#include "arm_control.h"
#include "cmsis_os2.h"
#include "../Core/Inc/fdcan.h"
#include "../ak/ak_can.h"
#include "../ak/ak_mit.h"
#include "../el05/el05_mit.h"
#include <math.h>
/* 控制周期必须与任务中的 osDelay(2) 对应，单位：s。 */
#define ARM_CONTROL_DT       (0.002f)
#define ARM_FEEDBACK_TIMEOUT (500U)  /* 运行中超过 500 ms 无反馈即停机。 */
#define ARM_STARTUP_TIMEOUT  (5000U) /* 上电等待三台电机反馈的最长时间。 */
#define ARM_PROBE_PERIOD     (100U)  /* 等待反馈时的安全查询周期。 */
#define ARM_DEBUG_MAX_STEP   (0.40f)  /* Ozone 单次最多移动 40 cm。 */
//大臂 运动时
#define ARM_AK1_KP            (40.0f)
#define ARM_AK1_KD            (1.5f)
//小臂 运动时
#define ARM_AK2_KP            (40.0f)
#define ARM_AK2_KD            (1.5f)
//大臂 保持时
#define ARM_AK1_HOLD_KP       (40.0f)
#define ARM_AK1_HOLD_KD       (2.0f)
//小臂 保持时
#define ARM_AK2_HOLD_KP       (40.0f)
#define ARM_AK2_HOLD_KD       (2.0f)
//腕部
#define ARM_EL05_KP           (100.0f)
#define ARM_EL05_KD           (1.0f)
//当前关节角
volatile ArmJoint arm_current_joint = {0.0f, 0.0f, 0.0f};
//目标关节角
volatile ArmJoint arm_target_joint = {0.0f, 0.0f, 0.0f};
//当前腕部点坐标
volatile ArmPoint arm_current_wrist_point = {0.0f, 0.0f};
//工具末端点坐标
//volatile ArmPoint arm_current_tool_point = {0.0f, 0.0f};
//运动标志 1表示正在执行轨迹 0表示保持中
volatile uint8_t arm_motion_active = 0U;
// EL05主动上报配置的最近一次发送结果，0表示成功。
volatile uint32_t arm_el05_report_status = 0U;
// 重力前馈力矩系数
volatile ArmDebug arm_debug = {
    .ak1_gravity_scale = 0.8f,
    .ak2_gravity_scale = 1.0f
};
// Ozone 坐标调试入口：修改 x、z 后将 execute 置 1。
volatile ArmPointDebug arm_point_debug = {0};
// 故障状态：active=1 时查看 code 和 action，排障后将 reset 置 1。
volatile ArmFaultState arm_fault = {0};
//有新目标待处理：Ozone 中改完 arm_target_joint 后置 1。
volatile uint8_t arm_target_pending = 0U;
//轨迹结构体 保存起点 终点 时长 已用时间 active标志
static ArmTrajectory arm_trajectory = {0};

/* EL05 不加入 AK 关节轨迹，单独做目标平滑和限速。 */
static float arm_wrist_command = 0.0f;//当前实际下发的腕部位置
static float arm_wrist_speed = 0.0f;//当前腕部速度
static float arm_wrist_smooth_target = 0.0f;//对腕部目标做低通平滑后的值，避免目标阶跃
static uint8_t arm_wrist_initialized = 0U;//腕部命令是否被初始化过（第一次用当前反馈位置做起点）
static uint8_t arm_auto_level_active = 0U;//腕部自动水平补偿模式（q1与q2变了就自动修wrist）
//角度归一化到-pai到pai
static float arm_wrap_pi(float angle)
{
    while (angle > ARM_PI) angle -= 2.0f * ARM_PI;
    while (angle < -ARM_PI) angle += 2.0f * ARM_PI;
    return angle;
}
//求最短角度差 179到-179 只差2 而非358
static float arm_angle_diff(float target, float current)
{
    return arm_wrap_pi(target - current);
}
//限幅
static float arm_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}
/* 腕部目标平滑 */
static void arm_update_wrist_command(float target)
{
    const float max_speed = 4.0f; /* rad/s */
    const float max_step = max_speed * ARM_CONTROL_DT;//每周期最大变化量
    float target_error;//原始目标与平滑目标的差
    float delta;//rad 平滑目标与实际命令之间的差 比如目标为1rad 当前命令是0rad 那delta=1
    float step;//rad 就是经过max_step限幅后的delta  表示实际允许走过的角度增量
     //对输入目标的限幅
    target = arm_clampf(target, ARM_WRIST_MIN, ARM_WRIST_MAX);
     //只在第一次调用时执行 就是一上电直接让wrist保持到当前位置而不是从默认的0开始 再到目标
    if (!arm_wrist_initialized)
    {
        arm_wrist_smooth_target = target;//平滑目标直接=目标
        arm_wrist_command = arm_clampf(el05_mit_state.position_rad,
                                       ARM_WRIST_MIN, ARM_WRIST_MAX);
        arm_wrist_initialized = 1U;
    }
     //一阶低通滤波器 指数平滑
    target_error = arm_angle_diff(target, arm_wrist_smooth_target);
    if (fabsf(target_error) < 0.01f)
        arm_wrist_smooth_target = target;
    else
        arm_wrist_smooth_target += 0.10f * target_error;

    arm_wrist_smooth_target = arm_clampf(arm_wrist_smooth_target,
                                         ARM_WRIST_MIN, ARM_WRIST_MAX);

    delta = arm_angle_diff(arm_wrist_smooth_target, arm_wrist_command);
    step = arm_clampf(delta, -max_step, max_step);
    arm_wrist_command = arm_clampf(arm_wrist_command + step,
                                   ARM_WRIST_MIN, ARM_WRIST_MAX);//作为命令下发
    arm_wrist_speed = step / ARM_CONTROL_DT;//作为速度前馈下发
    //这样相当于是两级平滑 一个是低通 让目标更加连续 软化；另外一个是限速 保护电机与机械
}

/* 根据当前两关节反馈实时计算腕部水平目标。腕部保持水平就是-（q1+q2） */
static float arm_get_level_wrist_target(const ArmJoint *current)
{
    float target;

    if (current == 0) return 0.0f;
    target = arm_wrap_pi(-(current->q1 + current->q2) +
                         arm_debug.wrist_level_trim);//arm_debug.wrist_level_trim微调 删？
    return arm_clampf(target, ARM_WRIST_MIN, ARM_WRIST_MAX);
}
//volatile ArmJoint arm_current_joint;  // 当前关节角，可能被中断、其他任务或调试器修改
//volatile ArmJoint arm_target_joint;   // 目标关节角，Ozone 调试时可能手动改
//从volatile源到普通目标 把arm_current_joint拷贝到一个普通的局部变量里 供后续计算使用 这样计算时编译器就可以优化了 相当于得到一个快照 就算全局变量被中断改变了 也不会影响这次计算
static void copy_joint_from_volatile(ArmJoint *destination,
                                     const volatile ArmJoint *source)
{
    if ((destination == 0) || (source == 0)) return;//空指针检查

    destination->q1 = source->q1;
    destination->q2 = source->q2;
    destination->wrist = source->wrist;
}
// 从普通变量拷贝到 volatile 把计算好的目标关节角写回全局的arm_target_joint里面
static void copy_joint_to_volatile(volatile ArmJoint *destination,
                                   const ArmJoint *source)
{
    if ((destination == 0) || (source == 0)) return;//空指针检查

    destination->q1 = source->q1;
    destination->q2 = source->q2;
    destination->wrist = source->wrist;
}
/* 硬故障：立即停止三台电机。 */
static void arm_disable_all(void)
{
    AK_MIT_Disable(&hfdcan1, 1U);
    AK_MIT_Disable(&hfdcan1, 2U);
    EL05_MIT_Disable(&hfdcan1);
}

/* 故障锁存：保持类故障锁当前位置，严重故障直接失能。
 * code-故障码用来标识发生了什么故障
 * action-hold or distable
 * current 就是在action为hold的时候使用的 让关节保持当前位置
 */
static void arm_raise_fault(uint8_t code, uint8_t action,
                            const ArmJoint *current)
{
    if (arm_fault.active) return;//如果已经故障了 那就不覆盖已有的故障信息
    arm_fault.active = 1U;
    arm_fault.code = code;
    arm_fault.action = action;
    arm_fault.reset = 0U;//ozone调试使用 若排除故障之后可以手动写1 可删？
    arm_trajectory.active = 0U;//停止当前正在执行的轨迹
    arm_motion_active = 0U;//标记机械臂不再处于运动状态
    arm_target_pending = 0U;//清除有新目标待处理的标志 不让发生故障之后还去执行就目标
//如果是hold就hold 否则失能
    if ((action == ARM_FAULT_ACTION_HOLD) && (current != 0))
        copy_joint_to_volatile(&arm_target_joint, current);
    else if (action == ARM_FAULT_ACTION_DISABLE)
        arm_disable_all();
}

/*
 * current用于故障选择保持
 */
static void arm_fault_monitor(const ArmJoint *current)
{
    if (arm_fault.active) return;//封存第一次故障
     //检查三个电机反馈结构体 结构体中带有错误标志 直接失能
    if (ak_mit_state[0].error)
    {
        arm_raise_fault(ARM_FAULT_AK1_DRIVER, ARM_FAULT_ACTION_DISABLE, current);
        return;
    }
    if (ak_mit_state[1].error)
    {
        arm_raise_fault(ARM_FAULT_AK2_DRIVER, ARM_FAULT_ACTION_DISABLE, current);
        return;
    }
    if (el05_mit_state.fault)
    {
        arm_raise_fault(ARM_FAULT_EL05_DRIVER, ARM_FAULT_ACTION_DISABLE, current);
        return;
    }

}

//设置关节目标
void Arm_SetJointTarget(float q1, float q2, float wrist)
{
    ArmJoint target = {q1, q2, wrist};
    if (arm_fault.active) return;
    ArmMath_ClampJoint(&target);
    copy_joint_to_volatile(&arm_target_joint, &target);
    arm_target_pending = 1U;
}
//设置腕根坐标目标，腕部自动补偿 q1+q2，使工具保持水平。
uint8_t Arm_SetPointTarget(float x, float z)
{
    ArmJoint current;
    ArmJoint target;

    if (arm_fault.active) return ARM_POINT_FAULT;//已经故障 不允许再设置目标
    copy_joint_from_volatile(&current, &arm_current_joint);
    if (!ArmMath_Inverse(x, z, &current, &target)) return ARM_POINT_IK_ERROR;
    target.wrist = arm_wrap_pi(-(target.q1 + target.q2) );
    if ((target.wrist < ARM_WRIST_MIN) || (target.wrist > ARM_WRIST_MAX))
        return ARM_POINT_WRIST_LIMIT;
    arm_auto_level_active = 1U;//启动自动水平补偿
    Arm_SetJointTarget(target.q1, target.q2, target.wrist);
    return ARM_POINT_OK;
}
//保持当前位姿
void Arm_HoldCurrent(void)
{
    ArmJoint current;

    copy_joint_from_volatile(&current, &arm_current_joint);
    arm_auto_level_active = 0U;//停止自动水平补偿
    Arm_SetJointTarget(current.q1, current.q2, current.wrist);
}
//反馈接收与解析
static void arm_receive_feedback(void)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    /* FDCAN1 只能由一个任务读取，避免 RX FIFO 被多个任务抢走。 */
    while (AK_CAN_Receive(&hfdcan1, &header, data))
    {
        AK_MIT_ParseFeedback(&header, data);
        EL05_MIT_ParseFeedback(&header, data);
    }
}
//反馈就绪检查
static uint8_t arm_feedback_ready(void)
{
    return (uint8_t)(AK_MIT_IsOnline(1U, ARM_FEEDBACK_TIMEOUT) &&
                     AK_MIT_IsOnline(2U, ARM_FEEDBACK_TIMEOUT) &&
                     EL05_MIT_IsOnline(ARM_FEEDBACK_TIMEOUT));
}
//反馈更新
static void arm_update_feedback(void)
{
    ArmJoint current;
    ArmPoint wrist_point;
    ArmPoint tool_point;

    ArmMath_MotorToJoint(ak_mit_state[0].position_rad,
                         ak_mit_state[1].position_rad,
                         &current);
    current.wrist = el05_mit_state.position_rad;

    copy_joint_to_volatile(&arm_current_joint, &current);
    ArmMath_Forward(&current, &wrist_point);
    arm_current_wrist_point.x = wrist_point.x;
    arm_current_wrist_point.z = wrist_point.z;
}
//发送命令函数
static void arm_send_command(const ArmJoint *position,
                             const ArmJoint *velocity,
                             float kp1, float kd1,
                             float kp2, float kd2,
                             float el05_kp, float el05_kd)
{
    float motor1_position;
    float motor2_position;
    float motor1_velocity;
    float motor2_velocity;
    float torque1;
    float torque2;
    ArmJoint current;
    if ((position == 0) || (velocity == 0)) return;
    //关节角转电机角
    ArmMath_JointToMotor(position, &motor1_position, &motor2_position);
    //关节速度转电机速度
    ArmMath_JointVelocityToMotor(velocity, &motor1_velocity, &motor2_velocity);
    copy_joint_from_volatile(&current, &arm_current_joint);
    ArmMath_GravityTorque(&current, &torque1, &torque2);
    torque1 *= arm_clampf(arm_debug.ak1_gravity_scale, 0.0f, 1.2f);
    torque2 *= arm_clampf(arm_debug.ak2_gravity_scale, 0.0f, 1.2f);
    arm_debug.ak1_gravity_torque = torque1;
    arm_debug.ak2_gravity_torque = torque2;
    arm_debug.el05_command_position = position->wrist;
    arm_debug.el05_command_kp = el05_kp;
    arm_debug.el05_tx_status = (uint32_t)EL05_MIT_Control(
        &hfdcan1, position->wrist, velocity->wrist,
        el05_kp, el05_kd, 0);
    arm_debug.ak1_tx_status = (uint32_t)AK_MIT_Control(
        &hfdcan1, 1U, motor1_position, motor1_velocity, kp1, kd1, torque1);
    arm_debug.ak2_tx_status = (uint32_t)AK_MIT_Control(
        &hfdcan1, 2U, motor2_position, motor2_velocity, kp2, kd2, torque2);
    // arm_debug.el05_tx_status = (uint32_t)EL05_MIT_Control(
    //     &hfdcan1, 0.0, 0.0,
    //     0.0, 0.0, 0.0);
    //
    // arm_debug.ak1_tx_status = (uint32_t)AK_MIT_Control(
    //     &hfdcan1, 1U, 0.0, 0.0, 0.0, 0.0f, torque1);
    // arm_debug.ak2_tx_status = (uint32_t)AK_MIT_Control(
    //     &hfdcan1, 2U, 0.0, 0.0, 0.0, 0.0f, torque2);
}
//主任务
void Arm_control(void *argument)
{
    ArmJoint current;//当前关节
    ArmJoint target;//目标关节
    ArmJoint command_position;//命令位置
    ArmJoint command_velocity;//命令速度
    ArmJoint command_acceleration;//命令加速度
    uint8_t initialized = 0U;//标志是否完成初次初始化
    uint32_t next_report_tick;//el05下次上报时间
    uint32_t next_probe_tick;//控制探测频率 唤醒ak回反馈
    uint32_t feedback_wait_start;//计算等待反馈的总时长
    (void)argument;//避免未使用参数警告
    /* 当前位置设为 AK 零点。 */
    AK_MIT_SetZero(&hfdcan1, 1U);
    osDelay(10);
    AK_MIT_SetZero(&hfdcan1, 2U);
    osDelay(10);
    /* 使能 */
    AK_MIT_Enable(&hfdcan1, 1U);
    osDelay(10);
    AK_MIT_Enable(&hfdcan1, 2U);
    osDelay(10);
    EL05_MIT_Enable(&hfdcan1);
    osDelay(10);
    arm_el05_report_status = (uint32_t)EL05_MIT_EnableAutoReport(&hfdcan1);//让el05主动上报
    next_report_tick = HAL_GetTick() + 100U;
    next_probe_tick = HAL_GetTick();
    feedback_wait_start = HAL_GetTick();
    for (;;)
    {
        /* 主动上报 */
        if ((int32_t)(HAL_GetTick() - next_report_tick) >= 0)
        {
            arm_el05_report_status = (uint32_t)EL05_MIT_EnableAutoReport(&hfdcan1);
            next_report_tick += 100U;
        }
        arm_receive_feedback();//接收与解析所有can反馈
        /* 三台电机都在线后才发送位置命令，避免目标默认为 0。 */
        if (!arm_feedback_ready())
        {
            /* AK 通常在收到控制帧后才反馈；零刚度查询不会驱动电机。 */
            if ((int32_t)(HAL_GetTick() - next_probe_tick) >= 0)
            {
                next_probe_tick += ARM_PROBE_PERIOD;
                if (!AK_MIT_IsOnline(1U, ARM_FEEDBACK_TIMEOUT))
                    AK_MIT_Control(&hfdcan1, 1U,
                                   0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                if (!AK_MIT_IsOnline(2U, ARM_FEEDBACK_TIMEOUT))
                    AK_MIT_Control(&hfdcan1, 2U,
                                   0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                if (!EL05_MIT_IsOnline(ARM_FEEDBACK_TIMEOUT))
                {
                    EL05_MIT_Enable(&hfdcan1);
                    arm_el05_report_status =
                        (uint32_t)EL05_MIT_EnableAutoReport(&hfdcan1);
                }
            }

            if (!arm_fault.active &&
                (initialized ||
                 ((HAL_GetTick() - feedback_wait_start) > ARM_STARTUP_TIMEOUT)))
            {
                if (!AK_MIT_IsOnline(1U, ARM_FEEDBACK_TIMEOUT))
                    arm_raise_fault(ARM_FAULT_AK1_OFFLINE,
                                    ARM_FAULT_ACTION_DISABLE, 0);
                else if (!AK_MIT_IsOnline(2U, ARM_FEEDBACK_TIMEOUT))
                    arm_raise_fault(ARM_FAULT_AK2_OFFLINE,
                                    ARM_FAULT_ACTION_DISABLE, 0);
                else
                    arm_raise_fault(ARM_FAULT_EL05_OFFLINE,
                                    ARM_FAULT_ACTION_DISABLE, 0);
            }
            osDelay(2);
            continue;
        }
        arm_update_feedback();//更新当前关节角 腕部点
        copy_joint_from_volatile(&current, &arm_current_joint);
        //首次初始化
        if (!initialized)
        {
            /* 没有外部目标时，上电默认保持当前位置。 */
            if (!arm_target_pending)
            {
                copy_joint_to_volatile(&arm_target_joint, &current);
            }
            arm_point_debug.x = arm_current_wrist_point.x;
            arm_point_debug.z = arm_current_wrist_point.z;
            arm_point_debug.execute = 0U;
            arm_point_debug.result = ARM_POINT_IDLE;
            initialized = 1U;
        }
        /* 硬故障保持失能状态，不再发送运动控制帧。 */
        if (arm_fault.active &&
            (arm_fault.action == ARM_FAULT_ACTION_DISABLE))
        {
            arm_point_debug.execute = 0U;
            arm_point_debug.result = ARM_POINT_FAULT;
            osDelay(2);
            continue;
        }
        /* Ozone调试：先改 x、z，最后把 execute 改为 1。 */
        if (arm_point_debug.execute)
        {
            float x = arm_point_debug.x;
            float z = arm_point_debug.z;
            float dx = x - arm_current_wrist_point.x;
            float dz = z - arm_current_wrist_point.z;
            arm_point_debug.execute = 0U;
            if (arm_fault.active)
                arm_point_debug.result = ARM_POINT_FAULT;
            else if (arm_motion_active)
                arm_point_debug.result = ARM_POINT_BUSY;
            else if ((dx * dx + dz * dz) >
                     (ARM_DEBUG_MAX_STEP * ARM_DEBUG_MAX_STEP))
                arm_point_debug.result = ARM_POINT_STEP_TOO_LARGE;
            else
                arm_point_debug.result = Arm_SetPointTarget(x, z);
        }
        arm_fault_monitor(&current);
        /* 每周期读取目标。 */
        copy_joint_from_volatile(&target, &arm_target_joint);
        //如果有新目标待处理
        if (arm_target_pending)
        {
            /* AK1、AK2 有位移时才启动轨迹；只有腕部变化时不启动 AK 轨迹。 */
            if ((fabsf(arm_angle_diff(target.q1, current.q1)) > 0.001f) ||
                (fabsf(arm_angle_diff(target.q2, current.q2)) > 0.001f))
            {
                ArmMath_TrajectoryStart(&arm_trajectory, &current, &target,
                                        ArmMath_CalcTime(&current, &target));
                arm_motion_active = 1U;
            }
            else
            {
                arm_trajectory.active = 0U;
                arm_motion_active = 0U;
            }
            arm_target_pending = 0U;//清理新目标待处理
        }
        /* 收到坐标目标后，才根据实际 q1/q2 每周期修正腕部水平。 */
        if (arm_auto_level_active)
        {
            target.wrist = arm_get_level_wrist_target(&current);
            arm_target_joint.wrist = target.wrist;
        }
        arm_debug.wrist_level_target = target.wrist;
        arm_debug.wrist_level_error = arm_wrap_pi(
            current.q1 + current.q2 + current.wrist -
            arm_debug.wrist_level_trim);
        arm_update_wrist_command(target.wrist);
        if (arm_trajectory.active)//如果有轨迹正在跑
        {
            if (ArmMath_TrajectoryUpdate(&arm_trajectory, ARM_CONTROL_DT,
                                         &command_position,
                                         &command_velocity,
                                         &command_acceleration))//推进轨迹 如果计算时间完成 置0 轨迹规划 只针对ak1 2
            {
                arm_motion_active = 0U;//如果轨迹完成时置零
            }
            //单独对wrist
            command_position.wrist = arm_wrist_command;
            command_velocity.wrist = arm_wrist_speed;
            //发送运动命令
            arm_send_command(&command_position, &command_velocity,
                             ARM_AK1_KP, ARM_AK1_KD,
                             ARM_AK2_KP, ARM_AK2_KD,
                             ARM_EL05_KP, ARM_EL05_KD);
        }
        else
        {
            /* 到位后持续保持目标位置。 */
            copy_joint_from_volatile(&target, &arm_target_joint);
            command_velocity.q1 = 0.0f;
            command_velocity.q2 = 0.0f;//速度为0
            command_position.wrist = arm_wrist_command;
            command_velocity.wrist = arm_wrist_speed;
            target.wrist = arm_wrist_command;
            arm_send_command(&target, &command_velocity,
                             ARM_AK1_HOLD_KP, ARM_AK1_HOLD_KD,
                             ARM_AK2_HOLD_KP, ARM_AK2_HOLD_KD,
                             ARM_EL05_KP, ARM_EL05_KD);
        }

        osDelay(2);
    }
}
