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
#define ARM_DEBUG_MAX_STEP   (0.20f)  /* Ozone 单次最多移动 20 cm。 */
#define ARM_FORBIDDEN_X      (-0.40f) /* 与参考工程相同的车体禁区。 */
#define ARM_FORBIDDEN_Z      (0.30f)
#define ARM_AK1_SPEED_LIMIT  (15.0f)  /* rad/s */
#define ARM_AK2_SPEED_LIMIT  (7.0f)

/* 先使用参考工程的常用刚度；需要调节时只改这里。 */
//大臂 运动时
#define ARM_AK1_KP            (12.0f)
#define ARM_AK1_KD            (2.5f)
//小臂 运动时
#define ARM_AK2_KP            (15.0f)
#define ARM_AK2_KD            (2.5f)
//大臂 保持时
#define ARM_AK1_HOLD_KP       (15.0f)
#define ARM_AK1_HOLD_KD       (3.0f)
//小臂 保持时
#define ARM_AK2_HOLD_KP       (18.0f)
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
volatile ArmPoint arm_current_tool_point = {0.0f, 0.0f};
//运动标志 1表示正在执行轨迹 0表示保持中
volatile uint8_t arm_motion_active = 0U;
// EL05主动上报配置的最近一次发送结果，0表示成功。
volatile uint32_t arm_el05_report_status = 0U;
// CAN发送诊断信息，在 Ozone 中展开 arm_debug 观察。
volatile ArmDebug arm_debug = {0};
// Ozone 坐标调试入口：修改 x、z 后将 execute 置 1。
volatile ArmPointDebug arm_point_debug = {0};
// 故障状态：active=1 时查看 code 和 action，排障后将 reset 置 1。
volatile ArmFaultState arm_fault = {0};
//有新目标待处理：Ozone 中改完 arm_target_joint 后置 1。
volatile uint8_t arm_target_pending = 0U;
//轨迹结构体 保存起点 终点 时长 已用时间 active标志
static ArmTrajectory arm_trajectory = {0};

/* EL05 不加入 AK 关节轨迹，单独做目标平滑和限速。 */
static float arm_wrist_command = 0.0f;
static float arm_wrist_speed = 0.0f;
static float arm_wrist_smooth_target = 0.0f;
static uint8_t arm_wrist_initialized = 0U;
static uint8_t arm_auto_level_active = 0U;
static uint8_t arm_fault_monitor_reset = 1U;

static float arm_wrap_pi(float angle)
{
    while (angle > ARM_PI) angle -= 2.0f * ARM_PI;
    while (angle < -ARM_PI) angle += 2.0f * ARM_PI;
    return angle;
}

static float arm_angle_diff(float target, float current)
{
    return arm_wrap_pi(target - current);
}

static float arm_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/* 参考工程的防撞禁区：腕根不能进入车体左上区域。 */
static uint8_t arm_point_is_forbidden(float x, float z)
{
    return (uint8_t)((x < ARM_FORBIDDEN_X) && (z > ARM_FORBIDDEN_Z));
}

/* 参考工程的腕部控制：目标平滑后按最大速度逐周期逼近。 */
static void arm_update_wrist_command(float target)
{
    const float max_speed = 4.0f; /* rad/s */
    const float max_step = max_speed * ARM_CONTROL_DT;
    float target_error;
    float delta;
    float step;

    target = arm_clampf(target, ARM_WRIST_MIN, ARM_WRIST_MAX);

    if (!arm_wrist_initialized)
    {
        arm_wrist_smooth_target = target;
        arm_wrist_command = arm_clampf(el05_mit_state.position_rad,
                                       ARM_WRIST_MIN, ARM_WRIST_MAX);
        arm_wrist_initialized = 1U;
    }

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
                                   ARM_WRIST_MIN, ARM_WRIST_MAX);
    arm_wrist_speed = step / ARM_CONTROL_DT;
}

/* 根据当前两关节反馈实时计算腕部水平目标。 */
static float arm_get_level_wrist_target(const ArmJoint *current)
{
    float target;

    if (current == 0) return 0.0f;
    target = arm_wrap_pi(-(current->q1 + current->q2) +
                         arm_debug.wrist_level_trim);
    return arm_clampf(target, ARM_WRIST_MIN, ARM_WRIST_MAX);
}
//？？？？从 volatile 拷贝到普通变量 volatile 不能直接整体赋值，只能逐成员拷贝。？？
static void copy_joint_from_volatile(ArmJoint *destination,
                                     const volatile ArmJoint *source)
{
    if ((destination == 0) || (source == 0)) return;

    destination->q1 = source->q1;
    destination->q2 = source->q2;
    destination->wrist = source->wrist;
}
// 从普通变量拷贝到 volatile 同样逐成员赋值。
static void copy_joint_to_volatile(volatile ArmJoint *destination,
                                   const ArmJoint *source)
{
    if ((destination == 0) || (source == 0)) return;

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

/* 故障锁存：保持类故障锁当前位置，严重故障直接失能。 */
static void arm_raise_fault(uint8_t code, uint8_t action,
                            const ArmJoint *current)
{
    if (arm_fault.active) return;

    arm_fault.active = 1U;
    arm_fault.code = code;
    arm_fault.action = action;
    arm_fault.reset = 0U;
    arm_fault.tick = HAL_GetTick();
    arm_trajectory.active = 0U;
    arm_motion_active = 0U;
    arm_target_pending = 0U;

    if ((action == ARM_FAULT_ACTION_HOLD) && (current != 0))
        copy_joint_to_volatile(&arm_target_joint, current);
    else if (action == ARM_FAULT_ACTION_DISABLE)
        arm_disable_all();
}

/* 驱动器、速度和碰撞检测。 */
static void arm_fault_monitor(const ArmJoint *current)
{
    static uint32_t last_tick[3] = {0};
    static float last_torque[2] = {0.0f, 0.0f};
    static uint8_t speed_count[2] = {0};
    static uint8_t collision_count = 0U;
    uint8_t new_ak1;
    uint8_t new_ak2;

    if (arm_fault.active) return;

    /* 上电或故障复位后重新建立检测基准，避免旧计数误触发。 */
    if (arm_fault_monitor_reset)
    {
        last_tick[0] = ak_mit_state[0].last_rx_tick;
        last_tick[1] = ak_mit_state[1].last_rx_tick;
        last_tick[2] = el05_mit_state.last_rx_tick;
        last_torque[0] = ak_mit_state[0].torque_nm;
        last_torque[1] = ak_mit_state[1].torque_nm;
        speed_count[0] = 0U;
        speed_count[1] = 0U;
        collision_count = 0U;
        arm_fault_monitor_reset = 0U;
        return;
    }

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

    new_ak1 = (ak_mit_state[0].last_rx_tick != last_tick[0]);
    new_ak2 = (ak_mit_state[1].last_rx_tick != last_tick[1]);

    if (new_ak1)
    {
        last_tick[0] = ak_mit_state[0].last_rx_tick;
        if (fabsf(ak_mit_state[0].velocity_rad_s) > ARM_AK1_SPEED_LIMIT)
            speed_count[0]++;
        else
            speed_count[0] = 0U;
    }
    if (new_ak2)
    {
        last_tick[1] = ak_mit_state[1].last_rx_tick;
        if (fabsf(ak_mit_state[1].velocity_rad_s) > ARM_AK2_SPEED_LIMIT)
            speed_count[1]++;
        else
            speed_count[1] = 0U;
    }
    last_tick[2] = el05_mit_state.last_rx_tick;

    if (speed_count[0] >= 8U)
        arm_raise_fault(ARM_FAULT_AK1_OVERSPEED, ARM_FAULT_ACTION_DISABLE, current);
    else if (speed_count[1] >= 8U)
        arm_raise_fault(ARM_FAULT_AK2_OVERSPEED, ARM_FAULT_ACTION_DISABLE, current);
    if (arm_fault.active) return;

    /* 与参考工程一致：AK 反馈力矩连续突变视为碰撞。 */
    if (new_ak1 || new_ak2)
    {
        uint8_t collision = 0U;

        if (new_ak1 &&
            (fabsf(ak_mit_state[0].torque_nm - last_torque[0]) > 5.0f))
            collision = 1U;
        if (new_ak2 &&
            (fabsf(ak_mit_state[1].torque_nm - last_torque[1]) > 4.5f))
            collision = 1U;

        if (new_ak1) last_torque[0] = ak_mit_state[0].torque_nm;
        if (new_ak2) last_torque[1] = ak_mit_state[1].torque_nm;

        if (collision)
            collision_count++;
        else if (collision_count > 0U)
            collision_count--;

        if (collision_count >= 5U)
            arm_raise_fault(ARM_FAULT_COLLISION, ARM_FAULT_ACTION_HOLD, current);
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

    if (arm_fault.active) return ARM_POINT_FAULT;
    if (!isfinite(x) || !isfinite(z)) return ARM_POINT_INVALID;
    if (arm_point_is_forbidden(x, z)) return ARM_POINT_FORBIDDEN;

    copy_joint_from_volatile(&current, &arm_current_joint);
    if (!ArmMath_Inverse(x, z, &current, &target)) return ARM_POINT_IK_ERROR;

    /* 与参考工程 READY 模式一致：wrist = -(q1 + q2)。 */
    target.wrist = arm_wrap_pi(-(target.q1 + target.q2) +
                               arm_debug.wrist_level_trim);
    if ((target.wrist < ARM_WRIST_MIN) || (target.wrist > ARM_WRIST_MAX))
        return ARM_POINT_WRIST_LIMIT;

    arm_auto_level_active = 1U;
    Arm_SetJointTarget(target.q1, target.q2, target.wrist);
    return ARM_POINT_OK;
}
//保持当前位姿
void Arm_HoldCurrent(void)
{
    ArmJoint current;

    copy_joint_from_volatile(&current, &arm_current_joint);
    arm_auto_level_active = 0U;
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
    ArmMath_Forward(&current, &wrist_point, &tool_point);
    arm_current_wrist_point.x = wrist_point.x;
    arm_current_wrist_point.z = wrist_point.z;
    arm_current_tool_point.x = tool_point.x;
    arm_current_tool_point.z = tool_point.z;
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
    float wrist_torque;
    ArmJoint current;

    if ((position == 0) || (velocity == 0)) return;

    ArmMath_JointToMotor(position, &motor1_position, &motor2_position);
    ArmMath_JointVelocityToMotor(velocity, &motor1_velocity, &motor2_velocity);
    copy_joint_from_volatile(&current, &arm_current_joint);
    ArmMath_GravityTorque(&current, &torque1, &torque2);

    /* 参考工程的空载腕部重力前馈：m=0.3 kg，质心距离=0.08 m。 */
    wrist_torque = 0.30f * 9.8f * 0.08f *
                   cosf(current.q1 + current.q2 + position->wrist);
    wrist_torque = arm_clampf(wrist_torque, -1.0f, 1.0f);

    /* EL05 先发送，避免它总排在两个 AK 控制帧之后。 */
    arm_debug.el05_command_position = position->wrist;
    arm_debug.el05_command_kp = el05_kp;
    arm_debug.el05_command_torque = wrist_torque;
    arm_debug.el05_tx_status = (uint32_t)EL05_MIT_Control(
        &hfdcan1, position->wrist, velocity->wrist,
        el05_kp, el05_kd, wrist_torque);

    arm_debug.ak1_tx_status = (uint32_t)AK_MIT_Control(
        &hfdcan1, 1U, motor1_position, motor1_velocity, kp1, kd1, torque1);
    arm_debug.ak2_tx_status = (uint32_t)AK_MIT_Control(
        &hfdcan1, 2U, motor2_position, motor2_velocity, kp2, kd2, torque2);
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
    uint32_t next_report_tick;
    uint32_t next_probe_tick;
    uint32_t feedback_wait_start;
    (void)argument;//避免未使用参数警告

    /* 按参考工程：先把手动摆好的当前位置设为 AK 零点。 */
    AK_MIT_SetZero(&hfdcan1, 1U);
    osDelay(10);
    AK_MIT_SetZero(&hfdcan1, 2U);
    osDelay(10);

    /* 置零后再使能，后续控制帧由本任务统一发送。 */
    AK_MIT_Enable(&hfdcan1, 1U);
    osDelay(10);
    AK_MIT_Enable(&hfdcan1, 2U);
    osDelay(10);
    EL05_MIT_Enable(&hfdcan1);
    osDelay(10);
    arm_el05_report_status = (uint32_t)EL05_MIT_EnableAutoReport(&hfdcan1);
    next_report_tick = HAL_GetTick() + 100U;
    next_probe_tick = HAL_GetTick();
    feedback_wait_start = HAL_GetTick();

    /* 任务必须一直运行，不能执行到函数末尾。 */
    for (;;)
    {
        /* 主动上报配置偶尔会丢帧，周期重发确保反馈持续开启。 */
        if ((int32_t)(HAL_GetTick() - next_report_tick) >= 0)
        {
            arm_el05_report_status = (uint32_t)EL05_MIT_EnableAutoReport(&hfdcan1);
            next_report_tick += 100U;
        }

        arm_receive_feedback();//接收与解析所有can反馈

        /* 故障排除后在 Ozone 将 arm_fault.reset 写 1，重新使能。 */
        if (arm_fault.reset)
        {
            arm_fault.reset = 0U;
            if (arm_fault.active)
            {
                arm_fault.active = 0U;
                arm_fault.code = ARM_FAULT_NONE;
                arm_fault.action = ARM_FAULT_ACTION_NONE;
                arm_fault.tick = 0U;
                AK_MIT_Enable(&hfdcan1, 1U);
                osDelay(10);
                AK_MIT_Enable(&hfdcan1, 2U);
                osDelay(10);
                EL05_MIT_Enable(&hfdcan1);
                osDelay(10);
                arm_el05_report_status =
                    (uint32_t)EL05_MIT_EnableAutoReport(&hfdcan1);
                arm_fault_monitor_reset = 1U;
                initialized = 0U;
                next_probe_tick = HAL_GetTick();
                feedback_wait_start = HAL_GetTick();
            }
        }

        //?????/* 三台电机都在线后才发送位置命令，避免目标默认为 0。 */
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

        arm_update_feedback();//更新当前关节角 腕部点 工具点
        //？？？把全局 arm_current_joint 拷贝到局部 current。
        copy_joint_from_volatile(&current, &arm_current_joint);
        //首次初始化
        if (!initialized)
        {
            /* 没有外部目标时，上电默认保持当前位置。 */
            if (!arm_target_pending)
            {
                copy_joint_to_volatile(&arm_target_joint, &current);
            }
            /* 调试输入默认显示当前位置，上电不会自行运动。 */
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

        /* Ozone：先改 x、z，最后把 execute 改为 1。 */
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

        /* 运行中进入参考工程禁区时，锁存故障并保持当前位置。 */
        if (arm_point_is_forbidden(arm_current_wrist_point.x,
                                   arm_current_wrist_point.z))
            arm_raise_fault(ARM_FAULT_FORBIDDEN,
                            ARM_FAULT_ACTION_HOLD, &current);

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

        if (arm_trajectory.active)
        {
            if (ArmMath_TrajectoryUpdate(&arm_trajectory, ARM_CONTROL_DT,
                                         &command_position,
                                         &command_velocity,
                                         &command_acceleration))
            {
                arm_motion_active = 0U;//如果轨迹完成时置零
            }
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
            command_velocity.q2 = 0.0f;
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
