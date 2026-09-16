#include "arm_control.h"

#include "cmsis_os2.h"
#include "../Core/Inc/fdcan.h"
#include "../ak/ak_can.h"
#include "../ak/ak_mit.h"
#include "../el05/el05_mit.h"

/* 控制周期必须与任务中的 osDelay(2) 对应，单位：s。 */
#define ARM_CONTROL_DT       (0.002f)
#define ARM_FEEDBACK_TIMEOUT (2000U) /* 超过该时间没有反馈则认为电机离线。 */

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
//有新目标待处理：Ozone 中改完 arm_target_joint 后置 1。
volatile uint8_t arm_target_pending = 0U;
//轨迹结构体 保存起点 终点 时长 已用时间 active标志
static ArmTrajectory arm_trajectory = {0};

/* EL05 不加入 AK 关节轨迹，单独做目标平滑和限速。 */
static float arm_wrist_command = 0.0f;
static float arm_wrist_speed = 0.0f;
static float arm_wrist_smooth_target = 0.0f;
static uint8_t arm_wrist_initialized = 0U;

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
//设置关节目标
void Arm_SetJointTarget(float q1, float q2, float wrist)
{
    ArmJoint target = {q1, q2, wrist};

    ArmMath_ClampJoint(&target);
    copy_joint_to_volatile(&arm_target_joint, &target);
    arm_target_pending = 1U;
}
//设置笛卡尔点目标
uint8_t Arm_SetPointTarget(float x, float z, float wrist)
{
    ArmJoint current;
    ArmJoint target;

    copy_joint_from_volatile(&current, &arm_current_joint);
    if (!ArmMath_Inverse(x, z, &current, &target)) return 0U;

    target.wrist = wrist;
    Arm_SetJointTarget(target.q1, target.q2, target.wrist);
    return 1U;
}
//保持当前位姿
void Arm_HoldCurrent(void)
{
    ArmJoint current;

    copy_joint_from_volatile(&current, &arm_current_joint);
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
    ArmJoint current;

    if ((position == 0) || (velocity == 0)) return;

    ArmMath_JointToMotor(position, &motor1_position, &motor2_position);
    ArmMath_JointVelocityToMotor(velocity, &motor1_velocity, &motor2_velocity);
    copy_joint_from_volatile(&current, &arm_current_joint);
    ArmMath_GravityTorque(&current, &torque1, &torque2);

    /* EL05 先发送，避免它总排在两个 AK 控制帧之后。 */
    arm_debug.el05_command_position = position->wrist;
    arm_debug.el05_command_kp = el05_kp;
    arm_debug.el05_tx_status = (uint32_t)EL05_MIT_Control(
        &hfdcan1, position->wrist, velocity->wrist, el05_kp, el05_kd, 0.0f);

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
    uint8_t ik_test_once = 0U;
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
        /* 只调用一次逆解 */
        if (!ik_test_once)
        {
            ik_test_once = 1U;

            Arm_SetPointTarget(
                0.30f,                       // 目标 x，单位 m
                0.20f,                       // 目标 z，单位 m
                arm_current_joint.wrist     // 腕部保持当前角度
            );
        }
        //?????/* 三台电机都在线后才发送位置命令，避免目标默认为 0。 */
        if (!arm_feedback_ready())
        {
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
            initialized = 1U;
        }

        /* 每周期读取目标；wrist 目标可直接在调试器中观察。 */
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

        /* EL05 始终独立跟踪 wrist 目标，不使用 AK 五次轨迹输出。 */
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
