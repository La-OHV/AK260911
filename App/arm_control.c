#include "arm_control.h"

#include "cmsis_os2.h"
#include "../Core/Inc/fdcan.h"
#include "../ak/ak_can.h"
#include "../ak/ak_mit.h"
#include "../el05/el05_mit.h"

/* 控制周期必须与任务中的 osDelay(2) 对应，单位：s。 */
#define ARM_CONTROL_DT       (0.002f)
#define ARM_FEEDBACK_TIMEOUT (500U)//反馈超时时间 如果500ms之内没有收到电机的反馈就认为它离线

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
//有新目标待处理-1 否为0
static volatile uint8_t arm_target_pending = 0U;
//轨迹结构体 保存
static ArmTrajectory arm_trajectory = {0};

static void copy_joint_from_volatile(ArmJoint *destination,
                                     const volatile ArmJoint *source)
{
    if ((destination == 0) || (source == 0)) return;

    destination->q1 = source->q1;
    destination->q2 = source->q2;
    destination->wrist = source->wrist;
}

static void copy_joint_to_volatile(volatile ArmJoint *destination,
                                   const ArmJoint *source)
{
    if ((destination == 0) || (source == 0)) return;

    destination->q1 = source->q1;
    destination->q2 = source->q2;
    destination->wrist = source->wrist;
}

void Arm_SetJointTarget(float q1, float q2, float wrist)
{
    ArmJoint target = {q1, q2, wrist};

    ArmMath_ClampJoint(&target);
    copy_joint_to_volatile(&arm_target_joint, &target);
    arm_target_pending = 1U;
}

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

void Arm_HoldCurrent(void)
{
    ArmJoint current;

    copy_joint_from_volatile(&current, &arm_current_joint);
    Arm_SetJointTarget(current.q1, current.q2, current.wrist);
}

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

static uint8_t arm_feedback_ready(void)
{
    return (uint8_t)(AK_MIT_IsOnline(1U, ARM_FEEDBACK_TIMEOUT) &&
                     AK_MIT_IsOnline(2U, ARM_FEEDBACK_TIMEOUT) &&
                     EL05_MIT_IsOnline(ARM_FEEDBACK_TIMEOUT));
}

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

    /* AK ID1、ID2 和 EL05 ID0x03 均挂在 FDCAN1。 */
    AK_MIT_Control(&hfdcan1, 1U, motor1_position, motor1_velocity,
                   kp1, kd1, torque1);
    AK_MIT_Control(&hfdcan1, 2U, motor2_position, motor2_velocity,
                   kp2, kd2, torque2);
    EL05_MIT_Control(&hfdcan1, position->wrist, velocity->wrist,
                     el05_kp, el05_kd, 0.0f);
}

void Arm_control(void *argument)
{
    ArmJoint current;
    ArmJoint target;
    ArmJoint command_position;
    ArmJoint command_velocity;
    ArmJoint command_acceleration;
    uint8_t initialized = 0U;

    (void)argument;

    /* 只发送一次使能，后续控制帧由本任务统一发送。 */
    AK_MIT_Enable(&hfdcan1, 1U);
    osDelay(10);
    AK_MIT_Enable(&hfdcan1, 2U);
    osDelay(10);
    EL05_MIT_Enable(&hfdcan1);

    /* 任务必须一直运行，不能执行到函数末尾。 */
    for (;;)
    {
        arm_receive_feedback();

        /* 三台电机都在线后才发送位置命令，避免目标默认为 0。 */
        if (!arm_feedback_ready())
        {
            osDelay(2);
            continue;
        }

        arm_update_feedback();
        copy_joint_from_volatile(&current, &arm_current_joint);

        if (!initialized)
        {
            /* 没有外部目标时，上电默认保持当前位置。 */
            if (!arm_target_pending)
            {
                copy_joint_to_volatile(&arm_target_joint, &current);
            }
            initialized = 1U;
        }

        if (arm_target_pending)
        {
            copy_joint_from_volatile(&target, &arm_target_joint);
            ArmMath_TrajectoryStart(&arm_trajectory, &current, &target,
                                    ArmMath_CalcTime(&current, &target));
            arm_target_pending = 0U;
            arm_motion_active = 1U;
        }

        if (arm_trajectory.active)
        {
            if (ArmMath_TrajectoryUpdate(&arm_trajectory, ARM_CONTROL_DT,
                                         &command_position,
                                         &command_velocity,
                                         &command_acceleration))
            {
                arm_motion_active = 0U;
            }

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
            command_velocity.wrist = 0.0f;
            arm_send_command(&target, &command_velocity,
                             ARM_AK1_HOLD_KP, ARM_AK1_HOLD_KD,
                             ARM_AK2_HOLD_KP, ARM_AK2_HOLD_KD,
                             ARM_EL05_KP, ARM_EL05_KD);
        }

        osDelay(2);
    }
}
