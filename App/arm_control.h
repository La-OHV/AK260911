#ifndef AK260911_ARM_CONTROL_H
#define AK260911_ARM_CONTROL_H

#include "../Service/Service_math.h"

/* 调试信息：在 Ozone 中添加 arm_debug 后展开观察。 */
typedef struct
{
    uint32_t ak1_tx_status;          /* AK1 发送结果，0 表示成功。 */
    uint32_t ak2_tx_status;          /* AK2 发送结果，0 表示成功。 */
    uint32_t el05_tx_status;         /* EL05 发送结果，0 表示成功。 */
    float el05_command_position;     /* EL05 实际发送的目标位置，rad。 */
    float el05_command_kp;           /* EL05 实际发送的 Kp。 */
    float el05_command_torque;       /* EL05 重力前馈力矩，N·m。 */
    float wrist_level_target;        /* 自动水平目标，rad。 */
    float wrist_level_error;         /* 工具相对水平的误差，rad。 */
    float wrist_level_trim;          /* 水平安装微调，默认 0 rad。 */
} ArmDebug;

/* Ozone 坐标调试结果。 */
typedef enum
{
    ARM_POINT_IDLE = 0,          /* 等待输入 */
    ARM_POINT_OK,                /* 目标已接受 */
    ARM_POINT_IK_ERROR,          /* 不可达或关节角超限 */
    ARM_POINT_FORBIDDEN,         /* 进入参考工程的车体禁区 */
    ARM_POINT_WRIST_LIMIT,       /* 腕部无法保持水平 */
    ARM_POINT_STEP_TOO_LARGE,    /* 单次移动超过 20 cm */
    ARM_POINT_BUSY,              /* 上一次运动尚未完成 */
    ARM_POINT_INVALID,           /* 输入不是有效数值 */
    ARM_POINT_FAULT              /* 当前存在故障 */
} ArmPointResult;

/* Ozone 中只需填写 x、z，最后将 execute 改为 1。 */
typedef struct
{
    float x;                     /* 腕根目标 x，单位：m */
    float z;                     /* 腕根目标 z，单位：m */
    uint8_t execute;             /* 写 1 执行，程序会自动清零 */
    uint8_t result;              /* 执行结果，见 ArmPointResult */
} ArmPointDebug;

/* 故障码。 */
typedef enum
{
    ARM_FAULT_NONE = 0,
    ARM_FAULT_AK1_OFFLINE,
    ARM_FAULT_AK2_OFFLINE,
    ARM_FAULT_EL05_OFFLINE,
    ARM_FAULT_AK1_DRIVER,
    ARM_FAULT_AK2_DRIVER,
    ARM_FAULT_EL05_DRIVER,
    ARM_FAULT_AK1_OVERSPEED,
    ARM_FAULT_AK2_OVERSPEED,
    ARM_FAULT_EL05_OVERSPEED,
    ARM_FAULT_COLLISION,
    ARM_FAULT_FORBIDDEN
} ArmFaultCode;

typedef enum
{
    ARM_FAULT_ACTION_NONE = 0,
    ARM_FAULT_ACTION_HOLD,       /* 停止轨迹并保持当前位置 */
    ARM_FAULT_ACTION_DISABLE     /* 三台电机全部失能 */
} ArmFaultAction;

/* Ozone 中展开 arm_fault 查看；排除故障后将 reset 写 1。 */
typedef struct
{
    uint8_t active;
    uint8_t code;
    uint8_t action;
    uint8_t reset;
    uint32_t tick;
} ArmFaultState;

/* 当前反馈和当前目标，便于在调试器中观察或修改。 */
extern volatile ArmJoint arm_current_joint;
extern volatile ArmJoint arm_target_joint;
extern volatile uint8_t arm_target_pending;
extern volatile ArmPoint arm_current_wrist_point;
extern volatile ArmPoint arm_current_tool_point;
extern volatile uint8_t arm_motion_active;
extern volatile uint32_t arm_el05_report_status;
extern volatile ArmDebug arm_debug;
extern volatile ArmPointDebug arm_point_debug;
extern volatile ArmFaultState arm_fault;

/* 关节目标：q1、q2、wrist 单位均为 rad。 */
void Arm_SetJointTarget(float q1, float q2, float wrist);

/* 腕根 XZ 目标；腕部自动保持水平，返回 ArmPointResult。 */
uint8_t Arm_SetPointTarget(float x, float z);

/* 将当前反馈位置设为目标并保持。 */
void Arm_HoldCurrent(void);

/* FreeRTOS 机械臂控制任务。 */
void Arm_control(void *argument);

#endif
