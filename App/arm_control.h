#ifndef AK260911_ARM_CONTROL_H
#define AK260911_ARM_CONTROL_H

#include "../Service/Service_math.h"

/* 当前反馈和当前目标，便于在调试器中观察或修改。 */
extern volatile ArmJoint arm_current_joint;
extern volatile ArmJoint arm_target_joint;
extern volatile ArmPoint arm_current_wrist_point;
extern volatile ArmPoint arm_current_tool_point;
extern volatile uint8_t arm_motion_active;

/* 关节目标：q1、q2、wrist 单位均为 rad。 */
void Arm_SetJointTarget(float q1, float q2, float wrist);

/* 腕根 XZ 目标 + 腕部角度目标，成功返回 1。 */
uint8_t Arm_SetPointTarget(float x, float z, float wrist);

/* 将当前反馈位置设为目标并保持。 */
void Arm_HoldCurrent(void);

/* FreeRTOS 机械臂控制任务。 */
void Arm_control(void *argument);

#endif
