#ifndef AK260911_SERVICE_MATH_H
#define AK260911_SERVICE_MATH_H

#include <stdint.h>

#ifndef ARM_PI
#define ARM_PI (3.14159265358979323846f)
#endif

/* 与实际机械臂一致的连杆长度，单位：m。 */
#define ARM_L1             (0.35f)          /* 大臂 */
#define ARM_L2             (0.25f)          /* 小臂 */
#define ARM_L3             (0.10f)          /* 腕部/吸盘 */

/* 电机角与关节角之间的机械安装偏置。 */
#define ARM_OFFSET_DOWN   (3.141593f)
#define ARM_OFFSET_UP     (-2.6511548f)

/* 关节安全范围，单位：rad。 */
#define ARM_Q1_MIN        (-0.4f * ARM_PI)
#define ARM_Q1_MAX        ( ARM_PI)
#define ARM_Q2_MIN        (-0.9f * ARM_PI)
#define ARM_Q2_MAX        ( 0.9f * ARM_PI)
#define ARM_WRIST_MIN     (-0.75f * ARM_PI)
#define ARM_WRIST_MAX     ( 0.6666667f * ARM_PI)

/* 机械臂关节角，q1/q2 为相对关节角，wrist 为腕部电机角。 */
typedef struct
{
    float q1;
    float q2;
    float wrist;
} ArmJoint;

/* XZ 平面上的点。 */
typedef struct
{
    float x;
    float z;
} ArmPoint;

/* 三个关节共用一条五次多项式轨迹。 */
typedef struct
{
    ArmJoint start;
    ArmJoint end;
    float duration;
    float elapsed;
    uint8_t active;
} ArmTrajectory;

/* 电机角与关节角转换。 */
void ArmMath_MotorToJoint(float motor1, float motor2, ArmJoint *joint);
void ArmMath_JointToMotor(const ArmJoint *joint, float *motor1, float *motor2);
void ArmMath_JointVelocityToMotor(const ArmJoint *joint_velocity,
                                  float *motor1_velocity,
                                  float *motor2_velocity);

/* 二连杆正运动学：同时计算腕根位置和吸盘末端位置。 */
void ArmMath_Forward(const ArmJoint *joint,
                     ArmPoint *wrist_point,
                     ArmPoint *tool_point);

/* 根据腕根目标 x/z 计算 q1/q2，成功返回 1。 */
uint8_t ArmMath_Inverse(float x, float z,
                        const ArmJoint *current,
                        ArmJoint *target);

/* 限制关节角；返回值为 1 表示输入曾经越界。 */
uint8_t ArmMath_ClampJoint(ArmJoint *joint);

/* 根据关节行程估计运动时间，范围为 0.5~4.0 s。 */
float ArmMath_CalcTime(const ArmJoint *start, const ArmJoint *end);

/* 启动和更新五次多项式轨迹。更新完成返回 1。 */
void ArmMath_TrajectoryStart(ArmTrajectory *trajectory,
                             const ArmJoint *start,
                             const ArmJoint *end,
                             float duration);
uint8_t ArmMath_TrajectoryUpdate(ArmTrajectory *trajectory,
                                 float dt,
                                 ArmJoint *position,
                                 ArmJoint *velocity,
                                 ArmJoint *acceleration);

/* 按参考机械参数计算两个 AK 电机的重力前馈力矩。 */
void ArmMath_GravityTorque(const ArmJoint *joint,
                           float *motor1_torque,
                           float *motor2_torque);

#endif
