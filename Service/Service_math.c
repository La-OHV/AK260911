#include "Service_math.h"

#include <math.h>

/* 限幅1 */
static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}
//限制角度到-pai到pai
static float wrap_pi(float angle)
{
    while (angle > ARM_PI) angle -= 2.0f * ARM_PI;
    while (angle < -ARM_PI) angle += 2.0f * ARM_PI;
    return angle;
}
//计算两个角度之间最短的差值
static float angle_diff(float target, float current)
{
    return wrap_pi(target - current);
}
//电机角转关节角
void ArmMath_MotorToJoint(float motor1, float motor2, ArmJoint *joint)
{
    if (joint == 0) return;

    /* 大臂方向与关节方向相反，小臂只是平移零点。 */
    joint->q1 = ARM_OFFSET_DOWN - motor1;
    joint->q2 = motor2 + ARM_OFFSET_UP;
}
//关节角转电机角
void ArmMath_JointToMotor(const ArmJoint *joint, float *motor1, float *motor2)
{
    if ((joint == 0) || (motor1 == 0) || (motor2 == 0)) return;

    *motor1 = ARM_OFFSET_DOWN - joint->q1;
    *motor2 = joint->q2 - ARM_OFFSET_UP;
}
//关节速度转电机速度
void ArmMath_JointVelocityToMotor(const ArmJoint *joint_velocity,
                                  float *motor1_velocity,
                                  float *motor2_velocity)
{
    if ((joint_velocity == 0) || (motor1_velocity == 0) ||
        (motor2_velocity == 0)) return;

    /* motor1 = offset - q1，因此大臂电机速度必须取反。 */
    *motor1_velocity = -joint_velocity->q1;
    *motor2_velocity =  joint_velocity->q2;
}
//正运动学
void ArmMath_Forward(const ArmJoint *joint,
                     ArmPoint *wrist_point,
                     ArmPoint *tool_point)
{
    float q12;
    float wrist_x;
    float wrist_z;

    if ((joint == 0) || (wrist_point == 0) || (tool_point == 0)) return;

    q12 = joint->q1 + joint->q2;
    wrist_x = ARM_L1 * cosf(joint->q1) + ARM_L2 * cosf(q12);
    wrist_z = ARM_L1 * sinf(joint->q1) + ARM_L2 * sinf(q12);

    wrist_point->x = wrist_x;
    wrist_point->z = wrist_z;

    /* 吸盘末端还要叠加腕部 L3。 */
    tool_point->x = wrist_x + ARM_L3 * cosf(q12 + joint->wrist);
    tool_point->z = wrist_z + ARM_L3 * sinf(q12 + joint->wrist);
}
//关节限幅
uint8_t ArmMath_ClampJoint(ArmJoint *joint)
{
    uint8_t changed = 0U;

    if (joint == 0) return 0U;

    if ((joint->q1 < ARM_Q1_MIN) || (joint->q1 > ARM_Q1_MAX)) changed = 1U;
    if ((joint->q2 < ARM_Q2_MIN) || (joint->q2 > ARM_Q2_MAX)) changed = 1U;
    if ((joint->wrist < ARM_WRIST_MIN) || (joint->wrist > ARM_WRIST_MAX)) changed = 1U;

    joint->q1 = clampf(joint->q1, ARM_Q1_MIN, ARM_Q1_MAX);
    joint->q2 = clampf(joint->q2, ARM_Q2_MIN, ARM_Q2_MAX);
    joint->wrist = clampf(joint->wrist, ARM_WRIST_MIN, ARM_WRIST_MAX);
    return changed;
}
//逆运动学
uint8_t ArmMath_Inverse(float x, float z,
                        const ArmJoint *current,
                        ArmJoint *target)
{
    const float eps = 1.0e-6f;
    const float min_radius = fabsf(ARM_L1 - ARM_L2);
    const float max_radius = ARM_L1 + ARM_L2;
    const float distance_sq = x * x + z * z;
    const float min_radius_sq = min_radius * min_radius;
    const float max_radius_sq = max_radius * max_radius;
    float cos_q2;
    float sin_q2;
    float best_cost = 1.0e30f;
    ArmJoint best = {0.0f, 0.0f, 0.0f};
    int solution;

    if (target == 0) return 0U;
    if ((distance_sq > max_radius_sq + eps) ||
        (distance_sq < min_radius_sq - eps)) {
        return 0U;
    }

    cos_q2 = (distance_sq - ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2) /
             (2.0f * ARM_L1 * ARM_L2);
    cos_q2 = clampf(cos_q2, -1.0f, 1.0f);
    sin_q2 = sqrtf(fmaxf(0.0f, 1.0f - cos_q2 * cos_q2));

    for (solution = 0; solution < 2; solution++)
    {
        const float signed_sin_q2 = (solution == 0) ? sin_q2 : -sin_q2;
        ArmJoint candidate;
        float phi;
        float psi;
        float cost;

        candidate.q2 = atan2f(signed_sin_q2, cos_q2);
        phi = atan2f(z, x);
        psi = atan2f(ARM_L2 * signed_sin_q2,
                     ARM_L1 + ARM_L2 * cos_q2);
        candidate.q1 = wrap_pi(phi - psi);
        candidate.wrist = (current != 0) ? current->wrist : 0.0f;

        if ((candidate.q1 < ARM_Q1_MIN) || (candidate.q1 > ARM_Q1_MAX) ||
            (candidate.q2 < ARM_Q2_MIN) || (candidate.q2 > ARM_Q2_MAX)) {
            continue;
        }

        if (current != 0) {
            cost = fabsf(angle_diff(candidate.q1, current->q1)) +
                   0.3f * fabsf(angle_diff(candidate.q2, current->q2));
        } else {
            cost = 0.0f;
        }

        if (cost < best_cost) {
            best_cost = cost;
            best = candidate;
        }
    }

    if (best_cost >= 1.0e29f) return 0U;
    *target = best;
    return 1U;
}
//轨迹时间估计
float ArmMath_CalcTime(const ArmJoint *start, const ArmJoint *end)
{
    float max_delta;
    float time;

    if ((start == 0) || (end == 0)) return 0.5f;

    max_delta = fmaxf(fabsf(angle_diff(end->q1, start->q1)),
                      fabsf(angle_diff(end->q2, start->q2)));
    /* 参考工程的轨迹只规划两个 AK，EL05 单独限速控制。 */
    time = max_delta / (3.5f * 0.64f);
    return clampf(time, 0.5f, 4.0f);
}
//轨迹规划
//初始化轨迹
void ArmMath_TrajectoryStart(ArmTrajectory *trajectory,
                             const ArmJoint *start,
                             const ArmJoint *end,
                             float duration)
{
    if ((trajectory == 0) || (start == 0) || (end == 0)) return;

    trajectory->start = *start;
    trajectory->end = *end;
    ArmMath_ClampJoint(&trajectory->start);
    ArmMath_ClampJoint(&trajectory->end);
    trajectory->duration = (duration > 0.01f) ? duration : 0.01f;
    trajectory->elapsed = 0.0f;
    trajectory->active = 1U;
}

uint8_t ArmMath_TrajectoryUpdate(ArmTrajectory *trajectory,
                                 float dt,
                                 ArmJoint *position,
                                 ArmJoint *velocity,
                                 ArmJoint *acceleration)
{
    float tau;
    float tau2;
    float tau3;
    float tau4;
    float tau5;
    float s;
    float ds_dt;
    float d2s_dt2;
    float dq1;
    float dq2;
    uint8_t done;

    if ((trajectory == 0) || (position == 0) || (velocity == 0) ||
        (acceleration == 0)) return 1U;
    if (!trajectory->active) return 1U;

    if (dt < 0.0f) dt = 0.0f;
    trajectory->elapsed += dt;
    tau = trajectory->elapsed / trajectory->duration;
    if (tau >= 1.0f) tau = 1.0f;
    if (tau < 0.0f) tau = 0.0f;

    tau2 = tau * tau;
    tau3 = tau2 * tau;
    tau4 = tau3 * tau;
    tau5 = tau4 * tau;

    /* 五次多项式：起止位置、速度、加速度都连续。 */
    s = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau5;
    ds_dt = (30.0f * tau2 - 60.0f * tau3 + 30.0f * tau4) /
            trajectory->duration;
    d2s_dt2 = (60.0f * tau - 180.0f * tau2 + 120.0f * tau3) /
              (trajectory->duration * trajectory->duration);

    dq1 = angle_diff(trajectory->end.q1, trajectory->start.q1);
    dq2 = angle_diff(trajectory->end.q2, trajectory->start.q2);
    position->q1 = trajectory->start.q1 + dq1 * s;
    position->q2 = trajectory->start.q2 + dq2 * s;
    /* EL05 不使用这条轨迹，腕部命令由上层独立限速生成。 */
    position->wrist = trajectory->start.wrist;

    velocity->q1 = dq1 * ds_dt;
    velocity->q2 = dq2 * ds_dt;
    velocity->wrist = 0.0f;

    acceleration->q1 = dq1 * d2s_dt2;
    acceleration->q2 = dq2 * d2s_dt2;
    acceleration->wrist = 0.0f;

    done = (tau >= 1.0f) ? 1U : 0U;
    if (done) trajectory->active = 0U;
    return done;
}
//重力补偿
void ArmMath_GravityTorque(const ArmJoint *joint,
                           float *motor1_torque,
                           float *motor2_torque)
{
    const float m_elbow = 0.50f;
    const float m_wrist = 0.45f;
    const float gravity = 9.81f;
    const float gear = 7.5f;
    float q12;
    float torque1;
    float torque2;

    if ((joint == 0) || (motor1_torque == 0) || (motor2_torque == 0)) return;

    q12 = joint->q1 + joint->q2;
    torque1 = m_elbow * gravity * ARM_L1 * cosf(joint->q1) +
              m_wrist * gravity *
              (ARM_L1 * cosf(joint->q1) + ARM_L2 * cosf(q12));
    torque2 = m_wrist * gravity * ARM_L2 * cosf(q12);

    /* 与参考工程相同，换算为电机侧前馈力矩。 */
    *motor1_torque = clampf(torque1 / gear, -6.0f, 6.0f);
    *motor2_torque = clampf(torque2 / gear, -5.5f, 5.5f);
}
