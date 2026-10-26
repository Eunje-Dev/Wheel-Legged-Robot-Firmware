#include "woki_slope_formula.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#include "woki_pitch_geometry_generated.h"
#include "woki_slope_lookup_generated.h"

// 종경사 최대 보정각
#define WOKI_LONGITUDINAL_COMMAND_MAX_DEG 10.5f

// PITCH 앞뒤 보정 최대각
#define WOKI_PITCH_DELTA_MAX_DEG 15.0f

#define WOKI_DEG_TO_RAD (0.01745329251994329577f)
#define WOKI_RAD_TO_DEG (57.29577951308232088f)

static float WokiSlope_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float WokiSlope_WrapDeg(float angle_deg)
{
    while (angle_deg <= -180.0f) {
        angle_deg += 360.0f;
    }
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

static uint8_t WokiSlope_InputFinite(const WokiSlopeInput *input)
{
    uint32_t leg;

    if (!isfinite(input->base_roll_deg) || !isfinite(input->base_pitch_deg)) {
        return 0U;
    }
    for (leg = 0U; leg < WOKI_SLOPE_LEG_COUNT; ++leg) {
        if (!isfinite(input->hip_roll_deg[leg])
                || !isfinite(input->hip_pitch_deg[leg])) {
            return 0U;
        }
    }
    return 1U;
}

static void WokiSlope_WheelCenterXZ(uint32_t leg, float pitch_deg,
        float *center_x_m, float *center_z_m)
{
    const float pivot_x = woki_pitch_pivot_xz_m[leg][0];
    const float pivot_z = woki_pitch_pivot_xz_m[leg][1];
    const float relative_x = woki_wheel_center0_xz_m[leg][0] - pivot_x;
    const float relative_z = woki_wheel_center0_xz_m[leg][1] - pivot_z;
    const float angle_rad = pitch_deg * WOKI_DEG_TO_RAD;
    const float cosine = cosf(angle_rad);
    const float sine = sinf(angle_rad);

    *center_x_m = pivot_x + cosine * relative_x + sine * relative_z;
    *center_z_m = pivot_z - sine * relative_x + cosine * relative_z;
}

static float WokiSlope_PairAngleDeg(uint32_t front_leg, uint32_t rear_leg,
        const float pitch_deg[WOKI_SLOPE_LEG_COUNT])
{
    float front_x;
    float front_z;
    float rear_x;
    float rear_z;

    WokiSlope_WheelCenterXZ(front_leg, pitch_deg[front_leg], &front_x, &front_z);
    WokiSlope_WheelCenterXZ(rear_leg, pitch_deg[rear_leg], &rear_x, &rear_z);
    return atan2f(front_z - rear_z, front_x - rear_x) * WOKI_RAD_TO_DEG;
}

static float WokiSlope_WheelChordBodyDeg(
        const float pitch_deg[WOKI_SLOPE_LEG_COUNT])
{
    const float left = WokiSlope_PairAngleDeg(
            WOKI_SLOPE_FL, WOKI_SLOPE_RL, pitch_deg);
    const float right = WokiSlope_PairAngleDeg(
            WOKI_SLOPE_FR, WOKI_SLOPE_RR, pitch_deg);
    return 0.5f * (left + right);
}

static float WokiSlope_LongitudinalDeltaDeg(float slope_deg)
{
    float position;
    uint32_t index;
    float fraction;

    slope_deg = WokiSlope_Clamp(slope_deg, 0.0f, WOKI_LONGITUDINAL_MAX_DEG);
    position = slope_deg / WOKI_SLOPE_TABLE_STEP_DEG;
    index = (uint32_t)position;
    if (index >= WOKI_LONGITUDINAL_TABLE_COUNT - 1U) {
        return woki_longitudinal_delta_deg[WOKI_LONGITUDINAL_TABLE_COUNT - 1U];
    }
    fraction = position - (float)index;
    return woki_longitudinal_delta_deg[index]
            + fraction * (woki_longitudinal_delta_deg[index + 1U]
                    - woki_longitudinal_delta_deg[index]);
}

static float WokiSlope_LateralInverseDeg(
        const float roll_deg[WOKI_SLOPE_LEG_COUNT])
{
    float best_beta_deg = -WOKI_LATERAL_MAX_DEG;
    float best_error = FLT_MAX;
    uint32_t segment;

    // 0.25도 간격 기구학 표의 최소제곱 역산
    for (segment = 0U; segment < WOKI_LATERAL_TABLE_COUNT - 1U; ++segment) {
        float denominator = 0.0f;
        float numerator = 0.0f;
        float error = 0.0f;
        float fraction;
        uint32_t leg;

        for (leg = 0U; leg < WOKI_SLOPE_LEG_COUNT; ++leg) {
            const float q0 = woki_lateral_roll_deg[segment][leg];
            const float dq = woki_lateral_roll_deg[segment + 1U][leg] - q0;
            denominator += dq * dq;
            numerator += dq * (roll_deg[leg] - q0);
        }
        if (denominator <= 0.0f) {
            continue;
        }

        fraction = WokiSlope_Clamp(numerator / denominator, 0.0f, 1.0f);
        for (leg = 0U; leg < WOKI_SLOPE_LEG_COUNT; ++leg) {
            const float q0 = woki_lateral_roll_deg[segment][leg];
            const float dq = woki_lateral_roll_deg[segment + 1U][leg] - q0;
            const float residual = roll_deg[leg] - (q0 + fraction * dq);
            error += residual * residual;
        }
        if (error < best_error) {
            best_error = error;
            best_beta_deg = -WOKI_LATERAL_MAX_DEG
                    + ((float)segment + fraction) * WOKI_SLOPE_TABLE_STEP_DEG;
        }
    }
    return best_beta_deg;
}

static void WokiSlope_LateralTargetDeg(float beta_deg,
        float target_deg[WOKI_SLOPE_LEG_COUNT])
{
    float position;
    float fraction;
    uint32_t index;
    uint32_t leg;

    beta_deg = WokiSlope_Clamp(beta_deg,
            -WOKI_LATERAL_MAX_DEG, WOKI_LATERAL_MAX_DEG);
    position = (beta_deg + WOKI_LATERAL_MAX_DEG)
            / WOKI_SLOPE_TABLE_STEP_DEG;
    index = (uint32_t)position;
    if (index >= WOKI_LATERAL_TABLE_COUNT - 1U) {
        index = WOKI_LATERAL_TABLE_COUNT - 1U;
        for (leg = 0U; leg < WOKI_SLOPE_LEG_COUNT; ++leg) {
            target_deg[leg] = woki_lateral_roll_deg[index][leg];
        }
        return;
    }

    fraction = position - (float)index;
    for (leg = 0U; leg < WOKI_SLOPE_LEG_COUNT; ++leg) {
        target_deg[leg] = woki_lateral_roll_deg[index][leg]
                + fraction * (woki_lateral_roll_deg[index + 1U][leg]
                        - woki_lateral_roll_deg[index][leg]);
    }
}

uint8_t WokiSlope_Compute(const WokiSlopeInput *input, WokiSlopeOutput *output)
{
    float lateral_clamped;
    float longitudinal_clamped;
    uint32_t leg;

    if (input == NULL || output == NULL || !WokiSlope_InputFinite(input)) {
        return 0U;
    }

    output->lateral_from_joint_deg = WokiSlope_LateralInverseDeg(
            input->hip_roll_deg);
    output->lateral_raw_deg = output->lateral_from_joint_deg
            + input->base_roll_deg;
    lateral_clamped = WokiSlope_Clamp(output->lateral_raw_deg,
            -WOKI_LATERAL_MAX_DEG, WOKI_LATERAL_MAX_DEG);
    output->lateral_clipped =
            (fabsf(lateral_clamped - output->lateral_raw_deg) > 1.0e-6f) ? 1U : 0U;
    output->lateral_command_deg = lateral_clamped;
    WokiSlope_LateralTargetDeg(lateral_clamped, output->hip_roll_target_deg);

    output->wheel_chord_body_deg = WokiSlope_WheelChordBodyDeg(
            input->hip_pitch_deg);
    output->longitudinal_raw_deg = WokiSlope_WrapDeg(
            output->wheel_chord_body_deg - input->base_pitch_deg);
    longitudinal_clamped = WokiSlope_Clamp(output->longitudinal_raw_deg,
            0.0f, WOKI_LONGITUDINAL_COMMAND_MAX_DEG);
    output->longitudinal_clipped =
            (fabsf(longitudinal_clamped - output->longitudinal_raw_deg) > 1.0e-6f) ? 1U : 0U;
    output->longitudinal_command_deg = longitudinal_clamped;
    output->pitch_delta_deg = WokiSlope_Clamp(
            WokiSlope_LongitudinalDeltaDeg(longitudinal_clamped),
            0.0f, WOKI_PITCH_DELTA_MAX_DEG);

    output->hip_pitch_target_deg[WOKI_SLOPE_FL] =
            WOKI_SLOPE_COMMON_PITCH_DEG + output->pitch_delta_deg;
    output->hip_pitch_target_deg[WOKI_SLOPE_FR] =
            WOKI_SLOPE_COMMON_PITCH_DEG + output->pitch_delta_deg;
    output->hip_pitch_target_deg[WOKI_SLOPE_RL] =
            WOKI_SLOPE_COMMON_PITCH_DEG - output->pitch_delta_deg;
    output->hip_pitch_target_deg[WOKI_SLOPE_RR] =
            WOKI_SLOPE_COMMON_PITCH_DEG - output->pitch_delta_deg;

    for (leg = 0U; leg < WOKI_SLOPE_LEG_COUNT; ++leg) {
        output->knee_target_deg[leg] = WOKI_SLOPE_FIXED_KNEE_DEG;
    }
    return 1U;
}
