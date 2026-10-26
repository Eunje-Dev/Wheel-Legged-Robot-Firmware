// KNEE 외부 감속비·기준 위치 및 기구 범위
// 관절각 기준·기구 한계·전원 인가 시작 자세 확인 후 위치 출력 허용
#ifndef INC_WOKI_JOINT_CONFIG_H_
#define INC_WOKI_JOINT_CONFIG_H_

#include <stdint.h>
#include <math.h>

#define WOKI_KNEE_GEAR_RATIO            5.0f
#define WOKI_KNEE_LEGACY_GEAR_RATIO     2.0f
#define WOKI_KNEE_LEGACY_TO_CURRENT     (WOKI_KNEE_GEAR_RATIO / WOKI_KNEE_LEGACY_GEAR_RATIO)
#define WOKI_KNEE_COUNTS_PER_DEG        (WOKI_KNEE_GEAR_RATIO * 4096.0f / 360.0f)
#define WOKI_KNEE_EXTENDED_MODE         4U

// 4개 KNEE 공통 조립 영점, 기존 1:2 조립 기준과 다른 기구 자세
#define WOKI_KNEE_ASSEMBLY_ZERO_RAW     2048

// 오른쪽 FR·RR 실측 기구 범위, Wheel·R26 목표값과 별도
#define WOKI_KNEE_RR_INDEX             11U
#define WOKI_KNEE_RR_ASSEMBLY_ZERO_RAW  WOKI_KNEE_ASSEMBLY_ZERO_RAW
// 재조립 후 확인한 기구 끝값, 실제 운전 목표는 여유를 둔 안쪽에서 설정
#define WOKI_KNEE_RR_MIN_RAW            (-1100)
#define WOKI_KNEE_RR_MAX_RAW            4500

// 왼쪽 FL·RL 실측 기구 범위, 공통 영점으로 좌우 끝값을 대칭 추정하지 않음
#define WOKI_KNEE_LEFT_MIN_RAW          (-700)
#define WOKI_KNEE_LEFT_MAX_RAW          5000

// 0: 관절 위치 출력 잠금, 1: 1:5 기준의 전환·보행·경사로 경로 검증 완료
// 값만 1로 바꾸는 것은 실물 확인을 대신하지 않음
#ifndef WOKI_KNEE_REFERENCE_VERIFIED
#define WOKI_KNEE_REFERENCE_VERIFIED     0U
#endif
#if WOKI_KNEE_REFERENCE_VERIFIED != 0 && WOKI_KNEE_REFERENCE_VERIFIED != 1
#error "WOKI_KNEE_REFERENCE_VERIFIED must be 0 or 1"
#endif

// Wheel↔Leg 실물 경로 확인용, 전체 보행·경사로 검증 완료와 구분
// 기존 Wheel↔Leg 동작 허가 유지, 전체 보행·경사로 허가와 별도
#ifndef WOKI_WHEEL_LEG_TEST_ENABLE
#define WOKI_WHEEL_LEG_TEST_ENABLE      1U
#endif
#if WOKI_WHEEL_LEG_TEST_ENABLE != 0 && WOKI_WHEEL_LEG_TEST_ENABLE != 1
#error "WOKI_WHEEL_LEG_TEST_ENABLE must be 0 or 1"
#endif
#define WOKI_KNEE_POSITION_OUTPUT_ENABLED \
    (WOKI_KNEE_REFERENCE_VERIFIED == 1U || WOKI_WHEEL_LEG_TEST_ENABLE == 1U)

// 사용자가 확인한 Wheel 시작 자세, 순서 FL·FR·RL·RR (모드 4 누적 위치)
// 이 자세를 기존 궤적의 시작 좌표 31.56도에 연결해 각도 변화량을 적용
// 왼쪽 4096은 0과 다른 기구 자세이므로 한 바퀴 나머지로 바꾸지 않음
#define WOKI_KNEE_REFERENCE_FL_RAW      4096.0f
#define WOKI_KNEE_REFERENCE_FR_RAW      0.0f
#define WOKI_KNEE_REFERENCE_RL_RAW      4096.0f
#define WOKI_KNEE_REFERENCE_RR_RAW      0.0f
#define WOKI_KNEE_STARTUP_TOLERANCE_RAW  64

static inline float WokiJoint_KneeReference(uint32_t leg)
{
    const float reference[4] = {
        WOKI_KNEE_REFERENCE_FL_RAW, WOKI_KNEE_REFERENCE_FR_RAW,
        WOKI_KNEE_REFERENCE_RL_RAW, WOKI_KNEE_REFERENCE_RR_RAW
    };
    return leg < 4U ? reference[leg] : 0.0f;
}

static inline float WokiJoint_LegacyKneeReference(uint32_t leg)
{
    // 기존 1:2 궤적의 기준값은 보존, 새 Wheel 기준값으로 덮어쓰지 않음
    return (leg % 2U == 0U) ? 2691.4569f : 1404.5431f;
}

static inline uint8_t WokiJoint_IsKneeIndex(uint32_t index)
{
    return index < 12U && index % 3U == 2U;
}

// 1:2 엔코더 목표·한계를 기준점 주위에서 환산, ROLL/PITCH는 그대로 유지
static inline float WokiJoint_FromLegacyRaw(uint32_t index, float raw)
{
    if (!WokiJoint_IsKneeIndex(index)) return raw;
    const uint32_t leg = index / 3U;
    return WokiJoint_KneeReference(leg)
            + (raw - WokiJoint_LegacyKneeReference(leg))
                    * WOKI_KNEE_LEGACY_TO_CURRENT;
}

// 음수 위치도 0 방향으로 잘리지 않도록 부호를 반영한 반올림
static inline int32_t WokiJoint_RoundRaw(float raw)
{
    return (int32_t)(raw < 0.0f ? raw - 0.5f : raw + 0.5f);
}

static inline int32_t WokiJoint_LegacyPosition(uint32_t index, float raw)
{
    return WokiJoint_RoundRaw(WokiJoint_FromLegacyRaw(index, raw));
}

// 기존 관절각 범위와 좌우별 실측 기구 범위의 공통 구간
// FL·RL은 왼쪽, FR·RR은 오른쪽 범위 적용, ROLL/PITCH는 기존 제한 유지
static inline int32_t WokiJoint_MinRaw(uint32_t index)
{
    const int32_t minimum = WokiJoint_LegacyPosition(index, 0.0f);
    if (!WokiJoint_IsKneeIndex(index)) return minimum;
    const int32_t mechanical_minimum = (index / 3U) % 2U == 0U
            ? WOKI_KNEE_LEFT_MIN_RAW : WOKI_KNEE_RR_MIN_RAW;
    return minimum < mechanical_minimum ? mechanical_minimum : minimum;
}

static inline int32_t WokiJoint_MaxRaw(uint32_t index)
{
    const int32_t maximum = WokiJoint_LegacyPosition(index, 4095.0f);
    if (!WokiJoint_IsKneeIndex(index)) return maximum;
    const int32_t mechanical_maximum = (index / 3U) % 2U == 0U
            ? WOKI_KNEE_LEFT_MAX_RAW : WOKI_KNEE_RR_MAX_RAW;
    return maximum > mechanical_maximum ? mechanical_maximum : maximum;
}

static inline uint8_t WokiJoint_PositionValid(uint32_t index, int32_t raw)
{
    return index < 12U && raw >= WokiJoint_MinRaw(index)
            && raw <= WokiJoint_MaxRaw(index)
            && raw >= -1048575 && raw <= 1048575;
}

static inline uint32_t WokiJoint_IndexFromId(uint8_t id)
{
    const uint32_t leg = id / 10U;
    const uint32_t joint = id % 10U;
    return leg < 4U && joint >= 1U && joint <= 3U
            ? leg * 3U + joint - 1U : 12U;
}

static inline uint8_t WokiJoint_PositionValidById(uint8_t id, int32_t raw)
{
    return WokiJoint_PositionValid(WokiJoint_IndexFromId(id), raw);
}

#endif /* INC_WOKI_JOINT_CONFIG_H_ */
