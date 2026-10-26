// 모드 4의 누적 회전수는 전원 재인가 후 보존되지 않으므로 실제 기준 자세 확인 필요
// 종료 준비는 전원 차단 명령이 아니며, 외부 지지 확인 전 토크를 자동 해제하지 않음
#include "woki_power_guard.h"
#include "woki_joint_config.h"
#include "dxl_2_0.h"
#include "robot_motion.h"
#include <string.h>

extern volatile uint8_t user_stop_flag;
volatile RobotPowerStatus robot_power_status;
volatile RobotPowerCommFault robot_power_comm_fault;
static RobotPowerCommFault pending_comm_failure;

static const uint8_t power_ids[12] = { 1,2,3,11,12,13,21,22,23,31,32,33 };
static const int32_t park_position[12] = WOKI_POWER_PARK_POSITION_INITIALIZER;
static int32_t park_start[12], private_goal[12];
static uint32_t saved_acceleration[12], saved_velocity[12];
static uint8_t private_goal_active, private_goal_allow_stop, saved_profile_count, failed_samples;
static uint8_t monitor_index, boot_check_attempted, initial_wheel_reached;
static uint32_t sample_tick, park_tick, settle_tick;
static uint8_t settling;
static uint8_t initial_output_owned;
static int32_t initial_start_position[12];

// 초기 출력 0·Torque ON 준비에서만 사용, 정상 운전·주차 목표 검사는 기존 그대로 유지
#define POWER_INITIAL_GOAL_CHECK_ATTEMPTS 3U
#define POWER_INITIAL_GOAL_WAIT_MS        2U
// 기존 부팅 Goal PWM 설정값, 기동 목표 확인 전에는 이 값으로 복원하지 않음
#define POWER_INITIAL_RUN_PWM            885U

static uint8_t Power_Close(int32_t a, int32_t b, int32_t tolerance)
{
    const int64_t distance = (int64_t)a - (int64_t)b;
    return distance >= -(int64_t)tolerance && distance <= (int64_t)tolerance;
}

// 시작 구간은 사용자가 실물 기준으로 확보, Wheel 목표와 기구 여유만 확인
static uint8_t Power_ConfigValid(void)
{
    int32_t wheel[12];
    if (!WOKI_KNEE_POSITION_OUTPUT_ENABLED) return 0U;
    if (WOKI_POWER_START_MIN_RAW < 0 || WOKI_POWER_START_MAX_RAW > 4095
            || WOKI_POWER_START_MIN_RAW > WOKI_POWER_START_MAX_RAW) return 0U;
    Robot_Get_Wheel_Pose_Targets(wheel);
    for (uint8_t i = 0U; i < 12U; ++i) {
        if (!WokiJoint_PositionValid(i, wheel[i])
                || (int64_t)wheel[i] - WOKI_POWER_POSITION_MARGIN_RAW < WokiJoint_MinRaw(i)
                || (int64_t)wheel[i] + WOKI_POWER_POSITION_MARGIN_RAW > WokiJoint_MaxRaw(i)
                || WOKI_POWER_START_MIN_RAW < WokiJoint_MinRaw(i)
                || WOKI_POWER_START_MAX_RAW > WokiJoint_MaxRaw(i))
            return 0U;
    }
    return 1U;
}

// 종료는 기존 Wheel 복귀 후에만 허용, 새 주차 위치·새 경로를 임의로 추가하지 않음
static uint8_t Power_ParkConfigValid(void)
{
    int32_t wheel[12];
    if (!Power_ConfigValid() || WOKI_POWER_PARK_POSE_VERIFIED != 1U
            || WOKI_POWER_PARK_PATH_VERIFIED != 1U) return 0U;
    Robot_Get_Wheel_Pose_Targets(wheel);
    for (uint8_t i = 0U; i < 12U; ++i) {
        const int32_t p = park_position[i];
        if (p != wheel[i]
                || (int64_t)p - WOKI_POWER_POSITION_MARGIN_RAW < WokiJoint_MinRaw(i)
                || (int64_t)p + WOKI_POWER_POSITION_MARGIN_RAW > WokiJoint_MaxRaw(i)) return 0U;
    }
    return 1U;
}

// 통신 실패가 확정된 조회의 정보만 준비, 오류 처리 중 재조회는 원래 기록을 건드리지 않음
static void Power_RecordCommFailure(uint8_t source, uint16_t address, uint8_t size, uint8_t attempts)
{
    if (robot_power_status.state == ROBOT_POWER_FAULT) return;
    memset(&pending_comm_failure, 0, sizeof(pending_comm_failure));
    pending_comm_failure.source = source;
    pending_comm_failure.startup_stage = robot_power_status.startup_stage;
    pending_comm_failure.failed_id = robot_power_status.failed_id;
    pending_comm_failure.read_address = address;
    pending_comm_failure.read_size = size;
    pending_comm_failure.query_attempts = attempts;
    pending_comm_failure.timestamp_ms = HAL_GetTick();
    if (source == ROBOT_COMM_SYNC) {
        dxl_get_last_sync_read_diagnostics(&pending_comm_failure.sync);
    } else {
        pending_comm_failure.read_fail_stage = robot_power_status.read_fail_stage;
        pending_comm_failure.device_error = robot_power_status.device_error;
    }
}

static uint8_t Power_Read(uint8_t id, uint16_t address, uint8_t size, uint32_t *value)
{
    uint8_t data[4];
    uint8_t used_attempts = 0U;
    // 시작 준비·최초 Wheel 이동 구간의 일시적인 조회 실패만 최대 3회 시도
    // 운전 중 감시 주기와 기존 연속 실패 판정은 변경하지 않음
    const uint8_t attempts = robot_power_status.state == ROBOT_POWER_CHECKING
            || (robot_power_status.state == ROBOT_POWER_READY && !initial_wheel_reached
                    && Robot_Get_Mode() == MODE_NONE) ? 3U : 1U;
    robot_power_status.failed_id = id;
    for (uint8_t attempt = 0U; attempt < attempts; ++attempt) {
        ++used_attempts;
        if (dxl_read_2_0(id, address, size, data,
                (uint8_t *)&robot_power_status.device_error,
                (uint8_t *)&robot_power_status.read_fail_stage)
                && robot_power_status.device_error == 0U) {
            *value = 0U;
            for (uint8_t b = 0U; b < size; ++b) *value |= (uint32_t)data[b] << (8U * b);
            return 1U;
        }
        // 정상 수신한 모터 오류나 호출 인자 오류를 단순 유실로 취급하지 않음
        // 수신 전 device_error=0xFF는 미확인 표시이므로 모터가 보낸 오류와 구분
        if (((robot_power_status.read_fail_stage == DXL_READ_FAIL_NONE
                        || robot_power_status.read_fail_stage == DXL_READ_FAIL_DATA)
                    && robot_power_status.device_error != 0U)
                || robot_power_status.read_fail_stage == DXL_READ_FAIL_ARGUMENT
                || user_stop_flag || attempt + 1U >= attempts) break;
        HAL_Delay(2U);
    }
    Power_RecordCommFailure(ROBOT_COMM_READ, address, size, used_attempts);
    robot_power_status.error = ROBOT_POWER_COMM;
    return 0U;
}

// 최초 값 불일치만 저장, 이후 Hold·프로파일 복원이 원인을 덮어쓰지 않도록 보존
static void Power_RecordWriteMismatch(uint8_t id, uint16_t address,
        uint32_t expected, uint32_t actual)
{
    if (robot_power_status.write_mismatch_valid) return;
    robot_power_status.write_failed_id = id;
    robot_power_status.write_address = address;
    robot_power_status.write_expected = expected;
    robot_power_status.write_actual = actual;
    robot_power_status.write_mismatch_valid = 1U;
}

static uint8_t Power_WriteVerifyConfig(uint8_t id, uint16_t address, uint8_t size,
        uint32_t value, uint8_t restoring)
{
    uint32_t actual;
    if (user_stop_flag && !restoring) { robot_power_status.error = ROBOT_POWER_STOP; return 0U; }
    robot_power_status.failed_id = id;
    robot_power_status.device_error = 0U;
    robot_power_status.read_fail_stage = DXL_READ_FAIL_NONE;
    const HAL_StatusTypeDef status = dxl_sync_write_config_2_0(id, address, size, value);
    if (status != HAL_OK) {
        robot_power_status.error = ROBOT_POWER_WRITE;
        robot_power_status.read_fail_stage = status == HAL_BUSY ? DXL_READ_FAIL_TX_BUSY
                : (status == HAL_TIMEOUT ? DXL_READ_FAIL_TX_TIMEOUT : DXL_READ_FAIL_TX_ERROR);
        return 0U;
    }
    if (!Power_Read(id, address, size, &actual)) return 0U;
    if (actual != value) {
        Power_RecordWriteMismatch(id, address, value, actual);
        robot_power_status.error = ROBOT_POWER_WRITE; return 0U;
    }
    return 1U;
}

static uint8_t Power_WriteVerify(uint8_t id, uint16_t address, uint8_t size, uint32_t value)
{
    return Power_WriteVerifyConfig(id, address, size, value, 0U);
}

// 값 0을 보낸 사실만으로 출력 차단을 가정하지 않고 12개 모두 다시 읽어 확인
static uint8_t Power_CheckInitialOutputZero(void)
{
    uint32_t value;
    robot_power_status.startup_pwm_zero_mask = 0U;
    for (uint8_t i = 0U; i < 12U; ++i) {
        if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; return 0U; }
        if (!Power_Read(power_ids[i], DXL_2_Goal_PWM, 2U, &value)) return 0U;
        if (value != 0U) {
            Power_RecordWriteMismatch(power_ids[i], DXL_2_Goal_PWM, 0U, value);
            robot_power_status.error = ROBOT_POWER_WRITE; return 0U;
        }
        robot_power_status.startup_pwm_zero_mask |= (uint16_t)(1U << i);
    }
    return 1U;
}

// 최초 기동 중 실패했을 때만 출력 0 재설정 시도, 정상 운전의 Hold 동작과 구분
// 통신 실패 모터의 실제 출력은 보장할 수 없으므로 확인된 비트만 남김
static void Power_ForceInitialOutputZero(void)
{
    robot_power_status.startup_pwm_zero_mask = 0U;
    for (uint8_t i = 0U; i < 12U; ++i) {
        if (Power_WriteVerifyConfig(power_ids[i], DXL_2_Goal_PWM, 2U, 0U, 1U))
            robot_power_status.startup_pwm_zero_mask |= (uint16_t)(1U << i);
    }
}

// 방향·영점은 변경하지 않고 확인만 수행, Homing Offset=0인 기준만 지원
static uint8_t Power_CheckMotor(uint8_t i, int8_t expected_torque)
{
    uint32_t value;
    const uint8_t id = power_ids[i];
    if (!Power_Read(id, DXL_2_Operating_Mode, 1U, &value)) return 0U;
    if (value != (WokiJoint_IsKneeIndex(i) ? 4U : 3U)) {
        robot_power_status.error = ROBOT_POWER_MODE; return 0U;
    }
    if (!Power_Read(id, DXL_2_Drive_Mode, 1U, &value)) return 0U;
    if (value != 0U) { robot_power_status.error = ROBOT_POWER_DRIVE; return 0U; }
    if (!Power_Read(id, DXL_2_Homing_Offset, 4U, &value)) return 0U;
    if (value != 0U) { robot_power_status.error = ROBOT_POWER_OFFSET; return 0U; }
    if (!Power_Read(id, DXL_2_Hardware_Error_Status, 1U, &value)) return 0U;
    if (value != 0U) { robot_power_status.error = ROBOT_POWER_HARDWARE; return 0U; }
    if (!Power_Read(id, DXL_2_Torque_Enable, 1U, &value)) return 0U;
    if (expected_torque >= 0 && value != (uint32_t)expected_torque) {
        robot_power_status.error = ROBOT_POWER_TORQUE; return 0U;
    }
    return 1U;
}

static uint8_t Power_CheckAllMotors(int8_t expected_torque)
{
    for (uint8_t i = 0U; i < 12U; ++i) {
        if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; return 0U; }
        if (!Power_CheckMotor(i, expected_torque)) return 0U;
    }
    return 1U;
}

static uint8_t Power_Feedback(DXL_Leg_State_t *state)
{
    // Torque ON 전 기동 검사에서만 부분 수신을 최대 3회 확인, 운전 중 조회 주기는 유지
    const uint8_t startup_off_check = !initial_wheel_reached && Robot_Get_Mode() == MODE_NONE
            && (robot_power_status.state == ROBOT_POWER_CHECKING
                    || robot_power_status.state == ROBOT_POWER_READY)
            && robot_power_status.startup_stage >= ROBOT_START_CHECK_OFF
            && robot_power_status.startup_stage < ROBOT_START_TORQUE_ZERO;
    const uint8_t attempts = startup_off_check ? 3U : 1U;
    robot_power_status.feedback_valid = 0U;
    for (uint8_t attempt = 0U; attempt < attempts; ++attempt) {
        if (startup_off_check && user_stop_flag) {
            robot_power_status.error = ROBOT_POWER_STOP; return 0U;
        }
        const uint16_t valid_mask = dxl_sync_read_leg_state(state);
        if (valid_mask == 0x0FFFU && state->received_count == 12U) break;
        if (startup_off_check) {
            // 누락만 재시도, 이미 받은 모터 오류·기구 범위 위반을 정상 응답으로 덮지 않음
            for (uint8_t i = 0U; i < 12U; ++i) {
                if (state->motors[i].valid
                        && (state->motors[i].id != power_ids[i]
                                || state->motors[i].status_error != 0U
                                || !WokiJoint_PositionValid(i, state->motors[i].position_raw))) {
                    robot_power_status.failed_id = power_ids[i];
                    robot_power_status.error = ROBOT_POWER_POSITION; return 0U;
                }
            }
        }
        if (attempt + 1U < attempts) {
            // 매번 새 Sync Read로 12개를 다시 조회, 이전 부분 수신 결과를 합치지 않음
            HAL_Delay(2U);
            continue;
        }
        // 마지막 개별 검사 ID(33)를 남기지 않고, 최종 조회의 첫 누락 ID를 기록
        // 전체 비트가 있어도 수신 개수가 다르면 특정 모터를 단정하지 않고 0으로 표시
        robot_power_status.failed_id = 0U;
        for (uint8_t i = 0U; i < 12U; ++i) {
            if ((valid_mask & (uint16_t)(1U << i)) == 0U) {
                robot_power_status.failed_id = power_ids[i];
                break;
            }
        }
        Power_RecordCommFailure(ROBOT_COMM_SYNC, DXL_2_Present_Current,
                DXL_SYNC_STATE_DATA_LENGTH, (uint8_t)(attempt + 1U));
        robot_power_status.error = ROBOT_POWER_COMM; return 0U;
    }
    for (uint8_t i = 0U; i < 12U; ++i) {
        robot_power_status.failed_id = power_ids[i];
        if (state->motors[i].id != power_ids[i] || !state->motors[i].valid
                || state->motors[i].status_error != 0U
                || !WokiJoint_PositionValid(i, state->motors[i].position_raw)) {
            robot_power_status.error = ROBOT_POWER_POSITION; return 0U;
        }
    }
    robot_power_status.last_feedback_ms = HAL_GetTick();
    robot_power_status.feedback_valid = 1U;
    return 1U;
}

static uint8_t Power_At(const DXL_Leg_State_t *state, const int32_t expected[12], int32_t tolerance)
{
    for (uint8_t i = 0U; i < 12U; ++i) {
        robot_power_status.failed_id = power_ids[i];
        if (!Power_Close(state->motors[i].position_raw, expected[i], tolerance)) {
            robot_power_status.error = ROBOT_POWER_POSITION; return 0U;
        }
        if (!Power_Close(state->motors[i].velocity_raw, 0, WOKI_POWER_SETTLED_VELOCITY_RAW)) {
            robot_power_status.error = ROBOT_POWER_VELOCITY; return 0U;
        }
    }
    return 1U;
}

// 초기 이동 전에는 Wheel 목표와의 거리 대신 실제 현재 위치 0~4095 및 정지 상태 확인
static uint8_t Power_StartInSingleTurn(const DXL_Leg_State_t *state)
{
    for (uint8_t i = 0U; i < 12U; ++i) {
        robot_power_status.failed_id = power_ids[i];
        if (state->motors[i].position_raw < WOKI_POWER_START_MIN_RAW
                || state->motors[i].position_raw > WOKI_POWER_START_MAX_RAW) {
            robot_power_status.error = ROBOT_POWER_POSITION; return 0U;
        }
        if (!Power_Close(state->motors[i].velocity_raw, 0, WOKI_POWER_SETTLED_VELOCITY_RAW)) {
            robot_power_status.error = ROBOT_POWER_VELOCITY; return 0U;
        }
    }
    return 1U;
}

uint8_t RobotPower_PrivateGoalAllowed(uint8_t id, int32_t position)
{
    const uint32_t i = WokiJoint_IndexFromId(id);
    return private_goal_active && (!user_stop_flag || private_goal_allow_stop)
            && Power_ConfigValid() && i < 12U
            && private_goal[i] == position;
}

// 전역 잠금을 풀지 않고 이번 함수가 준비한 정확한 12개 목표만 한 번 허용
static uint8_t Power_SendGoal(const int32_t goal[12], uint8_t stopping_hold)
{
    memcpy(private_goal, goal, sizeof(private_goal));
    private_goal_allow_stop = stopping_hold;
    private_goal_active = 1U;
    const uint8_t sent = dxl_sync_write_goal_positions_checked(power_ids, goal, 12U);
    private_goal_active = 0U;
    private_goal_allow_stop = 0U;
    if (!sent) robot_power_status.error = ROBOT_POWER_WRITE;
    return sent;
}

// 출력 0·Torque ON에서 현재 위치 목표를 전송한 뒤 같은 값만 재확인
// 목표 재전송·현재 위치로 목표 교체·허용오차 확대 없이, 끝까지 다르면 출발 차단
static uint8_t Power_CheckInitialGoal(const int32_t expected[12])
{
    uint32_t value;
    if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; return 0U; }
    HAL_Delay(POWER_INITIAL_GOAL_WAIT_MS);
    for (uint8_t i = 0U; i < 12U; ++i) {
        for (uint8_t attempt = 0U; attempt < POWER_INITIAL_GOAL_CHECK_ATTEMPTS; ++attempt) {
            if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; return 0U; }
            if (!Power_Read(power_ids[i], DXL_2_Goal_Position, 4U, &value)) return 0U;
            if ((int32_t)value == expected[i]) break;
            if (attempt + 1U >= POWER_INITIAL_GOAL_CHECK_ATTEMPTS) {
                Power_RecordWriteMismatch(power_ids[i], DXL_2_Goal_Position,
                        (uint32_t)expected[i], value);
                robot_power_status.error = ROBOT_POWER_WRITE; return 0U;
            }
            HAL_Delay(POWER_INITIAL_GOAL_WAIT_MS);
        }
    }
    if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; return 0U; }
    return 1U;
}

static uint8_t Power_CheckGoal(const int32_t expected[12])
{
    uint32_t value;
    for (uint8_t i = 0U; i < 12U; ++i) {
        if (!Power_Read(power_ids[i], DXL_2_Goal_Position, 4U, &value)) return 0U;
        if ((int32_t)value != expected[i]) {
            Power_RecordWriteMismatch(power_ids[i], DXL_2_Goal_Position,
                    (uint32_t)expected[i], value);
            robot_power_status.error = ROBOT_POWER_WRITE; return 0U;
        }
    }
    return 1U;
}

static uint8_t Power_RestoreProfiles(void)
{
    uint8_t restored = 1U;
    uint8_t failed_id = 0U, device_error = 0U, read_fail_stage = 0U;
    // 정지 요청 중에도 저장값 복원은 가능, 목표 변경·토크 인가 없음
    for (uint8_t i = 0U; i < saved_profile_count; ++i) {
        for (uint8_t profile = 0U; profile < 2U; ++profile) {
            const uint16_t address = profile == 0U ? DXL_2_Profile_Acceleration : DXL_2_Profile_Velocity;
            const uint32_t value = profile == 0U ? saved_acceleration[i] : saved_velocity[i];
            if (!Power_WriteVerifyConfig(power_ids[i], address, 4U, value, 1U)) {
                if (restored) {
                    failed_id = robot_power_status.failed_id;
                    device_error = robot_power_status.device_error;
                    read_fail_stage = robot_power_status.read_fail_stage;
                }
                restored = 0U;
            }
        }
    }
    saved_profile_count = 0U;
    // 뒤의 복원에 성공해도 먼저 실패한 모터·통신 단계는 보존
    if (!restored) {
        robot_power_status.error = ROBOT_POWER_WRITE;
        robot_power_status.failed_id = failed_id;
        robot_power_status.device_error = device_error;
        robot_power_status.read_fail_stage = read_fail_stage;
    }
    return restored;
}

// 통신 가능한 경우 현재 위치 목표를 전송해 주차 이동 중지 시도
// 전송 성공은 실제 정지 보장이 아니며, 실패 시 토크 OFF로 몸체를 떨어뜨리지 않음
static void Power_Fault(RobotPowerError error, uint8_t try_hold)
{
    // 최초 COMM 정지 원인을 Hold·설정 복원보다 먼저 고정, 부팅 초기화 전까지 유지
    if (error == ROBOT_POWER_COMM && !robot_power_comm_fault.valid
            && pending_comm_failure.source != ROBOT_COMM_NONE) {
        robot_power_comm_fault = pending_comm_failure;
        robot_power_comm_fault.valid = 1U; // 전체 복사가 끝난 뒤 유효 표시
    }
    const uint8_t failed_id = robot_power_status.failed_id;
    const uint8_t device_error = robot_power_status.device_error;
    const uint8_t read_fail_stage = robot_power_status.read_fail_stage;
    const uint8_t write_mismatch_valid = robot_power_status.write_mismatch_valid;
    const uint8_t write_failed_id = robot_power_status.write_failed_id;
    const uint16_t write_address = robot_power_status.write_address;
    const uint32_t write_expected = robot_power_status.write_expected;
    const uint32_t write_actual = robot_power_status.write_actual;
    DXL_Leg_State_t state;
    int32_t hold[12];
    if (robot_power_status.startup_stage > ROBOT_START_IDLE
            && robot_power_status.startup_stage < ROBOT_START_COMPLETE)
        robot_power_status.startup_failed_stage = robot_power_status.startup_stage;
    robot_power_status.state = ROBOT_POWER_FAULT;
    initial_wheel_reached = 0U;
    robot_power_status.supported_confirmed = 0U;
    Robot_Power_StopLocalMotion();
    robot_power_status.hold_sent = 0U;
    // 시작 목표 확인·출력 복원 중 실패하면 출력 0으로 복귀 시도, 자동 토크 OFF는 하지 않음
    if (initial_output_owned) Power_ForceInitialOutputZero();
    if (try_hold && Power_Feedback(&state)) {
        for (uint8_t i = 0U; i < 12U; ++i) hold[i] = state.motors[i].position_raw;
        robot_power_status.hold_sent = Power_SendGoal(hold, 1U);
    }
    (void)Power_RestoreProfiles();
    robot_power_status.feedback_valid = 0U;
    dxl_knee_position_status = DXL_KNEE_POSITION_LOCKED;
    robot_power_status.error = error;
    robot_power_status.failed_id = failed_id;
    // Hold·프로파일 복원 조회가 최초 실패의 통신 진단값을 덮어쓰지 않도록 보존
    robot_power_status.device_error = device_error;
    robot_power_status.read_fail_stage = read_fail_stage;
    // 오류 처리 중 처음 발생한 복원 실패를 원래 동작의 실패 원인으로 오인하지 않음
    robot_power_status.write_failed_id = write_failed_id;
    robot_power_status.write_address = write_address;
    robot_power_status.write_expected = write_expected;
    robot_power_status.write_actual = write_actual;
    robot_power_status.write_mismatch_valid = write_mismatch_valid;
}

void RobotPower_Init(void)
{
    memset((void *)&robot_power_status, 0, sizeof(robot_power_status));
    memset((void *)&robot_power_comm_fault, 0, sizeof(robot_power_comm_fault));
    memset(&pending_comm_failure, 0, sizeof(pending_comm_failure));
    robot_power_status.state = ROBOT_POWER_LOCKED;
    dxl_knee_position_status = DXL_KNEE_POSITION_LOCKED;
    private_goal_active = private_goal_allow_stop = saved_profile_count = failed_samples = monitor_index = 0U;
    settling = 0U;
    boot_check_attempted = 0U;
    initial_wheel_reached = 0U;
    initial_output_owned = 0U;
    memset(initial_start_position, 0, sizeof(initial_start_position));
    sample_tick = HAL_GetTick();
}

uint8_t RobotPower_CheckBootStart(void)
{
    // MCU 부팅 때 한 번만 검사, 운전 중 오류·모터 재부팅 뒤에는 자동 재허가하지 않음
    if (boot_check_attempted || robot_power_status.state != ROBOT_POWER_LOCKED
            || Robot_Get_Mode() != MODE_NONE) return 0U;
    boot_check_attempted = 1U;
    return RobotPower_ConfirmStartSupported();
}

uint8_t RobotPower_MotionAllowed(void)
{
    return robot_power_status.state == ROBOT_POWER_READY
            && Power_ConfigValid() && !user_stop_flag;
}

uint8_t RobotPower_IsParking(void)
{
    return robot_power_status.state == ROBOT_POWER_PARKING;
}

uint8_t RobotPower_SafeToPowerOff(void)
{
    return robot_power_status.state == ROBOT_POWER_SAFE_OFF
            && robot_power_status.supported_confirmed && robot_power_status.feedback_valid
            && (uint32_t)(HAL_GetTick() - robot_power_status.last_feedback_ms)
                    <= WOKI_POWER_FEEDBACK_MAX_AGE_MS && !user_stop_flag;
}

void RobotPower_InvalidateReference(void)
{
    Robot_Power_StopLocalMotion();
    private_goal_active = 0U;
    dxl_knee_position_status = DXL_KNEE_POSITION_LOCKED;
    robot_power_status.state = ROBOT_POWER_LOCKED;
    initial_wheel_reached = 0U;
    robot_power_status.error = ROBOT_POWER_REFERENCE_CHANGED;
    robot_power_status.supported_confirmed = robot_power_status.feedback_valid = 0U;
}

uint8_t RobotPower_CheckInitialPose(void)
{
    DXL_Leg_State_t state;
    uint32_t goal;
    if (!RobotPower_MotionAllowed()) return 0U;
    if (!Power_CheckAllMotors(initial_wheel_reached ? 1 : 0) || !Power_Feedback(&state)) goto failed;
    if (initial_wheel_reached) {
        int32_t wheel[12];
        Robot_Get_Wheel_Pose_Targets(wheel);
        if (!Power_At(&state, wheel, WOKI_POWER_POSITION_TOLERANCE_RAW)) goto failed;
    } else {
        // Torque OFF에서 Goal Position을 유지하지 않는 모터를 허용, 실제 자세·출력 0은 확인
        if (!initial_output_owned) { robot_power_status.error = ROBOT_POWER_STATE; goto failed; }
        if (!Power_StartInSingleTurn(&state)
                || !Power_At(&state, initial_start_position, WOKI_POWER_POSITION_TOLERANCE_RAW)
                || !Power_CheckInitialOutputZero()) goto failed;
        return 1U;
    }
    for (uint8_t i = 0U; i < 12U; ++i) {
        if (!Power_Read(power_ids[i], DXL_2_Goal_Position, 4U, &goal)) goto failed;
        if (!Power_Close((int32_t)goal, state.motors[i].position_raw,
                WOKI_POWER_POSITION_TOLERANCE_RAW)) {
            robot_power_status.error = ROBOT_POWER_POSITION; goto failed;
        }
    }
    return 1U;
failed:
    Power_Fault(robot_power_status.error, initial_wheel_reached);
    return 0U;
}

// 실제 0~4095 구간의 현재 위치에서 Wheel 목표로 직접 이동, 중간 2048 목표 없음
// 다회전 좌표 보정·영점 변경 없이 저속 이동과 실제 도착·정지 상태를 확인
uint8_t RobotPower_MoveInitialWheelPose(void)
{
    DXL_Leg_State_t state;
    int32_t move_start[12], wheel[12];
    uint8_t ids[12], torque_on[12] = { 1,1,1,1,1,1,1,1,1,1,1,1 };
    uint32_t started, settled_at = 0U;
    uint8_t settled = 0U, failures = 0U;
    int32_t hold[12];
    if (!RobotPower_MotionAllowed() || Robot_Get_Mode() != MODE_NONE) return 0U;
    if (!RobotPower_CheckInitialPose()) return 0U;
    if (initial_wheel_reached) return 1U;
    Robot_Get_Wheel_Pose_Targets(wheel);
    memcpy(ids, power_ids, sizeof(ids));

    robot_power_status.startup_stage = ROBOT_START_PROFILE;
    saved_profile_count = 0U;
    for (uint8_t i = 0U; i < 12U; ++i) {
        if (!Power_Read(power_ids[i], DXL_2_Profile_Acceleration, 4U, &saved_acceleration[i])
                || !Power_Read(power_ids[i], DXL_2_Profile_Velocity, 4U, &saved_velocity[i])) goto failed_off;
        ++saved_profile_count;
        const uint32_t acceleration = WokiJoint_IsKneeIndex(i)
                ? WOKI_POWER_INITIAL_WHEEL_KNEE_ACC : WOKI_POWER_INITIAL_WHEEL_ACC;
        const uint32_t velocity = WokiJoint_IsKneeIndex(i)
                ? WOKI_POWER_INITIAL_WHEEL_KNEE_VEL : WOKI_POWER_INITIAL_WHEEL_VEL;
        if (!Power_WriteVerify(power_ids[i], DXL_2_Profile_Acceleration, 4U, acceleration)
                || !Power_WriteVerify(power_ids[i], DXL_2_Profile_Velocity, 4U, velocity)) goto failed_off;
    }
    // 이전 목표가 남아 있어도 출력이 나오지 않도록 Torque OFF·출력 0을 재확인
    if (!RobotPower_CheckInitialPose()) return 0U;
    if (!Power_Feedback(&state) || !Power_StartInSingleTurn(&state)) goto failed_off;
    if (!Power_At(&state, initial_start_position, WOKI_POWER_POSITION_TOLERANCE_RAW)) goto failed_off;
    if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; goto failed_off; }
    robot_power_status.startup_stage = ROBOT_START_TORQUE_ZERO;
    dxl_sync_write_1byte(ids, torque_on, 12U, DXL_2_Torque_Enable);
    // Torque ON으로 출력 0 설정이 바뀌지 않았는지도 확인, 불일치 시 출력 복원 금지
    if (!Power_CheckInitialOutputZero() || !Power_CheckAllMotors(1)
            || !Power_Feedback(&state) || !Power_StartInSingleTurn(&state)
            || !Power_At(&state, initial_start_position, WOKI_POWER_POSITION_TOLERANCE_RAW)) goto failed_on;
    if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; goto failed_on; }
    for (uint8_t i = 0U; i < 12U; ++i) hold[i] = state.motors[i].position_raw;
    robot_power_status.startup_stage = ROBOT_START_SEED_GOAL;
    if (!Power_SendGoal(hold, 0U) || !Power_CheckInitialGoal(hold)) goto failed_on;
    robot_power_status.startup_stage = ROBOT_START_VERIFY_HOLD;
    if (!Power_CheckAllMotors(1) || !Power_CheckInitialOutputZero() || !Power_CheckGoal(hold)
            || !Power_Feedback(&state)
            || !Power_At(&state, hold, WOKI_POWER_POSITION_TOLERANCE_RAW)) goto failed_on;
    if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; goto failed_on; }
    robot_power_status.startup_stage = ROBOT_START_RESTORE_OUTPUT;
    for (uint8_t i = 0U; i < 12U; ++i) {
        // 복원 명령이 일부만 적용되어도 0 확인 비트를 그대로 남기지 않음
        robot_power_status.startup_pwm_zero_mask &= (uint16_t)~(uint16_t)(1U << i);
        if (!Power_WriteVerify(power_ids[i], DXL_2_Goal_PWM, 2U, POWER_INITIAL_RUN_PWM)) goto failed_on;
    }
    // 기존 현재 위치 목표로 출력 복원 완료 후에만 Wheel 이동 시작, 이후 실패는 기존 Hold 처리
    if (!Power_CheckAllMotors(1) || !Power_CheckGoal(hold) || !Power_Feedback(&state)
            || !Power_At(&state, hold, WOKI_POWER_POSITION_TOLERANCE_RAW)) goto failed_on;
    if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; goto failed_on; }
    for (uint8_t i = 0U; i < 12U; ++i) move_start[i] = state.motors[i].position_raw;
    initial_output_owned = 0U;
    robot_power_status.startup_stage = ROBOT_START_WHEEL_MOVE;
    if (!Power_SendGoal(wheel, 0U) || !Power_CheckGoal(wheel)) goto failed_on;
    started = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started) < WOKI_POWER_INITIAL_WHEEL_TIMEOUT_MS) {
        if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; goto failed_on; }
        if (!Power_CheckAllMotors(1) || !Power_Feedback(&state)) {
            settled = 0U;
            if (robot_power_status.error != ROBOT_POWER_COMM || ++failures >= 3U) {
                if (robot_power_status.error == ROBOT_POWER_COMM)
                    pending_comm_failure.consecutive_failures = failures;
                goto failed_on;
            }
        } else {
            failures = 0U;
            // 현재 위치~Wheel 목표 구간을 벗어나거나 반대 방향으로 이동하는지 확인
            for (uint8_t i = 0U; i < 12U; ++i) {
                const int32_t low = move_start[i] < wheel[i] ? move_start[i] : wheel[i];
                const int32_t high = move_start[i] > wheel[i] ? move_start[i] : wheel[i];
                if ((int64_t)state.motors[i].position_raw < (int64_t)low - 32
                        || (int64_t)state.motors[i].position_raw > (int64_t)high + 32) {
                    robot_power_status.failed_id = power_ids[i];
                    robot_power_status.error = ROBOT_POWER_POSITION; goto failed_on;
                }
            }
            if (Power_At(&state, wheel, WOKI_POWER_POSITION_TOLERANCE_RAW)) {
                if (!settled) { settled_at = HAL_GetTick(); settled = 1U; }
                if ((uint32_t)(HAL_GetTick() - settled_at) >= WOKI_POWER_SETTLE_MS) {
                    if (!Power_CheckAllMotors(1) || !Power_CheckGoal(wheel)) goto failed_on;
                    // 정상 완료 뒤에도 기존 최초 Wheel의 저속 프로파일 유지
                    // 중단 시에만 저장값 복원, 정상 완료 직후 토크 OFF/ON은 하지 않음
                    saved_profile_count = 0U;
                    initial_wheel_reached = 1U;
                    robot_power_status.startup_stage = ROBOT_START_COMPLETE;
                    robot_power_status.error = ROBOT_POWER_OK;
                    robot_power_status.failed_id = 0U;
                    return 1U;
                }
            } else settled = 0U;
        }
        HAL_Delay(WOKI_POWER_SAMPLE_MS);
    }
    robot_power_status.error = ROBOT_POWER_TIMEOUT;
failed_on:
    // 실패 시 현재 위치 유지 시도, 몸체를 떨어뜨릴 수 있는 자동 Torque OFF는 하지 않음
    Power_Fault(robot_power_status.error, 1U);
    return 0U;
failed_off:
    Power_Fault(robot_power_status.error, 0U);
    return 0U;
}

// 메인 루프 감시 사이에 모터 전원이 재인가되어도 다음 자세 전환 전에는 다시 확인
// 이 확인은 모터 전원 차단 자체를 막거나 전원 OFF 중의 기구 자세를 알아내지는 못함
uint8_t RobotPower_CheckRunningReference(void)
{
    DXL_Leg_State_t state;
    if (!RobotPower_MotionAllowed()) return 0U;
    if (!Power_CheckAllMotors(1) || !Power_Feedback(&state)) {
        Power_Fault(robot_power_status.error, 1U);
        return 0U;
    }
    robot_power_status.error = ROBOT_POWER_OK;
    robot_power_status.failed_id = 0U;
    return 1U;
}

uint8_t RobotPower_ConfirmStartSupported(void)
{
    DXL_Leg_State_t state;
    if (robot_power_status.state == ROBOT_POWER_READY || RobotPower_IsParking()
            || Robot_Get_Mode() != MODE_NONE) {
        robot_power_status.error = ROBOT_POWER_STATE; return 0U;
    }
    if (!Power_ConfigValid()) { robot_power_status.error = ROBOT_POWER_CONFIG; return 0U; }
    if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; return 0U; }
    robot_power_status.state = ROBOT_POWER_CHECKING;
    initial_wheel_reached = 0U;
    robot_power_status.startup_stage = ROBOT_START_CHECK_OFF;
    robot_power_status.startup_failed_stage = 0U;
    // 전원 ON 전 실제 한 바퀴 구간으로 복귀하는 운용 규칙, 부팅 숫자로 회전수를 추정하지 않음
    // Torque OFF는 여기서 자동 실행하지 않고, 12개가 이미 OFF인지 읽어 확인
    if (!Power_CheckAllMotors(0) || !Power_Feedback(&state)
            || !Power_StartInSingleTurn(&state)) goto failed;
    HAL_Delay(20U);
    if (!Power_Feedback(&state)
            || !Power_StartInSingleTurn(&state)) goto failed;
    for (uint8_t i = 0U; i < 12U; ++i) initial_start_position[i] = state.motors[i].position_raw;
    // 12개 Torque OFF·실제 시작 자세 확인 후에만 이 기동 세션이 출력 제한을 관리
    initial_output_owned = 1U;
    robot_power_status.startup_stage = ROBOT_START_ZERO_OUTPUT;
    // 기존 PID 값 유지, Goal PWM은 0으로 준비하고 목표 확인 뒤 기존 885로 복원
    for (uint8_t i = 0U; i < 12U; ++i) {
        if (!Power_WriteVerify(power_ids[i], DXL_2_Position_D_Gain, 2U, 0U)
                || !Power_WriteVerify(power_ids[i], DXL_2_Position_I_Gain, 2U, 0U)
                || !Power_WriteVerify(power_ids[i], DXL_2_Position_P_Gain, 2U, 850U)
                || !Power_WriteVerify(power_ids[i], DXL_2_Goal_PWM, 2U, 0U)) goto failed;
    }
    // Torque OFF에서는 목표 저장·완전 일치를 요구하지 않음, 출력 0·실제 자세만 준비
    if (!Power_CheckAllMotors(0) || !Power_CheckInitialOutputZero() || !Power_Feedback(&state)
            || !Power_StartInSingleTurn(&state)
            || !Power_At(&state, initial_start_position, WOKI_POWER_POSITION_TOLERANCE_RAW)) goto failed;
    if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; goto failed; }
    dxl_knee_position_status = DXL_KNEE_POSITION_READY;
    robot_power_status.state = ROBOT_POWER_READY;
    robot_power_status.startup_stage = ROBOT_START_PREPARED_OFF;
    robot_power_status.error = ROBOT_POWER_OK;
    robot_power_status.failed_id = 0U;
    robot_power_status.supported_confirmed = 0U;
    failed_samples = monitor_index = 0U;
    sample_tick = HAL_GetTick();
    return 1U;
failed:
    Power_Fault(robot_power_status.error, 0U);
    return 0U;
}

uint8_t RobotPower_PreparePowerOff(void)
{
    DXL_Leg_State_t state;
    int32_t wheel[12];
    if (!RobotPower_MotionAllowed() || Robot_Get_Mode() != MODE_WHEEL) {
        robot_power_status.error = ROBOT_POWER_STATE; return 0U;
    }
    if (!Power_ParkConfigValid()) {
        robot_power_status.error = ROBOT_POWER_CONFIG; return 0U;
    }
    // Leg·경사로·보행 중 즉시 주차 목표를 보내지 않음. 기존 WHEEL 복귀 완료 후 요청
    Robot_Get_Wheel_Pose_Targets(wheel);
    if (!Power_CheckAllMotors(1) || !Power_Feedback(&state)
            || !Power_At(&state, wheel, WOKI_KNEE_STARTUP_TOLERANCE_RAW)) goto failed;
    for (uint8_t i = 0U; i < 12U; ++i) park_start[i] = state.motors[i].position_raw;
    Robot_Power_StopLocalMotion();
    robot_power_status.state = ROBOT_POWER_PARKING;
    robot_power_status.supported_confirmed = 0U;
    saved_profile_count = 0U;
    // 주차에서만 저속 프로파일 적용, 완료·중단 시 원래 값 복원
    for (uint8_t i = 0U; i < 12U; ++i) {
        if (!Power_Read(power_ids[i], DXL_2_Profile_Acceleration, 4U, &saved_acceleration[i])
                || !Power_Read(power_ids[i], DXL_2_Profile_Velocity, 4U, &saved_velocity[i])) goto failed;
        ++saved_profile_count;
        const uint32_t acceleration = WokiJoint_IsKneeIndex(i) ? 13U : 5U;
        const uint32_t velocity = WokiJoint_IsKneeIndex(i) ? 25U : 10U;
        if (!Power_WriteVerify(power_ids[i], DXL_2_Profile_Acceleration, 4U, acceleration)
                || !Power_WriteVerify(power_ids[i], DXL_2_Profile_Velocity, 4U, velocity)) goto failed;
    }
    if (user_stop_flag) { robot_power_status.error = ROBOT_POWER_STOP; goto failed; }
    if (!Power_SendGoal(park_position, 0U) || !Power_CheckGoal(park_position)) goto failed;
    park_tick = sample_tick = HAL_GetTick();
    robot_power_status.elapsed_ms = 0U;
    failed_samples = monitor_index = settling = 0U;
    robot_power_status.error = ROBOT_POWER_OK;
    robot_power_status.failed_id = 0U;
    return 1U;
failed:
    Power_Fault(robot_power_status.error, 1U);
    return 0U;
}

uint8_t RobotPower_ConfirmPowerOffSupported(void)
{
    DXL_Leg_State_t state;
    if (robot_power_status.state != ROBOT_POWER_PARKED) {
        robot_power_status.error = ROBOT_POWER_STATE; return 0U;
    }
    // 사용자가 몸체 외부 지지·전원 차단 후 자세 유지 가능 여부를 확인한 뒤 요청
    if (!Power_CheckAllMotors(1) || !Power_Feedback(&state)
            || !Power_At(&state, park_position, WOKI_POWER_POSITION_TOLERANCE_RAW)) {
        Power_Fault(robot_power_status.error, 1U); return 0U;
    }
    robot_power_status.supported_confirmed = 1U;
    robot_power_status.state = ROBOT_POWER_SAFE_OFF;
    robot_power_status.error = ROBOT_POWER_OK;
    robot_power_status.failed_id = 0U;
    return RobotPower_SafeToPowerOff();
}

void RobotPower_Cancel(void)
{
    // 이미 발생한 원인을 이후 버튼 입력으로 STOP에 덮어쓰지 않음
    if (robot_power_status.state == ROBOT_POWER_FAULT) return;
    const uint8_t try_hold = robot_power_status.state == ROBOT_POWER_READY
            || robot_power_status.state == ROBOT_POWER_PARKING
            || robot_power_status.state == ROBOT_POWER_PARKED
            || robot_power_status.state == ROBOT_POWER_SAFE_OFF;
    Power_Fault(ROBOT_POWER_STOP, try_hold);
}

void RobotPower_Task(void)
{
    const uint32_t now = HAL_GetTick();
    const RobotPowerState phase = robot_power_status.state;
    DXL_Leg_State_t state;
    if (phase != ROBOT_POWER_READY && phase != ROBOT_POWER_PARKING
            && phase != ROBOT_POWER_PARKED && phase != ROBOT_POWER_SAFE_OFF) return;
    if (user_stop_flag) { Power_Fault(ROBOT_POWER_STOP, 1U); return; }
    if (phase == ROBOT_POWER_PARKING) {
        robot_power_status.elapsed_ms = now - park_tick;
        if (robot_power_status.elapsed_ms >= WOKI_POWER_PARK_TIMEOUT_MS) {
            Power_Fault(ROBOT_POWER_TIMEOUT, 1U); return;
        }
    }
    if ((uint32_t)(now - sample_tick) < WOKI_POWER_SAMPLE_MS) return;
    sample_tick = now;
    // 메인 루프에서 순환 조회해 모터만 재부팅된 경우의 Torque OFF·설정 변경 검출
    // 기존 blocking 전환 내부까지 즉시 감시하는 기능은 아니며 외부 비상정지를 대체하지 않음
    const int8_t torque = phase == ROBOT_POWER_READY && Robot_Get_Mode() == MODE_NONE ? -1 : 1;
    if (!Power_CheckMotor(monitor_index, torque)) {
        settling = 0U;
        robot_power_status.feedback_valid = 0U;
        if (++failed_samples >= 3U || robot_power_status.error != ROBOT_POWER_COMM)
            Power_Fault(robot_power_status.error, 1U);
        return;
    }
    monitor_index = (uint8_t)((monitor_index + 1U) % 12U);
    if (phase == ROBOT_POWER_READY) {
        failed_samples = 0U;
        robot_power_status.error = ROBOT_POWER_OK;
        robot_power_status.failed_id = 0U;
        return;
    }
    if (!Power_Feedback(&state)) {
        settling = 0U;
        if (++failed_samples >= 3U) Power_Fault(robot_power_status.error, 1U);
        return;
    }
    failed_samples = 0U;
    if (phase == ROBOT_POWER_PARKING) {
        for (uint8_t i = 0U; i < 12U; ++i) {
            const int32_t low = park_start[i] < park_position[i] ? park_start[i] : park_position[i];
            const int32_t high = park_start[i] > park_position[i] ? park_start[i] : park_position[i];
            if ((int64_t)state.motors[i].position_raw < (int64_t)low - 32
                    || (int64_t)state.motors[i].position_raw > (int64_t)high + 32) {
                robot_power_status.failed_id = power_ids[i];
                Power_Fault(ROBOT_POWER_POSITION, 1U); return;
            }
        }
        if (Power_At(&state, park_position, WOKI_POWER_POSITION_TOLERANCE_RAW)) {
            if (!settling) { settle_tick = now; settling = 1U; }
            if ((uint32_t)(now - settle_tick) >= WOKI_POWER_SETTLE_MS) {
                if (!Power_CheckAllMotors(1) || !Power_CheckGoal(park_position)
                        || !Power_RestoreProfiles()) {
                    Power_Fault(robot_power_status.error, 1U); return;
                }
                robot_power_status.state = ROBOT_POWER_PARKED;
            }
        } else settling = 0U;
        // 이동 중의 도착 위치 불일치는 별도 오류로 표시하지 않음
        robot_power_status.error = ROBOT_POWER_OK;
    } else if (!Power_At(&state, park_position, WOKI_POWER_POSITION_TOLERANCE_RAW)) {
        Power_Fault(robot_power_status.error, 1U); return;
    }
    robot_power_status.failed_id = 0U;
}
