// 전원 재인가 후 기준 자세 확인 및 종료 준비
#ifndef INC_WOKI_POWER_GUARD_H_
#define INC_WOKI_POWER_GUARD_H_

#include <stdint.h>
#include "woki_joint_config.h"
#include "dxl_2_0.h"

// 전원 ON 전 사용자가 Knee 네 개의 실제 누적 위치를 0~4095 구간으로 복귀
// 정상·비상 종료 모두 동일하게 적용, 부팅 후 표시 숫자만으로 실물 구간을 판단하지 않음
// 12개 관절이 0~4095 안에 있으면 현재 위치에서 저속으로 Wheel 자세에 직접 이동
// 별도 승인 명령·누적 좌표 자동 보정·영점 설정 변경 없음

// 종료 준비는 기존 WHEEL 복귀가 완료된 뒤에만 허용, 별도 주차 경로를 만들지 않음
// Wheel 4096/0으로 종료해도 다음 전원 ON 전에는 위의 실제 한 바퀴 구간 복귀 규칙 적용
#ifndef WOKI_POWER_PARK_POSE_VERIFIED
#define WOKI_POWER_PARK_POSE_VERIFIED 1U
#endif
#ifndef WOKI_POWER_PARK_PATH_VERIFIED
#define WOKI_POWER_PARK_PATH_VERIFIED 1U
#endif

// 순서: FL(1,2,3), FR(11,12,13), RL(21,22,23), RR(31,32,33)
// 종료용 Wheel 자세는 시작 검사 자세와 별개, 4096을 0으로 나머지 처리하지 않음
#ifndef WOKI_POWER_PARK_POSITION_INITIALIZER
#define WOKI_POWER_PARK_POSITION_INITIALIZER { \
    2048, 2048, (int32_t)WOKI_KNEE_REFERENCE_FL_RAW, \
    2048, 2048, (int32_t)WOKI_KNEE_REFERENCE_FR_RAW, \
    2048, 2048, (int32_t)WOKI_KNEE_REFERENCE_RL_RAW, \
    2048, 2048, (int32_t)WOKI_KNEE_REFERENCE_RR_RAW }
#endif

#define WOKI_POWER_POSITION_MARGIN_RAW     128
// 부팅 시 허용하는 현재 위치 구간, 2048 근처에 있어야 한다는 별도 조건 없음
#define WOKI_POWER_START_MIN_RAW           0
#define WOKI_POWER_START_MAX_RAW           4095
// 초기 Wheel·주차 완료 판정의 위치 허용오차 (모터 raw 카운트, ±50)
#define WOKI_POWER_POSITION_TOLERANCE_RAW  50
#define WOKI_POWER_SETTLED_VELOCITY_RAW    2
#define WOKI_POWER_SETTLE_MS              500U
#define WOKI_POWER_SAMPLE_MS              20U
#define WOKI_POWER_FEEDBACK_MAX_AGE_MS    100U
#define WOKI_POWER_PARK_TIMEOUT_MS        60000U
#define WOKI_POWER_INITIAL_WHEEL_TIMEOUT_MS 60000U
// 최초 Wheel 직접 이동 전용 저속 프로파일, 기존 Wheel↔Leg 궤적과 별개
#define WOKI_POWER_INITIAL_WHEEL_ACC       2U
#define WOKI_POWER_INITIAL_WHEEL_VEL       5U
#define WOKI_POWER_INITIAL_WHEEL_KNEE_ACC  5U
#define WOKI_POWER_INITIAL_WHEEL_KNEE_VEL  13U

typedef enum {
    ROBOT_POWER_LOCKED = 0, ROBOT_POWER_CHECKING, ROBOT_POWER_READY,
    ROBOT_POWER_PARKING, ROBOT_POWER_PARKED, ROBOT_POWER_SAFE_OFF,
    ROBOT_POWER_FAULT
} RobotPowerState;

typedef enum {
    ROBOT_POWER_OK = 0, ROBOT_POWER_CONFIG, ROBOT_POWER_STATE,
    ROBOT_POWER_STOP, ROBOT_POWER_COMM, ROBOT_POWER_MODE,
    ROBOT_POWER_DRIVE, ROBOT_POWER_OFFSET, ROBOT_POWER_TORQUE,
    ROBOT_POWER_HARDWARE, ROBOT_POWER_POSITION, ROBOT_POWER_VELOCITY,
    ROBOT_POWER_WRITE, ROBOT_POWER_TIMEOUT, ROBOT_POWER_REFERENCE_CHANGED
} RobotPowerError;

// 최초 기동 진단용 단계, 기존 전원 상태·Jetson 패킷 형식과 별개
typedef enum {
    ROBOT_START_IDLE = 0, ROBOT_START_CHECK_OFF, ROBOT_START_ZERO_OUTPUT,
    ROBOT_START_PREPARED_OFF, ROBOT_START_PROFILE, ROBOT_START_TORQUE_ZERO,
    ROBOT_START_SEED_GOAL, ROBOT_START_VERIFY_HOLD, ROBOT_START_RESTORE_OUTPUT,
    ROBOT_START_WHEEL_MOVE, ROBOT_START_COMPLETE
} RobotPowerStartupStage;

typedef struct {
    RobotPowerState state;
    RobotPowerError error;
    uint8_t failed_id, device_error, read_fail_stage, hold_sent;
    uint8_t feedback_valid, supported_confirmed;
    uint32_t last_feedback_ms, elapsed_ms;
    // MCU 부팅 후 최초 쓰기 값 불일치 기록, 정상 응답을 읽고 비교에 실패한 경우만 유효
    // 전송·수신 실패의 미확인 값을 실제 읽힌 값으로 표시하지 않음
    uint8_t write_mismatch_valid, write_failed_id;
    uint16_t write_address;
    uint32_t write_expected, write_actual; // 레지스터 raw 비트값 그대로 보존
    uint8_t startup_stage, startup_failed_stage;
    uint16_t startup_pwm_zero_mask; // 마지막 확인에서 Goal PWM=0을 읽은 모터 비트, 전체 0xFFF
} RobotPowerStatus;

extern volatile RobotPowerStatus robot_power_status;

// 정지를 확정한 COMM 오류만 보존, 정상 재조회·Hold·프로파일 복원으로 덮어쓰지 않음
enum RobotPowerCommSource { ROBOT_COMM_NONE = 0, ROBOT_COMM_READ, ROBOT_COMM_SYNC };
typedef struct {
    uint8_t valid, source, startup_stage, failed_id;
    uint8_t read_fail_stage, device_error, query_attempts, consecutive_failures;
    uint16_t read_address;
    uint8_t read_size;
    uint32_t timestamp_ms;
    DXL_Sync_Read_Diagnostics_t sync; // source=ROBOT_COMM_SYNC일 때만 유효
} RobotPowerCommFault;
extern volatile RobotPowerCommFault robot_power_comm_fault;

// 초기화는 RAM 상태만 변경, 모드 변경·토크 인가·자동 자세 이동 없음
void RobotPower_Init(void);
// MCU 부팅 시 한 번만 시작 검사, 출력 0 확인 후에도 여기서는 토크 ON·자세 이동 없음
uint8_t RobotPower_CheckBootStart(void);
// 출력 0에서 토크 ON → 현재 위치 목표 확인 → 출력 복원 후 최초 Wheel 저속 이동
uint8_t RobotPower_MoveInitialWheelPose(void);
void RobotPower_Task(void);
uint8_t RobotPower_ConfirmStartSupported(void);
uint8_t RobotPower_CheckInitialPose(void);
uint8_t RobotPower_CheckRunningReference(void);
uint8_t RobotPower_PreparePowerOff(void);
uint8_t RobotPower_ConfirmPowerOffSupported(void);
void RobotPower_Cancel(void);
uint8_t RobotPower_MotionAllowed(void);
uint8_t RobotPower_IsParking(void);
uint8_t RobotPower_SafeToPowerOff(void);

// 운전 모드·방향·영점 설정 변경 시 기존 기준 확인 무효화
void RobotPower_InvalidateReference(void);

// 드라이버의 위치 출력 잠금 확인, 내부 준비·정지용 목표는 일회성으로 제한
uint8_t RobotPower_PrivateGoalAllowed(uint8_t id, int32_t position);

#endif /* INC_WOKI_POWER_GUARD_H_ */
