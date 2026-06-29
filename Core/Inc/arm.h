/**
 * @file    arm.h
 * @brief   2-DOF 机械臂控制模块 — 直接操作 CAN 收发队列
 *
 * 移植自 Dog_arm_26 项目 ArmInTerface + Arm_Control,
 * 不再通过 CanManager (ROS2 topic) 收发 CAN, 改为直接读写
 * FreeRTOS 消息队列 canTxHandle / canRxHandle。
 *
 * 微 ROS (rclc) 指令接收 & 状态发布接口暂时留空, 仅留出:
 *   - Arm_PutCommand()   : 将上位机指令放入本模块内部目标寄存器
 *   - Arm_GetState()     : 提取当前状态供 micro-ROS publisher 使用
 */

#ifndef ARM_H
#define ARM_H

#include "can.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <cmsis_os.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 常量 ======================================================== */

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

#define RPM_TO_RADPS      (2.0f * M_PI / 60.0f) /* RPM → rad/s     */
#define DEG_TO_RAD        (M_PI / 180.0f)       /* deg → rad       */
#define CELSIUS_TO_KELVIN 273.15f               /* °C → K         */

/* C620 编码器线数 */
#define C620_ENCODER_NUM_PER_ROUND 8192U

/* C620 电流映射系数: int16 满量程 16384 → 20A */
#define C620_CURRENT_TO_OUT (16384.0f / 20.0f)

/* ========== 枚举 ======================================================== */

/** DM 电机控制状态 (传统模式) */
typedef enum {
    ArmDM_Status_DISABLE         = 0x0,
    ArmDM_Status_ENABLE          = 0x1,
    ArmDM_Status_OVERVOLTAGE     = 0x8,
    ArmDM_Status_UNDERVOLTAGE    = 0x9,
    ArmDM_Status_OVERCURRENT     = 0xA,
    ArmDM_Status_MOS_OVERTEMP    = 0xB,
    ArmDM_Status_ROTOR_OVERTEMP  = 0xC,
    ArmDM_Status_LOSE_CONNECTION = 0xD,
    ArmDM_Status_MOS_OVERLOAD    = 0xE,
} ArmDM_ControlStatus;

/** DJI 电机 CAN ID */
typedef enum {
    ArmDJI_ID_0x201 = 1,
    ArmDJI_ID_0x202,
    ArmDJI_ID_0x203,
    ArmDJI_ID_0x204,
    ArmDJI_ID_0x205,
    ArmDJI_ID_0x206,
    ArmDJI_ID_0x207,
    ArmDJI_ID_0x208,
} ArmDJI_MotorID;

/** DJI 电机控制模式 */
typedef enum {
    ArmDJI_Ctrl_VOLTAGE = 0,
    ArmDJI_Ctrl_CURRENT,
    ArmDJI_Ctrl_TORQUE,
    ArmDJI_Ctrl_OMEGA,
    ArmDJI_Ctrl_ANGLE,
} ArmDJI_ControlMethod;

/** 升降归零状态机 */
typedef enum {
    Arm_HeightHoming_IDLE         = 0,
    Arm_HeightHoming_MOVING_DOWN  = 1,
    Arm_HeightHoming_STALL_DETECT = 2,
    Arm_HeightHoming_COMPLETE     = 3,
} Arm_HeightHomingState;

/* ========== PID 控制器 ================================================== */

typedef struct {
    float kp, ki, kd, kf;     /* 增益            */
    float i_out_max, out_max; /* 限幅            */
    float dead_zone;          /* 死区            */
    float dt;                 /* 控制周期 (s)     */

    float target, now; /* 目标 / 当前值    */
    float pre_now, pre_target;
    float pre_error, pre_out;
    float integral_error;
    float out;
    bool deadband;
} Arm_PID;

/* ========== 五次多项式轨迹规划器 ========================================== */

typedef struct {
    float out;         /* 当前输出 */
    float target;      /* 目标终点 */
    float start_val;   /* 轨迹起点 */
    float prev_target; /* 上一帧目标 (检测变化) */
    float elapsed;     /* 轨迹已用时间 (s) */
    float duration;    /* 轨迹总时长 (s)   */
    float max_vel;     /* 最大速度 (单位/s) */
    float dt;          /* 时间步长 (s)     */
    bool active;       /* 轨迹进行中       */
} Arm_Slope;

/* ========== DM 4310 电机 (旋转关节, MIT 模式) ============================ */

typedef struct {
    uint16_t can_rx_id; /* 接收 CAN ID (上位机 Master_ID)   */
    uint16_t can_tx_id; /* 发送 CAN ID (CAN_ID + 偏移)      */
    float angle_max;    /* 最大角度 (rad)                   */
    float omega_max;    /* 最大角速度 (rad/s)               */
    float torque_max;   /* 最大扭矩 (Nm)                    */
    float current_max;  /* 最大电流 (A)                     */

    /* ── 解码后的接收数据 ── */
    ArmDM_ControlStatus control_status;
    float now_angle;  /* 当前角度 (rad)                   */
    float now_omega;  /* 当前角速度 (rad/s)               */
    float now_torque; /* 当前扭矩 (Nm)                    */
    float mos_temp;   /* MOS 温度 (K)                     */
    float rotor_temp; /* 转子温度 (K)                     */

    /* ── 累计量 ── */
    uint16_t pre_encoder;
    int32_t total_encoder;
    int32_t total_round;
    bool encoder_primed; /* 首帧编码器已基准化    */

    /* ── 接收标志 / DMA ISR 去抖动 ── */
    uint32_t flag;
    uint32_t pre_flag;
} Arm_DM_Motor;

/* ========== DJI C620 电机 (升降关节) ==================================== */

typedef struct {
    ArmDJI_MotorID can_rx_id; /* 接收 CAN ID */
    uint16_t can_tx_id;       /* 发送 CAN ID (0x1FF 或 0x200) */
    ArmDJI_ControlMethod control_method;

    float current_max;  /* 最大电流 (A)                    */
    float gearbox_rate; /* 减速比                          */

    Arm_PID pid_angle;
    Arm_PID pid_omega;

    /* ── 解码后的接收数据 ── */
    float now_angle;   /* 当前角度 (rad, 输出轴)          */
    float now_omega;   /* 当前角速度 (rad/s, 输出轴)      */
    float now_current; /* 当前电流 (A)                    */
    float temperature; /* 温度 (K)                        */

    /* ── 累计量 ── */
    uint16_t pre_encoder;
    uint16_t clear_encoder;
    int32_t total_encoder;
    int32_t total_round;
    bool encoder_primed; /* 首帧编码器已基准化    */

    /* ── 目标值 ── */
    float target_angle;
    float target_omega;
    float target_current;

    /* ── 内部 ── */
    float out; /* 映射到 int16 的输出值           */
    uint8_t angle_clear_flag;
    uint32_t flag;
    uint32_t pre_flag;

    /* ── C620 组帧发送缓冲区 (本电机在其中的偏移由 allocate 决定) ── */
    uint8_t *tx_data_ptr;
} Arm_DJI_MotorC620;

/* ========== Arm 全局配置 ================================================ */

typedef struct {
    /* ── DM 4310 (旋转) ── */
    uint8_t dm_rx_id;
    uint8_t dm_tx_id;
    float dm_angle_max;
    float dm_omega_max;
    float dm_torque_max;
    float dm_current_max;
    float dm_target_omega;  /* MIT 前馈角速度   */
    float dm_target_torque; /* MIT 前馈扭矩     */
    float dm_kp;            /* MIT Kp           */
    float dm_kd;            /* MIT Kd           */

    /* ── DJI 3508/C620 (升降) ── */
    ArmDJI_MotorID c620_rx_id;
    float c620_current_max;
    float c620_gearbox;   /* 减速比, 默认 3591/187 */
    float c620_pid_dt;    /* PID 时间步长 (s)       */
    float gear_radius_mm; /* 输出齿轮半径 (mm)      */

    /* 角度环 PID */
    float c620_angle_kp, c620_angle_ki, c620_angle_kd, c620_angle_kf;
    float c620_angle_i_out_max, c620_angle_out_max, c620_angle_dead_zone;

    /* 角速度环 PID */
    float c620_omega_kp, c620_omega_ki, c620_omega_kd, c620_omega_kf;
    float c620_omega_i_out_max, c620_omega_out_max, c620_omega_dead_zone;

    /* ── 五次多项式轨迹 ── */
    float loop_period_s;
    float base_angle_max_vel; /* rad/s */
    float height_max_vel;     /* mm/s  */

    /* ── 初始目标 ── */
    float target_base_angle; /* rad */
    float target_height_mm;  /* mm  */

    /* ── 升降归零 ── */
    float height_homing_current;             /* A        */
    float height_homing_stall_time_s;        /* s        */
    float height_homing_stall_threshold_rad; /* rad  */
} Arm_Config;

/* ========== Arm 控制实例 ================================================ */

typedef struct {
    Arm_Config cfg;

    /* 电机对象 */
    Arm_DM_Motor dm;
    Arm_DJI_MotorC620 c620;

    /* 五次多项式轨迹 */
    Arm_Slope base_angle_slope;
    Arm_Slope height_slope;

    /* 归零状态机 */
    Arm_HeightHomingState homing_state;
    float homing_last_angle;
    float homing_stall_timer;
    float homing_direction;
    bool homing_direction_decided;
    float height_zero_angle;

    /* DM 使能重试 */
    float dm_enable_retry_timer;
    bool dm_was_enabled;

    /* 状态 */
    bool inited;

    /* ── CAN 队列引用 (由外部注入) ── */
    osMessageQueueId_t canTxQueue;
    osMessageQueueId_t canRxQueue;
} Arm_Control;

/* ========== 默认配置 ==================================================== */

extern const Arm_Config Arm_Config_Default;

/* ========== API ========================================================= */

/**
 * @brief 初始化 Arm_Control 实例
 * @param arm   实例指针
 * @param cfg   配置 (可传 NULL 使用默认值)
 */
void Arm_Init(Arm_Control *arm, const Arm_Config *cfg);

/**
 * @brief 执行归零: DM 使能+回零, C620 升降堵转归零
 * @note  阻塞直到归零完成; 调用前队列必须已绑定, 调用后 arm 处于零位就绪状态
 */
void Arm_Homing(Arm_Control *arm);

/**
 * @brief 更新目标值 (等效于 ROS topic 回调)
 * @param arm             实例指针
 * @param base_angle_rad  基座旋转目标角度 (rad)
 * @param height_mm       末端期望高度 (mm)
 */
void Arm_SetTarget(Arm_Control *arm, float base_angle_rad, float height_mm);

/**
 * @brief 单步控制循环: 读 CAN → 解码 → 归零/PID → 组帧 → 写 CAN TX 队列
 * @note  调用频率应与 cfg.loop_period_s 一致
 */
void Arm_Update(Arm_Control *arm);

/**
 * @brief 喂入一帧 CAN 原始数据 (由 CAN ISR 线程调用)
 * @note  在 Arm_Update 之前调用, 将 ISR 环缓冲中的帧逐个交给 arm 解码
 */
void Arm_FeedRxFrame(Arm_Control *arm, const CANFrame_t *frame);

/**
 * @brief 失能电机, 退出前调用
 */
void Arm_Shutdown(Arm_Control *arm);

/* ── 状态查询 ── */

float Arm_GetBaseAngle(const Arm_Control *arm);
float Arm_GetBaseOmega(const Arm_Control *arm);
float Arm_GetHeightMm(const Arm_Control *arm);
float Arm_GetLiftAngle(const Arm_Control *arm);
bool Arm_IsHeightHomed(const Arm_Control *arm);
float Arm_GetLoopPeriodS(const Arm_Control *arm);

/**
 * @brief 将上位机下发的目标放入 arm (由 micro-ROS subscriber 回调调用)
 * @note  目前直接透传 Arm_SetTarget
 */
void Arm_PutCommand(Arm_Control *arm, float base_angle_rad, float height_mm);

/**
 * @brief 提取当前状态, 供 micro-ROS publisher 定时上报
 */
void Arm_GetState(const Arm_Control *arm,
                  float *base_angle_rad,
                  float *base_omega_radps,
                  float *height_mm,
                  bool *homed);

/**
 * @brief 将 arm 标记为需要失能 (供 service 回调)
 */
void Arm_RequestShutdown(Arm_Control *arm, bool shutdown);

#ifdef __cplusplus
}
#endif

#endif /* ARM_H */
