/**
 * @file    arm.c
 * @brief   2-DOF 机械臂控制 — 实现
 *
 * 移植自 Dog_arm_26 项目的 ArmInTerface + Arm_Control,
 * CAN 收发直接走 FreeRTOS 消息队列 canTxHandle / canRxHandle.
 */

#include "arm.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>

/* ========== 默认配置 ==================================================== */

const Arm_Config Arm_Config_Default = {
    /* DM 4310 */
    .dm_rx_id         = 0x00,
    .dm_tx_id         = 0x01,
    .dm_angle_max     = 12.5f,
    .dm_omega_max     = 30.0f,
    .dm_torque_max    = 10.0f,
    .dm_current_max   = 7.0f,
    .dm_target_omega  = 0.0f,
    .dm_target_torque = 0.0f,
    .dm_kp            = 100.0f,
    .dm_kd            = 1.0f,

    /* DJI 3508 */
    .c620_rx_id       = ArmDJI_ID_0x201,
    .c620_current_max = 20.0f,
    .c620_gearbox     = 3591.0f / 187.0f,
    .c620_pid_dt      = 0.001f,
    .gear_radius_mm   = 30.0f,

    .c620_angle_kp        = 15.0f,
    .c620_angle_ki        = 1.0f,
    .c620_angle_kd        = 0.0f,
    .c620_angle_kf        = 0.0f,
    .c620_angle_i_out_max = 10.0f, /* rad/s, 积分饱和上限 */
    .c620_angle_out_max   = 30.0f, /* rad/s, 限制最大角速度指令 */
    .c620_angle_dead_zone = 0.0f,

    .c620_omega_kp        = 1.0f,
    .c620_omega_ki        = 0.5f,
    .c620_omega_kd        = 0.0f,
    .c620_omega_kf        = 0.0f,
    .c620_omega_i_out_max = 20.0f, /* A, 积分饱和上限 */
    .c620_omega_out_max   = 5.0f,  /* A, 对应 c620_current_max */
    .c620_omega_dead_zone = 0.0f,

    /* 五次多项式轨迹 */
    .loop_period_s      = 0.001f,
    .base_angle_max_vel = 3.0f,
    .height_max_vel     = 80.0f,

    /* 初始目标 */
    .target_base_angle = 0.0f,
    .target_height_mm  = 0.0f,

    /* 归零 */
    .height_homing_current             = 1.0f,
    .height_homing_stall_time_s        = 0.5f,
    .height_homing_stall_threshold_rad = 0.015f,
};

/* ========== 工具函数 ==================================================== */

/**
 * @brief 浮点数 → 整数 线性映射
 */
static inline int32_t float_to_int(float x,
                                   float f1, float f2,
                                   int32_t i1, int32_t i2)
{
    float tmp = (x - f1) / (f2 - f1);
    return (int32_t) (tmp * (float) (i2 - i1) + (float) i1);
}

/**
 * @brief 整数 → 浮点数 线性映射
 */
static inline float int_to_float(int32_t x,
                                 int32_t i1, int32_t i2,
                                 float f1, float f2)
{
    float tmp = (float) (x - i1) / (float) (i2 - i1);
    return tmp * (f2 - f1) + f1;
}

/**
 * @brief 16-bit 大小端翻转 (原地)
 */
static inline uint16_t endian_reverse_16(const void *src)
{
    const uint8_t *p = (const uint8_t *) src;
    return ((uint16_t) p[0] << 8) | (uint16_t) p[1];
}

/**
 * @brief 数值限幅
 */
static inline void constrain_f(float *v, float min, float max)
{
    if (*v < min) {
        *v = min;

    } else if (*v > max) {
        *v = max;
    }
}

static float wrap_angle_error_rad(float angle)
{
    const float two_pi = 2.0f * M_PI;

    while (angle > M_PI) {
        angle -= two_pi;
    }
    while (angle < -M_PI) {
        angle += two_pi;
    }
    return angle;
}

static float expand_nearest_angle_rad(float reference, float target)
{
    return reference + wrap_angle_error_rad(target - reference);
}

static float wrap_to_symmetric_range(float value, float limit)
{
    const float span = 2.0f * limit;

    if (limit <= 0.0f) {
        return value;
    }

    while (value > limit) {
        value -= span;
    }
    while (value < -limit) {
        value += span;
    }
    return value;
}

/* ========== PID ========================================================== */

static void pid_init(Arm_PID *pid,
                     float kp, float ki, float kd, float kf,
                     float i_out_max, float out_max,
                     float dt, float dead_zone)
{
    memset(pid, 0, sizeof(*pid));
    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->kf        = kf;
    pid->i_out_max = i_out_max;
    pid->out_max   = out_max;
    pid->dt        = dt;
    pid->dead_zone = dead_zone;
}

static void pid_calc(Arm_PID *pid)
{
    float error   = pid->target - pid->now;
    float abs_err = fabsf(error);

    pid->deadband = false;

    /* 死区 */
    if (abs_err < pid->dead_zone) {
        pid->target     = pid->now;
        pid->out        = 0.0f;
        pid->pre_now    = pid->now;
        pid->pre_target = pid->target;
        pid->pre_out    = pid->out;
        pid->pre_error  = 0.0f;
        pid->deadband   = true;
        return;
    }

    if (error > 0.0f) {
        error -= pid->dead_zone;

    } else if (error < 0.0f) {
        error += pid->dead_zone;
    }

    /* P */
    float p_out = pid->kp * error;

    /* I */
    pid->integral_error += pid->dt * error;
    if (pid->i_out_max != 0.0f && pid->ki != 0.0f) {
        constrain_f(&pid->integral_error,
                    -pid->i_out_max / pid->ki,
                    pid->i_out_max / pid->ki);
    }
    float i_out = pid->ki * pid->integral_error;

    /* D */
    float d_out = 0.0f;
    if (pid->dt > 0.0f) {
        d_out = pid->kd * (error - pid->pre_error) / pid->dt;
    }

    /* F (前馈) */
    float f_out = pid->kf;

    pid->out = p_out + i_out + d_out + f_out;

    if (pid->out_max != 0.0f) {
        constrain_f(&pid->out, -pid->out_max, pid->out_max);
    }

    pid->pre_now    = pid->now;
    pid->pre_target = pid->target;
    pid->pre_out    = pid->out;
    pid->pre_error  = error;
}

/* ========== 五次多项式轨迹 (quintic S-curve) ============================== */

/**
 * @brief 初始化轨迹规划器
 * @param max_vel  最大速度 (单位/s), 五次峰值 = 平均 * 1.875
 */
static void slope_init(Arm_Slope *s, float max_vel, float period_s)
{
    memset(s, 0, sizeof(*s));
    s->max_vel = max_vel;
    s->dt      = period_s;
}

/**
 * @brief 重置轨迹到指定值 (打断当前轨迹)
 */
static void slope_reset(Arm_Slope *s, float value)
{
    s->out         = value;
    s->target      = value;
    s->start_val   = value;
    s->prev_target = value;
    s->elapsed     = 0.0f;
    s->duration    = 0.0f;
    s->active      = false;
}

/**
 * @brief 每控制周期调用, 根据 target 变化自动生成五次多项式轨迹
 *
 * 五次多项式: s(t) = 10t³ - 15t⁴ + 6t⁵  (归一化 t ∈ [0,1])
 * 边界条件: s(0)=0, s'(0)=0, s''(0)=0, s(1)=1, s'(1)=0, s''(1)=0
 * 峰值速度 v_peak = 1.875 * D / T → T = 1.875 * |D| / v_max
 */
static void slope_update(Arm_Slope *s)
{
    /* 目标变化 → 启动新轨迹 */
    if (s->target != s->prev_target) {
        s->start_val   = s->out;
        s->prev_target = s->target;
        float D        = s->target - s->start_val;
        if (fabsf(D) < 1e-6f || s->max_vel <= 0.0f) {
            s->out    = s->target;
            s->active = false;
            return;
        }
        /* T = 1.875 * |D| / v_max, 保证峰值速度不超过 v_max */
        s->duration = 1.875f * fabsf(D) / s->max_vel;
        if (s->duration < s->dt) {
            s->duration = s->dt;
        }
        s->elapsed = 0.0f;
        s->active  = true;
    }

    if (!s->active) {
        return;
    }

    s->elapsed += s->dt;
    if (s->elapsed >= s->duration) {
        s->out    = s->target;
        s->active = false;
        return;
    }

    /* 归一化时间 t∈[0,1), 五次多项式 */
    float t  = s->elapsed / s->duration;
    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;
    float t5 = t4 * t;
    float D  = s->target - s->start_val;
    s->out   = s->start_val + D * (10.0f * t3 - 15.0f * t4 + 6.0f * t5);
}

/* ========== DM 4310 电机 ================================================= */

/** DM 命令帧: clear error */
static const uint8_t dm_cmd_clear_error[8] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb};
/** DM 命令帧: enable */
static const uint8_t dm_cmd_enable[8] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
/** DM 命令帧: disable */
static const uint8_t dm_cmd_disable[8] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};

/**
 * @brief 将 DM 电机 MIT 模式控制帧推入 CAN TX 队列
 */
static void dm_output(const Arm_Control *arm,
                      float ctrl_angle,
                      float ctrl_omega,
                      float ctrl_torque,
                      float kp, float kd)
{
    const Arm_DM_Motor *dm = &arm->dm;
    float wrapped_angle    = wrap_to_symmetric_range(ctrl_angle, dm->angle_max);

    /* 浮点 → 整数映射 */
    uint16_t tmp_angle  = (uint16_t) float_to_int(wrapped_angle, -dm->angle_max, dm->angle_max, 0, 65535);
    uint16_t tmp_omega  = (uint16_t) float_to_int(ctrl_omega, -dm->omega_max, dm->omega_max, 0, 4095);
    uint16_t tmp_torque = (uint16_t) float_to_int(ctrl_torque, -dm->torque_max, dm->torque_max, 0, 4095);
    uint16_t tmp_kp     = (uint16_t) float_to_int(kp, 0.0f, 500.0f, 0, 4095);
    uint16_t tmp_kd     = (uint16_t) float_to_int(kd, 0.0f, 5.0f, 0, 4095);

    CANFrame_t frame;
    frame.CANFrame.id  = dm->can_tx_id;
    frame.CANFrame.dlc = 8;

    /* 角度 16-bit: */
    frame.CANFrame.data[0] = (uint8_t) (tmp_angle >> 8);
    frame.CANFrame.data[1] = (uint8_t) (tmp_angle & 0xFF);
    frame.CANFrame.data[2] = (uint8_t) (tmp_omega >> 4);
    frame.CANFrame.data[3] = (uint8_t) (((tmp_omega & 0x0F) << 4) | (tmp_kp >> 8));
    frame.CANFrame.data[4] = (uint8_t) (tmp_kp & 0xFF);
    frame.CANFrame.data[5] = (uint8_t) (tmp_kd >> 4);
    frame.CANFrame.data[6] = (uint8_t) (((tmp_kd & 0x0F) << 4) | (tmp_torque >> 8));
    frame.CANFrame.data[7] = (uint8_t) (tmp_torque & 0xFF);

    osMessageQueuePut(arm->canTxQueue, &frame, 0, 0);
}

/**
 * @brief 发送 DM 8 字节命令帧 (enable/disable/clear_error)
 */
static void dm_send_cmd(const Arm_Control *arm, const uint8_t cmd[8])
{
    CANFrame_t frame;
    frame.CANFrame.id  = arm->dm.can_tx_id;
    frame.CANFrame.dlc = 8;
    memcpy(frame.CANFrame.data, cmd, 8);
    osMessageQueuePut(arm->canTxQueue, &frame, 0, 0);
}

/**
 * @brief 单帧解码: 识别 DM / C620 帧并保存最新 raw 值
 *
 * 每种电机只保留最后一帧的有效 payload,
 * 实际解码 (圈数累计、物理量换算) 在 Arm_Update 中统一完成.
 */

/* ── 单帧 raw 缓冲 (arm_feed_single_frame 写入, arm_decode_latest 消费) ── */
static bool dm_has_new     = false;
static bool c620_has_new   = false;
static uint16_t dm_raw_enc = 0, dm_raw_omega = 0, dm_raw_torque = 0;
static uint8_t dm_raw_status = 0, dm_raw_mos = 0, dm_raw_rotor = 0;
static uint16_t c620_raw_enc    = 0;
static int16_t c620_raw_omega_r = 0, c620_raw_curr_r = 0;
static uint8_t c620_raw_temp = 0;

static void arm_feed_single_frame(Arm_Control *arm, const CANFrame_t *frame)
{
    Arm_DM_Motor *dm     = &arm->dm;
    Arm_DJI_MotorC620 *m = &arm->c620;

    uint16_t id = frame->CANFrame.id;

    /* ── DM 4310 ── */
    if (id == dm->can_rx_id) {
        uint8_t can_id_in_frame = frame->CANFrame.data[0] & 0x0F;
        if (can_id_in_frame == (dm->can_tx_id & 0x0F)) {
            dm->flag   = 1; /* 标记有数据待解码 */
            dm_has_new = true;
            /* 仅保存原始 int 值, 不做 float 换算 */
            dm_raw_enc    = endian_reverse_16(&frame->CANFrame.data[1]);
            dm_raw_omega  = (uint16_t) ((frame->CANFrame.data[3] << 4) |
                                       (frame->CANFrame.data[4] >> 4));
            dm_raw_torque = (uint16_t) (((frame->CANFrame.data[4] & 0x0F) << 8) |
                                        frame->CANFrame.data[5]);
            dm_raw_status = (frame->CANFrame.data[0] >> 4) & 0x0F;
            dm_raw_mos    = frame->CANFrame.data[6];
            dm_raw_rotor  = frame->CANFrame.data[7];
        }
        return;
    }

    /* ── C620 ── */
    if (id == (uint16_t) (0x200 + m->can_rx_id)) {
        m->flag          = 1;
        c620_has_new     = true;
        c620_raw_enc     = endian_reverse_16(&frame->CANFrame.data[0]);
        c620_raw_omega_r = (int16_t) endian_reverse_16(&frame->CANFrame.data[2]);
        c620_raw_curr_r  = (int16_t) endian_reverse_16(&frame->CANFrame.data[4]);
        c620_raw_temp    = frame->CANFrame.data[6];
        return;
    }
}

void Arm_FeedRxFrame(Arm_Control *arm, const CANFrame_t *frame)
{
    arm_feed_single_frame(arm, frame);
}

/**
 * @brief 将上一周期缓存的 raw 值解码为物理量 (圈数累计、float 换算)
 */
static void arm_decode_latest(Arm_Control *arm)
{
    Arm_DM_Motor *dm     = &arm->dm;
    Arm_DJI_MotorC620 *m = &arm->c620;

    /* ── 解码 DM ── */
    if (dm_has_new) {
        dm->control_status = (ArmDM_ControlStatus) dm_raw_status;

        if (!dm->encoder_primed) {
            /* 首帧: 记录当前编码器为基准, 不计算圈数增量 */
            dm->pre_encoder    = dm_raw_enc;
            dm->total_encoder  = (int32_t) dm_raw_enc;
            dm->total_round    = 0;
            dm->encoder_primed = true;
        } else {
            int32_t delta = (int32_t) dm_raw_enc - (int32_t) dm->pre_encoder;
            /* DM MIT 全 16-bit 范围跨度 65536, 半程 32768 为翻转阈值 */
            if (delta < -(1 << 15)) {
                dm->total_round++;
            } else if (delta > (1 << 15)) {
                dm->total_round--;
            }
            dm->total_encoder = dm->total_round * 65536 +
                                (int32_t) dm_raw_enc;
            dm->pre_encoder = dm_raw_enc;
        }

        /* now_angle 使用 int_to_float, 与 dm_output 的 float_to_int 互逆 */
        dm->now_angle = int_to_float((int32_t) dm_raw_enc, 0, 65535,
                                     -dm->angle_max, dm->angle_max) +
                        (float) dm->total_round * 2.0f * dm->angle_max;
        dm->now_omega   = int_to_float((int32_t) dm_raw_omega, 0, 4095,
                                       -dm->omega_max, dm->omega_max);
        dm->now_torque  = int_to_float((int32_t) dm_raw_torque, 0, 4095,
                                       -dm->torque_max, dm->torque_max);
        dm->mos_temp    = (float) dm_raw_mos + CELSIUS_TO_KELVIN;
        dm->rotor_temp  = (float) dm_raw_rotor + CELSIUS_TO_KELVIN;
        dm->pre_encoder = dm_raw_enc;
        dm_has_new      = false;
    }

    /* ── 解码 C620 ── */
    if (c620_has_new) {
        if (m->angle_clear_flag == 1) {
            m->clear_encoder    = c620_raw_enc;
            m->angle_clear_flag = 0;
        }

        if (!m->encoder_primed) {
            /* 首帧: 记录当前编码器为基准, 不计算圈数增量 */
            m->pre_encoder    = c620_raw_enc;
            m->total_encoder  = (int32_t) c620_raw_enc;
            m->total_round    = 0;
            m->encoder_primed = true;
        } else {
            int16_t delta = (int16_t) (c620_raw_enc - m->pre_encoder);
            if (delta < -(int16_t) (C620_ENCODER_NUM_PER_ROUND / 2)) {
                m->total_round++;
            } else if (delta > (int16_t) (C620_ENCODER_NUM_PER_ROUND / 2)) {
                m->total_round--;
            }
            m->total_encoder = m->total_round * (int32_t) C620_ENCODER_NUM_PER_ROUND +
                               (int32_t) c620_raw_enc - (int32_t) m->clear_encoder;
            m->pre_encoder = c620_raw_enc;
        }

        m->now_angle = (float) m->total_encoder /
                       (float) C620_ENCODER_NUM_PER_ROUND *
                       2.0f * M_PI / m->gearbox_rate;
        m->now_omega   = (float) c620_raw_omega_r * RPM_TO_RADPS / m->gearbox_rate;
        m->now_current = (float) c620_raw_curr_r / C620_CURRENT_TO_OUT;
        m->temperature = (float) c620_raw_temp + CELSIUS_TO_KELVIN;
        m->pre_encoder = c620_raw_enc;
        c620_has_new   = false;
    }
}

/* ========== DJI C620 电机 ================================================ */

/**
 * @brief 根据电机 ID 分配 C620 在 TX 缓冲 (0x200 / 0x1FF) 中的偏移
 */
static uint8_t *c620_allocate_tx_data(ArmDJI_MotorID id,
                                      uint8_t *buf_200,
                                      uint8_t *buf_1ff)
{
    switch (id) {
        case ArmDJI_ID_0x201:
            return &buf_200[0];
        case ArmDJI_ID_0x202:
            return &buf_200[2];
        case ArmDJI_ID_0x203:
            return &buf_200[4];
        case ArmDJI_ID_0x204:
            return &buf_200[6];
        case ArmDJI_ID_0x205:
            return &buf_1ff[0];
        case ArmDJI_ID_0x206:
            return &buf_1ff[2];
        case ArmDJI_ID_0x207:
            return &buf_1ff[4];
        case ArmDJI_ID_0x208:
            return &buf_1ff[6];
        default:
            return &buf_200[0];
    }
}

/**
 * @brief 获取 C620 的 CAN TX ID
 */
static uint16_t c620_get_tx_id(ArmDJI_MotorID id)
{
    if (id >= ArmDJI_ID_0x201 && id <= ArmDJI_ID_0x204) {
        return 0x200;
    }
    return 0x1FF;
}

/**
 * @brief 将 int16 电流值写入 C620 TX 缓冲对应位置 (大端)
 */
static void c620_output(Arm_DJI_MotorC620 *m)
{
    int16_t val       = (int16_t) m->out;
    m->tx_data_ptr[0] = (uint8_t) (val >> 8);
    m->tx_data_ptr[1] = (uint8_t) (val & 0xFF);
}

/**
 * @brief C620 PID 计算 + 发送
 *
 * 根据 control_method 级联:
 *   ANGLE  : pos PID → omega PID → current
 *   OMEGA  : omega PID → current
 *   CURRENT: 直通 Target_Current
 */
static void c620_pid_calc_and_send(Arm_Control *arm,
                                   uint8_t *tx_buf_200,
                                   uint8_t *tx_buf_1ff)
{
    Arm_DJI_MotorC620 *m = &arm->c620;

    switch (m->control_method) {
        case ArmDJI_Ctrl_CURRENT:
            break;
        case ArmDJI_Ctrl_OMEGA:
            m->pid_omega.target = m->target_omega;
            m->pid_omega.now    = m->now_omega;
            pid_calc(&m->pid_omega);
            m->target_current = m->pid_omega.out;
            break;
        case ArmDJI_Ctrl_ANGLE:
            m->pid_angle.target = m->target_angle;
            m->pid_angle.now    = m->now_angle;
            pid_calc(&m->pid_angle);
            m->target_omega = m->pid_angle.out;

            m->pid_omega.target = m->target_omega;
            m->pid_omega.now    = m->now_omega;
            pid_calc(&m->pid_omega);
            m->target_current = m->pid_omega.out;
            break;
        default:
            m->target_current = 0.0f;
            break;
    }

    /* 电流限幅 → int16 映射 */
    float tmp = m->target_current;
    constrain_f(&tmp, -m->current_max, m->current_max);
    m->out = tmp * C620_CURRENT_TO_OUT;

    c620_output(m);

    /* 发送整组帧 (4 电机共用同一 CAN ID) */
    CANFrame_t frame;
    frame.CANFrame.id  = m->can_tx_id;
    frame.CANFrame.dlc = 8;

    if (m->can_tx_id == 0x200) {
        memcpy(frame.CANFrame.data, tx_buf_200, 8);
    } else {
        memcpy(frame.CANFrame.data, tx_buf_1ff, 8);
    }
    osMessageQueuePut(arm->canTxQueue, &frame, 0, 0);
}

/* ========== 辅助转换 ===================================================== */

static inline float raw_lift_angle_to_height_mm(const Arm_Control *arm,
                                                float raw_angle)
{
    return (raw_angle - arm->height_zero_angle) * arm->cfg.gear_radius_mm;
}

static inline float height_mm_to_raw_lift_angle(const Arm_Control *arm,
                                                float height_mm)
{
    return height_mm / arm->cfg.gear_radius_mm + arm->height_zero_angle;
}

/* ========== 归零状态机 =================================================== */

static void update_height_homing(Arm_Control *arm, float dt_s,
                                 uint8_t *tx_200, uint8_t *tx_1ff);

/* ========== DM 使能检查与重试 ============================================ */

static void check_and_retry_dm_enable(Arm_Control *arm, float dt_s)
{
    ArmDM_ControlStatus status = arm->dm.control_status;
    bool is_enabled            = (status == ArmDM_Status_ENABLE);

    if (is_enabled) {
        arm->dm_was_enabled        = true;
        arm->dm_enable_retry_timer = 0.0f;
        return;
    }

    arm->dm_was_enabled = false;
    arm->dm_enable_retry_timer -= dt_s;
    if (arm->dm_enable_retry_timer > 0.0f) {
        return;
    }

    /* 重试: clear error → enable */
    dm_send_cmd(arm, dm_cmd_clear_error);
    dm_send_cmd(arm, dm_cmd_enable);
    arm->dm_enable_retry_timer = 0.5f;
}

/* ========== 公共 API ===================================================== */

/**
 * @brief 初始化 Arm_Control 实例
 */
void Arm_Init(Arm_Control *arm, const Arm_Config *cfg)
{
    memset(arm, 0, sizeof(*arm));

    /* 配置 */
    if (cfg) {
        arm->cfg = *cfg;
    } else {
        arm->cfg = Arm_Config_Default;
    }
    const Arm_Config *c = &arm->cfg;

    /* ── DM 电机初始化 ── */
    arm->dm.can_rx_id   = c->dm_rx_id;
    arm->dm.can_tx_id   = c->dm_tx_id;
    arm->dm.angle_max   = c->dm_angle_max;
    arm->dm.omega_max   = c->dm_omega_max;
    arm->dm.torque_max  = c->dm_torque_max;
    arm->dm.current_max = c->dm_current_max;

    /* ── C620 电机初始化 ── */
    arm->c620.can_rx_id      = c->c620_rx_id;
    arm->c620.can_tx_id      = c620_get_tx_id(c->c620_rx_id);
    arm->c620.control_method = ArmDJI_Ctrl_ANGLE;
    arm->c620.current_max    = c->c620_current_max;
    arm->c620.gearbox_rate   = c->c620_gearbox;

    /* C620 PID */
    pid_init(&arm->c620.pid_angle,
             c->c620_angle_kp, c->c620_angle_ki,
             c->c620_angle_kd, c->c620_angle_kf,
             c->c620_angle_i_out_max, c->c620_angle_out_max,
             c->c620_pid_dt, c->c620_angle_dead_zone);
    pid_init(&arm->c620.pid_omega,
             c->c620_omega_kp, c->c620_omega_ki,
             c->c620_omega_kd, c->c620_omega_kf,
             c->c620_omega_i_out_max, c->c620_omega_out_max,
             c->c620_pid_dt, c->c620_omega_dead_zone);

    /* 分配 C620 TX 缓冲指针 (静态缓冲由 update 内提供) */
    arm->c620.tx_data_ptr = NULL; /* 延迟到首次 update 绑定 */

    /* ── 五次多项式轨迹 ── */
    slope_init(&arm->base_angle_slope, c->base_angle_max_vel, c->loop_period_s);
    slope_init(&arm->height_slope, c->height_max_vel, c->loop_period_s);

    /* inited 由 Arm_Homing 完成归零后置 true */
    arm->inited = false;
}

/**
 * @brief 执行归零: DM 使能+回零, C620 升降堵转归零
 *
 * 阻塞运行, 完成后 arm 处于零位 (DM angle=0, C620 height=0).
 * 调用前 must have canTxQueue/canRxQueue bound.
 */
void Arm_Homing(Arm_Control *arm)
{
    const float dt_s             = arm->cfg.loop_period_s;
    const TickType_t delay_ticks = (TickType_t) (dt_s * 1000.0f);

    /* C620 TX 缓冲 */
    static uint8_t c620_tx_buf_200[8] = {0};
    static uint8_t c620_tx_buf_1ff[8] = {0};

    /* 绑定 TX 指针 */
    arm->c620.tx_data_ptr = c620_allocate_tx_data(
        arm->c620.can_rx_id, c620_tx_buf_200, c620_tx_buf_1ff);

    /* ── 1. DM 使能 ── */
    // dm_send_cmd(arm, dm_cmd_clear_error);
    dm_send_cmd(arm, dm_cmd_enable);
    arm->dm_enable_retry_timer = 0.0f;
    arm->dm_was_enabled        = false;

    /* 等待 enable 反馈帧 */
    {
        static const int kMaxRetries = 50;
        int retry                    = 0;
        while (retry < kMaxRetries) {
            CANFrame_t rx;
            while (osMessageQueueGet(arm->canRxQueue, &rx, NULL, 0) == osOK) {
                Arm_FeedRxFrame(arm, &rx);
            }
            arm_decode_latest(arm);
            check_and_retry_dm_enable(arm, dt_s);

            if (arm->dm.control_status == ArmDM_Status_ENABLE) {
                break;
            }
            retry++;
            osDelay(delay_ticks > 0 ? delay_ticks : 1);
        }
    }

    /* ── 2. DM: 信任绝对值编码器, 以当前位置为起点, 不旋转归零 ── */
    slope_reset(&arm->base_angle_slope, arm->dm.now_angle);
    arm->base_angle_slope.target = arm->dm.now_angle;

    /* ── 3. C620 升降归零 (期间持续给 DM 发送 MIT 帧以保持使能) ── */
    arm->homing_state             = Arm_HeightHoming_IDLE;
    arm->homing_direction_decided = false;

    while (arm->homing_state != Arm_HeightHoming_COMPLETE) {
        CANFrame_t rx;
        while (osMessageQueueGet(arm->canRxQueue, &rx, NULL, 0) == osOK) {
            Arm_FeedRxFrame(arm, &rx);
        }
        arm_decode_latest(arm);
        check_and_retry_dm_enable(arm, dt_s);

        slope_update(&arm->base_angle_slope);

        dm_output(arm,
                  arm->base_angle_slope.out,
                  arm->cfg.dm_target_omega,
                  arm->cfg.dm_target_torque,
                  arm->cfg.dm_kp,
                  arm->cfg.dm_kd);

        update_height_homing(arm, dt_s, c620_tx_buf_200, c620_tx_buf_1ff);
        osDelay(delay_ticks);
    }

    /* ── 4. 就绪: 用当前实际位置 prime 斜坡, 确保 Arm_Update 首帧不乱动 ── */
    slope_reset(&arm->base_angle_slope, arm->dm.now_angle);
    arm->base_angle_slope.target = arm->dm.now_angle;

    arm->height_slope.target = 0.0f;
    arm->inited              = true;
}

/**
 * @brief 设置目标
 */
void Arm_SetTarget(Arm_Control *arm, float base_angle_rad, float height_mm)
{
    arm->base_angle_slope.target =
        expand_nearest_angle_rad(arm->base_angle_slope.out, base_angle_rad);
    arm->height_slope.target = height_mm;
}

/**
 * @brief 主控制循环
 *
 * 调用频率应与 arm->cfg.loop_period_s 一致.
 * C620 的 TX 缓冲 (static local) 在本函数内维护, 保证跨调用持久化.
 */
void Arm_Update(Arm_Control *arm)
{
    if (!arm->inited) {
        return;
    }

    /* static 持久化 C620 TX 缓冲: 必须在同一 CAN 总线上组合多个电机 */
    static uint8_t c620_tx_buf_200[8] = {0};
    static uint8_t c620_tx_buf_1ff[8] = {0};

    /* 首次绑定时 / Homing 后重新绑定 tx_data_ptr (两者 buffer 地址不同) */
    arm->c620.tx_data_ptr = c620_allocate_tx_data(
        arm->c620.can_rx_id, c620_tx_buf_200, c620_tx_buf_1ff);

    const float dt_s = arm->cfg.loop_period_s;

    /* ── 1. 解码上一周期缓存的电机反馈 (由外部 Arm_FeedRxFrame 喂入) ── */
    arm_decode_latest(arm);

    /* ── 2. DM 使能检查与重试 ── */
    check_and_retry_dm_enable(arm, dt_s);

    /* ── 3. 旋转五次多项式轨迹 ── */
    slope_update(&arm->base_angle_slope);

    /* ── 4. 升降控制 ── */
    slope_update(&arm->height_slope);

    float height_cmd_mm    = arm->height_slope.out;
    arm->c620.target_angle = height_mm_to_raw_lift_angle(arm, height_cmd_mm);

    /* ── 5. 发送电机指令 ── */
    dm_output(arm,
              arm->base_angle_slope.out,
              arm->cfg.dm_target_omega,
              arm->cfg.dm_target_torque,
              arm->cfg.dm_kp,
              arm->cfg.dm_kd);

    c620_pid_calc_and_send(arm, c620_tx_buf_200, c620_tx_buf_1ff);
}

/**
 * @brief 失能
 */
void Arm_Shutdown(Arm_Control *arm)
{
    if (!arm->inited) {
        return;
    }
    dm_send_cmd(arm, dm_cmd_disable);
    arm->inited = false;
}

/* ── 状态查询 ── */

float Arm_GetBaseAngle(const Arm_Control *arm)
{
    return arm->dm.now_angle;
}
float Arm_GetBaseOmega(const Arm_Control *arm)
{
    return arm->dm.now_omega;
}

float Arm_GetHeightMm(const Arm_Control *arm)
{
    return raw_lift_angle_to_height_mm(arm, arm->c620.now_angle);
}

float Arm_GetLiftAngle(const Arm_Control *arm)
{
    return arm->c620.now_angle - arm->height_zero_angle;
}

bool Arm_IsHeightHomed(const Arm_Control *arm)
{
    return arm->homing_state == Arm_HeightHoming_COMPLETE;
}

float Arm_GetLoopPeriodS(const Arm_Control *arm)
{
    return arm->cfg.loop_period_s;
}

/* ========== 微 ROS 接口占位 ============================================== */

void Arm_PutCommand(Arm_Control *arm, float base_angle_rad, float height_mm)
{
    Arm_SetTarget(arm, base_angle_rad, height_mm);
}

void Arm_GetState(const Arm_Control *arm,
                  float *base_angle_rad,
                  float *base_omega_radps,
                  float *height_mm,
                  bool *homed)
{
    if (base_angle_rad) {
        *base_angle_rad = Arm_GetBaseAngle(arm);
    }
    if (base_omega_radps) {
        *base_omega_radps = Arm_GetBaseOmega(arm);
    }
    if (height_mm) {
        *height_mm = Arm_GetHeightMm(arm);
    }
    if (homed) {
        *homed = Arm_IsHeightHomed(arm);
    }
}

void Arm_RequestShutdown(Arm_Control *arm, bool shutdown)
{
    if (shutdown) {
        Arm_Shutdown(arm);
    }
}

/* ========== 归零状态机实现 =============================================== */

static void update_height_homing(Arm_Control *arm, float dt_s,
                                 uint8_t *tx_200, uint8_t *tx_1ff)
{
    if (arm->homing_state == Arm_HeightHoming_COMPLETE) {
        return;
    }

    Arm_DJI_MotorC620 *m = &arm->c620;
    float raw_angle      = m->now_angle;

    /* ── IDLE: 首次进入, 判定归零方向 ── */
    if (arm->homing_state == Arm_HeightHoming_IDLE) {
        if (!arm->homing_direction_decided) {
            arm->homing_last_angle = raw_angle;

            /* 试探: 正向电流驱动一帧, 观察角度变化方向 */
            m->control_method = ArmDJI_Ctrl_CURRENT;
            m->target_current = arm->cfg.height_homing_current;
            c620_pid_calc_and_send(arm, tx_200, tx_1ff);

            arm->homing_direction_decided = true;
            return;
        }

        /* 第二帧: 观察角度变化方向 */
        float delta           = raw_angle - arm->homing_last_angle;
        const float kMinDelta = 0.001f;

        if (fabsf(delta) > kMinDelta) {
            /* delta > 0 说明正方向使角度增大; 归零向低位 → 取反 */
            arm->homing_direction = (delta > 0.0f) ? -1.0f : 1.0f;
        } else {
            arm->homing_direction = -1.0f;
        }

        arm->homing_state       = Arm_HeightHoming_MOVING_DOWN;
        arm->homing_last_angle  = raw_angle;
        arm->homing_stall_timer = 0.0f;
    }

    /* ── MOVING_DOWN: 持续向下运动 ── */
    if (arm->homing_state == Arm_HeightHoming_MOVING_DOWN) {
        m->control_method = ArmDJI_Ctrl_CURRENT;
        m->target_current = arm->cfg.height_homing_current *
                            arm->homing_direction;
        c620_pid_calc_and_send(arm, tx_200, tx_1ff);

        float delta = fabsf(raw_angle - arm->homing_last_angle);
        if (delta <= arm->cfg.height_homing_stall_threshold_rad) {
            arm->homing_state       = Arm_HeightHoming_STALL_DETECT;
            arm->homing_stall_timer = 0.0f;
        } else {
            arm->homing_last_angle = raw_angle;
        }
        return;
    }

    /* ── STALL_DETECT: 堵转确认 ── */
    if (arm->homing_state == Arm_HeightHoming_STALL_DETECT) {
        m->control_method = ArmDJI_Ctrl_CURRENT;
        m->target_current = arm->cfg.height_homing_current *
                            arm->homing_direction;
        c620_pid_calc_and_send(arm, tx_200, tx_1ff);

        float delta = fabsf(raw_angle - arm->homing_last_angle);
        if (delta <= arm->cfg.height_homing_stall_threshold_rad) {
            arm->homing_stall_timer += dt_s;
            if (arm->homing_stall_timer >= arm->cfg.height_homing_stall_time_s) {
                /* ── 归零完成 ── */
                arm->height_zero_angle = raw_angle;

                /* 切回 ANGLE 模式, 停在当前位置 */
                m->control_method = ArmDJI_Ctrl_ANGLE;
                m->target_current = 0.0f;
                m->target_angle   = raw_angle;

                slope_reset(&arm->height_slope, 0.0f);
                arm->homing_state = Arm_HeightHoming_COMPLETE;
            }
        } else {
            /* 又动了, 回到 MOVING_DOWN */
            arm->homing_state       = Arm_HeightHoming_MOVING_DOWN;
            arm->homing_last_angle  = raw_angle;
            arm->homing_stall_timer = 0.0f;
        }
    }
}
