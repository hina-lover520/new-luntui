/*********************************************************************************************************************
 * Name:blance.c
 *      轮腿平衡控制 (两轮自平衡)
 *
 *      控制结构 (串级五环):
 *        位置环   (最外环): 轮速积分位置 -> 位置误差 -> 目标前进速度(RPM) (定点保持)
 *        速度环   (外环):   目标速度误差(RPM) -> 目标前倾角(度)
 *        角度环   (中环):   四元数俯仰角 s_angle.agy -> 目标角速度
 *        角速度环 (内环):   卡尔曼滤波角速度 imu660rb_gyro_y_l -> 目标轮速(RPM)
 *        电机 PWM (最内环): 目标轮速(RPM) 直接 -> 占空比 (比例驱动, 无速度反馈)
 *
 *      数据来源 (在 cm7_1_isr.c 的 pit0_ch0_isr 中已更新):
 *        s_angle            -> my_quaternion() 四元数解算出的欧拉角 (单位: 度)
 *        imu660rb_gyro_y_l  -> kaermanb_Updata_1() 卡尔曼滤波后的陀螺仪 (单位: raw, 14.3 raw ≈ 1°/s)
 *
 *      速度反馈: 无刷驱动回传的 receive_left/right_speed_data (单位 RPM),
 *      由 uart4_isr -> small_driver_control_callback 解析, 需周期性 small_driver_get_speed() 请求刷新。
 *
 *      输出: 小驱动两路无刷电机 small_driver_set_duty, 占空比 -10000 ~ 10000。
 * Created on:
 * Author:
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <math.h>
#include <stdlib.h>
#include "pid.h"
#include "ztjx.h"
#include "quaternion.h"
#include "small_driver_uart_control.h"
#include "blance.h"

#define BALANCE_MAX_INTEGRAL      (100.0f)        // PID 积分限幅
#define BALANCE_DUTY_MAX_PERCENT  (85.0f)         // 最大占空比 85% (整定安全限幅); 起步推不动(静摩擦)就往大调, 车冲/翻就往小调
#define BALANCE_SPEED_REQ_PERIOD  (20)            // 每 20 次 balance_star (1ms×20=20ms) 请求一次速度回传 (差速环需要更快的反馈)
#define BALANCE_STARTUP_DELAY_MS  (6000)          // 上电延迟启动(ms): 先等四元数解算收敛, 期间轮子/舵机保持不动, 蜂鸣器最后一秒响

// ---- (已废弃) 角度环 P ramp: 之前起立时 700->稳态 400 自动切换; 现在角度环 P 直接在 balance_Init 里调, 运行时不再 ramp ----


// ---- 位置环 (最外环): 轮速积分得到位置, 位置误差 -> 目标前进速度(RPM) ----
#define BALANCE_POSITION_MAX_INTEGRAL (100.0f)   // 位置环积分限幅
#define BALANCE_POSITION_MAX_SPEED    (200.0f)   // 位置环输出限幅(RPM): 目标前进速度最大
// ---- 速度环: 目标速度误差(RPM) -> 目标前倾角(度) ----
#define BALANCE_SPEED_MAX_INTEGRAL    (50.0f)    // 速度环积分限幅
#define BALANCE_SPEED_MAX_ANGLE       (10.0f)    // 速度环输出限幅(度): 目标前倾角最大
#define BALANCE_RPM_TO_DUTY           (0.1f)     // 电机 PWM 内环: 目标轮速(RPM) -> 占空比(%) 固定比例 (≈原速度环 P)
#define BALANCE_FORWARD_BIAS_RPM      (0.0f)     // 基础前进转速(RPM): 直接前馈到轮速设定值, 只作起步辅助, 别给太大
// ---- 转向差速环 (内环, 1ms): 视觉给目标差速(RPM), 实测两轮差速反馈 -> 修正差速(RPM) ----
#define BALANCE_TURN_MAX_INTEGRAL     (80.0f)    // 转向差速环积分限幅
#define BALANCE_TURN_MAX_CORR         (100.0f)   // 转向差速环输出限幅(RPM): 修正量上限, 防止猛打

PIDController balance_angle;                    // 角度环 (外环)
PIDController balance_gyro;                     // 角速度环 (中环)
PIDController balance_wheel_l;                  // 左轮电机速度环 (内环)
PIDController balance_wheel_r;                  // 右轮电机速度环 (内环)
PIDController balance_position;                 // 位置环(最外环): 位置误差 -> 目标前进速度(RPM)
PIDController balance_speed;                    // 速度环(外环): 目标速度误差(RPM) -> 目标前倾角(度)
PIDController balance_turn;                     // 转向差速环(内环): 目标差速(RPM) -> 修正差速(RPM)

float balance_target_angle = 0.0f;              // 目标俯仰角 (度), 竖直站立为 0
float balance_mech_zero   = 4.5f;               // 机械零点 (度): 车体物理水平(平衡位)时 s_angle.agy 的读数 (2.63 往后漂 -> 往大调)
float balance_gyro_zero   = -8.47f;            // 角速度零点 (raw): 静止时 imu660rb_gyro_y_l 的读数 (回退到原值, 会漂移)

float balance_angle_out = 0.0f;                 // 角度环输出 (目标角速度)
float balance_gyro_out  = 0.0f;                 // 角速度环输出 (目标轮速 RPM)
int   balance_left_duty  = 0;                   // 左轮占空比
int   balance_right_duty = 0;                   // 右轮占空比

float balance_target_position   = 0.0f;         // 位置环目标(定点), 用户可设 (单位: 轮转数, 相对起点)
float balance_position_measured = 0.0f;         // 位置环反馈(轮速积分, 轮转数)
float balance_position_out      = 0.0f;         // 位置环输出(RPM), 目标前进速度
float balance_forward_speed     = 0.0f;         // 前向速度(RPM), 前进=正值 (位置环积分反馈 + 调试)
float balance_forward_target    = 0.0f;         // 目标前进速度(RPM), 仅显示用 (实际前进靠 balance_forward_tilt)
float balance_forward_tilt      = 0.0f;         // 前进前倾角(度): 循迹时给正小角度让车前进, 定点时 0
uint8  balance_line_follow      = 0;            // 1=循迹中(位置环旁路, 前倾角前进), 0=定点(位置环保持)
float balance_speed_out         = 0.0f;         // 速度环输出(度), 目标前倾角

float balance_steer_error = 0.0f;              // steer error (px) from core0 shared memory, debug
float balance_turn_out    = 0.0f;              // turn differential target wheel speed (RPM), 视觉外环设定(前馈)
float balance_turn_corr   = 0.0f;              // 转向差速环输出(RPM): 加到两轮的实际修正差速

uint8  balance_stand_up_done = 0;               // 0=起立中, 1=已起立(进入轮子+舵机平衡)
static uint32 balance_startup_cnt = 0;          // 上电延迟启动计数(1ms/次), balance_star 与 servo_balance_star 共用

//===================================================================================================================
// 舵机辅助平衡 (两轴: 俯仰 + 横滚)
//   原理: 车身向某侧倾斜时, 用四个腿舵机把重心往反方向顶, 辅助轮子一起把车身扶正。
//   与轮子平衡环并行, 在同一个 1ms 中断里 balance_star() 之后调用 servo_balance_star()。
//   PID 输出单位是"舵机角度偏移(度)", 叠加到每个舵机的中性角上。
//===================================================================================================================
#define SERVO_BALANCE_MAX_ANGLE    (30.0f)   // 每轴舵机偏移限幅(度), 防止舵机猛打/顶到限位
#define SERVO_ANGLE_LIMIT          (35.0f)   // 最终舵机角相对中性角的限幅(度): 中性±35
#define SERVO_ROLL_GAIN            (3.0f)    // 横滚放大倍数: 横滚舵机幅度不明显就往大调
#define SERVO_FILT_ALPHA           (0.05f)   // 舵机命令低通滤波(0~1): 越小越平滑(反应越慢); 还抖就往小调

static PIDController servo_pitch_pid;       // 俯仰辅助 (角度误差 -> 舵机偏移 度)
static PIDController servo_roll_pid;        // 横滚辅助 (角度误差 -> 舵机偏移 度)
static float servo_pitch_out_filt = 0.0f;   // 俯仰舵机命令低通滤波值(平滑)
static float servo_roll_out_filt  = 0.0f;   // 横滚舵机命令低通滤波值(平滑)

// ============ 四条腿舵机配置 (按实际接线 / 机械中位改这里) ============
// 每条腿: PIN=接的PWM口, NEUTRAL=初始/中性角(度), PITCH_DIR/ROLL_DIR=俯仰/横滚方向
//   - 改接线:               改 PIN
//   - 改初始角度:           改 NEUTRAL
//   - 方向反了(越顶越倒):   把该腿 PITCH_DIR / ROLL_DIR 取反
// 前左腿
#define SERVO_FL_PIN         SERVO_MOTOR_PWM1
#define SERVO_FL_NEUTRAL     (90.0f)
#define SERVO_FL_PITCH_DIR   (1.0f)
#define SERVO_FL_ROLL_DIR    (-1.0f)
// 前右腿
#define SERVO_FR_PIN         SERVO_MOTOR_PWM2
#define SERVO_FR_NEUTRAL     (90.0f)
#define SERVO_FR_PITCH_DIR   (-1.0f)
#define SERVO_FR_ROLL_DIR    (-1.0f)
// 后左腿
#define SERVO_RL_PIN         SERVO_MOTOR_PWM3
#define SERVO_RL_NEUTRAL     (90.0f)
#define SERVO_RL_PITCH_DIR   (1.0f)
#define SERVO_RL_ROLL_DIR    (1.0f)
// 后右腿
#define SERVO_RR_PIN         SERVO_MOTOR_PWM4
#define SERVO_RR_NEUTRAL     (90.0f)
#define SERVO_RR_PITCH_DIR   (-1.0f)
#define SERVO_RR_ROLL_DIR    (1.0f)

// ---- 站立阶段判定 ----
#define SERVO_STANDUP_HOLD_MS      (1500)   // 站立阶段时长(ms): 舵机保持中立不动, 站满这么久后才让舵机 PID 运行

float servo_roll_zero   = -0.99f;           // 横滚零点(度): 车身水平时 s_angle.agx 的读数
float servo_target_roll = 0.0f;             // 目标横滚角(度)
float servo_pitch_out   = 0.0f;             // 俯仰辅助输出(度), 调试
float servo_roll_out    = 0.0f;             // 横滚辅助输出(度), 调试
float servo_angle[4]    = {90.0f, 90.0f, 90.0f, 90.0f};   // 四个舵机最终角度(度), 调试

// 四个舵机全部打到中性角
static void servo_set_neutral_all(void)
{
    pwm_set_duty(SERVO_FL_PIN, (uint32)SERVO_MOTOR_DUTY(SERVO_FL_NEUTRAL));   // 前左
    pwm_set_duty(SERVO_FR_PIN, (uint32)SERVO_MOTOR_DUTY(SERVO_FR_NEUTRAL));   // 前右
    pwm_set_duty(SERVO_RL_PIN, (uint32)SERVO_MOTOR_DUTY(SERVO_RL_NEUTRAL));   // 后左
    pwm_set_duty(SERVO_RR_PIN, (uint32)SERVO_MOTOR_DUTY(SERVO_RR_NEUTRAL));   // 后右
}

// 站立计时(1ms/次), 在 servo_balance_Init 清零
static uint32 servo_standup_time_cnt = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      轮腿平衡控制初始化
// 参数说明     angle_p/i/d   角度环 PID 参数 (俯仰角 -> 目标角速度)
// 参数说明     gyro_p/i/d    角速度环 PID 参数 (目标角速度 -> 目标轮速 RPM)
// 参数说明     wheel_p/i/d   电机速度环 PID 参数 (目标轮速 RPM -> 占空比)
//-------------------------------------------------------------------------------------------------------------------
void balance_Init(float angle_p, float angle_i, float angle_d,
                  float gyro_p,  float gyro_i,  float gyro_d,
                  float wheel_p, float wheel_i, float wheel_d)
{
    PID_Init(&balance_angle,   angle_p, angle_i, angle_d);
    PID_Init(&balance_gyro,    gyro_p,  gyro_i,  gyro_d);
    PID_Init(&balance_wheel_l, wheel_p, wheel_i, wheel_d);
    PID_Init(&balance_wheel_r, wheel_p, wheel_i, wheel_d);
    PID_Init(&balance_position, 0.0f, 0.0f, 0.0f);   // 位置环默认关闭, 用 balance_position_Init 打开

    balance_angle_out = 0.0f;
    balance_gyro_out  = 0.0f;
    balance_left_duty  = 0;
    balance_right_duty = 0;

    balance_target_position   = 0.0f;
    balance_position_measured = 0.0f;
    balance_position_out      = 0.0f;
    balance_forward_speed     = 0.0f;
    balance_forward_tilt      = 0.0f;
    balance_line_follow       = 0;

    small_driver_set_duty(&small_driver_value, 0, 0);           // 电机输出清零
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      位置环(定点)初始化, P>0 才生效; P 越大位置保持越硬(太大易抖), I 消除稳态漂移
//-------------------------------------------------------------------------------------------------------------------
void balance_position_Init(float p, float i, float d)
{
    PID_Init(&balance_position, p, i, d);
    balance_target_position   = 0.0f;
    balance_position_measured = 0.0f;
    balance_position_out      = 0.0f;
    balance_forward_speed     = 0.0f;
    balance_forward_tilt      = 0.0f;
    balance_line_follow       = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      前进速度环初始化 (目标前进速度 RPM -> 目标倾斜角 度)
// 参数说明     p  比例: 速度误差立即映射成前倾角, 越大响应越快(太大易抖)
// 参数说明     i  积分: 消除速度静差, 让实际速度最终等于目标速度(速度不准就慢慢加)
//-------------------------------------------------------------------------------------------------------------------
void balance_speed_Init(float p, float i, float d)
{
    PID_Init(&balance_speed, p, i, d);
    balance_speed_out = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      转向差速环初始化 (内环: 目标差速 RPM -> 修正差速 RPM)
// 参数说明     p  比例: 实测差速与目标差速的误差 -> 修正量; 转过头就调大(更早收住), 拐不动就调小
// 参数说明     i  积分: 消除两轮差速静差(保证实际差速最终等于目标差速)
// 参数说明     d  微分: 阻尼差速变化; 先给 0, 还转过头再慢慢加
//-------------------------------------------------------------------------------------------------------------------
void balance_turn_Init(float p, float i, float d)
{
    PID_Init(&balance_turn, p, i, d);
    balance_turn_corr = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      轮腿平衡控制主函数, 在 PIT 中断中周期调用 (1ms)
//-------------------------------------------------------------------------------------------------------------------
void balance_star(void)
{
    static uint32 wheel_on_cnt = 0;   // 轮子启动后计时(ms), 驱动 P ramp 时间线; 舵机站起前清零

    // ---- 上电延迟启动: 先等四元数解算收敛, 期间轮子保持 0, 蜂鸣器最后一秒响 ----
    if(balance_startup_cnt < BALANCE_STARTUP_DELAY_MS)
    {
        balance_startup_cnt++;

        // 蜂鸣器: 只在延迟最后一秒响, 其余静音; 最后一拍(==DELAY)自动关掉
        if(balance_startup_cnt >= (BALANCE_STARTUP_DELAY_MS - 1000) && balance_startup_cnt < BALANCE_STARTUP_DELAY_MS)
            gpio_set_level(BUZZER_PIN, GPIO_HIGH);
        else
            gpio_set_level(BUZZER_PIN, GPIO_LOW);

        balance_left_duty  = 0;
        balance_right_duty = 0;
        small_driver_set_duty(&small_driver_value, 0, 0);    //地址  0  0
        return;
    }

    // ---- 起立阶段: 舵机负责把车身顶起来, 轮子保持 0, 等舵机站好之后再动 ----
    if(!balance_stand_up_done)
    {
        wheel_on_cnt = 0;               // 站起前清零轮子启动计时
        balance_left_duty  = 0;
        balance_right_duty = 0;
        small_driver_set_duty(&small_driver_value, 0, 0);
        return;
    }

    wheel_on_cnt++;                     // 站起后轮子开始计时

    // ============ 多速率串级: balance_star 每 1ms 调用一次 ============
    // 外环慢、内环快: 位置环+角度环 5ms -> 角速度环 2ms -> 电机速度环 1ms
    // 外环输出保持(零阶保持)到下一个周期, 内环直接用上一次保持的值。
    static uint8 angle_cnt = 0;    // 位置环/角度环分频计数 (5ms)
    static uint8 gyro_cnt  = 0;    // 角速度环分频计数 (2ms)

    // ---- 位置环 + 角度环 (最外环, 每 5ms) ----
    if(++angle_cnt >= 5)
    {
        angle_cnt = 0;

        // 实测前向速度(RPM), 前进=正值: 位置环积分反馈 + 屏幕显示
        // 前进=左轮正/右轮负(与 PWM 段"左 +/右 - 前进"一致), 用 (左-右)/2 使前进为正值
        
        balance_forward_speed = (small_driver_value.receive_left_speed_data - small_driver_value.receive_right_speed_data) * 0.5f;  // 前向速度(RPM)

        // ---- 位置环 (最外环): 轮速积分得到位置, 位置误差 -> 目标前进速度(RPM) ----
        balance_position_measured += balance_forward_speed * (0.005f / 60.0f);   // RPM -> 轮转数 (每 5ms 累加)

        if(balance_line_follow)
        {
            // 固定速度模式: 位置环旁路, 速度环直接以 balance_forward_target 为设定值 -> 前倾角
            balance_position_out        = balance_forward_target;   // 速度环目标速度(RPM)
            balance_position.integral   = 0.0f;
            balance_position.last_error = 0.0f;

            // 速度环: 速度误差(RPM) -> 目标前倾角(度)
            balance_speed_out = PID_Update(&balance_speed, balance_position_out, balance_forward_speed, BALANCE_SPEED_MAX_INTEGRAL);
            balance_speed_out = func_limit_ab(balance_speed_out, -BALANCE_SPEED_MAX_ANGLE, BALANCE_SPEED_MAX_ANGLE);
        }
        else
        {
            // 定点: 位置误差 -> 目标前进速度(RPM)
            balance_position_out = PID_Update(&balance_position, balance_target_position, balance_position_measured, BALANCE_POSITION_MAX_INTEGRAL);
            balance_position_out = func_limit_ab(balance_position_out, -BALANCE_POSITION_MAX_SPEED, BALANCE_POSITION_MAX_SPEED);

            // 速度环: 速度误差(RPM) -> 目标前倾角(度), 叠加到角度环目标
            balance_speed_out = PID_Update(&balance_speed, balance_position_out, balance_forward_speed, BALANCE_SPEED_MAX_INTEGRAL);
            balance_speed_out = func_limit_ab(balance_speed_out, -BALANCE_SPEED_MAX_ANGLE, BALANCE_SPEED_MAX_ANGLE);
        }

        // 角度环: 俯仰角 -> 目标角速度 (目标=机械零点 + 速度环前倾角)
        balance_angle_out = PID_Update(&balance_angle,
                                       balance_target_angle + balance_mech_zero + balance_speed_out,
                                       s_angle.agy,
                                       BALANCE_MAX_INTEGRAL);
    }

    // ---- 角速度环 (中环, 每 2ms) ----
    if(++gyro_cnt >= 2)
    {
        gyro_cnt = 0;
        balance_gyro_out = PID_Update(&balance_gyro,
                                      balance_angle_out,
                                      imu660rb_gyro_y_l - balance_gyro_zero,
                                      BALANCE_MAX_INTEGRAL);
    }

    // ---- 转向差速环 (内环, 每 1ms): 视觉给目标差速(balance_turn_out), 实测两轮差速闭环 -> 修正差速 ----
    // 实测差速 = (左轮速 + 右轮速)/2, 与 balance_turn_out 同号(前进=左+/右-, 转向加同号)。
    // 前馈 balance_turn_out 直接进 PWM(反应快), PID 修正量 balance_turn_corr 消除实际差速与目标的误差(转过头就靠它收)。
    float balance_turn_meas = (small_driver_value.receive_left_speed_data + small_driver_value.receive_right_speed_data) * 0.5f;
    balance_turn_corr = PID_Update(&balance_turn, balance_turn_out, balance_turn_meas, BALANCE_TURN_MAX_INTEGRAL);
    balance_turn_corr = func_limit_ab(balance_turn_corr, -BALANCE_TURN_MAX_CORR, BALANCE_TURN_MAX_CORR);

    // ---- 电机 PWM (内环, 每 1ms): 角速度环输出(目标轮速 RPM) 直接 -> 占空比 ----
    // 左右电机镜像安装(左 +/右 - 前进)。
    //   balance_turn_out        前馈转向差速(方向反了就把循迹 KP 取反)
    //   balance_turn_corr       差速环修正(消除实际差速与目标的误差)
    //   BALANCE_FORWARD_BIAS_RPM 反符号偏移 -> 基础前进转速(左 +/右 -)
    // 原电机速度环(带反馈 PID)去掉, 直接按 RPM->占空比 比例驱动, 避免速度反馈 50ms 延迟/噪声
    float left_pct  = (+balance_gyro_out + balance_turn_out + balance_turn_corr + BALANCE_FORWARD_BIAS_RPM) * BALANCE_RPM_TO_DUTY;
    float right_pct = (-balance_gyro_out + balance_turn_out + balance_turn_corr - BALANCE_FORWARD_BIAS_RPM) * BALANCE_RPM_TO_DUTY;

    // 占空比转换 (每 1ms): 限幅 ±85% 再 ×(PWM_DUTY_MAX/100)
    left_pct  = func_limit_ab(left_pct,  -BALANCE_DUTY_MAX_PERCENT, BALANCE_DUTY_MAX_PERCENT) * (PWM_DUTY_MAX / 100.0f);
    right_pct = func_limit_ab(right_pct, -BALANCE_DUTY_MAX_PERCENT, BALANCE_DUTY_MAX_PERCENT) * (PWM_DUTY_MAX / 100.0f);

    balance_left_duty  = (int)left_pct;
    balance_right_duty = (int)right_pct;

    // 轮子实际方向与占空比相反(实测 速度≈-占空比), 这里整体取反修正方向
    small_driver_set_duty(&small_driver_value, -balance_left_duty, -balance_right_duty);

    // ---- 周期性请求速度回传 (驱动板需收到请求才回传速度, 参考例程) ----
    static uint8 speed_req_cnt = 0;
    if(++speed_req_cnt >= BALANCE_SPEED_REQ_PERIOD)
    {
        speed_req_cnt = 0;
        small_driver_get_speed(&small_driver_value);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      舵机辅助平衡初始化
// 参数说明     pitch_p/d   俯仰辅助 PD (角度误差(度) -> 舵机偏移(度))
// 参数说明     roll_p/d    横滚辅助 PD
//-------------------------------------------------------------------------------------------------------------------
void servo_balance_Init(float pitch_p, float pitch_d, float roll_p, float roll_d)
{
    PID_Init(&servo_pitch_pid, pitch_p, 0.0f, pitch_d);
    PID_Init(&servo_roll_pid,  roll_p,  0.0f, roll_d);
    servo_pitch_out = 0.0f;
    servo_roll_out  = 0.0f;
    balance_stand_up_done  = 0;          // 上电先进入"舵机起立"阶段
    servo_standup_time_cnt = 0;

    pwm_init(SERVO_FL_PIN, SERVO_MOTOR_FREQ, 0);                // 前左 50Hz 舵机 PWM
    pwm_init(SERVO_FR_PIN, SERVO_MOTOR_FREQ, 0);                // 前右
    pwm_init(SERVO_RL_PIN, SERVO_MOTOR_FREQ, 0);                // 后左
    pwm_init(SERVO_RR_PIN, SERVO_MOTOR_FREQ, 0);                // 后右
    servo_angle[0] = SERVO_FL_NEUTRAL;
    servo_angle[1] = SERVO_FR_NEUTRAL;
    servo_angle[2] = SERVO_RL_NEUTRAL;
    servo_angle[3] = SERVO_RR_NEUTRAL;
    servo_set_neutral_all();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      舵机辅助平衡主函数, 在 balance_star() 之后、同一 1ms 中断里调用
//-------------------------------------------------------------------------------------------------------------------
void servo_balance_star(void)
{
    // ---- 上电延迟启动: 与 balance_star 共用同一计数器, 期间舵机保持中立不动 ----
    if(balance_startup_cnt < BALANCE_STARTUP_DELAY_MS)
    {
        servo_set_neutral_all();
        return;
    }

    // ---- 站立阶段: 舵机保持中性别动, 站满时长后才让舵机 PID 运行 ----
    if(!balance_stand_up_done)
    {
        servo_set_neutral_all();
        servo_standup_time_cnt++;

        // 站立动作完整完成(车身稳定)后, 才切到平衡 PID
        if(servo_standup_time_cnt >= SERVO_STANDUP_HOLD_MS)
            balance_stand_up_done = 1;
        return;
    }

    // 俯仰参考 = 目标竖直角 + 机械零点 + 速度环前倾角; 与轮子角度环目标一致, 舵机辅助顶前倾(辅助定速), 不互相抵消
    float pitch_ref = balance_target_angle + balance_mech_zero + balance_speed_out;
    float roll_ref  = servo_target_roll + servo_roll_zero;

    // 两轴 PD: 角度误差 -> 舵机偏移(度)
    servo_pitch_out = PID_Update(&servo_pitch_pid, pitch_ref, s_angle.agy, SERVO_BALANCE_MAX_ANGLE);
    servo_roll_out  = PID_Update(&servo_roll_pid,  roll_ref,  s_angle.agx, SERVO_BALANCE_MAX_ANGLE);
    servo_pitch_out = func_limit_ab(servo_pitch_out, -SERVO_BALANCE_MAX_ANGLE, SERVO_BALANCE_MAX_ANGLE);
    servo_roll_out  = func_limit_ab(servo_roll_out * SERVO_ROLL_GAIN, -SERVO_BALANCE_MAX_ANGLE, SERVO_BALANCE_MAX_ANGLE);

    // 输出低通滤波: 舵机机械响应慢(~20ms 一个 PWM 周期), 1ms 更新会追高频噪声而抖; 平滑命令 = 减慢反应
    servo_pitch_out_filt += SERVO_FILT_ALPHA * (servo_pitch_out - servo_pitch_out_filt);
    servo_roll_out_filt  += SERVO_FILT_ALPHA * (servo_roll_out  - servo_roll_out_filt);
    servo_pitch_out = servo_pitch_out_filt;
    servo_roll_out  = servo_roll_out_filt;

    // 前左腿
    servo_angle[0] = func_limit_ab(SERVO_FL_NEUTRAL + SERVO_FL_PITCH_DIR * servo_pitch_out + SERVO_FL_ROLL_DIR * servo_roll_out,     
                                   SERVO_FL_NEUTRAL - SERVO_ANGLE_LIMIT, SERVO_FL_NEUTRAL + SERVO_ANGLE_LIMIT);                   //限幅
    servo_angle[0] = func_limit_ab(servo_angle[0], (float)SERVO_MOTOR_L_MAX, (float)SERVO_MOTOR_R_MAX);   // 再卡物理范围 10~170
    pwm_set_duty(SERVO_FL_PIN, (uint32)SERVO_MOTOR_DUTY(servo_angle[0]));

    // 前右腿
    servo_angle[1] = func_limit_ab(SERVO_FR_NEUTRAL + SERVO_FR_PITCH_DIR * servo_pitch_out + SERVO_FR_ROLL_DIR * servo_roll_out,
                                   SERVO_FR_NEUTRAL - SERVO_ANGLE_LIMIT, SERVO_FR_NEUTRAL + SERVO_ANGLE_LIMIT);
    servo_angle[1] = func_limit_ab(servo_angle[1], (float)SERVO_MOTOR_L_MAX, (float)SERVO_MOTOR_R_MAX);   // 再卡物理范围 10~170
    pwm_set_duty(SERVO_FR_PIN, (uint32)SERVO_MOTOR_DUTY(servo_angle[1]));

    // 后左腿
    servo_angle[2] = func_limit_ab(SERVO_RL_NEUTRAL + SERVO_RL_PITCH_DIR * servo_pitch_out + SERVO_RL_ROLL_DIR * servo_roll_out,
                                   SERVO_RL_NEUTRAL - SERVO_ANGLE_LIMIT, SERVO_RL_NEUTRAL + SERVO_ANGLE_LIMIT);
    servo_angle[2] = func_limit_ab(servo_angle[2], (float)SERVO_MOTOR_L_MAX, (float)SERVO_MOTOR_R_MAX);   // 再卡物理范围 10~170
    pwm_set_duty(SERVO_RL_PIN, (uint32)SERVO_MOTOR_DUTY(servo_angle[2]));

    // 后右腿
    servo_angle[3] = func_limit_ab(SERVO_RR_NEUTRAL + SERVO_RR_PITCH_DIR * servo_pitch_out + SERVO_RR_ROLL_DIR * servo_roll_out,
                                   SERVO_RR_NEUTRAL - SERVO_ANGLE_LIMIT, SERVO_RR_NEUTRAL + SERVO_ANGLE_LIMIT);
    servo_angle[3] = func_limit_ab(servo_angle[3], (float)SERVO_MOTOR_L_MAX, (float)SERVO_MOTOR_R_MAX);   // 再卡物理范围 10~170
    pwm_set_duty(SERVO_RR_PIN, (uint32)SERVO_MOTOR_DUTY(servo_angle[3]));
}
