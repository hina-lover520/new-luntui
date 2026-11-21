/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          main_cm7_0
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "find.h"
#include "globals.h"
#include "image.h"
#include "car_control.h"
#include "kaerman.h"
#include "quaternion.h"
#include "ztjx.h"
#include "my_encoder.h"
#include "blance.h"
#include "small_driver_uart_control.h"
#include <math.h>                       // fabsf 用于循迹转向非线性项

/* RING_ADDON_BEGIN */
#include "ring_runtime.h"
#include "ring_uart_diag.h"
/* RING_ADDON_END */
/* ================= dual-core shared memory (copied from CYT4bb7Libraryhere0) =================
 * CM7_0 = producer (vision), CM7_1 = consumer (balance / line-follow control).
 * __no_init stops CM7_1 startup from zeroing this region (it is owned/written by CM7_0).
 * Protocol: [0]=steer error(px)  [1]=track state(0 lost/1 normal/2 cross)  <- CM7_0 writes
 *           [2..14]=screen debug data <- CM7_1 writes:
 *             [2]fwd spd*10 [3]speed_out*10(前倾角) [4]position_out*10(RPM) [5]L duty [6]R duty
 *             [7]L wheel RPM [8]R wheel RPM [9]stand_up_done [10]turn*10 [11..14]servo deg*10
 * ============================================================================================= */
#pragma location = 0x28001000
__no_init int m7_1_data_1[16];

// 循迹转向 (饱和 P + 微分, 无陀螺): 误差低通 -> 饱和P + 微分 -> 输出低通
#define TURN_KP1          (1.35f)    /* P 增益(原始误差): 反应慢就调大, 左右晃就调小 */
#define TURN_SAT          (40.0f)   /* 饱和尺度(px): 大误差封顶(KP1*SAT), 大弯不冲过头 */
#define TURN_KD           (1.5f)    /* 微分阻尼(基于滤波误差): 左右晃就调大, 急弯甩出去就调小 */
#define TURN_ERR_ALPHA    (0.6f)    /* 微分用误差低通(0~1): 只平滑微分, 不给 P 加滞后 */
#define TURN_SMOOTH_ALPHA (0.4f)    /* 输出低通(0~1): 越小越丝滑但反应越慢 */
#define TURN_MAX_RPM      (180.0f)  /* clamp the differential target wheel speed */
#define LINE_FOLLOW_FWD_SPEED (60.0f)  /* 循迹目标前进速度(RPM), 仅显示用 */   50   65
#define LINE_FOLLOW_FWD_TILT  (4.0f)   /* 循迹前进前倾角(度), 缓慢前行; 快/慢调这里 (方向反了取负); 3°推不动静摩擦 -> 6° */
#define LINE_FOLLOW_ENABLE    (1)      /* 1=开循迹转向(速度环固定速度前进+循线转向), 0=关转向(只直行) */
#define FIXED_SPEED_ENABLE    (1)      /* 1=速度环固定目标速度前进(暂不循迹不转向), 0=只定点平衡 */
#define FWD_SPEED_SMALL       (195.0f) //140 /* 小幅度拐弯/直行: 目标速度上限(RPM) */100   150  165稳   
#define FWD_SPEED_LARGE       (155.0f) //125 /* 大幅度拐弯: 目标速度下限(RPM), 再急也刹到这 */  85    125  125（配165）
#define FWD_TURN_SLOW_GAIN    (0.40f)   /* 减速斜率: 减速度(RPM) = 增益 × |转向差速|; 急弯甩出去就调大(更早减速) */

// 舵机引脚 / 频率 / 角度映射宏 已移到 blance.h (舵机辅助平衡共用)
/* OPERATOR_UI_BEGIN: only CM7_1 owns P20_2/P20_3. */
volatile uint8 balance_run_enabled = 0;
volatile uint8 balance_ring_display = 0;
/* OPERATOR_UI_END */
static int len =0;
static float steer_err_filt   = 0.0f;   // 误差低通后的值(平滑 + 微分用)
static float steer_last_err   = 0.0f;   // 上一次滤波误差(微分用)
static float turn_out_filt    = 0.0f;   // 转向输出低通滤波后的值(拐弯丝滑)

// ******************************* �������� *******************************
/*#define UART_INDEX              (DEBUG_UART_INDEX   )                           // Ĭ�� UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // Ĭ�� 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // Ĭ�� UART0_TX_P00_1
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // Ĭ�� UART0_RX_P00_0
*/
uint8 uart_get_data[64];                                                        // ���ڽ������ݻ�����
uint8 fifo_get_data[64];                                                        // fifo �������������

uint8  get_data = 0;                                                            // �������ݱ���
uint32 fifo_data_count = 0;                                                     // fifo ���ݸ���

uint8 by_fly[7];
fifo_struct uart_data_fifo;

extern volatile uint32 uart4_rx_isr_count;   // cm7_1_isr.c 中的 UART4 接收中断计数



int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_init();                   // ���Դ�����Ϣ��ʼ��
    uart_init(UART_1, 115200, UART1_TX_P04_1, UART1_RX_P04_0);    
    uart_rx_interrupt(UART_1, 1);  
    kaerman_b_Init(&gyro_x,2.0f,2.0f,80.0f,80.0f,0.002f,0.002f);
    kaerman_b_Init(&gyro_y,1.0f,1.0f,80.0f,80.0f,0.002f,0.002f);
    kaerman_b_Init(&gyro_z,1.0f,1.0f,80.0f,80.0f,0.002f,0.002f);
    // �˴���д�û����� ���������ʼ�������
    quaternion_Init(&sc,&s_angle,&s_shu,1.2,0.001,0.002f);//pi,�Ƕȣ���Ԫ����kp,kd,�ж�ʱ��,2.0,0.001,0.05f
    imu660rb_init();
    
    
    
    //imu660rc_init(IMU660RC_QUARTERNION_120HZ);

    small_driver_uart_init();           // 小驱动两路无刷串口初始化 (UART4, 与飞控一致放在 CM7_1)

    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);   // 蜂鸣器 (上电延迟启动提示音)

    servo_balance_Init(0.5f, 0.08f, 0.0f, 0.0f);   // 舵机俯仰辅助启用(P=1.0 D=0.1): 顶前倾辅助定速; 横滚仍禁用  //后为横滚
    //servo_balance_Init(0.0f, 0.00f, 1.0f, 0.1f);
    // 轮腿平衡控制初始化 (角度环 + 角速度环 + 电机速度环)
    // 参数: 角度环 P I D / 角速度环 P I D / 电机速度环 P I D, 待整定
    balance_Init(50.0f,  2.0f,  1.5f,   // 角度环 5ms (度 -> 目标角速度), 前后剧烈抖(超调) -> P 700 降到 500
                 -0.12f, 0.0f, -0.1f,   // 角速度环 2ms (-> 目标轮速 RPM), 抖 -> P -0.15 回到 -0.12
                 0.1f, 0.01f, 0.0f);  // 电机速度环 1ms (RPM -> 占空比), 已废弃(直连 PWM), 参数无影响

    // 位置环(最外环): 位置误差(轮转数) -> 目标前进速度(RPM); 先关(速度环调好再开)
    balance_position_Init(0.0f, 0.0f, 0.0f);
    balance_target_position = 0.0f;     // 0=原地定点保持; 改正值让车往前走到该位置, 负值往后

    // 速度环: 速度误差(RPM) -> 目标前倾角(度); P=+0.01 (0.06 太硬→原地打转/振荡, 降到 0.01)
    balance_speed_Init(0.15f, 0.0f, 0.08f);

    // 转向差速环(内环): 目标差速(RPM) -> 修正差速(RPM); P I D 待整定
    balance_turn_Init(0.5f, 0.05f, 0.0f);

    balance_forward_target = 0.0f;    // 目标前进速度(RPM): 0=原地保持; 循迹时循环里改成 LINE_FOLLOW_FWD_SPEED

    balance_stand_up_done = 0;          // 启用起立: 延迟结束后先起身(轮子+舵机), 站直后再进平衡

    //888888888888888
    //car_Init(0.0,0.0,0.0,0.0,0.0,0.0,  0.0,0.0,0.0);
    //car_Init(0.0,0.0,0.0,  0.0,0.0,0.0,  4.5,0.0,5.0);   // 旧四轮有刷电机, 轮腿不用
    //car_Init(2.5,0.0,0.25,  2.5,0.0,0.25, 5.5,0.0,0.35);
    
    //my_encoder_Init(1.0,0.0,0.8, 1.0,0.0,0.8, 0.6,0.0,0.8, 1.2,0.0,0.8);//1.0,0.0,0.5
    
    //my_encoder_Init(1.2,0.0,0.6, 1.2,0.0,0.5, 0.8,0.0,0.5, 0.8,0.0,0.6);//1.0,0.0,0.5   // 旧编码器, 轮腿不用
    
    //88888888888888888
    //encoder_quad_init(ENCODER_QUADDEC1, ENCODER_QUADDEC_A1, ENCODER_QUADDEC_B1);// ��ʼ��������ģ�������� ������������ģʽ
    //encoder_quad_init(ENCODER_QUADDEC2, ENCODER_QUADDEC_A2, ENCODER_QUADDEC_B2);// ��ʼ��������ģ�������� ������������ģʽ
    //encoder_quad_init(ENCODER_QUADDEC3, ENCODER_QUADDEC_A3, ENCODER_QUADDEC_B3);// ��ʼ��������ģ�������� ������������ģʽ
    //encoder_quad_init(ENCODER_QUADDEC4, ENCODER_QUADDEC_A4, ENCODER_QUADDEC_B4);// ��ʼ��������ģ�������� ������������ģʽ
    
    /* RING_ADDON_BEGIN */
    ring_runtime_init(imu660rb_gyro_transition(1));
    /* OPERATOR_UI_BEGIN */
    gpio_init(P20_2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(P20_3, GPI, GPIO_HIGH, GPI_PULL_UP);
    /* OPERATOR_UI_END */
    /* RING_ADDON_END */
    pit_ms_init(PIT_CH0,1);
    pit_enable(PIT_CH0);
    pit_ms_init(PIT_CH1,1);
    pit_enable(PIT_CH1);
    pit_ms_init(PIT_CH2,5);
    pit_enable(PIT_CH2);
    pit_ms_init(PIT_CH10,2);
    pit_enable(PIT_CH10);
    
    // �˴���д�û����� ���������ʼ�������
    while(true)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���

       // fifo_data_count = fifo_used(&uart_data_fifo);                             // �鿴 fifo �Ƿ�������
       // if(fifo_data_count != 0)                                                // ��ȡ��������
       // {
        //   fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // �� fifo �����ݶ�������� fifo ���صĻ���
       //     uart_write_string(UART_2, "\r\nUART get data:");                // ���������Ϣ
            //uart_write_buffer(UART_2, fifo_get_data, fifo_data_count);      // ����ȡ�������ݷ��ͳ�ȥ
            //if(by_fly[0]==0xAA&&by_fly[4]==0x55)
            //{
        /* ---- line-follow: read steer error written by CM7_0 (flush D-cache first) ---- */
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data_1, sizeof(m7_1_data_1));
        balance_steer_error = (float)m7_1_data_1[0];
        if (balance_run_enabled && FIXED_SPEED_ENABLE)                        /* 固定速度模式: 速度环目标=固定速度前进, 叠加循迹转向 */
        {
            balance_line_follow    = 1;
            balance_forward_tilt   = 0.0f;

            if (LINE_FOLLOW_ENABLE && (uint8_t)m7_1_data_1[1] >= 1)  /* 循迹转向: 检测到线才转, 丢线不转 */
            {
                float e = balance_steer_error;

                /* P 用原始误差: 无滞后, 反应快(修 反应慢/急弯慢) */
                float turn_p = TURN_KP1 * e / (1.0f + fabsf(e) / TURN_SAT);

                /* D 用轻滤波误差: 只平滑微分(整数像素量化), 不给 P 加滞后 */
                steer_err_filt += TURN_ERR_ALPHA * (e - steer_err_filt);
                float turn_d = TURN_KD * (steer_err_filt - steer_last_err);
                steer_last_err = steer_err_filt;

                float turn_des = turn_p + turn_d;
                turn_out_filt += TURN_SMOOTH_ALPHA * (turn_des - turn_out_filt);  // 输出低通 -> 拐弯丝滑
                balance_turn_out = turn_out_filt;
            }
            else
            {
                turn_out_filt   = 0.0f;
                steer_err_filt  = 0.0f;
                steer_last_err  = 0.0f;
                balance_turn_out = 0.0f;
            }

            /* 速度随转向差速线性降: 小拐/直行≈125, 大拐刹到下限 100 (|差速|>=100 时到底) */
            balance_forward_target = FWD_SPEED_SMALL - FWD_TURN_SLOW_GAIN * fabsf(balance_turn_out);
            if (balance_forward_target < FWD_SPEED_LARGE)
                balance_forward_target = FWD_SPEED_LARGE;
        }
        else                                            /* 关前进: 只定点平衡(位置环+速度环) */
        {
            balance_line_follow    = 0;
            balance_forward_tilt   = 0.0f;
            balance_forward_target = 0.0f;
            balance_turn_out = 0.0f;
        }
        balance_turn_out = func_limit_ab(balance_turn_out, -TURN_MAX_RPM, TURN_MAX_RPM);
        /* RING_ADDON_BEGIN */
        ring_runtime_set_operator(balance_run_enabled, balance_ring_display);
        ring_runtime_observe_motion(balance_forward_speed, s_angle.agy - balance_mech_zero,
            balance_speed_out, balance_left_duty, balance_right_duty);
        ring_runtime_apply(&balance_forward_target, &balance_turn_out, balance_position_measured, m7_1_data_1[1]);
        /* RING_ADDON_END */

        /* ---- 写调试数据给 CM7_0 屏幕显示 (共享内存 [2..14]) ---- */
        m7_1_data_1[2]  = (int)(balance_forward_speed * 10.0f);   // 实测前向速度 RPM*10
        m7_1_data_1[3]  = (int)(balance_speed_out * 10.0f);       // 速度环输出(目标前倾角) 度*10
        m7_1_data_1[4]  = (int)(balance_position_out * 10.0f);    // 位置环输出(目标速度) RPM*10
        m7_1_data_1[5]  = balance_left_duty;                      // 左轮占空比(PWM 值)
        m7_1_data_1[6]  = balance_right_duty;                     // 右轮占空比
        m7_1_data_1[7]  = small_driver_value.receive_left_speed_data;   // 左轮速 RPM
        m7_1_data_1[8]  = small_driver_value.receive_right_speed_data;  // 右轮速 RPM
        m7_1_data_1[9]  = balance_stand_up_done;                  // 是否站起 0/1
        m7_1_data_1[10] = (int)(balance_turn_out * 10.0f);        // 转向差速 RPM*10
        m7_1_data_1[11] = (int)(servo_angle[0] * 10.0f);          // 舵机角 度*10
        m7_1_data_1[12] = (int)(servo_angle[1] * 10.0f);
        m7_1_data_1[13] = (int)(servo_angle[2] * 10.0f);
        m7_1_data_1[14] = (int)(servo_angle[3] * 10.0f);
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data_1, sizeof(m7_1_data_1));

        system_delay_ms(30);
        rcc_roll = imu660rc_roll;
        rcc_pitch=imu660rc_pitch;
        rcc_yaw = imu660rc_yaw;
        // 旧四轮编码器读取, 轮腿不用 (速度反馈已改为无刷驱动回传的 receive_*_speed_data)
        //encoder_data_dir[0] = encoder_get_count(ENCODER_QUADDEC1);
        //encoder_clear_count(ENCODER_QUADDEC1);
        //encoder_data_dir[1] = encoder_get_count(ENCODER_QUADDEC2);
        //encoder_clear_count(ENCODER_QUADDEC2);
        //encoder_data_dir[2] = encoder_get_count(ENCODER_QUADDEC3);
        //encoder_clear_count(ENCODER_QUADDEC3);
        //encoder_data_dir[3] = encoder_get_count(ENCODER_QUADDEC4);
        //encoder_clear_count(ENCODER_QUADDEC4);
        //pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY(90.0f));
        printf("agy=%.2f agx=%.2f gyro=%.2f spdL=%d spdR=%d L=%d R=%d pos=%.3f tgt=%.2f pout=%.1f sout=%.1f sp=%.1f sr=%.1f s1=%.0f s2=%.0f s3=%.0f s4=%.0f st=%d rxISR=%d\n", s_angle.agy, s_angle.agx, imu660rb_gyro_y_l, small_driver_value.receive_left_speed_data, small_driver_value.receive_right_speed_data, balance_left_duty, balance_right_duty, balance_position_measured, balance_target_position, balance_position_out, balance_speed_out, servo_pitch_out, servo_roll_out, servo_angle[0], servo_angle[1], servo_angle[2], servo_angle[3], balance_stand_up_done, uart4_rx_isr_count);
        /* UART1 diagnostic: bounded FIFO writes, no wait for transmission. */
        ring_uart_diag_poll((int)balance_steer_error,
            small_driver_value.receive_left_speed_data, small_driver_value.receive_right_speed_data);
        // printf("%f,%f,%f\n",s_angle.agz,imu660rc_pitch,imu660rc_roll -2.0);
        //printf("1");
       //printf("%f,%f,%f\n",imu660rc_roll,imu660rc_pitch,imu660rc_yaw);
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������       UART_INDEX �Ľ����жϴ������� ����������� UART_INDEX ��Ӧ���жϵ���
// ����˵��       void
// ���ز���       void
// ʹ��ʾ��       uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt_handler (void)
{
//    get_data = uart_read_byte(UART_INDEX);        // �������� while �ȴ�ʽ ���������ж�ʹ��
  
    //if(uart_query_buffer(UART_2, by_fly))                                  // �������� ��ѯʽ �����ݻ᷵�� TRUE û�����ݻ᷵�� FALSE
    //{
    //    fifo_write_buffer(&uart_data_fifo, by_fly, 5);     
    //}
    //uart_query_buffer(UART_2, by_fly);
  len++;
  if(len>6)len=0;
  uart_query_byte(UART_1, &by_fly[0+len]);
}

// **************************** �������� ****************************
// **************************** ���̳�������˵�� ****************************
// ��������ʱ�밴�������������б����
// ����1������û������
//      �鿴�������ִ򿪵��Ƿ�����ȷ�Ĵ��ڣ����򿪵� COM ���Ƿ��Ӧ���ǵ������������� USB-TTL ģ��� COM ��
//      �����ʹ����ɿƼ� CMSIS-DAP �������������ӣ���ô������������Ƿ��ɶ��������İ崮�������Ƿ��Ѿ����ӣ��������߲鿴���İ�ԭ��ͼ�����ҵ�
//      �����ʹ�� USB-TTL ģ�����ӣ���ô��������Ƿ������Ƿ��ɶ���ģ�� TX �Ƿ����ӵĺ��İ�� RX��ģ�� RX �Ƿ����ӵĺ��İ�� TX
// ����2��������������
//      �鿴�����������õĲ������Ƿ����������һ�£������� zf_common_debug.h �ļ��� DEBUG_UART_BAUDRATE �궨��Ϊ debug uart ʹ�õĴ��ڲ�����