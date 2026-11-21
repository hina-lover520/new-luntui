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
* �ļ�����          cm7_0_isr
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            ����12��pit�����ж� ���Ӳ���ע��˵��
* 2025-2-4      pudding            �Ż������ж��߼�����ֹ������ŵ��µĿ������⣬�Ż����ڲ����ʼ����߼�
* 2025-2-4      pudding            �����������ڽӿ�
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "kaerman.h"
#include "quaternion.h"
#include "car_control.h"
#include "my_encoder.h"
#include "ztjx.h"
#include "blance.h"
#include "small_driver_uart_control.h"

/* RING_ADDON_BEGIN */
#include "ring_runtime.h"
/* OPERATOR_UI_BEGIN */
#include "run_buttons.h"
extern volatile uint8 balance_run_enabled;
extern volatile uint8 balance_ring_display;
static run_buttons_t operator_buttons; /* BSS = run_buttons_init() defaults. */
/* OPERATOR_UI_END */
/* RING_ADDON_END */
volatile uint32 uart4_rx_isr_count = 0;   // 诊断: UART4 接收中断触发次数 (0=中断没进来, >0=中断进来但解析有问题)

// **************************** PIT�жϺ��� ****************************
void pit0_ch0_isr()                     // ��ʱ��ͨ�� 0 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH0);
    /* OPERATOR_UI_BEGIN: bounded 1ms scan; no waits or display work. */
    run_buttons_tick(&operator_buttons,
        (uint8_t)(gpio_get_level(P20_2) == GPIO_LOW),
        (uint8_t)(gpio_get_level(P20_3) == GPIO_LOW));
    balance_run_enabled = operator_buttons.run_enabled;
    balance_ring_display = operator_buttons.display_ring;
    /* OPERATOR_UI_END */
    imu660rb_get_acc();
    imu660rb_get_gyro();
    imurb_get_values();
    imu660rb_gyro_x_l=kaermanb_Updata_1(&gyro_x,imu660rb_gyro_x);
    imu660rb_gyro_y_l=kaermanb_Updata_1(&gyro_y,imu660rb_gyro_y);
    imu660rb_gyro_z_l=kaermanb_Updata_1(&gyro_z,imu660rb_gyro_z);
    my_quaternion(&sc,&s_angle,&s_shu);
    /* RING_ADDON_BEGIN */
    {
        int8_t ring_beep = ring_runtime_tick(imu660rb_gyro_z, balance_stand_up_done);
        if (ring_beep >= 0)
            gpio_set_level(BUZZER_PIN, ring_beep ? GPIO_HIGH : GPIO_LOW);
        ring_runtime_poll_timeout(&balance_forward_target, &balance_turn_out);
    }
    /* RING_ADDON_END */
    //balance_star();     // 轮腿平衡控制 (角度环 + 角速度环)

    //printf("%f\n",s_angle.agz);
    /*printf("yaw=%.1f tgt=%.1f valid=%d rec=%d signed=%.2f max=%.2f\n",
       s_angle.agz, yaw_stable_deg_dbg,
       yaw_stable_valid_dbg, yaw_recover_active_dbg,
       gz_signed_integral_dbg, gz_max_abs_dbg);*/
    //imurb_get_values();
    //car_star();
}

void pit0_ch1_isr()                     // ��ʱ��ͨ�� 1 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH1);
    /* OPERATOR_UI_BEGIN: standby runs IMU, not balance or servo PID. */
    if (balance_run_enabled) {
    balance_star();     // 轮腿平衡控制 (角度环 + 角速度环)
    servo_balance_star();   // 舵机辅助平衡 (俯仰 + 横滚)
    }
    /* OPERATOR_UI_END */
   //my_quaternion(&sc,&s_angle,&s_shu);
   //car_pid1();
   //car_pid();
   //car_star();
   //printf("%f,%f,%f\n",s_angle.agx,s_angle.agy,s_angle.agz);
   //printf("%f,%f,%f\n",1.0f,1.0f,1.0f);
}

void pit0_ch2_isr()                     // ��ʱ��ͨ�� 2 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH2);
    //Rc_Get();
    //car_pid();
    //printf("%f,%f,%f\n",s_angle.agx,s_angle.agy,s_angle.agz);
    //car_star();   // 旧四轮有刷电机, 轮腿不用
}

void pit0_ch10_isr()                    // ��ʱ��ͨ�� 10 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH10);
    
    // 旧四轮里程计积分, 轮腿不用
    //xx_count+=(-encoder_data_dir[2]+encoder_data_dir[1]-encoder_data_dir[3]+encoder_data_dir[0])/4;
    //yy_count+=(encoder_data_dir[2]+encoder_data_dir[1]-encoder_data_dir[3]-encoder_data_dir[0])/4;
}

void pit0_ch11_isr()                    // ��ʱ��ͨ�� 11 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH11);
    
}

void pit0_ch12_isr()                    // ��ʱ��ͨ�� 12 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH12);
    
}

void pit0_ch13_isr()                    // ��ʱ��ͨ�� 13 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH13);
    
}

void pit0_ch14_isr()                    // ��ʱ��ͨ�� 14 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH14);
    
}

void pit0_ch15_isr()                    // ��ʱ��ͨ�� 15 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH15);
    
}

void pit0_ch16_isr()                    // ��ʱ��ͨ�� 16 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH16);
    
}

void pit0_ch17_isr()                    // ��ʱ��ͨ�� 17 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH17);
    
}

void pit0_ch18_isr()                    // ��ʱ��ͨ�� 18 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH18);
    
}

void pit0_ch19_isr()                    // ��ʱ��ͨ�� 19 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH19);
    
}

void pit0_ch20_isr()                    // ��ʱ��ͨ�� 20 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH20);
    
}

void pit0_ch21_isr()                    // ��ʱ��ͨ�� 21 �����жϷ�����      
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT�жϺ��� ****************************

void uart_rx_interrupt_handler (void);
// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // ����0�����ж�
    {
        
//#if DEBUG_UART_USE_INTERRUPT             // ������� debug �����ж�
//        debug_interrupr_handler();       // ���� debug ���ڽ��մ������� ���ݻᱻ debug ���λ�������ȡ
//#endif                                   // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
      //uart_rx_interrupt_handler();
    }
    else                                 // ����0�����ж�
    {           
        
        
        
    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // ����1�����ж�
    {
        
        //wireless_module_uart_handler();  // ����ģ��ͳһ�ص�����
      uart_rx_interrupt_handler();
    }
    else                                // ����1�����ж�
    {
      
        
        
    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // ����2�����ж�
    {
        
       // gnss_uart_callback();            // GPSģ��ص�����      
        uart_rx_interrupt_handler();
    }
    else                                // ����2�����ж�
    {
        
        
       
    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // ����3�����ж�
    {
        
        
        
    }
    else                                // ����3�����ж�
    {
      
        
        
    }
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))            // ����4�����ж�
    {

        uart4_rx_isr_count++;                                                                      // 诊断计数
        small_driver_control_callback(&small_driver_value);                                      // 小驱动两路无刷串口接收回调 (UART4)

    }
    else                                // ����4�����ж�
    {
      
        
        
    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))            // ����5�����ж�
    {
        
        
       
    }
    else                                // ����5�����ж�
    {
      
        
        
    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))            // ����6�����ж�
    {

        
       
    }
    else                                // ����6�����ж�
    {
      
        
        
    }
}
// **************************** �����жϺ��� ****************************

// **************************** �ⲿ�жϺ��� ****************************
void gpio_0_exti_isr()                  // �ⲿ GPIO_0 �жϷ�����     
{
    
  
  
}

void gpio_1_exti_isr()                  // �ⲿ GPIO_1 �жϷ�����     
{
    if(exti_flag_get(P01_0))		// ʾ��P1_0�˿��ⲿ�ж��ж�
    {

      
      
            
    }
    if(exti_flag_get(P01_1))
    {

            
            
    }
}

void gpio_2_exti_isr()                  // �ⲿ GPIO_2 �жϷ�����     
{
    if(exti_flag_get(P02_0))
    {
            
            
    }
    if(exti_flag_get(P02_4))
    {
            
            
    }

}

void gpio_3_exti_isr()                  // �ⲿ GPIO_3 �жϷ�����     
{



}

void gpio_4_exti_isr()                  // �ⲿ GPIO_4 �жϷ�����     
{



}

void gpio_5_exti_isr()                  // �ⲿ GPIO_5 �жϷ�����     
{



}


void gpio_6_exti_isr()                  // �ⲿ GPIO_6 �жϷ�����     
{
      if(exti_flag_get(P06_7))
    {
        imu660rc_callback();
    }
}

void gpio_7_exti_isr()                  // �ⲿ GPIO_7 �жϷ�����     
{



}

void gpio_8_exti_isr()                  // �ⲿ GPIO_8 �жϷ�����     
{



}

void gpio_9_exti_isr()                  // �ⲿ GPIO_9 �жϷ�����     
{



}

void gpio_10_exti_isr()                  // �ⲿ GPIO_10 �жϷ�����     
{



}

void gpio_11_exti_isr()                  // �ⲿ GPIO_11 �жϷ�����     
{



}

void gpio_12_exti_isr()                  // �ⲿ GPIO_12 �жϷ�����     
{



}

void gpio_13_exti_isr()                  // �ⲿ GPIO_13 �жϷ�����     
{



}

void gpio_14_exti_isr()                  // �ⲿ GPIO_14 �жϷ�����     
{



}

void gpio_15_exti_isr()                  // �ⲿ GPIO_15 �жϷ�����     
{



}

void gpio_16_exti_isr()                  // �ⲿ GPIO_16 �жϷ�����     
{



}

void gpio_17_exti_isr()                  // �ⲿ GPIO_17 �жϷ�����     
{



}

void gpio_18_exti_isr()                  // �ⲿ GPIO_18 �жϷ�����     
{



}

void gpio_19_exti_isr()                  // �ⲿ GPIO_19 �жϷ�����     
{



}

void gpio_20_exti_isr()                  // �ⲿ GPIO_20 �жϷ�����     
{



}

void gpio_21_exti_isr()                  // �ⲿ GPIO_21 �жϷ�����     
{



}

void gpio_22_exti_isr()                  // �ⲿ GPIO_22 �жϷ�����     
{



}

void gpio_23_exti_isr()                  // �ⲿ GPIO_23 �жϷ�����     
{



}
// **************************** �ⲿ�жϺ��� ****************************