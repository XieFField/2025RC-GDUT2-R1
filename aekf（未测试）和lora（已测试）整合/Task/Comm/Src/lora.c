//#include "lora.h"
//#include "usart.h"
//#include <string.h>
//#include <stdlib.h>
//#include "drive_atk_mw1278d.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "drive_uart.h"
///* ATK-MW1278Dģ�����ò������� */
////#define DEMO_ADDR       1                               /* �豸��ַ */
////#define DEMO_WLRATE     ATK_MW1278D_WLRATE_19K2        /* �������� */
////#define DEMO_CHANNEL    1                               /* �ŵ� */
////#define DEMO_TPOWER     ATK_MW1278D_TPOWER_20DBM       /* ���书�� */
////#define DEMO_WORKMODE   ATK_MW1278D_WORKMODE_NORMAL    /* ����ģʽ */
////#define DEMO_TMODE      ATK_MW1278D_TMODE_TT           /* ����ģʽ */
////#define DEMO_WLTIME     ATK_MW1278D_WLTIME_1S          /* ����ʱ�� */
////#define DEMO_UARTRATE   ATK_MW1278D_UARTRATE_115200BPS /* UARTͨѶ������ */
////#define DEMO_UARTPARI   ATK_MW1278D_UARTPARI_NONE      /* UARTͨѶУ��λ */

//uint8_t times_error = 1;



//// ģ���ʼ������
//void All_Init(void) {
//    uint8_t ret;
//    
//    // ��ʼ��ATK-MW1278Dģ��
////    ret = atk_mw1278d_init(115200);
////    if(ret != 0) while(1) osDelay(2);
//    
////    // ��������ģʽ
////    atk_mw1278d_enter_config();
////    
////    // ���ò���
////    ret  = atk_mw1278d_addr_config(DEMO_ADDR);
////    ret += atk_mw1278d_wlrate_channel_config(DEMO_WLRATE, DEMO_CHANNEL);
////    ret += atk_mw1278d_tpower_config(DEMO_TPOWER);
////    ret += atk_mw1278d_workmode_config(DEMO_WORKMODE);
////    ret += atk_mw1278d_tmode_config(DEMO_TMODE);
////    ret += atk_mw1278d_wltime_config(DEMO_WLTIME);
////    ret += atk_mw1278d_uart_config(DEMO_UARTRATE, DEMO_UARTPARI);
////    
////    // �˳�����ģʽ
////    atk_mw1278d_exit_config();
//    
////    if(ret != 0) while(1) osDelay(2);
//    
//    // ��������
////    atk_mw1278d_uart_rx_restart();
//}
//int clock=0;
//float valid_num1;
//float valid_num2;
//float valid_num3;
///**
// * @brief       LORA��������(�Ż���)
// * @param       argument: �������
// * @retval      ��
// */
void Lora_Task(void *argument) {
//        All_Init();
//	for(;;) {
//		g_buf = atk_mw1278d_uart_rx_get_frame();
//		   // ����������
//        
//        if(g_buf != NULL) {
//		if(clock==0){
//     
//            g_len = atk_mw1278d_uart_rx_get_frame_len();
//            parse_data(g_buf, g_len);  // ��������
//            
//            // ���»��λ�����
//            g_data_buf.num1[g_idx] = g_num1;
//            g_data_buf.num2[g_idx] = g_num2;
//            g_data_buf.num3[g_idx] = g_num3;
//            g_idx = (g_idx + 1) % 3;
//            if(g_count < 3) g_count++;
//            
//            // ���Ŷ��ж�
//            if(g_count == 3) {
//                g_valid = is_valid_data();
//                if(g_valid) {
//					uart_putc('V');  // ��Ч���ݱ��
//					valid_num1=g_num1;
//					valid_num2=g_num2;
//					valid_num3=g_num3;
//                }
//            }
//            times_error = 0;
//            atk_mw1278d_uart_rx_restart();
//        }        

//    }
//        stack_high_water_mark = uxTaskGetStackHighWaterMark(NULL);
//        
//		osDelay(5);
//	}
}

///**
// * @brief       LORA��������(�Ż���)
// * @param       argument: �������
// * @retval      ��
// */
void Lora_Task1(void *argument) {
//    uint8_t len1, len2;
//    char *p;

//	
//    for(;;) {
////		if(clock==1){
////        if(atk_mw1278d_free() != ATK_MW1278D_EBUSY) {

//atk_mw1278d_uart_printf("%f,%f,%d" ,
//    (double)flt1,
//    (double)flt2,temp);
// osDelay(2);
//    }
		

         
	}
