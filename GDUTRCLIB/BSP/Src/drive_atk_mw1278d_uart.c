#include "drive_atk_mw1278d_uart.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define PKG_HEAD_SIZE    2
#define PKG_FLOAT_SIZE   (sizeof(float) * 2)
#define PKG_TAIL_SIZE    2
#define PKG_CRC_SIZE     1
#define PKG_TOTAL_SIZE   (PKG_HEAD_SIZE + PKG_FLOAT_SIZE + PKG_CRC_SIZE + PKG_TAIL_SIZE)
TIM_HandleTypeDef g_tim_handle;                      /* ATK_MW1278D Timer */
                    /* ATK_MW1278D UART */
static UART_HandleTypeDef g_uart_handle;                    /* ATK_MW1278D UART */

static struct
{
    uint8_t buf[ATK_MW1278D_UART_RX_BUF_SIZE];              /* ֡���ջ��� */
    struct
    {
        uint16_t len    : 20;                               /* ֡���ճ��ȣ�sta[14:0] */
        uint16_t finsh  : 1;                                /* ֡������ɱ�־��sta[15] */
    } sta;                                                  /* ֡״̬��Ϣ */
} g_uart_rx_frame = {0};                                    /* ATK_MW1278D UART����֡������Ϣ�ṹ�� */
static uint8_t g_uart_tx_buf[ATK_MW1278D_UART_TX_BUF_SIZE]; /* ATK_MW1278D UART���ͻ��� */



// CRCУ�麯��
uint8_t crc8(uint8_t *data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}


/**
 * @brief       ATK_MW1278D UART printf
 * @param       fmt: ����ӡ������
 * @retval      ��
 */
void atk_mw1278d_uart_printf(char *fmt, ...)
{
    va_list ap;
    uint16_t len;
    
    va_start(ap, fmt);
    // ��� vsprintf �Ƿ�ɹ�
    if (vsprintf((char *)g_uart_tx_buf, fmt, ap) < 0) {
        // �����ʽ������
    }    va_end(ap);
    
    len = strlen((const char *)g_uart_tx_buf);
	HAL_UART_Transmit(&huart2, g_uart_tx_buf, len, HAL_MAX_DELAY);
}





/**
 * @brief       ATK_MW1278D UART���¿�ʼ��������
 * @param       ��
 * @retval      ��
 */
void atk_mw1278d_uart_rx_restart(void)
{
    g_uart_rx_frame.sta.len     = 0;
    g_uart_rx_frame.sta.finsh   = 0;
}

/**
 * @brief       ��ȡATK_MW1278D UART���յ���һ֡����
 * @param       ��
 * @retval      NULL: δ���յ�һ֡����
 *              ����: ���յ���һ֡����
 */
uint8_t *atk_mw1278d_uart_rx_get_frame(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
		
        g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = '\0';
        return g_uart_rx_frame.buf;
    }
    else
    {
        return NULL;
    }
}



/**
 * @brief       ��ȡATK_MW1278D UART���յ���һ֡���ݵĳ���
 * @param       ��
 * @retval      0   : δ���յ�һ֡����
 *              ����: ���յ���һ֡���ݵĳ���
 */
uint16_t atk_mw1278d_uart_rx_get_frame_len(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        return g_uart_rx_frame.sta.len;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief       ATK_MW1278D UART��ʼ��
 * @param       baudrate: UARTͨѶ������
 * @retval      ��
 */
void atk_mw1278d_uart_init(uint32_t baudrate)
{
	g_uart_handle.Instance          = ATK_MW1278D_UART_INTERFACE;   /* ATK_MW1278D UART */
		  GPIO_InitTypeDef gpio_init_struct;
	        ATK_MW1278D_UART_TX_GPIO_CLK_ENABLE();                              /* ʹ��UART TX����ʱ�� */
        ATK_MW1278D_UART_RX_GPIO_CLK_ENABLE();                              /* ʹ��UART RX����ʱ�� */
        ATK_MW1278D_UART_CLK_ENABLE();                                      /* ʹ��UARTʱ�� */
        	        gpio_init_struct.Pin        = ATK_MW1278D_UART_TX_GPIO_PIN;         /* UART TX���� */
        gpio_init_struct.Mode       = GPIO_MODE_AF_PP;                      /* ����������� */
        gpio_init_struct.Pull       = GPIO_NOPULL;                          /* �������� */
        gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;                 /* ���� */
        gpio_init_struct.Alternate  = ATK_MW1278D_UART_TX_GPIO_AF;          /* ����ΪUART4 */
        HAL_GPIO_Init(ATK_MW1278D_UART_TX_GPIO_PORT, &gpio_init_struct);    /* ��ʼ��UART TX���� */
        
        gpio_init_struct.Pin        = ATK_MW1278D_UART_RX_GPIO_PIN;         /* UART RX���� */
        gpio_init_struct.Mode       = GPIO_MODE_AF_PP;                      /* ����������� */
        gpio_init_struct.Pull       = GPIO_NOPULL;                          /* �������� */
        gpio_init_struct.Speed      = GPIO_SPEED_FREQ_HIGH;                 /* ���� */
        gpio_init_struct.Alternate  = ATK_MW1278D_UART_RX_GPIO_AF;          /* ����ΪUART4 */
        HAL_GPIO_Init(ATK_MW1278D_UART_RX_GPIO_PORT, &gpio_init_struct);    /* ��ʼ��UART RX���� */		


	   
    g_uart_handle.Init.BaudRate     = baudrate;                     /* ������ */
    g_uart_handle.Init.WordLength   = UART_WORDLENGTH_8B;           /* ����λ */
    g_uart_handle.Init.StopBits     = UART_STOPBITS_1;              /* ֹͣλ */
    g_uart_handle.Init.Parity       = UART_PARITY_NONE;             /* У��λ */
    g_uart_handle.Init.Mode         = UART_MODE_TX_RX;              /* �շ�ģʽ */
    g_uart_handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;          /* ��Ӳ������ */
    g_uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;         /* ������ */
    HAL_UART_Init(&g_uart_handle);                                  /* ʹ��ATK_MW1278D UART
                                                                     * HAL_UART_Init()����ú���HAL_UART_MspInit()
                                                                     * �ú����������ļ�usart.c��
                                                                     */
														 
			HAL_NVIC_SetPriority(ATK_MW1278D_UART_IRQn, 0, 0);                  /* ��ռ���ȼ�0�������ȼ�0 */
        HAL_NVIC_EnableIRQ(ATK_MW1278D_UART_IRQn);                          /* ʹ��UART�ж�ͨ�� */
        
        __HAL_UART_ENABLE_IT(&g_uart_handle, UART_IT_RXNE);                          /* ʹ��UART�����ж� */																 
																 
																 
    g_tim_handle.Instance           = ATK_MW1278D_TIM_INTERFACE;    /* ATK_MW1278D Timer */
    g_tim_handle.Init.Prescaler     = ATK_MW1278D_TIM_PRESCALER - 1;/* Ԥ��Ƶϵ�� */
    g_tim_handle.Init.Period        = 100 - 1;                      /* �Զ���װ��ֵ */
    HAL_TIM_Base_Init(&g_tim_handle);                               /* ��ʼ��Timer����UART���ճ�ʱ��� */
	

        

	

}


/**
 * @brief       ATK_MW1278D Timer��ʼ��MSP�ص�����
 * @param       htim: Timer���ָ��
 * @retval      ��
 */
void TIM6_For_Lora(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATK_MW1278D_TIM_INTERFACE)
    {
        ATK_MW1278D_TIM_CLK_ENABLE();                       /* ʹ��Timerʱ�� */
        
        HAL_NVIC_SetPriority(ATK_MW1278D_TIM_IRQn, 0, 0);   /* ��ռ���ȼ�0�������ȼ�0 */
        HAL_NVIC_EnableIRQ(ATK_MW1278D_TIM_IRQn);           /* ʹ��Timer�ж� */
        
        __HAL_TIM_ENABLE_IT(&g_tim_handle, TIM_IT_UPDATE);  /* ʹ��Timer�����ж� */
    }
}

/**
 * @brief       ATK_MW1278D Timer�жϻص�����
 * @param       ��
 * @retval      ��
 */
void ATK_MW1278D_TIM_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&g_tim_handle, TIM_FLAG_UPDATE) != RESET)    /* Timer�����ж� */
    {
        __HAL_TIM_CLEAR_IT(&g_tim_handle, TIM_IT_UPDATE);               /* ��������жϱ�־ */
		 __HAL_TIM_DISABLE_IT(&g_tim_handle, TIM_IT_UPDATE);   // ���ø����ж�
        g_uart_rx_frame.sta.finsh = 1;                                  /* ���֡������� */
        __HAL_TIM_DISABLE(&g_tim_handle);                               /* ֹͣTimer���� */
    }
}

/**
 * @brief       ATK_MW1278D UART�жϻص�����
 * @param       ��
 * @retval      ��
 */

void ATK_MW1278D_UART_IRQHandler(void)
{
    uint8_t tmp;
    
    if (__HAL_UART_GET_FLAG(&g_uart_handle, UART_FLAG_ORE) != RESET)        /* UART���չ��ش����ж� */
    {
        __HAL_UART_CLEAR_OREFLAG(&g_uart_handle);                           /* ������չ��ش����жϱ�־ */
        (void)g_uart_handle.Instance->SR;                                   /* �ȶ�SR�Ĵ������ٶ�DR�Ĵ��� */
        (void)g_uart_handle.Instance->DR;
    }
    
    if (__HAL_UART_GET_FLAG(&g_uart_handle, UART_FLAG_RXNE) != RESET)       /* UART�����ж� */
    {
        HAL_UART_Receive(&g_uart_handle, &tmp, 1, HAL_MAX_DELAY);           /* UART�������� */
        
        if (g_uart_rx_frame.sta.len < (ATK_MW1278D_UART_RX_BUF_SIZE - 1))   /* �ж�UART���ջ����Ƿ����
                                                                             * ����һλ��������'\0'
                                                                             */
        {
            __HAL_TIM_SET_COUNTER(&g_tim_handle, 0);                        /* ����Timer����ֵ */
            if (g_uart_rx_frame.sta.len == 0)                               /* �����һ֡�ĵ�һ������ */
            {
                __HAL_TIM_CLEAR_FLAG(&g_tim_handle, TIM_FLAG_UPDATE);  // ������±�־
                __HAL_TIM_ENABLE_IT(&g_tim_handle, TIM_IT_UPDATE);  // ʹ�ܸ����ж�
                __HAL_TIM_ENABLE(&g_tim_handle);                            /* ����Timer���� */
            }
            
            // ֱ�ӽ����յ������ݴ��뻺���������������ݰ�����
            g_uart_rx_frame.buf[g_uart_rx_frame.sta.len++] = tmp;
        }
        else                                                                /* UART���ջ������ */
        {
            g_uart_rx_frame.sta.len = 0;                                    /* ����֮ǰ�յ������� */
            g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = tmp;             /* �����յ�������д�뻺�� */
            g_uart_rx_frame.sta.len++;                                      /* ���½��յ������ݳ��� */
        }
    }
}