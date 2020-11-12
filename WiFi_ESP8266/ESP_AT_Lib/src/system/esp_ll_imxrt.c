/**
 * \file            esp_ll_template.c
 * \brief           Low-level communication with ESP device template
 */

/*
 * Copyright (c) 2018 Tilen Majerle
 *  
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software, 
 * and to permit persons to whom the Software is furnished to do so, 
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of ESP-AT library.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 */
#include "system/esp_ll.h"
#include "esp/esp.h"
#include "esp/esp_mem.h"
#include "esp/esp_input.h"

#include "board.h"
#include "fsl_lpuart.h"
#include "fsl_lpuart_edma.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "fsl_cache.h"

#define CFG_ESP8266_UART_TX_DMA_MODE            0
#define CFG_ESP_MEM_SIZE                        8192
#define CFG_ESP8266_UART_RX_PINGPONG_BUF_SIZE   4096  /* 4KB is minimum */

#define ESP8266_UART_RX_DMA_CHNL                4
#define ESP8266_UART_RX_DMA_IRQn                DMA4_DMA20_IRQn 
#if (defined(CFG_ESP8266_UART_TX_DMA_MODE) && (CFG_ESP8266_UART_TX_DMA_MODE == 1))
#define ESP8266_UART_TX_DMA_CHNL                5
#define ESP8266_UART_TX_DMA_IRQn                DMA5_DMA21_IRQn 
#endif

static uint8_t initialized = 0;
AT_NONCACHEABLE_SECTION(static uint8_t s_Esp8266RecvPingPongBuffer[2][CFG_ESP8266_UART_RX_PINGPONG_BUF_SIZE]);

static edma_handle_t        s_Esp8266RxEdmaHandle;
static lpuart_edma_handle_t s_Esp8266lpuartEdmaHandle;
static SemaphoreHandle_t    s_IdleSemaphr;
#if (defined(CFG_ESP8266_UART_TX_DMA_MODE) && (CFG_ESP8266_UART_TX_DMA_MODE == 1))
static edma_handle_t        s_Esp8266TxEdmaHandle;
static SemaphoreHandle_t    s_Esp8266UartTxDoneSemaphr;
#endif

#if (defined(CFG_ESP8266_UART_TX_DMA_MODE) && (CFG_ESP8266_UART_TX_DMA_MODE == 1))
/**
 * \brief           dma callback handler
 */
static void
__Esp8266DmaUserCallback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData) {
    
    userData = userData;

    if (status == kStatus_LPUART_TxIdle) {

        BaseType_t xHigherPriorityTaskWoken;
        BaseType_t rt;

        rt = xSemaphoreGiveFromISR(s_Esp8266UartTxDoneSemaphr, &xHigherPriorityTaskWoken);
        assert(rt == pdPASS);

    	if (xHigherPriorityTaskWoken) {
    		taskYIELD();
    	}
    }
}

#endif

/**
 * \brief           UART global interrupt handler
 */
void 
BOARD_ESP8266_UART_IRQHandler(void) {
    uint32_t status = LPUART_GetStatusFlags(BOARD_ESP8266_UART_BASEADDR);
    
    if (status & kLPUART_RxOverrunFlag) {
        LPUART_ClearStatusFlags(BOARD_ESP8266_UART_BASEADDR, kLPUART_RxOverrunFlag);
    }
    
    if (status & kLPUART_IdleLineFlag) {
        LPUART_ClearStatusFlags(BOARD_ESP8266_UART_BASEADDR, kLPUART_IdleLineFlag);

        BaseType_t xHigherPriorityTaskWoken;
        BaseType_t rt;

        rt = xSemaphoreGiveFromISR(s_IdleSemaphr, &xHigherPriorityTaskWoken);
        assert(rt == pdPASS);

    	if (xHigherPriorityTaskWoken) {
    		taskYIELD();
    	}
    }
    
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/**
 * \brief           USART data processing
 */
static void
usart_ll_thread(void *p_param) {
    
    lpuart_transfer_t xfer;
    uint32_t count = 0;
    uint32_t index = 0;
    
    while (1) {
        (void)xSemaphoreTake(s_IdleSemaphr, portMAX_DELAY);
        
        LPUART_TransferGetReceiveCountEDMA(BOARD_ESP8266_UART_BASEADDR, &s_Esp8266lpuartEdmaHandle, &count);
        LPUART_TransferAbortReceiveEDMA(BOARD_ESP8266_UART_BASEADDR, &s_Esp8266lpuartEdmaHandle);
        xfer.data = s_Esp8266RecvPingPongBuffer[index ^ 1];
        xfer.dataSize = CFG_ESP8266_UART_RX_PINGPONG_BUF_SIZE;
        LPUART_ReceiveEDMA(BOARD_ESP8266_UART_BASEADDR, &s_Esp8266lpuartEdmaHandle, &xfer);

        esp_input_process(s_Esp8266RecvPingPongBuffer[index], count);
        index ^= 1;
    }
}

/**
 * \brief           Configure UART using DMA for receive in double buffer mode and IDLE line detection
 */
void
configure_uart(uint32_t br) {
    
    lpuart_config_t config;
    
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = br;
    config.enableTx = true;
    config.enableRx = true;
    
    if (!initialized) {
    
        IOMUXC_SetPinMux(BOARD_ESP8266_HW_RESET_IOMUXC, 0U);
        IOMUXC_SetPinConfig(BOARD_ESP8266_HW_RESET_IOMUXC, 0x10B0u);
        gpio_pin_config_t gpioConfig = {kGPIO_DigitalOutput, 1, kGPIO_NoIntmode};
        GPIO_PinInit(BOARD_ESP8266_HW_RESET_PORT, BOARD_ESP8266_HW_RESET_PIN, &gpioConfig);

        GPIO_PortClear(BOARD_ESP8266_HW_RESET_PORT, 1 << BOARD_ESP8266_HW_RESET_PIN);
        vTaskDelay(2);
        GPIO_PortSet(BOARD_ESP8266_HW_RESET_PORT, 1 << BOARD_ESP8266_HW_RESET_PIN);
        vTaskDelay(500);                    /* By default, it takes ~200ms to get ready @ 115200 */
        
        IOMUXC_SetPinMux(BOARD_ESP8266_UART_TX_IOMUX, 0U);
        IOMUXC_SetPinMux(BOARD_ESP8266_UART_RX_IOMUX, 0U);
        IOMUXC_SetPinConfig(BOARD_ESP8266_UART_TX_IOMUX, 0x10B0u);
        IOMUXC_SetPinConfig(BOARD_ESP8266_UART_RX_IOMUX, 0x10B0u);

        LPUART_Init(BOARD_ESP8266_UART_BASEADDR, &config, BOARD_ESP8266_UART_CLK_FREQ);
        
        s_IdleSemaphr = xSemaphoreCreateBinary();
        assert(s_IdleSemaphr != NULL);
        
        xTaskCreate(usart_ll_thread,
                    "Esp8266Recv", 
                    ESP8266_UART_STACK_SIZE, 
                    NULL,
                    ESP8266_UART_TASK_PRIORITY,
                    NULL);

        NVIC_SetPriority(BOARD_ESP8266_UART_IRQn, IRQPRIO_HIGH_OS);
        LPUART_EnableInterrupts(BOARD_ESP8266_UART_BASEADDR, kLPUART_IdleLineInterruptEnable | kLPUART_RxOverrunInterruptEnable);
        EnableIRQ(BOARD_ESP8266_UART_IRQn);

        DMAMUX_Init(DMAMUX);
        DMAMUX_SetSource(DMAMUX, ESP8266_UART_RX_DMA_CHNL, BOARD_ESP8266_UART_RXDMAMUX);
        DMAMUX_EnableChannel(DMAMUX, ESP8266_UART_RX_DMA_CHNL);
#if (defined(CFG_ESP8266_UART_TX_DMA_MODE) && (CFG_ESP8266_UART_TX_DMA_MODE == 1))
        DMAMUX_SetSource(DMAMUX, ESP8266_UART_TX_DMA_CHNL, BOARD_ESP8266_UART_TXDMAMUX);
        DMAMUX_EnableChannel(DMAMUX, ESP8266_UART_TX_DMA_CHNL);
#endif
        
        /* DMA initialization is done in ether QCA driver or ESP driver, TODO: move this part out of driver */
        edma_config_t dma_config;
        EDMA_GetDefaultConfig(&dma_config);
        EDMA_Init(DMA0, &dma_config);
        
        EDMA_CreateHandle(&s_Esp8266RxEdmaHandle, DMA0, ESP8266_UART_RX_DMA_CHNL);
#if (defined(CFG_ESP8266_UART_TX_DMA_MODE) && (CFG_ESP8266_UART_TX_DMA_MODE == 1))
        EDMA_CreateHandle(&s_Esp8266TxEdmaHandle, DMA0, ESP8266_UART_TX_DMA_CHNL);
        LPUART_TransferCreateHandleEDMA(BOARD_ESP8266_UART_BASEADDR, &s_Esp8266lpuartEdmaHandle, __Esp8266DmaUserCallback, NULL, &s_Esp8266TxEdmaHandle, &s_Esp8266RxEdmaHandle);
        NVIC_SetPriority(ESP8266_UART_TX_DMA_IRQn, IRQPRIO_LOW);
        s_Esp8266UartTxDoneSemaphr = xSemaphoreCreateBinary();
        assert(s_Esp8266UartTxDoneSemaphr != NULL);
#else
        LPUART_TransferCreateHandleEDMA(BOARD_ESP8266_UART_BASEADDR, &s_Esp8266lpuartEdmaHandle, NULL, NULL, NULL, &s_Esp8266RxEdmaHandle);
#endif
        
        lpuart_transfer_t xfer;
        xfer.data     = s_Esp8266RecvPingPongBuffer[0];
        xfer.dataSize = CFG_ESP8266_UART_RX_PINGPONG_BUF_SIZE;
        LPUART_ReceiveEDMA(BOARD_ESP8266_UART_BASEADDR, &s_Esp8266lpuartEdmaHandle, &xfer);
        
    } else {
        LPUART_SetBaudRate(BOARD_ESP8266_UART_BASEADDR, br, BOARD_ESP8266_UART_CLK_FREQ);
    }
}


/**
 * \brief           Send data to ESP device
 * \param[in]       data: Pointer to data to send
 * \param[in]       len: Number of bytes to send
 * \return          Number of bytes sent
 */
static size_t
send_data(const void* data, size_t len) {

#if (defined(CFG_ESP8266_UART_TX_DMA_MODE) && (CFG_ESP8266_UART_TX_DMA_MODE == 1))

    status_t st;
    
    lpuart_transfer_t sendXfer;
    sendXfer.data     = (uint8_t *)data;
    sendXfer.dataSize = len;
    
#if (defined(CACHE_MAINTENANCE) && (CACHE_MAINTENANCE == 1))
    /* Flush Dcache before start DMA */
    DCACHE_CleanByRange((uint32_t)data, len);
#endif    
    
    st = LPUART_SendEDMA(BOARD_ESP8266_UART_BASEADDR, &s_Esp8266lpuartEdmaHandle, &sendXfer);
    if (st != kStatus_Success) {
        return 0;
    }

    (void)xSemaphoreTake(s_Esp8266UartTxDoneSemaphr, portMAX_DELAY);

#else
    LPUART_WriteBlocking(BOARD_ESP8266_UART_BASEADDR, data, len);
#endif
    
    while (!(LPUART_GetStatusFlags(BOARD_ESP8266_UART_BASEADDR) & kLPUART_TransmissionCompleteFlag));
    
    return len;                                 /* Return number of bytes actually sent to AT port */
}

/**
 * \brief           Callback function called from initialization process
 * \note            This function may be called multiple times if AT baudrate is changed from application
 * \param[in,out]   ll: Pointer to \ref esp_ll_t structure to fill data for communication functions
 * \param[in]       baudrate: Baudrate to use on AT port
 * \return          Member of \ref espr_t enumeration
 */
espr_t
esp_ll_init(esp_ll_t* ll) {
    /*
     * Step 1: Configure memory for dynamic allocations
     */
    static uint8_t memory[CFG_ESP_MEM_SIZE];             /* Create memory for dynamic allocations with specific size */

    /*
     * Create region(s) of memory.
     * If device has internal/external memory available,
     * multiple memories may be used
     */
    esp_mem_region_t mem_regions[] = {
        { memory, sizeof(memory) }
    };
    if (!initialized) {
        esp_mem_assignmemory(mem_regions, ESP_ARRAYSIZE(mem_regions));  /* Assign memory for allocations to ESP library */
    }
    
    /*
     * Step 2: Set AT port send function to use when we have data to transmit
     */
    if (!initialized) {
        ll->send_fn = send_data;                /* Set callback function to send data */
    }

    /*
     * Step 3: Configure AT port to be able to send/receive data to/from ESP device
     */
    configure_uart(ll->uart.baudrate);            /* Initialize UART for communication */
    initialized = 1;
    return espOK;
}

/**
 * \brief           Callback function to de-init low-level communication part
 * \param[in,out]   ll: Pointer to \ref esp_ll_t structure to fill data for communication functions
 * \return          \ref espOK on success, member of \ref espr_t enumeration otherwise
 */
espr_t
esp_ll_deinit(esp_ll_t* ll) {
    initialized = 0;                            /* Clear initialized flag */
    return espOK;
}
