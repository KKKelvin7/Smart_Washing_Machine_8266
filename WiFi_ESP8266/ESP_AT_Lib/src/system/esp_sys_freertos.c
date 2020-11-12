/**
 * \file            esp_sys_freertos.c
 * \brief           System dependant functions for native FreeRTOS
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
 *                  Neo Xiong <xiongyu0523@gmail.com>
 */
#include "system/esp_sys.h"

#if !__DOXYGEN__

static SemaphoreHandle_t s_sys_mutex;

uint8_t
esp_sys_init(void) {
    s_sys_mutex = xSemaphoreCreateRecursiveMutex();
    return s_sys_mutex == NULL ? 0 : 1;
}

uint32_t
esp_sys_now(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

uint8_t
esp_sys_protect(void) {
    return xSemaphoreTakeRecursive(s_sys_mutex, portMAX_DELAY) == pdTRUE ? 1 : 0;
}

uint8_t
esp_sys_unprotect(void) {
    return xSemaphoreGiveRecursive(s_sys_mutex) == pdTRUE ? 1 : 0;
}

uint8_t
esp_sys_mutex_create(esp_sys_mutex_t* p) {
    *p = xSemaphoreCreateRecursiveMutex();
    return *p == NULL ? 0 : 1;
}

uint8_t
esp_sys_mutex_delete(esp_sys_mutex_t* p) {
    vSemaphoreDelete(*p);
    return 1;
}

uint8_t
esp_sys_mutex_lock(esp_sys_mutex_t* p) {
    return xSemaphoreTakeRecursive(*p, portMAX_DELAY) == pdTRUE ? 1 : 0;
}

uint8_t
esp_sys_mutex_unlock(esp_sys_mutex_t* p) {
    return xSemaphoreGiveRecursive(*p) == pdTRUE ? 1 : 0;
}

uint8_t
esp_sys_mutex_isvalid(esp_sys_mutex_t* p) {
    return p != NULL ? 1 : 0;
}

uint8_t
esp_sys_mutex_invalid(esp_sys_mutex_t* p) {
    *p = ESP_SYS_MUTEX_NULL;
    return 1;
}

uint8_t
esp_sys_sem_create(esp_sys_sem_t* p, uint8_t cnt) {
    *p = xSemaphoreCreateBinary();
    
    if (*p == NULL) {
        return 0;
    }

    /* the above API xSemaphoreCreateBinary() gives a sem with 0 set */
    if (cnt == 1) {
        if(xSemaphoreGive(*p) != pdTRUE) {
            return 0;
        }
    }
    
    return 1;
}

uint8_t
esp_sys_sem_delete(esp_sys_sem_t* p) {
    vSemaphoreDelete(*p);
    return 1;
}

uint32_t
esp_sys_sem_wait(esp_sys_sem_t* p, uint32_t timeout) {
    
    uint32_t time_in, time_out;
    time_in = esp_sys_now();

    if (timeout == 0) {
        (void)xSemaphoreTake(*p, portMAX_DELAY);
    } else {
        TickType_t tick = pdMS_TO_TICKS(timeout);
        if (xSemaphoreTake(*p, tick) == pdFALSE) {
            return ESP_SYS_TIMEOUT;
        }
    }
    
    time_out = esp_sys_now();

    return time_out - time_in;
}

uint8_t
esp_sys_sem_release(esp_sys_sem_t* p) {
    return xSemaphoreGive(*p) == pdTRUE ? 1 : 0;
}

uint8_t
esp_sys_sem_isvalid(esp_sys_sem_t* p) {
    return p != NULL ? 1 : 0;
}

uint8_t
esp_sys_sem_invalid(esp_sys_sem_t* p) {
    *p = ESP_SYS_SEM_NULL;
    return 1;
}

uint8_t
esp_sys_mbox_create(esp_sys_mbox_t* b, size_t size) {
    *b = xQueueCreate(size, sizeof(void *));
    return *b != NULL ? 1 : 0;
}

uint8_t
esp_sys_mbox_delete(esp_sys_mbox_t* b) {
    if (uxQueueMessagesWaiting(*b)) {
        return 0;
    }

    vQueueDelete(*b);
    return 1;
}

uint32_t
esp_sys_mbox_put(esp_sys_mbox_t* b, void* m) {
    uint32_t time_in = esp_sys_now();
    
    if (xQueueSendToBack(*b, &m, portMAX_DELAY) == pdPASS) {
        return esp_sys_now() - time_in;
    } else {
        return ESP_SYS_TIMEOUT;
    }
}

uint32_t 
esp_sys_mbox_get(esp_sys_mbox_t* b, void** m, uint32_t timeout) {
    void *p_msg;
    uint32_t time_in, time_out;

    time_in = esp_sys_now();

    if (m == NULL) {
        m = &p_msg;
    }

    if (timeout == 0) {
        (void)xQueueReceive(*b, m, portMAX_DELAY);
    }
    else {
        TickType_t tick = pdMS_TO_TICKS(timeout);
        if (xQueueReceive(*b, m, tick) != pdTRUE) {
            *m = NULL;
            return ESP_SYS_TIMEOUT;
        }
    }
    
    time_out = esp_sys_now();

    return time_out - time_in;
}

uint8_t 
esp_sys_mbox_putnow(esp_sys_mbox_t* b, void* m) {
    
    /* esp_sys_mbox_putnow may be called from IRQ context when esp_input() is used */
    
    if (__isIRQContext()) {
        BaseType_t rt;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        rt = xQueueSendToBackFromISR(*b, &m, &xHigherPriorityTaskWoken);
        if (rt == pdTRUE) {
            if (xHigherPriorityTaskWoken == pdTRUE) {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            return 1;
        } else {
            return 0;
        }
    } else {
        return xQueueSendToBack(*b, &m, 0) == pdTRUE ? 1 : 0;
    }
}

uint8_t 
esp_sys_mbox_getnow(esp_sys_mbox_t* b, void** m) {
    void *p_msg;

    if (m == NULL) {
        m = &p_msg;
    }

    if (xQueueReceive(*b, m, 0) != pdTRUE) {
        *m = NULL;
        return 0;
    }

    return 1;
}

uint8_t
esp_sys_mbox_isvalid(esp_sys_mbox_t* b) {
    return b != NULL ? 1 : 0;
}

uint8_t
esp_sys_mbox_invalid(esp_sys_mbox_t* b) {
    *b = ESP_SYS_MBOX_NULL;
    return 1;
}

uint8_t
esp_sys_thread_create(esp_sys_thread_t* t, const char* name, esp_sys_thread_fn thread_func, void* const arg, size_t stack_size, esp_sys_thread_prio_t prio) {
    return xTaskCreate(thread_func, name, stack_size, arg, prio, t) == pdPASS ? 1 : 0;
}

uint8_t
esp_sys_thread_terminate(esp_sys_thread_t* t) {
    vTaskDelete(*t);
    return 1;
}

uint8_t 
esp_sys_thread_yield(void) {
    taskYIELD();
    return 1;
}

#endif /* !__DOXYGEN__ */
