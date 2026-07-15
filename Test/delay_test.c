//
// Created by gaoxu on 2026/4/1.
//

#include "delay.h"
#include "delay_test.h"

#include <stdio.h>
#include <string.h>

#include "task.h"
#include "usart.h"
#include "../Application/App_Usartptint/app_usart.h"



void Delay_Test(void) {
    char string[100]="start\n";
    HAL_UART_Transmit(&huart6, (uint8_t *)string, strlen(string), HAL_MAX_DELAY);
    while (1){

        uint64_t time_delay_us = gx_GetUs();
        PERIODIC_START(DELAY_TEST,10)
        float time_delay_s = 1.0 * time_delay_us * 1.0e-6;
        sprintf(string,"%.3lf\n",time_delay_s);
        HAL_UART_Transmit(&huart6, (uint8_t *)string, strlen(string), HAL_MAX_DELAY);
        PERIODIC_END
    }
}