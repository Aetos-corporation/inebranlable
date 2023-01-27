/*
 * trace.c
 *
 *  Created on: Jan 9, 2023
 *      Author: Bastien
 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "usart.h"

void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}

void PRINT(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}
