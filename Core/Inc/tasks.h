#ifndef MY_TASK_H
#define MY_TASK_H

#include "cmsis_os.h" // Include FreeRTOS API headers
#include "stm32l4xx_hal.h"


void measureTask(void *argument);
void presKalmanTask(void *arguments);
void saveTask(void *arguments);
void readTask(void *arguments);
void startSaving();
void startReading();

#endif // MY_TASK_H
