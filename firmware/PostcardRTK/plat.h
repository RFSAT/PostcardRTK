// plat.h - small ESP32 equivalents of the Raspberry Pi Pico SDK calls the shared modules use
#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct mutex_t
{
    SemaphoreHandle_t h = nullptr;
};
inline void mutex_init(mutex_t *m) { m->h = xSemaphoreCreateMutex(); }
inline void mutex_enter_blocking(mutex_t *m) { xSemaphoreTake(m->h, portMAX_DELAY); }
inline void mutex_exit(mutex_t *m) { xSemaphoreGive(m->h); }
