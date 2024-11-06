#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <cmath>
#include <string>

class Module
{
public:
    Module() : TAG("Module") {}
    static void init();
    void log(const char *msg);

private:
    const std::string TAG;
};

void Module::log(const char *msg)
{
    ESP_LOGI(TAG.c_str(), "%s", msg);
}
