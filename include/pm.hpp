#pragma once
#include "esp_pm.h"
#include "common.hpp"

class DHt_pm : public Module
{
private:
    static const std::string TAG;

public:
    void init(); // Declare init function inside the class
};

// Define the static TAG member
const std::string DHt_pm::TAG = "DHt_pm";

// Define the init function outside the class
void DHt_pm::init()
{
    ESP_LOGI(TAG.c_str(), "Initializing PM module");
    esp_pm_config_t config = {
        .max_freq_mhz = 240, // Frequenza massima
        .min_freq_mhz = 80,  // Frequenza minima
        .light_sleep_enable = false};
    esp_pm_configure(&config);
}
