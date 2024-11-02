#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "stdio.h"
#define LED1 2
#define LED2 3

void task_led()
{
    while (true)
    { /*
        gpio_set_level(LED, 1);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        gpio_set_level(LED, 0);
        vTaskDelay(5000 / portTICK_PERIOD_MS)*/
        printf("LED diobon\n");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
void app_main()
{
    gpio_config_t io_config = {

    };
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pin_bit_mask = (1ULL << LED1) || (1ULL << LED2);
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);
    xTaskCreate(&task_led, NULL, 2048, NULL, 1, NULL);
}
