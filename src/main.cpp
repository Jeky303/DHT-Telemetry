#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_pm.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "common.hpp"

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/spi_common.h"

static const char *TAG = "AS5600_SD";

#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_PORT I2C_NUM_0
#define AS5600_I2C_ADDRESS 0x36
#define AS5600_REG_RAW_ANGLE 0x0C

// SD Card Configuration
#define MOUNT_POINT "/sdcard"
#define PIN_NUM_MISO 37
#define PIN_NUM_MOSI 35
#define PIN_NUM_CLK 36
#define PIN_NUM_CS GPIO_NUM_5

void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master{.clk_speed = I2C_MASTER_FREQ_HZ},
    };
    i2c_param_config(I2C_MASTER_PORT, &conf);
    i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

uint16_t read_as5600_angle()
{
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, AS5600_REG_RAW_ANGLE, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? (data[0] << 8) | data[1] : 0xFFFF;
}

esp_err_t sd_card_init(sdmmc_card_t **out_card)
{
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = (spi_host_device_t)host.slot;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, out_card);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount filesystem.");
        spi_bus_free((spi_host_device_t)host.slot);
    }
    return ret;
}

void log_to_sdcard(sdmmc_card_t *card, uint16_t angle, float delta_time)
{
    const char *file_path = MOUNT_POINT "/data.csv";
    FILE *f = fopen(file_path, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    fprintf(f, "%d,%.2f\n", angle, delta_time); // Write as CSV: angle, delta_time
    fclose(f);
    ESP_LOGI(TAG, "Data logged to SD card");
}

void task_read_encoder(void *arg)
{
    sdmmc_card_t *card;
    if (sd_card_init(&card) != ESP_OK)
    {
        ESP_LOGE(TAG, "SD card initialization failed");
        vTaskDelete(NULL);
        return;
    }

    int64_t previous_time = esp_timer_get_time();
    while (true)
    {
        int64_t current_time = esp_timer_get_time();
        float delta_time_ms = (current_time - previous_time) / 1000.0;
        previous_time = current_time;

        uint16_t angle = read_as5600_angle();
        if (angle != 0xFFFF)
        {
            ESP_LOGI(TAG, "Current Angle: %d, Time Delta: %.2f ms", angle, delta_time_ms);
            log_to_sdcard(card, angle, delta_time_ms);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    spi_bus_free(SDSPI_DEFAULT_HOST);
}

extern "C" void app_main()
{
    i2c_master_init();
    xTaskCreate(task_read_encoder, "ReadEncoderTask", 4096, NULL, 5, NULL);
}
