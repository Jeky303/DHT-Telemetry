#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "math.h"

#define I2C_MASTER_SCL_IO 9       // Pin SCL per ESP32-S3 DevkitC
#define I2C_MASTER_SDA_IO 8       // Pin SDA per ESP32-S3 DevkitC
#define I2C_MASTER_FREQ_HZ 100000 // Frequenza I2C
#define I2C_MASTER_PORT I2C_NUM_0 // Numero del bus I2C
#define AS5600_I2C_ADDRESS 0x36   // Indirizzo I2C dell'AS5600

#define AS5600_REG_RAW_ANGLE 0x0C // Registro dell'angolo grezzo

// Pulley specifications
const float raggioPuleggia = 16.5;                               // Pulley radius in mm
const float circonferenzaPuleggia = 2.0 * M_PI * raggioPuleggia; // Pulley circumference

uint16_t angoloPrecedente = 0;
float distanzaTotale = 0;

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

void calculate_distance(uint16_t angoloAttuale)
{
    int16_t differenzaAngolare = (int16_t)angoloPrecedente - (int16_t)angoloAttuale;

    // Handle multi-turn angles
    if (differenzaAngolare > 2048)
    {
        differenzaAngolare -= 4096;
    }
    else if (differenzaAngolare < -2048)
    {
        differenzaAngolare += 4096;
    }

    // Calculate partial and total linear distance based on angle difference
    float distanzaParziale = differenzaAngolare * (circonferenzaPuleggia / 4096.0);
    distanzaTotale += distanzaParziale;

    printf("Distance Traveled: %.2f mm\n", distanzaTotale);

    // Update previous angle
    angoloPrecedente = angoloAttuale;
}

uint16_t read_as5600_angle()
{
    uint8_t data[2];
    esp_err_t ret;

    // Crea un comando I2C per leggere l'angolo grezzo
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, AS5600_REG_RAW_ANGLE, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK)
    {
        return (data[0] << 8) | data[1]; // Combina MSB e LSB per ottenere l'angolo
    }
    else
    {
        printf("Errore di lettura I2C: %s\n", esp_err_to_name(ret));
        return 0xFFFF; // Valore di errore
    }
}

void task_read_encoder()
{
    angoloPrecedente = read_as5600_angle(); // Initial angle for reference

    while (true)
    {
        uint16_t angoloAttuale = read_as5600_angle();
        if (angoloAttuale != 0xFFFF)
        { // Check for error
            calculate_distance(angoloAttuale);
        }
    }
}

extern "C" void app_main()
{
    // Inizializza I2C
    i2c_master_init();

    // Crea il task per la lettura dell'encoder
    xTaskCreate((TaskFunction_t)task_read_encoder, "ReadEncoder", 2048, NULL, 5, NULL);
}
