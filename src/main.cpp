#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define I2C_PORT i2c1
#define I2C_SDA_PIN 14
#define I2C_SCL_PIN 15
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

#define ACCEL_SENSITIVITY 16384.0f
#define GYRO_SENSITIVITY 131.0f
#define FILTER_ALPHA 0.96f

typedef struct {
    float pitch;
    float roll;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
} plot_data_t;

QueueHandle_t plot_queue;

/**
 * @brief FreeRTOS task to blink the onboard LED, indicating scheduler is running.
 */
void led_task(void *pvParameters) {
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    for (;;) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_put(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief FreeRTOS task to initialize, calibrate, and read the sensor, then
 * run the filter and send the combined data to a queue.
 */
void sensor_task(void *pvParameters) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    uint8_t buf[] = {MPU6050_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);

    long gx_sum = 0, gy_sum = 0, ax_sum = 0, ay_sum = 0, az_sum = 0;
    uint8_t buffer[14];
    uint8_t reg = MPU6050_ACCEL_XOUT_H;

    for (int i = 0; i < 500; i++) {
        i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
        i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);
        ax_sum += (int16_t)(buffer[0] << 8 | buffer[1]);
        ay_sum += (int16_t)(buffer[2] << 8 | buffer[3]);
        az_sum += (int16_t)(buffer[4] << 8 | buffer[5]);
        gx_sum += (int16_t)(buffer[8] << 8 | buffer[9]);
        gy_sum += (int16_t)(buffer[10] << 8 | buffer[11]);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    int16_t gyro_cal_x = gx_sum / 500;
    int16_t gyro_cal_y = gy_sum / 500;
    int16_t accel_cal_x = ax_sum / 500;
    int16_t accel_cal_y = ay_sum / 500;
    int16_t accel_cal_z = (az_sum / 500) - ACCEL_SENSITIVITY;

    float pitch = 0.0f, roll = 0.0f;
    absolute_time_t last_update_time = get_absolute_time();

    for (;;) {
        i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
        i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);

        int16_t raw_ax = (int16_t)(buffer[0] << 8 | buffer[1]);
        int16_t raw_ay = (int16_t)(buffer[2] << 8 | buffer[3]);
        int16_t raw_az = (int16_t)(buffer[4] << 8 | buffer[5]);
        int16_t raw_gx = (int16_t)(buffer[8] << 8 | buffer[9]);
        int16_t raw_gy = (int16_t)(buffer[10] << 8 | buffer[11]);
        int16_t raw_gz = (int16_t)(buffer[12] << 8 | buffer[13]);

        float ax = (float)(raw_ax - accel_cal_x);
        float ay = (float)(raw_ay - accel_cal_y);
        float az = (float)(raw_az - accel_cal_z);
        float gyro_x_dps = (float)(raw_gx - gyro_cal_x) / GYRO_SENSITIVITY;
        float gyro_y_dps = (float)(raw_gy - gyro_cal_y) / GYRO_SENSITIVITY;

        absolute_time_t now = get_absolute_time();
        float dt = (float)absolute_time_diff_us(last_update_time, now) / 1000000.0f;
        last_update_time = now;

        float accel_pitch = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) * (180.0f / M_PI);
        float accel_roll = atan2(-ax, sqrt(pow(ay, 2) + pow(az, 2))) * (180.0f / M_PI);

        pitch = FILTER_ALPHA * (pitch + gyro_y_dps * dt) + (1.0f - FILTER_ALPHA) * accel_pitch;
        roll = FILTER_ALPHA * (roll + gyro_x_dps * dt) + (1.0f - FILTER_ALPHA) * accel_roll;

        plot_data_t data_to_send = {
            .pitch = pitch,
            .roll = roll,
            .accel_x = raw_ax,
            .accel_y = raw_ay,
            .accel_z = raw_az,
            .gyro_x = raw_gx,
            .gyro_y = raw_gy,
            .gyro_z = raw_gz
        };
        xQueueSend(plot_queue, &data_to_send, 0);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief FreeRTOS task to receive all sensor data from the queue and print it.
 */
void logger_task(void *pvParameters) {
    plot_data_t received_data;
    for (;;) {
        if (xQueueReceive(plot_queue, &received_data, portMAX_DELAY) == pdPASS) {
            printf("%.2f,%.2f,%d,%d,%d,%d,%d,%d\n",
                   received_data.pitch, received_data.roll,
                   received_data.accel_x, received_data.accel_y, received_data.accel_z,
                   received_data.gyro_x, received_data.gyro_y, received_data.gyro_z);
        }
    }
}

/**
 * @brief Main application entry point.
 */
int main() {
    stdio_init_all();
    sleep_ms(2000);

    plot_queue = xQueueCreate(1, sizeof(plot_data_t));

    xTaskCreate(led_task, "LED_Task", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(sensor_task, "Sensor_Task", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(logger_task, "Logger_Task", 256, NULL, tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();

    while (true);
}