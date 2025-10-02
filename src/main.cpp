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
} angles_t;

QueueHandle_t angles_queue;

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
 * run the filter and send the results to a queue.
 */
void sensor_task(void *pvParameters) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Wake up MPU-6050
    uint8_t buf[] = {MPU6050_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);

    printf("Sensor Task: Calibrating... Keep sensor flat and motionless.\n");
    int num_readings = 500;
    long gx_sum = 0, gy_sum = 0, ax_sum = 0, ay_sum = 0, az_sum = 0;
    uint8_t buffer[14];
    uint8_t reg = MPU6050_ACCEL_XOUT_H;

    for (int i = 0; i < num_readings; i++) {
        i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
        i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);
        ax_sum += (int16_t)(buffer[0] << 8 | buffer[1]);
        ay_sum += (int16_t)(buffer[2] << 8 | buffer[3]);
        az_sum += (int16_t)(buffer[4] << 8 | buffer[5]);
        gx_sum += (int16_t)(buffer[8] << 8 | buffer[9]);
        gy_sum += (int16_t)(buffer[10] << 8 | buffer[11]);
        vTaskDelay(pdMS_TO_TICKS(2)); // Short delay between calibration readings
    }
    
    int16_t gyro_cal_x = gx_sum / num_readings;
    int16_t gyro_cal_y = gy_sum / num_readings;
    int16_t accel_cal_x = ax_sum / num_readings;
    int16_t accel_cal_y = ay_sum / num_readings;
    int16_t accel_cal_z = (az_sum / num_readings) - ACCEL_SENSITIVITY;
    printf("Sensor Task: Calibration complete.\n");

    float pitch = 0.0f, roll = 0.0f;
    absolute_time_t last_update_time = get_absolute_time();

    for (;;) {
        // Read raw data
        i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
        i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);

        // Apply calibration and convert to physical units
        float ax = (float)((int16_t)(buffer[0] << 8 | buffer[1]) - accel_cal_x);
        float ay = (float)((int16_t)(buffer[2] << 8 | buffer[3]) - accel_cal_y);
        float az = (float)((int16_t)(buffer[4] << 8 | buffer[5]) - accel_cal_z);
        float gyro_x_dps = (float)((int16_t)(buffer[8] << 8 | buffer[9]) - gyro_cal_x) / GYRO_SENSITIVITY;
        float gyro_y_dps = (float)((int16_t)(buffer[10] << 8 | buffer[11]) - gyro_cal_y) / GYRO_SENSITIVITY;

        // Calculate time delta
        absolute_time_t now = get_absolute_time();
        float dt = (float)absolute_time_diff_us(last_update_time, now) / 1000000.0f;
        last_update_time = now;

        // Calculate angles from accelerometer
        float accel_pitch = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) * (180.0f / M_PI);
        float accel_roll = atan2(-ax, sqrt(pow(ay, 2) + pow(az, 2))) * (180.0f / M_PI);

        // Complementary Filter
        pitch = FILTER_ALPHA * (pitch + gyro_y_dps * dt) + (1.0f - FILTER_ALPHA) * accel_pitch;
        roll = FILTER_ALPHA * (roll + gyro_x_dps * dt) + (1.0f - FILTER_ALPHA) * accel_roll;

        angles_t current_angles = { .pitch = pitch, .roll = roll };
        
        // Send the filtered data to the queue
        xQueueSend(angles_queue, &current_angles, 0); // Use 0 timeout as we overwrite data anyway

        vTaskDelay(pdMS_TO_TICKS(20)); // ~50Hz update rate
    }
}

/**
 * @brief FreeRTOS task to receive filtered data from the queue and print it.
 */
void logger_task(void *pvParameters) {
    angles_t received_angles;
    for (;;) {
        if (xQueueReceive(angles_queue, &received_angles, portMAX_DELAY) == pdPASS) {
            printf("Pitch: %.2f, Roll: %.2f\n", received_angles.pitch, received_angles.roll);
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    angles_queue = xQueueCreate(1, sizeof(angles_t));

    xTaskCreate(led_task, "LED_Task", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(sensor_task, "Sensor_Task", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(logger_task, "Logger_Task", 256, NULL, tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();

    while (true);
}