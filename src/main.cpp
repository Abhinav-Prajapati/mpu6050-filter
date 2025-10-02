#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// Use the default LED pin on the Pico
const uint LED_PIN = PICO_DEFAULT_LED_PIN;

// This is the function that will be executed by our FreeRTOS task
void blink_task(void *params) {
    // This is a one-time setup
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // This is the infinite loop for the task
    while (true) {
        gpio_put(LED_PIN, 1); // Turn the LED on
        vTaskDelay(500 / portTICK_PERIOD_MS); // Wait 500ms
        gpio_put(LED_PIN, 0); // Turn the LED off
        vTaskDelay(500 / portTICK_PERIOD_MS); // Wait 500ms
    }
}

int main() {
    // Initialize standard I/O for debugging (optional)
    stdio_init_all();

    // Create the blink task
    xTaskCreate(blink_task, "BlinkTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // The scheduler will never return, so this part is never reached
    while (true);
}