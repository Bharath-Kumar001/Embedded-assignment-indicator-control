#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"

// Hardware Configuration
#define LEFT_BUTTON_GPIO    GPIO_NUM_4
#define RIGHT_BUTTON_GPIO   GPIO_NUM_5
#define LEFT_LED_GPIO       GPIO_NUM_18
#define RIGHT_LED_GPIO      GPIO_NUM_19

#define UART_TXD_PIN        GPIO_NUM_17
#define UART_RXD_PIN        GPIO_NUM_16
#define UART_BAUD_RATE      115200

// Timing Constants
#define DEBOUNCE_DELAY_MS   50
#define LONG_PRESS_MS       1000
#define TOGGLE_INTERVAL_MS  300
#define SCHEDULER_INTERVAL_MS 100

// LEDC (PWM) Configuration
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_LEFT   LEDC_CHANNEL_0
#define LEDC_CHANNEL_RIGHT  LEDC_CHANNEL_1
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY      5000

typedef enum {
    INDICATOR_OFF,
    INDICATOR_LEFT,
    INDICATOR_RIGHT,
    INDICATOR_HAZARD
} indicator_state_t;

typedef struct {
    gpio_num_t pin;  // Changed from uint8_t to gpio_num_t
    bool pressed;
    bool long_press_detected;
    uint32_t press_start_time;
    bool state_changed;
} button_t;

// Global variables
static indicator_state_t current_state = INDICATOR_OFF;
static bool toggle_state = false;
static uint32_t last_toggle_time = 0;
static button_t left_button = {LEFT_BUTTON_GPIO, false, false, 0, false};
static button_t right_button = {RIGHT_BUTTON_GPIO, false, false, 0, false};
static TimerHandle_t scheduler_timer = NULL;

// Function prototypes
void init_hardware();
void init_uart();
void init_pwm();
void button_isr_handler(void* arg);
void check_button_press(button_t* button);
void process_indicator_events();
void update_leds();
void toggle_indicators();
void uart_log_event(const char* event);
void scheduler_callback(TimerHandle_t xTimer);

void setup() {
    init_hardware();
    
    // Create 100ms scheduler timer
    scheduler_timer = xTimerCreate(
        "Scheduler",
        pdMS_TO_TICKS(SCHEDULER_INTERVAL_MS),
        pdTRUE,
        (void*)0,
        scheduler_callback
    );
    xTimerStart(scheduler_timer, 0);
    
    uart_log_event("System initialized");
}
void loop(){

}
void init_hardware() {
    // Initialize GPIO for buttons
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LEFT_BUTTON_GPIO) | (1ULL << RIGHT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);
    
    // Install GPIO ISR service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(LEFT_BUTTON_GPIO, button_isr_handler, &left_button);
    gpio_isr_handler_add(RIGHT_BUTTON_GPIO, button_isr_handler, &right_button);
    
    // Initialize UART
    init_uart();
    
    // Initialize PWM for LEDs
    init_pwm();
}

void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void init_pwm() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    
    ledc_channel_config_t ledc_channel_left = {
        .gpio_num = LEFT_LED_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_LEFT,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_left);
    
    ledc_channel_config_t ledc_channel_right = {
        .gpio_num = RIGHT_LED_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_RIGHT,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel_right);
}

void button_isr_handler(void* arg) {
    button_t* button = (button_t*) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Debounce logic
    static uint32_t last_interrupt_time = 0;
    uint32_t interrupt_time = xTaskGetTickCountFromISR();
    
    if (interrupt_time - last_interrupt_time > pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)) {
        bool current_state = gpio_get_level(button->pin);
        
        if (current_state == 0) { // Button pressed (assuming active low)
            button->press_start_time = interrupt_time;
            button->pressed = true;
        } else { // Button released
            button->pressed = false;
            button->long_press_detected = false;
        }
        
        button->state_changed = true;
    }
    
    last_interrupt_time = interrupt_time;
    
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void check_button_press(button_t* button) {
    if (!button->state_changed) return;
    
    uint32_t current_time = xTaskGetTickCount();
    
    if (button->pressed && !button->long_press_detected) {
        if (current_time - button->press_start_time >= pdMS_TO_TICKS(LONG_PRESS_MS)) {
            button->long_press_detected = true;
            button->state_changed = false;
            
            // Process the long press
            if (left_button.long_press_detected && right_button.long_press_detected) {
                // Hazard light toggle
                if (current_state == INDICATOR_HAZARD) {
                    current_state = INDICATOR_OFF;
                    uart_log_event("Hazard lights deactivated");
                } else {
                    current_state = INDICATOR_HAZARD;
                    uart_log_event("Hazard lights activated");
                }
            } else if (button == &left_button) {
                // Left button long press
                if (current_state == INDICATOR_LEFT) {
                    current_state = INDICATOR_OFF;
                    uart_log_event("Left indicator deactivated");
                } else {
                    current_state = INDICATOR_LEFT;
                    uart_log_event("Left indicator activated");
                }
            } else if (button == &right_button) {
                // Right button long press
                if (current_state == INDICATOR_RIGHT) {
                    current_state = INDICATOR_OFF;
                    uart_log_event("Right indicator deactivated");
                } else {
                    current_state = INDICATOR_RIGHT;
                    uart_log_event("Right indicator activated");
                }
            }
            
            // Reset other button if only one was pressed
            if (button == &left_button) {
                right_button.long_press_detected = false;
                right_button.state_changed = false;
            } else if (button == &right_button) {
                left_button.long_press_detected = false;
                left_button.state_changed = false;
            }
        }
    }
    
    button->state_changed = false;
}

void process_indicator_events() {
    check_button_press(&left_button);
    check_button_press(&right_button);
    
    // Handle indicator toggle timing
    uint32_t current_time = xTaskGetTickCount();
    if (current_state != INDICATOR_OFF) {
        if (current_time - last_toggle_time >= pdMS_TO_TICKS(TOGGLE_INTERVAL_MS)) {
            toggle_indicators();
            last_toggle_time = current_time;
        }
    } else {
        // Turn off both LEDs when indicators are off
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT, 0);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT);
        toggle_state = false;
    }
}

void update_leds() {
    uint32_t duty = toggle_state ? (1 << LEDC_DUTY_RES) - 1 : 0;
    
    switch (current_state) {
        case INDICATOR_LEFT:
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT, duty);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT, 0);
            break;
        case INDICATOR_RIGHT:
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT, 0);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT, duty);
            break;
        case INDICATOR_HAZARD:
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT, duty);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT, duty);
            break;
        case INDICATOR_OFF:
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LEFT, 0);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT, 0);
            break;
    }
    
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LEFT);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RIGHT);
}

void toggle_indicators() {
    toggle_state = !toggle_state;
    update_leds();
    
    // Log toggle state
    char log_msg[64];
    snprintf(log_msg, sizeof(log_msg), "Toggle: %s %s", 
             toggle_state ? "ON" : "OFF",
             current_state == INDICATOR_LEFT ? "Left" :
             current_state == INDICATOR_RIGHT ? "Right" :
             current_state == INDICATOR_HAZARD ? "Hazard" : "Off");
    uart_log_event(log_msg);
}

void uart_log_event(const char* event) {
    char log_buffer[128];
    uint32_t timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    snprintf(log_buffer, sizeof(log_buffer), "[%lu ms] %s\n", timestamp, event);
    uart_write_bytes(UART_NUM_1, log_buffer, strlen(log_buffer));
}

void scheduler_callback(TimerHandle_t xTimer) {
    process_indicator_events();
}