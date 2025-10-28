#include "uart_cmd.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "dshot.h"

// RX buffer
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile uint16_t uart_rx_write_pos = 0;
volatile bool uart_new_data_available = false;

// TX handling - use DMA but different approach
volatile bool uart_tx_busy = false;
char uart_tx_buffer[128];

// External symbols
extern UART_HandleTypeDef huart1;
extern volatile float pid_target_speed_rpms[MOTORS_COUNT];
extern uint16_t pwm_targets[4];


void UART_CMD_Init(UART_HandleTypeDef *huart) {
    HAL_UART_Receive_DMA(huart, uart_rx_buffer, UART_RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

// Non-blocking UART transmit with timeout protection
void Debug_Send_DMA(const char* format, ...) {
    // If UART is busy, skip this message to avoid blocking
    if (uart_tx_busy) {
        return;
    }

    va_list args;
    va_start(args, format);
    int len = vsnprintf(uart_tx_buffer, sizeof(uart_tx_buffer), format, args);
    va_end(args);

    if (len > 0 && len < (int)sizeof(uart_tx_buffer)) {
        uart_tx_busy = true;

        // Use HAL_UART_Transmit with reasonable timeout
        // This will block briefly but not disrupt DShot timing too much
        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buffer, len, 10);

        if (status != HAL_OK) {
            // Transmission failed or timed out
            uart_tx_busy = false;
        } else {
            uart_tx_busy = false;
        }
    }
}

// Alternative: Use DMA but with interrupt-based completion
void Debug_Send_DMA_Interrupt(const char* format, ...) {
    static char dma_tx_buffer[128];

    va_list args;
    va_start(args, format);
    int len = vsnprintf(dma_tx_buffer, sizeof(dma_tx_buffer), format, args);
    va_end(args);

    if (len > 0) {
        HAL_UART_Transmit_IT(&huart1, (uint8_t*)dma_tx_buffer, len);
    }
}
void process_uart_command(void) {
    static uint16_t read_pos_tracker = 0;
    uint16_t current_end_pos = uart_rx_write_pos;

    uint16_t bytes_received;
    if (current_end_pos >= read_pos_tracker)
        bytes_received = current_end_pos - read_pos_tracker;
    else
        bytes_received = UART_RX_BUFFER_SIZE - read_pos_tracker + current_end_pos;

    uart_new_data_available = false;
    if (bytes_received == 0) return;

    char temp_buffer[UART_RX_BUFFER_SIZE + 1];
    if (current_end_pos >= read_pos_tracker) {
        memcpy(temp_buffer, &uart_rx_buffer[read_pos_tracker], bytes_received);
    } else {
        memcpy(temp_buffer, &uart_rx_buffer[read_pos_tracker], UART_RX_BUFFER_SIZE - read_pos_tracker);
        memcpy(temp_buffer + (UART_RX_BUFFER_SIZE - read_pos_tracker), uart_rx_buffer, current_end_pos);
    }
    temp_buffer[bytes_received] = '\0';
    read_pos_tracker = current_end_pos;

    char *line = strtok(temp_buffer, "\r\n");
    while (line != NULL) {
        while (*line == ' ' || *line == '\t') line++;
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\t')) line[--len] = '\0';
        if (len == 0) { line = strtok(NULL, "\r\n"); continue; }

        if (strcmp(line, "0.00") == 0) {
            for (unsigned int i = 0; i < MOTORS_COUNT; i++) pid_target_speed_rpms[i] = 0.0f;
            pwm_targets[0] = pwm_targets[1] = pwm_targets[2] = pwm_targets[3] = 1500; // neutral
            Debug_Send_DMA("CMD: Reset all motors\r\n");
        } else {
            unsigned int motor_idx;
            float new_value;
            int parsed = sscanf(line, "%u.%f", &motor_idx, &new_value);

            if (parsed == 2) {
                // --- Handle DShot motors (0–9)
                if (motor_idx < MOTORS_COUNT) {
                    if (fabsf(new_value) <= 10000.0f) {
                        pid_target_speed_rpms[motor_idx] = new_value;
                        Debug_Send_DMA("CMD: M%u -> %.0f RPM\r\n", motor_idx, new_value);
                    }
                }
                // --- Handle PWM motors (10–13)
                else if (motor_idx >= 10 && motor_idx <= 13) {
                    if (new_value >= 500 && new_value <= 2500) {
                        pwm_targets[motor_idx - 10] = (uint16_t)new_value;
                        Debug_Send_DMA("CMD: PWM%u -> %.0f us\r\n", motor_idx - 9, new_value);
                    } else {
                        Debug_Send_DMA("CMD: PWM value %.0f out of range\r\n", new_value);
                    }
                } else {
                    Debug_Send_DMA("CMD: Invalid motor index %u\r\n", motor_idx);
                }
            } else {
                Debug_Send_DMA("CMD: Bad format. Use <motor>.<value>\r\n");
            }
        }
        line = strtok(NULL, "\r\n");
    }
}
