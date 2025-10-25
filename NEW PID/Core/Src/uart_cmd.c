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
    uint16_t current_message_end_pos = uart_rx_write_pos;

    uint16_t bytes_received;
    if (current_message_end_pos >= read_pos_tracker) {
        bytes_received = current_message_end_pos - read_pos_tracker;
    } else {
        bytes_received = UART_RX_BUFFER_SIZE - read_pos_tracker + current_message_end_pos;
    }

    uart_new_data_available = false;
    if (bytes_received == 0) return;

    char temp_command_buffer[UART_RX_BUFFER_SIZE + 1];
    if (current_message_end_pos >= read_pos_tracker) {
        memcpy(temp_command_buffer, (char*)&uart_rx_buffer[read_pos_tracker], bytes_received);
    } else {
        memcpy(temp_command_buffer, (char*)&uart_rx_buffer[read_pos_tracker], UART_RX_BUFFER_SIZE - read_pos_tracker);
        memcpy(temp_command_buffer + (UART_RX_BUFFER_SIZE - read_pos_tracker), (char*)uart_rx_buffer, current_message_end_pos);
    }
    temp_command_buffer[bytes_received] = '\0';
    read_pos_tracker = current_message_end_pos;

    // Trim whitespace and line endings
    size_t len = strlen(temp_command_buffer);
    while (len > 0 && (temp_command_buffer[len-1] == '\n' || temp_command_buffer[len-1] == '\r' ||
                       temp_command_buffer[len-1] == ' ' || temp_command_buffer[len-1] == '\t')) {
        temp_command_buffer[len-1] = '\0';
        len--;
    }

    // **REDUCED DEBUG OUTPUT** - Only essential messages
    // Debug_Send_DMA("CMD: Received '%s'\r\n", temp_command_buffer);

    // Check for special "0.00" command to zero ALL motors
    if (strcmp(temp_command_buffer, "0.00") == 0) {
        for (unsigned int i = 0; i < MOTORS_COUNT; i++) {
            pid_target_speed_rpms[i] = 0.0f;
        }
        Debug_Send_DMA("CMD: Set ALL motors to 0 RPM\r\n");
        return;
    }

    // Parse motor command format: "<motor_index>.<rpm_value>"
    unsigned int motor_idx;
    float new_value;

    int parsed_items = sscanf(temp_command_buffer, "%u.%f", &motor_idx, &new_value);

    if (parsed_items == 2) {
        if (motor_idx < MOTORS_COUNT) {
            #define MAX_ALLOWED_VALUE 10000.0f
            #define MIN_ALLOWED_VALUE -10000.0f

            if (new_value >= MIN_ALLOWED_VALUE && new_value <= MAX_ALLOWED_VALUE) {
                pid_target_speed_rpms[motor_idx] = new_value;
                Debug_Send_DMA("CMD: M%u -> %.0f RPM\r\n", motor_idx, new_value);
            } else {
                Debug_Send_DMA("CMD: Value %.0f out of range\r\n", new_value);
            }
        } else {
            Debug_Send_DMA("CMD: Invalid motor %u\r\n", motor_idx);
        }
    } else {
        Debug_Send_DMA("CMD: Bad format. Use: <motor>.<rpm>\r\n");
    }
}

