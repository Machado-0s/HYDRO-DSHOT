/*
 * uart_cmd.c
 *
 * Created on: Aug 19, 2025
 * Author: Chard Ethern
 */
// Core/Src/uart_cmd.c

#include "uart_cmd.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "dshot.h" // Include dshot.h to get MOTORS_COUNT

// RX buffer
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile uint16_t uart_rx_write_pos = 0;
volatile bool uart_new_data_available = false;

// External symbols
extern UART_HandleTypeDef huart1;
extern volatile bool uart_tx_ready;
char dma_uart_buffer[128];

// Extern declaration for pid_target_speed_rpms from dshot.c
extern volatile float pid_target_speed_rpms[MOTORS_COUNT];


void UART_CMD_Init(UART_HandleTypeDef *huart) {
    HAL_UART_Receive_DMA(huart, uart_rx_buffer, UART_RX_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
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

    size_t len = strlen(temp_command_buffer);
    while (len > 0 && (temp_command_buffer[len-1] == '\n' || temp_command_buffer[len-1] == '\r')) {
        temp_command_buffer[len-1] = '\0';
        len--;
    }

    // Debug_Send_DMA("PARSER_DEBUG: Raw RX Buffer: '%s' (Len: %u)\r\n",
    //              temp_command_buffer, (unsigned int)len);

    // Check for special "0.00" command to zero ALL motors
    if (strcmp(temp_command_buffer, "0.00") == 0) {
        for (unsigned int i = 0; i < MOTORS_COUNT; i++) {
            pid_target_speed_rpms[i] = 0.0f;
        }
        Debug_Send_DMA("CMD: Set ALL motors to 0 RPM (special command)\r\n");
        return;
    }

    unsigned int motor_idx;
    float new_value;
    // Use sscanf to parse "<idx>.<rpm>" format
    int parsed_items = sscanf(temp_command_buffer, "%u.%f", &motor_idx, &new_value);

    if (parsed_items == 2) { // Successfully parsed both motor index and value
        if (motor_idx < MOTORS_COUNT) { // Validate motor index
            // Define allowed range for bi-directional control
            #define MAX_ALLOWED_VALUE 10000.0f // Max forward RPM
            #define MIN_ALLOWED_VALUE -10000.0f // Max reverse RPM (negative)

            if (new_value >= MIN_ALLOWED_VALUE && new_value <= MAX_ALLOWED_VALUE) {
                // Update the specific motor's target RPM
                pid_target_speed_rpms[motor_idx] = new_value;
                Debug_Send_DMA("CMD: Set M%u Target RPM to %.0f\r\n", motor_idx, new_value);
            } else {
                Debug_Send_DMA("CMD: Invalid value (%.0f) for M%u. Range %.0f-%.0f.\r\n", new_value, motor_idx, MIN_ALLOWED_VALUE, MAX_ALLOWED_VALUE);
            }
        } else {
            Debug_Send_DMA("CMD: Invalid Motor Index %u. Max allowed is %u.\r\n", motor_idx, MOTORS_COUNT - 1);
        }
    } else {
        Debug_Send_DMA("CMD: Invalid command format. Expected '<idx>.<rpm>'. Received: '%s'\r\n", temp_command_buffer);
    }
}

void Debug_Send_DMA(const char* format, ...) {
    if (!uart_tx_ready) return;
    va_list args;
    va_start(args, format);
    int len = vsnprintf(dma_uart_buffer, sizeof(dma_uart_buffer), format, args);
    va_end(args);
    if (len > 0) {
        uart_tx_ready = false;
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dma_uart_buffer, len);
    }
}
