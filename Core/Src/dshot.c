#include "dshot.h"
#include <main.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "uart_cmd.h"

static uint16_t last_sent_dshot_command = 0; // Initialize to 0 (Motor Stop or Disarmed)

float pid_integral = 0;
float pid_last_error = 0;
float pid_filtered_error = 0;
volatile float pid_target_speed_rpm = 2000.0f;

// PID Target RPMs - Array to hold target RPM for each motor
volatile float pid_target_speed_rpms[MOTORS_COUNT]; // DEFINITION: Memory allocated here

extern UART_HandleTypeDef huart1;


//char dma_uart_buffer[128];

uint16_t motor_command_dshot_value = 0;
volatile bool telemetry_data_ready_flag = false;
uint16_t motor_values[MOTORS_COUNT] = {0};
volatile uint32_t dshot_bb_buffer_10[DSHOT_BB_BUFFER_LENGTH] __attribute__((aligned(4)));
volatile uint8_t telemetry_done_flag = 0;

volatile uint32_t dshot_bb_buffer_1_4_r[(int)(31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
MotorTelemetry_t motor_telemetry_data[MOTORS_COUNT];

volatile bool request_voltage_next = false;
bool bdshot_reception_1 = true;

const uint8_t motor_gpio_bit_positions[MOTORS_COUNT] = {
    0, 1, 2, 4, 5, 6, 7, 8, 11, 12
};

static uint32_t get_all_motors_gpio_mask() {
    uint32_t mask = 0;
    for (uint8_t m = 0; m < MOTORS_COUNT; m++) {
        mask |= (1 << motor_gpio_bit_positions[m]);
    }
    return mask;
}

void preset_bb_Dshot_buffers(void) {
    memset((void*)dshot_bb_buffer_10, 0, sizeof(dshot_bb_buffer_10));
    uint32_t all_motors_mask = get_all_motors_gpio_mask();

    dshot_bb_buffer_10[0] = all_motors_mask;

    for (uint16_t bit = 0; bit < DSHOT_BB_BUFFER_BITS; bit++) {
        uint16_t idx_base = (bit * DSHOT_BB_FRAME_SECTIONS)+1;
        uint16_t idx_end = idx_base + DSHOT_BB_1_LENGTH;

        dshot_bb_buffer_10[idx_base] |= (all_motors_mask << 16); // LOW start
        if (idx_end < DSHOT_BB_BUFFER_LENGTH) {
            dshot_bb_buffer_10[idx_end] |= all_motors_mask;       // HIGH restore
        }
    }

    // Ensure final output is HIGH (idle)
    dshot_bb_buffer_10[DSHOT_BB_BUFFER_LENGTH - 1] |= all_motors_mask;
}

static void fill_bb_Dshot_buffers(const uint16_t motor_packets[MOTORS_COUNT]) {
    preset_bb_Dshot_buffers();

    for (uint8_t bit = 0; bit < BITS_PER_FRAME; bit++) {
        uint16_t idx_base = bit * DSHOT_BB_FRAME_SECTIONS;
        uint8_t bit_pos = 15 - bit;
        uint16_t idx_one = idx_base + DSHOT_BB_1_LENGTH;
        uint16_t idx_zero = idx_base + DSHOT_BB_0_LENGTH;

        for (uint8_t m = 0; m < MOTORS_COUNT; m++) {
            uint32_t pin_mask = (1 << motor_gpio_bit_positions[m]);
            bool is_one = (motor_packets[m] >> bit_pos) & 1;

            if (!is_one) {
                if (idx_one < DSHOT_BB_BUFFER_LENGTH)
                    dshot_bb_buffer_10[idx_one] &= ~pin_mask;     // prevent early high
                if (idx_zero < DSHOT_BB_BUFFER_LENGTH)
                    dshot_bb_buffer_10[idx_zero] |= pin_mask;     // rise later for '0'
            }
        }
    }
}

uint16_t prepare_Dshot_package(uint16_t value, bool telemetry) {
    value = (value << 1) | (telemetry ? 1 : 0);
    uint16_t crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
    return (value << 4) | crc;
}


void update_motors_Tx_Only(void) {
    GPIOA->BSRR = get_all_motors_gpio_mask();

    fill_bb_Dshot_buffers(motor_values);
    bdshot_reception_1 = true;

    // Set GPIOs to output mode
    GPIOA->MODER |= (
        GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 |
        GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 |
        GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 |
        GPIO_MODER_MODER12_0
    );
    GPIOA->MODER &= ~(
        GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 |
        GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 |
        GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER11_1 |
        GPIO_MODER_MODER12_1
    );

    // Output settings
    GPIOA->OTYPER &= ~(
        GPIO_OTYPER_OT0  | GPIO_OTYPER_OT1  | GPIO_OTYPER_OT2  |
        GPIO_OTYPER_OT4  | GPIO_OTYPER_OT5  | GPIO_OTYPER_OT6  |
        GPIO_OTYPER_OT7  | GPIO_OTYPER_OT8  | GPIO_OTYPER_OT11 |
        GPIO_OTYPER_OT12
    );
    GPIOA->PUPDR |= (
        GPIO_PUPDR_PUPDR0_0  | GPIO_PUPDR_PUPDR1_0  | GPIO_PUPDR_PUPDR2_0  |
        GPIO_PUPDR_PUPDR4_0  | GPIO_PUPDR_PUPDR5_0  | GPIO_PUPDR_PUPDR6_0  |
        GPIO_PUPDR_PUPDR7_0  | GPIO_PUPDR_PUPDR8_0  | GPIO_PUPDR_PUPDR11_0 |
        GPIO_PUPDR_PUPDR12_0
    );
    GPIOA->PUPDR &= ~(
        GPIO_PUPDR_PUPDR0_1  | GPIO_PUPDR_PUPDR1_1  | GPIO_PUPDR_PUPDR2_1  |
        GPIO_PUPDR_PUPDR4_1  | GPIO_PUPDR_PUPDR5_1  | GPIO_PUPDR_PUPDR6_1  |
        GPIO_PUPDR_PUPDR7_1  | GPIO_PUPDR_PUPDR8_1  | GPIO_PUPDR_PUPDR11_1 |
        GPIO_PUPDR_PUPDR12_1
    );
    GPIOA->OSPEEDR |= (
        GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1 | GPIO_OSPEEDER_OSPEEDR2 |
        GPIO_OSPEEDER_OSPEEDR4 | GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 |
        GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR11 |
        GPIO_OSPEEDER_OSPEEDR12
    );

    // --- CRITICAL SYNCHRONIZATION FOR FIRST BIT ---

        // 5. Cleanly stop TIM1 and DMA stream
        TIM1->CR1 &= ~TIM_CR1_CEN;       // Stop TIM1
        DMA2_Stream5->CR &= ~DMA_SxCR_EN; // Disable DMA
        while (DMA2_Stream5->CR & DMA_SxCR_EN) { __NOP(); } // Wait for DMA to fully disable

        // Clear any pending DMA interrupt flags for stream5
        DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                      DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;

        // 6. Configure DMA for TX (Memory-to-Peripheral)
        DMA2_Stream5->PAR = (uint32_t)(&GPIOA->BSRR); // Peripheral address is BSRR
        DMA2_Stream5->M0AR = (uint32_t)(dshot_bb_buffer_10); // Source is your TX buffer
        DMA2_Stream5->NDTR = DSHOT_BB_BUFFER_LENGTH; // Number of transfers

        // Re-assign CR completely to ensure all bits are set correctly for TX
         //This atomic write is important to avoid intermediate states.
        uint32_t cr_val = (6 << DMA_SxCR_CHSEL_Pos)      // DMA Channel 6 for TIM1_UP (verify this is correct for your setup)
                        | DMA_SxCR_PL                    // Keep same priority (e.g., high)
                        | DMA_SxCR_MSIZE_1               // Memory size = 32-bit (word aligned)
                        | DMA_SxCR_PSIZE_1               // Peripheral size = 32-bit (for BSRR)
                         | DMA_SxCR_MINC                  // Memory increment enabled
                        | DMA_SxCR_DIR_0                 // Direction: Memory-to-Peripheral (00)
                        | DMA_SxCR_TCIE;                // Enable transfer complete interrupt

       DMA2_Stream5->CR = cr_val; // Write the compiled CR value

       TIM1->ARR = DSHOT_BB_SECTION_LENGTH - 1; // <--- ENSURE THIS IS EXPLICITLY SET FOR TX


        // 7. Reset TIM1's counter and clear update flag
        TIM1->EGR |= TIM_EGR_UG;   // Generate Update event (resets CNT, PSC, ARR)
        TIM1->CNT = 0;             // Ensure counter is explicitly at 0

        // 8. Ensure all memory operations are completed before enabling peripherals
        __DSB(); // Data Synchronization Barrier
        __ISB(); // Instruction Synchronization Barrier

        // 9. Enable both TIM1 and DMA with precise timing
        // Start TIM1 first, then enable DMA to ensure TIM1 is ready to generate the first trigger
        TIM1->CR1 |= TIM_CR1_CEN;        // Enable TIM1
        DMA2_Stream5->CR |= DMA_SxCR_EN; // Enable DMA to begin receiving triggers from TIM1

}

void setup_Dshot_Tx_Only(void) {

    __HAL_RCC_GPIOA_CLK_ENABLE();
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_URS;
    TIM1->PSC = 0;
    TIM1->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;
    TIM1->CCR1 = (DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS);///2;
    TIM1->CR2 = TIM_CR2_MMS_1;
    TIM1->DIER |= TIM_DIER_UDE;
    TIM1->EGR |= TIM_EGR_UG;
    TIM1->SR &= ~TIM_SR_UIF;

    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream5->CR & DMA_SxCR_EN);
    DMA2->HIFCR = 0xFFFFFFFF;

    DMA2_Stream5->CR = (6 << DMA_SxCR_CHSEL_Pos) |
                       DMA_SxCR_PL |
                       DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 |
                       DMA_SxCR_MINC | DMA_SxCR_TCIE;

    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    NVIC_SetPriority(DMA2_Stream5_IRQn, 3);
}


 static inline void short_delay_us(uint32_t us)
 {
     volatile uint32_t count = us * (84); // Roughly 1 cycle per loop at 84 MHz
     while (count--) {
         __NOP();
     }
 }

 // Helper: arm DMA + TIM1 to capture GPIOA->IDR into dshot_bb_buffer_1_4_r
 static inline void arm_bdshot_rx_capture(void)
 {
     const uint32_t pins_mask =
         (GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 |
          GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 |
          GPIO_MODER_MODER7 | GPIO_MODER_MODER8 | GPIO_MODER_MODER11 |
          GPIO_MODER_MODER12);

     const uint32_t pupdr_clear_mask =
         ((3U << (0*2)) | (3U << (1*2)) | (3U << (2*2)) |
          (3U << (4*2)) | (3U << (5*2)) | (3U << (6*2)) |
          (3U << (7*2)) | (3U << (8*2)) | (3U << (11*2)) |
          (3U << (12*2)));

     const uint32_t pupdr_pullup_mask =
         ((1U << (0*2)) | (1U << (1*2)) | (1U << (2*2)) |
          (1U << (4*2)) | (1U << (5*2)) | (1U << (6*2)) |
          (1U << (7*2)) | (1U << (8*2)) | (1U << (11*2)) |
          (1U << (12*2)));

     // 1) Set pins to input mode (00)
     GPIOA->MODER &= ~pins_mask;

     // 2) Configure pull-ups atomically: clear then set
     GPIOA->PUPDR &= ~pupdr_clear_mask;
     GPIOA->PUPDR |= pupdr_pullup_mask;

     uint32_t sample_period =  (DSHOT_BB_FRAME_LENGTH * DSHOT_MODE) /(BDSHOT_RESPONSE_BITRATE * BDSHOT_RESPONSE_OVERSAMPLING);

     if (sample_period == 0) sample_period = 1;
     TIM1->CR1 &= ~TIM_CR1_CEN;
     TIM1->ARR = sample_period - 1;
     TIM1->CCR1 = sample_period/2  ; // small safe value, not critical for DMA TRGO if CR2 MMS is set
     TIM1->EGR |= TIM_EGR_UG; // update registers

     // 4) Configure DMA2 Stream5 for peripheral->memory (GPIOA->IDR -> buffer)
     // Disable stream before reconfiguring
     DMA2_Stream5->CR &= ~DMA_SxCR_EN;
     while (DMA2_Stream5->CR & DMA_SxCR_EN) { __NOP(); }

     // Clear pending flags for stream5
     DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                   DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;

     // Peripheral address = input data register
     DMA2_Stream5->PAR = (uint32_t)(&GPIOA->IDR);

     // Memory address = your rx buffer (ensure buffer is word aligned)
     DMA2_Stream5->M0AR = (uint32_t)dshot_bb_buffer_1_4_r;

     // NDTR = desired sample count (make sure this fits)
     uint32_t ndtr = (uint32_t)((31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING);
     if (ndtr > 0xFFFF) ndtr = 0xFFFF;
     DMA2_Stream5->NDTR = ndtr;

     // CR: channel select, medium/high priority, peripheral size = 32-bit (PSIZE=10), memory size = 32-bit (MSIZE=10),
     // MN_INC = memory increment, DIR = Peripheral-to-memory (00 = P2M, actually on F4: DIR bits 0..1 = 00: peripheral-to-memory),
     // TCIE = transfer complete interrupt enabled.
     // Build CR value based on earlier configuration in setup_Dshot_Tx_Only:
     uint32_t cr_val = (6 << DMA_SxCR_CHSEL_Pos)      // channel 6
                     | DMA_SxCR_PL                    // keep same priority
                     | DMA_SxCR_MSIZE_1               // memory size = 32-bit (MSIZE = 10)
                     | DMA_SxCR_PSIZE_1               // peripheral size = 32-bit
                     | DMA_SxCR_MINC                  // memory increment
                     /* DIR = 00 for peripheral-to-memory so leave DIR bits cleared */
                     | DMA_SxCR_TCIE;                // enable transfer complete interrupt

     DMA2_Stream5->CR = cr_val;

     __DSB(); __ISB(); // ensure memory ops complete before enabling DMA

     // 5) Enable DMA stream (TCIE already set)
     DMA2_Stream5->CR |= DMA_SxCR_EN;

     // 6) Reset and start TIM1 so DMA requests begin immediately
     TIM1->CNT = 0;
     // Ensure TIM1 TRGO is configured to generate DMA requests (setup_Dshot_Tx_Only used CR2 MMS_1 earlier)
     TIM1->CR1 |= TIM_CR1_CEN;
 }

 // IRQ handler: stream5 used for both TX and RX transitions: TC toggles bdshot_reception_1
 void DMA2_Stream5_IRQHandler(void)
 {
     // Transfer complete for Stream5?
     if (DMA2->HISR & DMA_HISR_TCIF5) {
         // Clear the flag
         DMA2->HIFCR |= DMA_HIFCR_CTCIF5;

         if (bdshot_reception_1) {
             // We just finished the TX DMA. Immediately arm RX capture (no long loops).
             // This function will switch to input, enable pull-ups, configure DMA and start TIM1.
             arm_bdshot_rx_capture();

             // After arming RX we flip the state so next TC means RX complete.
             bdshot_reception_1 = false;

         } else {
             // We just finished RX capture - stop TIM1 and DMA, and mark telemetry ready.
             // Disable TIM1
             TIM1->CR1 &= ~TIM_CR1_CEN;

             // Disable DMA stream cleanly
             DMA2_Stream5->CR &= ~DMA_SxCR_EN;
             while (DMA2_Stream5->CR & DMA_SxCR_EN) { __NOP(); }

             // Clear any remaining flags for stream5
             DMA2->HIFCR |= DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                            DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;

             // Optionally: set pins back to output mode now or leave for next TX
             // (do not re-enable outputs here if you plan to start TX immediately from same ISR context)
             telemetry_done_flag = 1;

             // Next round, expect TX (so reset flag)
             bdshot_reception_1 = true;
         }
     }

     // Half-transfer / transfer error / direct mode error / transfer error housekeeping:
     if (DMA2->HISR & DMA_HISR_HTIF5) {
         DMA2->HIFCR |= DMA_HIFCR_CHTIF5;
     }
     if (DMA2->HISR & DMA_HISR_TEIF5) {
         DMA2->HIFCR |= DMA_HIFCR_CTEIF5;
     }
     if (DMA2->HISR & DMA_HISR_DMEIF5) {
         DMA2->HIFCR |= DMA_HIFCR_CDMEIF5;
     }
     if (DMA2->HISR & DMA_HISR_FEIF5) {
         DMA2->HIFCR |= DMA_HIFCR_CFEIF5;
     }
 }



// --- NEW GCR DECODING LOGIC ---
#define iv 0xFFFFFFFF
static const uint32_t GCR_table[32] = {
    iv, iv, iv, iv, iv, iv, iv, iv, iv, 9, 10, 11, iv, 13, 14, 15,
    iv, iv, 2, 3, iv, 5, 6, 7, iv, 0, 8, 1, iv, 4, 12, iv };


 uint32_t get_BDshot_response(uint32_t raw_buffer[], const uint8_t motor_shift){
    uint32_t* buffer_end = raw_buffer + 31 * BDSHOT_RESPONSE_BITRATE / 1000 * BDSHOT_RESPONSE_OVERSAMPLING;
    while (raw_buffer < buffer_end)
    {
        if (__builtin_expect(!(*raw_buffer++ & 1 << motor_shift), 0) ||
            __builtin_expect(!(*raw_buffer++ & 1 << motor_shift), 0) ||
            __builtin_expect(!(*raw_buffer++ & 1 << motor_shift), 0) ||
            __builtin_expect(!(*raw_buffer++ & 1 << motor_shift), 0))
        {
            uint32_t* buffer_previous = raw_buffer - 1;
            buffer_end = raw_buffer + BDSHOT_RESPONSE_LENGTH * BDSHOT_RESPONSE_OVERSAMPLING;
            uint32_t motor_response = 0;
            uint8_t bits = 0;
            while (raw_buffer <= buffer_end)
            {
                if (__builtin_expect((*raw_buffer++ & (1 << motor_shift)), 0) ||
                    __builtin_expect((*raw_buffer++ & (1 << motor_shift)), 0) ||
                    __builtin_expect((*raw_buffer++ & (1 << motor_shift)), 0) ||
                    __builtin_expect((*raw_buffer++ & (1 << motor_shift)), 0))
                {
                    if (raw_buffer <= buffer_end) {
                        uint8_t len = MAX((raw_buffer - buffer_previous) / BDSHOT_RESPONSE_OVERSAMPLING, 1);
                        bits += len;
                        motor_response <<= len;
                        buffer_previous = raw_buffer - 1;
                        while (raw_buffer < buffer_end)
                        {
                            if (__builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0) ||
                                __builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0) ||
                                __builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0) ||
                                __builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0)) {
                                if (raw_buffer <= buffer_end) {
                                    len = MAX((raw_buffer - buffer_previous) / BDSHOT_RESPONSE_OVERSAMPLING, 1);
                                    bits += len;
                                    motor_response <<= len;
                                    motor_response |= 0x1FFFFF >> (BDSHOT_RESPONSE_LENGTH - len);
                                    buffer_previous = raw_buffer - 1;
                                }
                                break;
                            }
                        }
                    }
                }
            }
            motor_response <<= (BDSHOT_RESPONSE_LENGTH - bits);
            if (*buffer_previous & (1 << motor_shift)) {
                motor_response |= 0x1FFFFF >> bits;
            }
            return motor_response;
        }
    }
    return 0xFFFFFFFF;
}


  bool BDshot_check_checksum(uint32_t decoded_value) {
     uint8_t crc = (decoded_value & 0x0F);
     uint16_t value = (decoded_value >> 4);
     uint8_t calculated_crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
     return (crc == calculated_crc);
 }


 // Modified read_BDshot_response function with DMA-based debugging
 void read_BDshot_response(uint32_t value, uint8_t motor){
  // Debug_Send_DMA("Raw telemetry value: 0x%lX\r\n", value);

     if (value < 0xFFFFFFF) {
        //uint32_t raw_gcr_value = value;
         value = (value ^ (value >> 1));
       // Debug_Send_DMA("After GCR XOR decoding: 0x%lX\r\n", value);

         uint32_t nibble1 = (value & 0x1F);
         uint32_t nibble2 = ((value >> 5) & 0x1F);
         uint32_t nibble3 = ((value >> 10) & 0x1F);
         uint32_t nibble4 = ((value >> 15) & 0x1F);

       // Debug_Send_DMA("Nibbles (GCR): 0x%lX, 0x%lX, 0x%lX, 0x%lX\r\n", nibble1, nibble2, nibble3, nibble4);

         if (GCR_table[nibble1] == iv || GCR_table[nibble2] == iv || GCR_table[nibble3] == iv || GCR_table[nibble4] == iv) {
             //Debug_Send_DMA("GCR lookup failed for one or more nibbles.\r\n");
             motor_telemetry_data[motor].valid_rpm = false;
             motor_telemetry_data[motor].valid_voltage = false;
             return;
         }

         uint32_t decoded_value = GCR_table[nibble1];
         decoded_value |= GCR_table[nibble2] << 4;
         decoded_value |= GCR_table[nibble3] << 8;
         decoded_value |= GCR_table[nibble4] << 12;

         //Debug_Send_DMA("Final decoded value: 0x%lX\r\n", decoded_value);

         if (BDshot_check_checksum(decoded_value))
         {
             //Debug_Send_DMA("CRC Check Passed.\r\n");
             motor_telemetry_data[motor].valid_rpm = true;
             motor_telemetry_data[motor].raw_rpm_value = ((decoded_value & 0x1FF0) >> 4) << (decoded_value >> 13);
             motor_telemetry_data[motor].raw_rpm_value = 60 * 1000000 / motor_telemetry_data[motor].raw_rpm_value * 2 / MOTOR_POLES_NUMBER;
             //Debug_Send_DMA("RPM: %u\r\n",motor_telemetry_data[motor].raw_rpm_value);
         } else {
            // Debug_Send_DMA("CRC Check FAILED. Data will be discarded.\r\n");
             motor_telemetry_data[motor].valid_rpm = false;
         }
     } else {
      //  Debug_Send_DMA("Invalid raw telemetry value.\r\n");
         motor_telemetry_data[motor].valid_rpm = false;
     }
 }



// --- NEW main telemetry processing function ---
void process_telemetry_with_new_method(void) {
    for (int m = 0; m < MOTORS_COUNT; m++) {
        uint32_t decoded_gcr_value = get_BDshot_response((uint32_t*)dshot_bb_buffer_1_4_r, motor_gpio_bit_positions[m]);
        read_BDshot_response(decoded_gcr_value, m); // Pass motor index 0-based
       // Debug_Send_DMA("M%d RAW: 0x%08lX\r\n", m, decoded_gcr_value); // Verify raw values
    }
}



uint16_t get_rpm_from_telemetry(uint16_t raw_value) {
    if (raw_value == 0) return 0;
    uint16_t rpm_x10 = (raw_value >> 4);
    return (uint16_t)(((float)rpm_x10 / MOTOR_POLES_NUMBER) * 12);
}

/*
uint16_t pid_calculate_command(uint16_t current_rpm,float target_rpm) {

    float error =  target_rpm - (float)current_rpm;

    pid_filtered_error = (PID_D_FILTER_ALPHA * error) + ((1.0f - PID_D_FILTER_ALPHA) * pid_filtered_error);

    float p_term = PID_KP * error;

    pid_integral += error;
    if (pid_integral > 1000) pid_integral = 1000;
    if (pid_integral < -1000) pid_integral = -1000;
    float i_term = PID_KI * pid_integral;

    float d_term = PID_KD * (pid_filtered_error - pid_last_error);
    pid_last_error = pid_filtered_error;

    float new_command_float = DSHOT_BASE_COMMAND + p_term + i_term + d_term;

    if (new_command_float < 48.0f) new_command_float = 48.0f;
    if (new_command_float > 2047.0f) new_command_float = 2047.0f;

    return (uint16_t)new_command_float;
}
*/


/**
 * @brief Calculates the DShot command based on PID controller logic for bi-directional motors.
 * @param current_rpm_unsigned The current measured RPM of the motor (always positive from telemetry).
 * @param target_rpm_signed The desired target RPM (signed: positive for forward, negative for reverse, 0 for stop/brake).
 * @return The calculated DShot command (1-2047 bi-directional).
 */
/*
uint16_t pid_calculate_command(uint32_t current_rpm_unsigned, float target_rpm_signed) {
    float error;
    float current_rpm_for_error_calc; // Signed version of current_rpm_unsigned for PID error calc

    // --- Step 1: Infer Current Motor Direction from last_sent_dshot_command ---
    // This is the key: we use the last command sent to infer the *actual* direction.
    if (last_sent_dshot_command > DSHOT_BI_DIR_DEADBAND_HIGH) { // Last command was in forward range
        current_rpm_for_error_calc = (float)current_rpm_unsigned; // Treat as positive RPM
    } else if (last_sent_dshot_command < DSHOT_BI_DIR_DEADBAND_LOW && last_sent_dshot_command > 0) { // Last command was in reverse range (and not stop)
        current_rpm_for_error_calc = -(float)current_rpm_unsigned; // Treat as negative RPM
    } else { // Last command was in deadband/stop zone, or was 0 (motor stop)
        current_rpm_for_error_calc = 0.0f; // Assume motor is stopped or very near stop
    }

    // --- Step 2: Calculate Error ---
    // The error is the difference between the desired signed target and the inferred signed current.
    error = target_rpm_signed - current_rpm_for_error_calc;

    // --- Step 3: Calculate PID Terms ---
    // Proportional term
    float p_term = PID_KP * error;

    // Integral term (with windup protection). Reset if passing through zero or at zero target.
    // Reset integral if target is 0 to avoid massive windup from previous positive/negative errors.
    if (target_rpm_signed == 0.0f) {
        pid_integral = 0.0f;
    } else {
        pid_integral += error;
        if (pid_integral > 1000) pid_integral = 1000;
        if (pid_integral < -1000) pid_integral = -1000;
    }
    float i_term = PID_KI * pid_integral;

    // Derivative term (with low-pass filter)
    pid_filtered_error = (PID_D_FILTER_ALPHA * error) + ((1.0f - PID_D_FILTER_ALPHA) * pid_filtered_error);
    float d_term = PID_KD * (pid_filtered_error - pid_last_error);
    pid_last_error = pid_filtered_error;

    // --- Step 4: Calculate Raw PID Output (signed) ---
    float raw_pid_output = p_term + i_term + d_term;

    // --- Step 5: Map PID Output to Bi-directional DShot Command ---
    float dshot_command_float;

    if (target_rpm_signed == 0) {
        // If target is explicitly zero, command the deadband center
        dshot_command_float = (float)DSHOT_BI_DIR_MIDPOINT;
        // Also reset integral and derivative components for a clean stop
        pid_integral = 0.0f;
        pid_last_error = 0.0f;
        pid_filtered_error = 0.0f;
    } else if (raw_pid_output >= 0) { // PID wants forward thrust
        // Map positive raw_pid_output (0 to MAX_ABS_RPM_FOR_MAPPING) to DShot forward range
        float scaled_output_percent = raw_pid_output / MAX_ABS_RPM_FOR_MAPPING;
        if (scaled_output_percent > 1.0f) scaled_output_percent = 1.0f; // Cap at max

        dshot_command_float = (scaled_output_percent * (DSHOT_BI_DIR_MAX_FORWARD - DSHOT_BI_DIR_DEADBAND_HIGH)) + DSHOT_BI_DIR_DEADBAND_HIGH;

        // Ensure command is at least the minimum forward value
        if (dshot_command_float < DSHOT_BI_DIR_DEADBAND_HIGH) dshot_command_float = DSHOT_BI_DIR_DEADBAND_HIGH;
        // Clamp to max DShot command
        if (dshot_command_float > DSHOT_BI_DIR_MAX_FORWARD) dshot_command_float = DSHOT_BI_DIR_MAX_FORWARD;

    } else { // PID wants reverse thrust (raw_pid_output < 0)
        // Map negative raw_pid_output (0 to -MAX_ABS_RPM_FOR_MAPPING) to DShot reverse range
        float scaled_output_percent = fabsf(raw_pid_output) / MAX_ABS_RPM_FOR_MAPPING; // Use absolute value
        if (scaled_output_percent > 1.0f) scaled_output_percent = 1.0f; // Cap at max absolute

        dshot_command_float = DSHOT_BI_DIR_DEADBAND_LOW - (scaled_output_percent * (DSHOT_BI_DIR_DEADBAND_LOW - DSHOT_BI_DIR_MIN_REVERSE));

        // Ensure command is at most the maximum reverse value
        if (dshot_command_float > DSHOT_BI_DIR_DEADBAND_LOW) dshot_command_float = DSHOT_BI_DIR_DEADBAND_LOW;
        // Clamp to min DShot command
        if (dshot_command_float < DSHOT_BI_DIR_MIN_REVERSE) dshot_command_float = DSHOT_BI_DIR_MIN_REVERSE;
    }

    // --- Step 6: Store and Return the Final Command ---
    uint16_t final_clamped_command = (uint16_t)dshot_command_float;
    last_sent_dshot_command = final_clamped_command; // Store for next iteration's direction inference

    // Debug output (uncomment for debugging)
    // Debug_Send_DMA("PID | Tgt:%.0f | Curr:%.0f | Err:%.0f | P:%.2f | I:%.2f | D:%.2f | Cmd:%u | LastCmd:%u\r\n",
    //                target_rpm_signed, current_rpm_for_error_calc, error, p_term, i_term, d_term, final_clamped_command, last_sent_dshot_command);

    return final_clamped_command;
}
*/


/**
 * @brief Calculates the DShot command based on PID controller logic for unidirectional motors.
 * For testing purposes, this function maps all negative or zero target RPMs to DSHOT_BASE_COMMAND (48).
 * @param current_rpm_unsigned The current measured RPM of the motor (always positive from telemetry).
 * @param target_rpm_signed The desired target RPM (signed: positive for forward, negative for reverse, 0 for stop/brake).
 * @return The calculated DShot command (48-2047 unidirectional).
 */
uint16_t pid_calculate_command(uint32_t current_rpm_unsigned, float target_rpm_signed) {
    // --- For Unidirectional Test: Treat all current RPM as positive ---
    float current_rpm_for_error_calc = (float)current_rpm_unsigned;

    // --- Step 2: Calculate Error ---
    // The error is the difference between the desired signed target and the current motor RPM.
    // Negative target RPMs are treated as 0 for error calculation to prevent erratic behavior.
    float effective_target_rpm = (target_rpm_signed < 0) ? 0.0f : target_rpm_signed;

    float error = effective_target_rpm - current_rpm_for_error_calc;

    // --- Step 3: Calculate PID Terms ---
    float p_term = PID_KP * error;

    // Integral term (with windup protection). Reset if effective_target_rpm is 0.
    if (effective_target_rpm == 0.0f) {
        pid_integral = 0.0f;
    } else {
        pid_integral += error;
        // Clamp integral term
        if (pid_integral > 1000) pid_integral = 1000;
        if (pid_integral < -1000) pid_integral = -1000;
    }
    float i_term = PID_KI * pid_integral;

    // Derivative term (with low-pass filter)
    pid_filtered_error = (PID_D_FILTER_ALPHA * error) + ((1.0f - PID_D_FILTER_ALPHA) * pid_filtered_error);
    float d_term = PID_KD * (pid_filtered_error - pid_last_error);
    pid_last_error = pid_filtered_error;

    // --- Step 4: Calculate Raw PID Output (signed for internal calculation) ---
    float raw_pid_output = p_term + i_term + d_term;

    // --- Step 5: Map PID Output to Unidirectional DShot Command (48-2047) ---
    float dshot_command_float;

    if (effective_target_rpm <= 0.0f) { // If target RPM is 0 or negative
        // Command motor stop/idle (DSHOT_BASE_COMMAND = 48)
        dshot_command_float = (float)DSHOT_BASE_COMMAND;
        // Reset integral and derivative components for a clean stop
        pid_integral = 0.0f;
        pid_last_error = 0.0f;
        pid_filtered_error = 0.0f;
    } else { // Positive target RPM
        // Map raw_pid_output (0 to MAX_ABS_RPM_FOR_MAPPING) to DSHOT_BASE_COMMAND to 2047
        float scaled_output_percent = raw_pid_output / MAX_ABS_RPM_FOR_MAPPING;
        if (scaled_output_percent > 1.0f) scaled_output_percent = 1.0f; // Cap at max
        if (scaled_output_percent < 0.0f) scaled_output_percent = 0.0f; // No reverse in unidirectional

        dshot_command_float = (scaled_output_percent * (2047.0f - DSHOT_BASE_COMMAND)) + DSHOT_BASE_COMMAND;

        // Ensure command is at least DSHOT_BASE_COMMAND (48)
        if (dshot_command_float < DSHOT_BASE_COMMAND) dshot_command_float = DSHOT_BASE_COMMAND;
        // Clamp to max DShot command
        if (dshot_command_float > 2047.0f) dshot_command_float = 2047.0f;
    }

    // --- Step 6: Store and Return the Final Command ---
    uint16_t final_clamped_command = (uint16_t)dshot_command_float;
    // last_sent_dshot_command is still updated, but its role in direction inference is suppressed
    last_sent_dshot_command = final_clamped_command;

    // Debug output (uncomment for debugging)
    // Debug_Send_DMA("PID | Tgt:%.0f | Curr:%.0f | Err:%.0f | P:%.2f | I:%.2f | D:%.2f | Cmd:%u\r\n",
    //                target_rpm_signed, current_rpm_for_error_calc, error, p_term, i_term, d_term, final_clamped_command);

    return final_clamped_command;
}
