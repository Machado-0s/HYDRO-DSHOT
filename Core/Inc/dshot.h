#ifndef INC_DSHOT_H_
#define INC_DSHOT_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define MOTORS_COUNT                10
#define DSHOT_MODE                  600
#define TIMER_CLK                   84000000
#define BITS_PER_FRAME              16
#define MOTOR_POLES_NUMBER          14   // number of poles in your motors (usually 14 or 12)

#define DSHOT_BB_FRAME_LENGTH       (TIMER_CLK / (DSHOT_MODE * 1000))
#define DSHOT_BB_FRAME_SECTIONS      8//4
#define DSHOT_BB_SECTION_LENGTH     (DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS)

#define DSHOT_BB_0_LENGTH           (DSHOT_BB_FRAME_SECTIONS  *  4/8 )   // 37.5%//2//3/8///1/3/3
#define DSHOT_BB_1_LENGTH           (DSHOT_BB_FRAME_SECTIONS  * 3/4)   // 75%//5/3/4/////2/3/6

#define DSHOT_BB_BUFFER_BITS        (BITS_PER_FRAME)
#define DSHOT_BB_BUFFER_LENGTH      ((DSHOT_BB_BUFFER_BITS * DSHOT_BB_FRAME_SECTIONS)+1)

#define DSHOT_DMA_CHANNEL           6
#define DSHOT_DMA_STREAM            DMA2_Stream5

#define BDSHOT_RESPONSE_LENGTH          21
#define BDSHOT_RESPONSE_BITRATE         (DSHOT_MODE * 4/3)
#define BDSHOT_RESPONSE_OVERSAMPLING    3// CAN only do in pairs of 3s and should conrespond to your dshot mode


// PID gains - these will need to be tuned

#define NOMINAL_VOLTAGE 12.4f // Nominal voltage for PID tuning
#define DSHOT_BASE_COMMAND 48.0f //48 // A safe base throttle value

#define PID_D_FILTER_ALPHA 0.3f // A value between 0.0 and 1.0
                                // Lower values = more filtering (more lag)
                                // Higher values = less filtering (less lag)
// Constants for Bi-directional DShot mapping
#define DSHOT_BI_DIR_MIDPOINT       1024 // DShot command for zero thrust (e.g., 1024 for BLHeli_32)
#define DSHOT_BI_DIR_DEADBAND_LOW   1001 // Lowest command in deadband/brake zone
#define DSHOT_BI_DIR_DEADBAND_HIGH  1047 // Highest command in deadband/brake zone
#define DSHOT_BI_DIR_MAX_FORWARD    2047 // Max DShot command for full forward
#define DSHOT_BI_DIR_MIN_REVERSE    1    // Min DShot command for full reverse
#define MAX_ABS_RPM_FOR_MAPPING     6000.0f // Max RPM the motor can achieve in either direction

// PID Target RPMs - Array to hold target RPM for each motor
extern volatile float pid_target_speed_rpms[MOTORS_COUNT];

// PID state variables
extern float pid_integral;
extern float pid_last_error;
//extern float pid_target_speed_rpm; // Your target speed, in RPM
extern float pid_filtered_error; // New variable to hold the filtered error
extern volatile float pid_target_speed_rpm; // 'volatile' is good for variables modified by external/ISR contexts


/// (if you plan to use them in the future)
typedef struct {
    uint32_t raw_rpm_value;
    bool valid_rpm;
    float voltage;
    bool valid_voltage;
    float current;      // Added for current
    bool valid_current; // Added for current
    float temperature;  // Added for temperature
    bool valid_temperature; // Added for temperature
} MotorTelemetry_t;

extern MotorTelemetry_t motor_telemetry_data[MOTORS_COUNT];
extern volatile uint8_t telemetry_done_flag;

extern volatile bool telemetry_data_ready_flag;

// External buffers and GPIO position mapping
extern uint16_t motor_values[MOTORS_COUNT];
extern volatile uint32_t dshot_bb_buffer_10[DSHOT_BB_BUFFER_LENGTH] __attribute__((aligned(4)));
extern const uint8_t motor_gpio_bit_positions[MOTORS_COUNT];
extern volatile uint32_t dshot_bb_buffer_1_4_r[(int)(31 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];

// Function prototypes
void setup_Dshot_Tx_Only(void);
void update_motors_Tx_Only(void);
uint16_t prepare_Dshot_package(uint16_t value, bool telemetry);
void preset_bb_Dshot_buffers(void);

// PID and Telemetry functions
uint16_t pid_calculate_command(uint32_t  current_rpm,float target_rpm);
uint16_t get_rpm_from_telemetry(uint16_t raw_value);

// New functions for the GCR decoding method
static const uint32_t GCR_table[32];
 uint32_t get_BDshot_response(uint32_t raw_buffer[], const uint8_t motor_shift);
 void read_BDshot_response(uint32_t value, uint8_t motor);
 bool BDshot_check_checksum(uint32_t decoded_value);
#define MAX(a,b) (((a)>(b))?(a):(b))


void process_telemetry_with_new_method(void);
void Debug_Send_DMA(const char* format, ...);


#endif /* INC_DSHOT_H_ */
