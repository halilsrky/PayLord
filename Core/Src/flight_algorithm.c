/**
 * @file flight_algorithm.c
 * @brief Core flight algorithm for rocket flight control system
 * @date 2025-07-03
 * @author halilsrky
 */

#include "flight_algorithm.h"
#include "queternion.h"
#include "bme280.h"
#include "main.h"
#include <math.h>

// External dependencies
extern int is_BME_ok;

// Private defines
#define ALT_DECREASE_THRESHOLD   3     // consecutive altitude decreases

// Private variables
static FlightPhase_t current_phase = PHASE_IDLE;

// Flight parameters (configurable)
static float launch_accel_threshold = 10.0f;  // m/s²
static float min_arming_altitude = 2000.0f;   // meters
static float main_chute_altitude = 500.0f;    // meters
static float max_angle_threshold = 70.0f;     // degrees

// Flight state tracking
static uint8_t is_rising = 0;
static uint8_t is_stabilized = 1;
static uint8_t is_armed = 0;
static uint8_t deployed_angle = 1;
static uint8_t deployed_velocity = 1;
static uint8_t deployed_magnetic = 1;
static uint8_t drogue_deployed = 0;
static uint8_t main_deployed = 0;
static uint8_t durum_verisi = 1;

int apogee_counter = 0;
float prev_velocity = 0;

static uint32_t flight_start_time = 0;
static uint8_t altitude_decrease_count = 0;
static float prev_altitude = 0.0f;

// Status tracking
static uint16_t status_bits = 0;

// Private function prototypes
static float calculate_total_acceleration(bmi088_struct_t* bmi);

// Drogue parachute deployment state
static uint8_t drogue_pulse_active = 0;
static uint32_t drogue_pulse_start_time = 0;

// Main parachute deployment state
static uint8_t main_pulse_active = 0;
static uint32_t main_pulse_start_time = 0;





/**
 * @brief Initialize the flight algorithm
 */
void flight_algorithm_init(void)
{
    flight_algorithm_reset();
}

/**
 * @brief Reset flight algorithm to initial state
 */
void flight_algorithm_reset(void)
{
    current_phase = PHASE_IDLE;
    is_rising = 0;
    is_stabilized = 1;
    is_armed = 0;
    drogue_deployed = 0;
    main_deployed = 0;
    altitude_decrease_count = 0;
    status_bits = 0;
    prev_altitude = 0.0f;
    flight_start_time = 0;
    deployed_velocity = 1;
    deployed_angle = 1;
    apogee_counter = 0;
    prev_velocity = 0;
    deployed_magnetic = 1;
    durum_verisi = 1;
}

/**
 * @brief Update flight algorithm with sensor data
 */
void flight_algorithm_update(BME_280_t* bme, bmi088_struct_t* bmi, sensor_fusion_t* sensor_fusion)
{
    // Calculate key metrics
    float total_acceleration = calculate_total_acceleration(bmi);
    //float theta = sensor_fusion->angle; // Use sensor fusion output

    // Status bits are cumulative - once set they remain set
    // Each phase builds on the previous phase's status bits

    // State machine for flight phases
    switch (current_phase) {
        case PHASE_IDLE:
            // Detect launch using acceleration threshold
            if (total_acceleration > launch_accel_threshold) {
                current_phase = PHASE_BOOST;
                is_rising = 1;
                flight_start_time = HAL_GetTick();
                durum_verisi = 2;
            }
            break;

        case PHASE_BOOST:
            // After boost phase (typically 3-5 seconds)
            if (HAL_GetTick() - flight_start_time > 7000) {
                current_phase = PHASE_COAST;
                is_stabilized = 1;
                durum_verisi = 3;
            }
            break;

        case PHASE_COAST:
            // Check minimum arming altitude
            if (bme->altitude > min_arming_altitude) {
                is_armed = 1;
                durum_verisi = 4;
            }

            // Check if angle exceeds threshold
            if (is_armed && (fabs(bmi->datas.angle_y) > max_angle_threshold) && deployed_angle) {
                drogue_deployed = 1;
                deployed_angle = 0;
                durum_verisi = 5;
            }

                 // Detect magnetic field decrease (apogee)
            if (is_armed && sensor_fusion->apogeeDetect == 1 && deployed_magnetic) {
                drogue_deployed = 1;
                deployed_magnetic = 0;
                durum_verisi = 6;
            }

            if (is_armed && sensor_fusion->velocity < 0.0f && sensor_fusion->velocity < prev_velocity && deployed_velocity) {
                apogee_counter++;
                if (apogee_counter >= 10) {  // Confirm apogee after 5 consecutive samples
                    drogue_deployed = 1;
                    deployed_velocity = 0;
                    durum_verisi = 7;
                }
            } else {
                apogee_counter = 0;
            }
            prev_velocity = sensor_fusion->velocity;

            // Deploy main parachute at designated altitude
            if (drogue_deployed && bme->altitude < main_chute_altitude) {
                current_phase = PHASE_MAIN_DESCENT;
                main_deployed = 1;
                drogue_deployed = 0;
                durum_verisi = 8;
            }
            break;

        case PHASE_MAIN_DESCENT:
            // Monitor descent under main parachute
            break;

        case PHASE_LANDED:
            // No additional status bits to set after landing
            break;
    }
    prev_altitude = bme->altitude;
}


/**
 * @brief Calculate total acceleration magnitude
 */
static float calculate_total_acceleration(bmi088_struct_t* bmi)
{
    return sqrtf((bmi->datas.acc_x * bmi->datas.acc_x) +
                 (bmi->datas.acc_y * bmi->datas.acc_y) +
                 (bmi->datas.acc_z * bmi->datas.acc_z));
}

/**
 * @brief Get the current flight phase
 */
FlightPhase_t flight_algorithm_get_phase(void)
{
    return current_phase;
}

uint8_t flight_algorithm_get_durum_verisi(void)
{
    return durum_verisi;
}
/**
 * @brief Set flight parameters
 */
void flight_algorithm_set_parameters(float launch_accel_threshold_param,
                                   float min_arming_altitude_param,
                                   float main_chute_altitude_param,
                                   float max_angle_threshold_param)
{
    launch_accel_threshold = launch_accel_threshold_param;
    min_arming_altitude = min_arming_altitude_param;
    main_chute_altitude = main_chute_altitude_param;
    max_angle_threshold = max_angle_threshold_param;
}

uint32_t flight_algorithm_get_start_time(void)
{
    return flight_start_time;
}



