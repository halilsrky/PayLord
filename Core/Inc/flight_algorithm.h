/**
 * @file flight_algorithm.h
 * @brief Core flight algorithm for rocket flight control system
 * @date 2025-07-03
 * @author halilsrky
 */

#ifndef INC_FLIGHT_ALGORITHM_H_
#define INC_FLIGHT_ALGORITHM_H_

#include "bme280.h"
#include "bmi088.h"
#include "sensor_fusion.h"
#include <stdint.h>

// Flight phases
typedef enum {
    PHASE_IDLE,             // Pre-launch, on pad
    PHASE_BOOST,            // Rocket engines burning
    PHASE_COAST,            // After burnout, still ascending
    PHASE_MAIN_DESCENT,     // Descent with main parachute
    PHASE_LANDED            // Landed
} FlightPhase_t;

/**
 * @brief Initialize the flight algorithm
 */
void flight_algorithm_init(void);

/**
 * @brief Reset flight algorithm to initial state
 */
void flight_algorithm_reset(void);

/**
 * @brief Update flight algorithm with sensor data
 * @param bme Pointer to BME280 data structure
 * @param bmi Pointer to BMI088 data structure
 * @param sensor_fusion Pointer to sensor fusion output
 * @return Current status bits
 */
void flight_algorithm_update(BME_280_t* bme, bmi088_struct_t* bmi, sensor_fusion_t* sensor_fusion);
uint32_t flight_algorithm_get_start_time(void);
/**
 * @brief Get the current flight phase
 * @return Current flight phase
 */
FlightPhase_t flight_algorithm_get_phase(void);

/**
 * @brief Get the current status bits
 * @return Status bits as a 16-bit value
 */
uint8_t flight_algorithm_get_durum_verisi(void);

/**
 * @brief Set flight parameters
 * @param launch_accel_threshold Launch detection acceleration threshold (m/s²)
 * @param min_arming_altitude Minimum altitude for arming (m)
 * @param main_chute_altitude Main parachute deployment altitude (m)
 * @param max_angle_threshold Maximum angle threshold for drogue deployment (degrees)
 */
void flight_algorithm_set_parameters(float launch_accel_threshold,
                                    float min_arming_altitude,
                                    float main_chute_altitude,
                                    float max_angle_threshold);


void deploy_drogue_parachute(void);
void deploy_main_parachute(void);
void deploy_parachute_update(void);
#endif /* INC_FLIGHT_ALGORITHM_H_ */
