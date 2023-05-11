/**
 * @file stm32_vl53l1x.h
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief More simplified and clear initialization with VL53L1X API
 * @date 2023-02-19
 */

#ifndef STM32_VL53L1X_H
#define STM32_VL53L1X_H

#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"

#define VL53L1X_ADDR 0x52

enum vl53l1x_dist_mode {
        VL53L1X_DIST_SHORT = 1,
        VL53L1X_DIST_LONG
};

/* 15ms timing budget must be used in short mode */
enum vl53l1x_timing_budget {
        VL53L1X_TIMING_BUDGET_15 = 15,
        VL53L1X_TIMING_BUDGET_20 = 20,
        VL53L1X_TIMING_BUDGET_33 = 33,
        VL53L1X_TIMING_BUDGET_50 = 50,
        VL53L1X_TIMING_BUDGET_100 = 100,
        VL53L1X_TIMING_BUDGET_200 = 200,
        VL53L1X_TIMING_BUDGET_500 = 500
};

struct vl53l1x_param {
        enum vl53l1x_dist_mode mode;
        enum vl53l1x_timing_budget tim_budget;
        uint16_t inter_measurement;
};

int vl53l1x_init(struct vl53l1x_param param);
uint16_t vl53l1x_get_distance(void);

#endif /* STM32_VL53L1X_H */