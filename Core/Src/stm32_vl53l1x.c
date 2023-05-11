/**
 * @file stm32_vl53l1x.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief More simplified and clear initialization with VL53L1X API
 * @date 2023-02-19
 */

#include "stm32_vl53l1x.h"

int vl53l1x_init(struct vl53l1x_param param)
{
        uint8_t sensor_state = 0;
        int result;
        /* try to boot 100 times */
        for (uint8_t i = 0; i < 100; i++) {
                result = VL53L1X_BootState(VL53L1X_ADDR, &sensor_state);
                if (sensor_state)
                        break;
        }
        if (result)
                return result;
        else if (!sensor_state)
                return 1;

        result = VL53L1X_SensorInit(VL53L1X_ADDR);
        if (result)
                return result;

        result = VL53L1X_SetDistanceMode(VL53L1X_ADDR, param.mode);
        if (result)
                return result;

        result = VL53L1X_SetTimingBudgetInMs(VL53L1X_ADDR, param.tim_budget);
        if (result)
                return result;

        result = VL53L1X_SetInterMeasurementInMs(VL53L1X_ADDR,
                                                 param.inter_measurement);
        if (result)
                return result;

        result = VL53L1X_StartRanging(VL53L1X_ADDR);
        
        return result;
}

uint16_t vl53l1x_get_distance(void)
{
        uint8_t is_data_ready;
        uint8_t range_status;
        static uint16_t distance = 0;

        VL53L1X_CheckForDataReady(VL53L1X_ADDR, &is_data_ready);

        if (is_data_ready)
                VL53L1X_GetRangeStatus(VL53L1X_ADDR, &range_status);
        else
                return distance;

        if (!range_status) {
                VL53L1X_GetDistance(VL53L1X_ADDR, &distance);
                VL53L1X_ClearInterrupt(VL53L1X_ADDR);
        }
        
        return distance;
}