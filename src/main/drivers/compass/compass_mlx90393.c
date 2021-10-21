/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_MAG_MLX90393

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/bus.h"

#include "drivers/sensor.h"
#include "drivers/compass/compass.h"

#include "common/log.h"

#define MLX90393_MEASURE_Z                          0b00001000
#define MLX90393_MEASURE_Y                          0b00000100
#define MLX90393_MEASURE_X                          0b00000010
#define MLX90393_MEASURE_T                          0b00000001

#define MLX90393_MEASURE_3D                         0b00001110


#define MLX90393_NOP                                0b00000000
#define MLX90393_START_BURST_MODE                   0b00010000 //uses with zyxt flags
#define MLX90393_START_WAKE_UP_ON_CHANGE_MODE       0b00100000 //uses with zyxt flags
#define MLX90393_START_SINGLE_MEASUREMENT_MODE      0b00110000 //uses with zyxt flags
#define MLX90393_READ_MEASUREMENT                   0b01000000 //uses with zyxt flags
#define MLX90393_READ_REGISTER                      0b01010000
#define MLX90393_WRITE_REGISTER                     0b01100000
#define MLX90393_EXIT_MODE                          0b10000000
#define MLX90393_MEMORY_RECALL                      0b11010000
#define MLX90393_MEMORY_STORE                       0b11100000
#define MLX90393_RESET                              0b11110000

#define MLX90393_BURST_MODE_FLAG                    0b10000000
#define MLX90393_WAKE_UP_ON_CHANGE_MODE_FLAG        0b01000000
#define MLX90393_SINGLE_MEASUREMENT_MODE_FLAG       0b00100000
#define MLX90393_ERROR_FLAG                         0b00010000
#define MLX90393_SINGLE_ERROR_DETECTION_FLAG        0b00001000
#define MLX90393_RESET_FLAG                         0b00000100
#define MLX90393_D1_FLAG                            0b00000010
#define MLX90393_D0_FLAG                            0b00000001

#define DETECTION_MAX_RETRY_COUNT   5

#define REG_BUF_LEN                 3

#define GAIN_SEL_HALLCONF_REG       0x0 //from datasheet 0x0 << 2 = 0x0 
#define GAIN_SEL_MASK               0x0070
#define GAIN_SEL_SHIFT              4
#define HALLCONF_MASK               0x000f
#define HALLCONF_SHIFT              0
#define Z_SERIES_MASK               0x0080
#define Z_SERIES_SHIFT              7


#define TCMP_EN_REG                 0x4 //from datasheet 0x1 << 2 = 0x4
#define TCMP_EN_MASK                0x0400
#define TCMP_EN_SHIFT               10

#define BDR_EN_MASK                 0x003f
#define BDR_EN_SHIFT                0

#define RES_XYZ_OSR_DIG_FLT_REG     0x8 //from datasheet 0x2 << 2 = 0x8 
#define RES_XYZ_MASK                0x07e0
#define RES_XYZ_SHIFT               5
#define OSR_MASK                    0x0003
#define OSR_SHIFT                   0
#define DIG_FLT_MASK                0x001c
#define DIG_FLT_SHIFT               2 

float gain_multipliers[8] = {5.f, 4.f, 3.f, 2.5f, 2.f, 1.66666667f, 1.33333333f, 1.f};
// for hallconf = 0
float base_xy_sens_hc0 = 0.196f;
float base_z_sens_hc0 = 0.316f;
// for hallconf = 0xc
float base_xy_sens_hc0xc = 0.150f;
float base_z_sens_hc0xc = 0.242f;

struct mlx90393_struct {
    uint8_t gain_sel;
    uint8_t hallconf;
    uint8_t tcmp_en;
    uint8_t res_x;
    uint8_t res_y;
    uint8_t res_z;
};

struct mlx90393_struct mlx90393 = {0};

static bool check_ack(bool ack, const char * func_name)
{
    if (!ack) {
        LOG_E(SYSTEM, "Problem with acknowledge in function %s", func_name);
        return false;
    }
    return true;
}

static void nop_command(magDev_t * mag) {
    uint8_t sig = 0;

    bool ack = busRead(mag->busDev, MLX90393_NOP, &sig);

    if (!ack) {
        LOG_E(SYSTEM, "ERROR in ACK of NOP");
    } else {
        LOG_E(SYSTEM, "MLX90393_NOP cmd answer: 0x%02X", sig);
    }
}

static bool mlx90393SetGainSel(magDev_t * mag, uint8_t gain_sel)
{
    const char* func_name;
    func_name = __func__;

    uint16_t old_val, new_val;
    uint8_t buf[REG_BUF_LEN] = {0};
    
    // FIRST READ
    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | GAIN_SEL_HALLCONF_REG, buf, REG_BUF_LEN) , func_name)) {
        return false;
    }
    delay(20);

    //buf[0] - status byte
    old_val = ((uint16_t)buf[1] << 8) | buf[2];

    new_val = (old_val & ~GAIN_SEL_MASK) | (((uint16_t)gain_sel << GAIN_SEL_SHIFT) & GAIN_SEL_MASK);

    LOG_E(SYSTEM, "mlx90393SetGainSel readres: 0x%02X old_val: 0x%04X new_val: 0x%04X", buf[0], old_val, new_val);

    buf[0] = (new_val >> 8) & 0xFF;
    buf[1] = (new_val >> 0) & 0xFF;
    buf[2] = GAIN_SEL_HALLCONF_REG;

    // WRITE
    if (!check_ack(busWriteBuf(mag->busDev, MLX90393_WRITE_REGISTER, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    mlx90393.gain_sel = gain_sel;

    return true;
}

static bool mlx90393SetHallConf(magDev_t * mag, uint8_t hall_conf)
{
    const char* func_name;
    func_name = __func__;

    uint16_t old_val, new_val;
    uint8_t buf[REG_BUF_LEN] = {0};
    
    // FIRST READ
    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | GAIN_SEL_HALLCONF_REG, buf, REG_BUF_LEN) , func_name)) {
        return false;
    }
    delay(20);

    //buf[0] - status byte
    old_val = ((uint16_t)buf[1] << 8) | buf[2];

    new_val = (old_val & ~HALLCONF_MASK) | (((uint16_t)hall_conf << HALLCONF_SHIFT) & HALLCONF_MASK);
    

    LOG_E(SYSTEM, "mlx90393SetHallConf readres: 0x%02X old_val: 0x%04X new_val: 0x%04X", buf[0], old_val, new_val);

    buf[0] = (new_val >> 8) & 0xFF;
    buf[1] = (new_val >> 0) & 0xFF;
    buf[2] = GAIN_SEL_HALLCONF_REG;

    // WRITE
    if (!check_ack(busWriteBuf(mag->busDev, MLX90393_WRITE_REGISTER, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    return true;
}

static bool mlx90393SetZSeries(magDev_t * mag, uint8_t z_series)
{
    const char* func_name;
    func_name = __func__;

    uint16_t old_val, new_val;
    uint8_t buf[REG_BUF_LEN] = {0};
    
    // FIRST READ
    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | GAIN_SEL_HALLCONF_REG, buf, REG_BUF_LEN) , func_name)) {
        return false;
    }
    delay(20);

    //buf[0] - status byte
    old_val = ((uint16_t)buf[1] << 8) | buf[2];

    new_val = (old_val & ~Z_SERIES_MASK) | (((uint16_t)z_series << Z_SERIES_SHIFT) & Z_SERIES_MASK);
    

    LOG_E(SYSTEM, "mlx90393SetZSeries readres: 0x%02X old_val: 0x%04X new_val: 0x%04X", buf[0], old_val, new_val);

    buf[0] = (new_val >> 8) & 0xFF;
    buf[1] = (new_val >> 0) & 0xFF;
    buf[2] = GAIN_SEL_HALLCONF_REG;

    // WRITE
    if (!check_ack(busWriteBuf(mag->busDev, MLX90393_WRITE_REGISTER, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    return true;
}

static bool mlx90393SetResolution(magDev_t * mag, uint8_t res_x, uint8_t res_y, uint8_t res_z) {
    const char* func_name;
    func_name = __func__;

    uint16_t res_xyz = ((res_z & 0x3)<<4) | ((res_y & 0x3)<<2) | (res_x & 0x3);
    uint16_t old_val, new_val;
    uint8_t buf[REG_BUF_LEN] = {0};

    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | RES_XYZ_OSR_DIG_FLT_REG, buf, REG_BUF_LEN), func_name))
    {
        return false;
    }

    //buf[0] - status byte
    old_val = ((uint16_t)buf[1] << 8) | buf[2];

    new_val = (old_val & ~RES_XYZ_MASK) | ((res_xyz << RES_XYZ_SHIFT) & RES_XYZ_MASK);

    LOG_E(SYSTEM, "mlx90393SetResolution old_val: 0x%04X new_val: 0x%04X", old_val, new_val);

    buf[0] = (new_val >> 8) & 0xFF;
    buf[1] = (new_val >> 0) & 0xFF;
    buf[2] = RES_XYZ_OSR_DIG_FLT_REG;

    if (!check_ack(busWriteBuf(mag->busDev, MLX90393_WRITE_REGISTER, buf, REG_BUF_LEN), func_name)){
        return false;
    };

    mlx90393.res_x = res_x;
    mlx90393.res_y = res_y;
    mlx90393.res_z = res_z;

    return true;
}

static bool mlx90393SetOverSampling(magDev_t * mag, uint8_t osr) {
    const char* func_name;
    func_name = __func__;

    uint16_t old_val, new_val;
    uint8_t buf[REG_BUF_LEN] = {0};

    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | RES_XYZ_OSR_DIG_FLT_REG, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    //buf[0] - status byte
    old_val = ((uint16_t)buf[1] << 8) | buf[2];
    
    new_val = (old_val & ~OSR_MASK) | (((uint16_t)osr << OSR_SHIFT) & OSR_MASK);

    LOG_E(SYSTEM, "mlx90393SetOverSampling old_val: 0x%04X new_val: 0x%04X", old_val, new_val);

    buf[0] = (new_val >> 8) & 0xFF;
    buf[1] = (new_val >> 0) & 0xFF;
    buf[2] = RES_XYZ_OSR_DIG_FLT_REG;

    if (!check_ack(busWriteBuf(mag->busDev, MLX90393_WRITE_REGISTER, buf, REG_BUF_LEN), func_name)) {
        return false;
    }

    return true;
}

static bool mlx90393SetDigitalFiltering(magDev_t * mag, uint8_t dig_flt) {
    const char* func_name;
    func_name = __func__;

    uint16_t old_val, new_val;
    uint8_t buf[REG_BUF_LEN] = {0};

    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | RES_XYZ_OSR_DIG_FLT_REG, buf, REG_BUF_LEN), func_name)){
        return false;
    };

    //buf[0] - status byte
    old_val = ((uint16_t)buf[1] << 8) | buf[2];

    new_val = (old_val & ~DIG_FLT_MASK) | (((uint16_t)dig_flt << DIG_FLT_SHIFT) & DIG_FLT_MASK);

    LOG_E(SYSTEM, "mlx90393SetDigitalFiltering old_val: 0x%04X new_val: 0x%04X", old_val, new_val);

    buf[0] = (new_val >> 8) & 0xFF;
    buf[1] = (new_val >> 0) & 0xFF;
    buf[2] = RES_XYZ_OSR_DIG_FLT_REG;

    if (!check_ack(busWriteBuf(mag->busDev, MLX90393_WRITE_REGISTER, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    return true;
}

static bool mlx90393SetTemperatureCompensation(magDev_t * mag, uint8_t enabled) {
    const char* func_name;
    func_name = __func__;

    uint8_t tcmp_en = enabled?1:0;
    uint16_t old_val, new_val;
    uint8_t buf[REG_BUF_LEN] = {0};

    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | TCMP_EN_REG, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    //buf[0] - status byte
    old_val = ((uint16_t)buf[1] << 8) | buf[2];

    new_val = (old_val & ~TCMP_EN_MASK) | (((uint16_t)tcmp_en << TCMP_EN_SHIFT) & TCMP_EN_MASK);

    LOG_E(SYSTEM, "mlx90393SetTemperatureCompensation old_val: 0x%04X new_val: 0x%04X", old_val, new_val);

    buf[0] = (new_val >> 8) & 0xFF;
    buf[1] = (new_val >> 0) & 0xFF;
    buf[2] = TCMP_EN_REG;

    if (!check_ack(busWriteBuf(mag->busDev, MLX90393_WRITE_REGISTER, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    mlx90393.tcmp_en = tcmp_en;
    
    return true;
}

static bool mlx90393SetBurstDataRate(magDev_t * mag, uint8_t bdr) {
    const char* func_name;
    func_name = __func__;

    uint16_t old_val, new_val;
    uint8_t buf[REG_BUF_LEN] = {0};

    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | TCMP_EN_REG, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    //buf[0] - status byte
    old_val = ((uint16_t)buf[1] << 8) | buf[2];
    new_val = (old_val & ~BDR_EN_MASK) | (((uint16_t) bdr << BDR_EN_SHIFT) & BDR_EN_MASK);

    LOG_E(SYSTEM, "mlx90393SetBurstDataRate old_val: 0x%04X new_val: 0x%04X", old_val, new_val);

    buf[0] = (new_val >> 8) & 0xFF;
    buf[1] = (new_val >> 0) & 0xFF;
    buf[2] = TCMP_EN_REG;

    if (!check_ack(busWriteBuf(mag->busDev, MLX90393_WRITE_REGISTER, buf, REG_BUF_LEN), func_name)){
        return false;
    }
    
    return true;
}

// =======================================================================================
static bool mlx90393Read(magDev_t * mag)
{
    const char* func_name;
    func_name = __func__;

    uint8_t buf[7] = {0};

    if (!check_ack(busReadBuf(mag->busDev, MLX90393_READ_MEASUREMENT | MLX90393_MEASURE_3D, buf, 7), func_name)){
        mag->magADCRaw[X] = 0;
        mag->magADCRaw[Y] = 0;
        mag->magADCRaw[Z] = 0;
        return false;
    }

    LOG_E(SYSTEM, "MLX90393_READ_MEASUREMENT cmd answer: 0x%02X", buf[0]);

    // READING. (Different resolution - different encoding.)

    // 2's complement MANUAL. For Res 0-1
    /* 
    if (buf[1] >> 7) {
        // minus sign
        mag->magADCRaw[X] = (int16_t)((~(buf[1] << 8 | buf[2]) & 0xffff) + 1);
        mag->magADCRaw[Y] = (int16_t)((~(buf[3] << 8 | buf[4]) & 0xffff) + 1);
        mag->magADCRaw[Z] = (int16_t)((~(buf[5] << 8 | buf[6]) & 0xffff) + 1);
    } else {
        // plus sign
        mag->magADCRaw[X] = (int16_t)(buf[1] << 8 | buf[2]);
        mag->magADCRaw[Y] = (int16_t)(buf[3] << 8 | buf[4]);
        mag->magADCRaw[Z] = (int16_t)(buf[5] << 8 | buf[6]);
    } */
    
    // 2's complement Auto. For Res 0-1
    mag->magADCRaw[X] = ((short)(buf[1] << 8 | buf[2]));
    mag->magADCRaw[Y] = ((short)(buf[3] << 8 | buf[4]));
    mag->magADCRaw[Z] = ((short)(buf[5] << 8 | buf[6]));

    // Unsigned Manual. For Res 2
    /* 
    if (buf[1] >> 7) {
        // minus sign
        mag->magADCRaw[X] = (int16_t)((buf[1] << 8 | buf[2]) & 0x7FFF);
        mag->magADCRaw[Y] = (int16_t)((buf[3] << 8 | buf[4]) & 0x7FFF);
        mag->magADCRaw[Z] = (int16_t)((buf[5] << 8 | buf[6]) & 0x7FFF);
    } else {
        // plus sign
        mag->magADCRaw[X] = (int16_t)(buf[1] << 8 | buf[2]);
        mag->magADCRaw[Y] = (int16_t)(buf[3] << 8 | buf[4]);
        mag->magADCRaw[Z] = (int16_t)(buf[5] << 8 | buf[6]);
    } */

    // Unsigned Manual. For Res 3
    /* 
    if (buf[1] >> 6) {
        // minus sign
        mag->magADCRaw[X] = (int16_t)((buf[1] << 8 | buf[2]) & 0x3FFF);
        mag->magADCRaw[Y] = (int16_t)((buf[3] << 8 | buf[4]) & 0x3FFF);
        mag->magADCRaw[Z] = (int16_t)((buf[5] << 8 | buf[6]) & 0x3FFF);
    } else {
        // plus sign
        mag->magADCRaw[X] = (int16_t)(buf[1] << 8 | buf[2]);
        mag->magADCRaw[Y] = (int16_t)(buf[3] << 8 | buf[4]);
        mag->magADCRaw[Z] = (int16_t)(buf[5] << 8 | buf[6]);
    } */

    return true;

}
/*
static bool mlx90393Read(magDev_t * mag)
{
    const char* func_name;
    func_name = __func__;

    uint8_t buf[7] = {0};

    if (!check_ack(busReadBuf(mag->busDev, MLX90393_READ_MEASUREMENT | MLX90393_MEASURE_3D, buf, 7), func_name)){
        mag->magADCRaw[X] = 0;
        mag->magADCRaw[Y] = 0;
        mag->magADCRaw[Z] = 0;
        return false;
    }

    LOG_E(SYSTEM, "MLX90393_READ_MEASUREMENT cmd answer: 0x%02X", buf[0]);

    int16_t raw_x = (int16_t)(buf[1] << 8 | buf[2]);
    int16_t raw_y = (int16_t)(buf[3] << 8 | buf[4]);
    int16_t raw_z = (int16_t)(buf[5] << 8 | buf[6]);

    float xy_sens;
    float z_sens;

    switch(mlx90393.hallconf){
    default:
    case 0:
        xy_sens = base_xy_sens_hc0;
        z_sens = base_z_sens_hc0;
        break;
    case 0xC:
        xy_sens = base_xy_sens_hc0xc;
        z_sens = base_z_sens_hc0xc;
        break;
    }

    float gain_factor = gain_multipliers[mlx90393.gain_sel & 0x7];

    if (mlx90393.tcmp_en){
        mag->magADCRaw[X] = ((raw_x - 32768.f) * xy_sens * gain_factor * (1 << mlx90393.res_x));
    } else {
        switch(mlx90393.res_x) {
        case 0:
        case 1:
            mag->magADCRaw[X] = ((int16_t)raw_x) * xy_sens * gain_factor * (1 << mlx90393.res_x);
            break;
        case 2:
            mag->magADCRaw[X] = ((raw_x - 32768.f) * xy_sens * gain_factor * (1 << mlx90393.res_x));
            break;
        case 3:
            mag->magADCRaw[X] = ((raw_x - 16384.f) * xy_sens * gain_factor * (1 << mlx90393.res_x));
            break;
        }
    }

    if (mlx90393.tcmp_en){
        mag->magADCRaw[Y] = ((raw_y - 32768.f) * xy_sens * gain_factor * (1 << mlx90393.res_y));
    } else {
        switch(mlx90393.res_y) {
        case 0:
        case 1:
            mag->magADCRaw[Y] = ((int16_t)raw_y) * xy_sens * gain_factor * (1 << mlx90393.res_y);
            break;
        case 2:
            mag->magADCRaw[Y] = ((raw_y - 32768.f) * xy_sens * gain_factor * (1 << mlx90393.res_y));
            break;
        case 3:
            mag->magADCRaw[Y] = ((raw_y - 16384.f) * xy_sens * gain_factor * (1 << mlx90393.res_y));
            break;
        }
    }

    if (mlx90393.tcmp_en){
        mag->magADCRaw[Z] = ((raw_z - 32768.f) * z_sens * gain_factor * (1 << mlx90393.res_z) );
    } else {
        switch(mlx90393.res_z) {
        case 0:
        case 1:
            mag->magADCRaw[Z] = ((int16_t)raw_z) * z_sens * gain_factor * (1 << mlx90393.res_z);
            break;
        case 2:
            mag->magADCRaw[Z] = ((raw_z - 32768.f) * z_sens * gain_factor * (1 << mlx90393.res_z));
            break;
        case 3:
            mag->magADCRaw[Z] = ((raw_z - 16384.f) * z_sens * gain_factor * (1 << mlx90393.res_z));
            break;
        }
    }

    return true;
}
*/

static bool deviceDetect(magDev_t * mag)
{
    LOG_E(SYSTEM, "Detecting MLX90393...");

    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        uint8_t sig = 0;

        bool ack = busRead(mag->busDev, MLX90393_RESET, &sig);
        delay(20);
 
        if (ack && ((sig & MLX90393_RESET_FLAG) == MLX90393_RESET_FLAG)) { // Check Reset Flag -> MLX90393
            LOG_E(SYSTEM, "MLX90393 found.");
            return true;
        }
    }
    LOG_E(SYSTEM, "MLX90393 NOT found.");
    return false;
}

//--------------------------------------------------
static bool mlx90393Init(magDev_t * mag)
{
    LOG_E(SYSTEM, "START INIT MLX90393.");
    const char* func_name;
    func_name = __func__;

    uint8_t sig = 0;

    nop_command(mag);
    delay(20);
    nop_command(mag);

    if (!check_ack(busRead(mag->busDev, MLX90393_EXIT_MODE, &sig), func_name)){
        LOG_E(SYSTEM, "Exit from previous mode success.");
    } else {
        LOG_E(SYSTEM, "Can't exit from previous mode!");
    }

    uint16_t reg_val = 0;
    uint8_t buf[REG_BUF_LEN] = {0};

    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | GAIN_SEL_HALLCONF_REG, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    delay(20);

    //buf[0] - status byte
    reg_val = ((uint16_t)buf[1] << 8) | buf[2];

    LOG_E(SYSTEM, "MLX90393_READ_REGISTER 1: cmd answer: 0x%02X value: 0x%04X", buf[0], reg_val);

    mlx90393.gain_sel = (reg_val & GAIN_SEL_MASK) >> GAIN_SEL_SHIFT;
    mlx90393.hallconf = (reg_val & HALLCONF_MASK) >> HALLCONF_SHIFT;

    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | TCMP_EN_REG, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    delay(20);

    //buf[0] - status byte
    reg_val = ((uint16_t)buf[1] << 8) | buf[2];

    LOG_E(SYSTEM, "MLX90393_READ_REGISTER 2: cmd answer: 0x%02X value: 0x%04X", buf[0], reg_val);

    mlx90393.tcmp_en = (reg_val & TCMP_EN_MASK) >> TCMP_EN_SHIFT;

    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | RES_XYZ_OSR_DIG_FLT_REG, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    delay(20);

    //buf[0] - status byte
    reg_val = ((uint16_t)buf[1] << 8) | buf[2];

    LOG_E(SYSTEM, "MLX90393_READ_REGISTER 3: cmd answer: 0x%02X value: 0x%04X", buf[0], reg_val);

    // END READING REGISTERS...

    delay(20);

    uint8_t res_xyz = (reg_val & RES_XYZ_MASK) >> RES_XYZ_SHIFT;
    mlx90393.res_x = (res_xyz >> 0) & 0x3;
    mlx90393.res_y = (res_xyz >> 2) & 0x3;
    mlx90393.res_z = (res_xyz >> 4) & 0x3;

    // 1 Tl = 10000 Gs
    // 1 mTl = 10 Gs
    // 1 mTl = 10000 mGs
    // 1 mkTl = 10 mGs

    // REGISTER 1. Recommended 7. (+- 49Gs) 
    // Gain Selector (3 bits)
    // For Hall_Conf 0xC And Resolution 0
    // 0 - .787 mkTl per LSB
    // 7 - .157 mkTl per LSB
    // For Hall_Conf 0xC And Resolution 3
    // 0 - 6.0097 mkTl per LSB
    // 7 - 1.202 mkTl per LSB
    
    if (!mlx90393SetGainSel(mag, 7)) {
        LOG_E(SYSTEM, "mlx90393SetGainSel unsuccessfull!");
        return false;
    }
    delay(20);
    nop_command(mag);

    // REGISTER 1. Recommended 12 or 0.
    // Hall Plate spinning rate adjustment (4 bits)
    if (!mlx90393SetHallConf(mag, 12)) {
        LOG_E(SYSTEM, "mlx90393SetHallConf unsuccessfull!");
        return false;
    }
    delay(20);
    nop_command(mag);

    // REGISTER 1. Recommended 1.
    // Activate Z-axis (1 bit)
    if (!mlx90393SetZSeries(mag, 1)) {
        LOG_E(SYSTEM, "mlx90393SetZSeries unsuccessfull!");
        return false;
    }
    delay(20);
    nop_command(mag);

    // REGISTER 2. 
    // Burst Data Rate (6 bit). Time of rate = BURST_DATA_RATE * 20ms. For inav=100ms (BDR=5). 1 and 3 - good too.
    if (!mlx90393SetBurstDataRate(mag, 4)) {
        LOG_E(SYSTEM, "mlx90393SetBurstDataRate unsuccessfull!");
        return false;
    }
    delay(20);
    nop_command(mag);

    // REGISTER 2. 0 or 1
    // Temperature compensation (1 bit)
    if (!mlx90393SetTemperatureCompensation(mag, 0)) {
        LOG_E(SYSTEM, "mlx90393SetTemperatureCompensation unsuccessfull!");
        return false;
    }
    delay(20);
    nop_command(mag);

    // REGISTER 3. (0-3)
    // Resolution for 3 axis (2 bits per axis)
    //              Temperature compensation: 0                         1
    // Res0. +- 2^15. 0mkTl=0LSB. Encoding: 2's complement.     0mkTl = 2^15LSB Encoding: unsigned.
    // Res1. +- 2^15. 0mkTl=0LSB. Encoding: 2's complement.     0mkTl = 2^15LSB Encoding: unsigned.
    // Res2. +- 22000. 0mkTl=2^15LSB. Encoding: unsigned.               NONE
    // Res3. +- 11000. 0mkTl=2^14LSB. Encoding: unsigned.               NONE
    
    // Res0 gain 7 - +- 51 Gs. (+- 0.0051 Tl)
    // Res3 gain 7 - +-138 Gs.
    // Res0 gain 0 - +-257 Gs.
    // Res3 gain 0 - +-692 Gs. (+- 0.0692 Tl)

    // Earth                    .5 Gs   (0.00005 Tl) 50mkTl
    // Small Magnet(Ferrit)     100 Gs  (0.01 Tl)    10000mkTl
    // Small Magnet(Neodium)    2000 Gs (0.2 Tl)     200000mkTl
    
    // Earth           0.25 Gs - 0.65 Gs.

    if (!mlx90393SetResolution(mag, 0, 0, 0)) {
        LOG_E(SYSTEM, "mlx90393SetResolution unsuccessfull!");
        return false;
    }
    delay(20);
    nop_command(mag);

    // Digital filtering/OSR to BDR
    //          OSR:    0   1   2   3
    //                  --------------
    // Filtering 5:     1   1   2   3
    // Filtering 6:     1   2   3   ~>5
    // Filtering 7:     2   3   ~>5   ~>10

    // REGISTER 3. 0-3. 3 - maximum OSR. Affects BDR
    // Oversampling (2 bits).
    if (!mlx90393SetOverSampling(mag, 3)) {
        LOG_E(SYSTEM, "mlx90393SetOverSampling unsuccessfull!");
        return false;
    }
    delay(20);
    nop_command(mag);

    // REGISTER 3. 0-7. 7 - maximum. Affects BDR
    // Digital Filtering
    if (!mlx90393SetDigitalFiltering(mag, 5)) {
        LOG_E(SYSTEM, "mlx90393SetDigitalFiltering unsuccessfull!");
        return false;
    }
    delay(20);
    nop_command(mag);


    // READ REGISTERS AGAIN ==============================================================================
    // 1 REGISTER
    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | GAIN_SEL_HALLCONF_REG, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    delay(20);
    LOG_E(SYSTEM, "MLX90393_READ_REGISTER 1 AGAIN(Need 0x00FC): cmd answer: 0x%02X value: 0x%04X", buf[0], ((uint16_t)buf[1] << 8) | buf[2]);

    // 2 REGISTER
    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | TCMP_EN_REG, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    delay(20);
    LOG_E(SYSTEM, "MLX90393_READ_REGISTER 2 AGAIN(Need 0x62C1): cmd answer: 0x%02X value: 0x%04X", buf[0], ((uint16_t)buf[1] << 8) | buf[2]);

    // 3 REGISTER
    if (!check_ack(busExtReadBuf(mag->busDev, ((uint16_t)MLX90393_READ_REGISTER << 8) | RES_XYZ_OSR_DIG_FLT_REG, buf, REG_BUF_LEN), func_name)){
        return false;
    }

    delay(20);
    LOG_E(SYSTEM, "MLX90393_READ_REGISTER 3 AGAIN: cmd answer: 0x%02X value: 0x%04X", buf[0], ((uint16_t)buf[1] << 8) | buf[2]);

    // END READING REGISTERS AGAIN...

    if (!check_ack(busRead(mag->busDev, MLX90393_START_BURST_MODE | MLX90393_MEASURE_3D, &sig), func_name)){
        return false;
    }
    delay(20);

    LOG_E(SYSTEM, "MLX90393_START_BURST_MODE cmd answer: 0x%02X", sig);

    return true;
}

// ==========================================================================

bool mlx90393Detect(magDev_t * mag)
{
    mag->busDev = busDeviceInit(BUSTYPE_ANY, DEVHW_MLX90393, mag->magSensorToUse, OWNER_COMPASS);
    if (mag->busDev == NULL) {
        return false;
    }

    if (!deviceDetect(mag)) {
        busDeviceDeInit(mag->busDev);
        return false;
    }

    mag->init = mlx90393Init;
    mag->read = mlx90393Read;

    return true;
}


#endif
