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

#define MLX90393_MEASURE_Z                           0b00001000
#define MLX90393_MEASURE_Y                           0b00000100
#define MLX90393_MEASURE_X                           0b00000010
#define MLX90393_MEASURE_T                           0b00000001

#define MLX90393_START_BURST_MODE                    0b00010000 //uses with zyxt flags
#define MLX90393_START_WAKE_UP_ON_CHANGE_MODE        0b00100000 //uses with zyxt flags
#define MLX90393_START_MEASUREMENT_MODE              0b00110000 //uses with zyxt flags
#define MLX90393_READ_MEASUREMENT                    0b01000000 //uses with zyxt flags

#define MLX90393_RESET                               0b11110000
#define MLX90393_RESET_FLAG			     0b00000100

#define DETECTION_MAX_RETRY_COUNT   5

static bool deviceDetect(magDev_t * mag)
{
    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
	uint8_t sig = 0;
        bool ack = busRead(mag->busDev, MLX90393_RESET, &sig);
        delay(20);
 
        if (ack && ((sig & MLX90393_RESET_FLAG) == MLX90393_RESET_FLAG)) { // Check Reset Flag -> MLX90393
            return true;
        }
    }

    return false;
}

static bool mlx90393Init(magDev_t * mag)
{
    uint8_t sig = 0;

    busRead(mag->busDev, MLX90393_START_MEASUREMENT_MODE | MLX90393_MEASURE_Z | MLX90393_MEASURE_Y | MLX90393_MEASURE_X, &sig); //TODO: Start Single Measurement Mode zyx (if device was reset after deviceDetect)
    delay(20);

    return true;
}

static bool mlx90393Read(magDev_t * mag)
{
    bool ack = false;
    uint8_t buf[7] = {0};

    ack = busReadBuf(mag->busDev, MLX90393_READ_MEASUREMENT | MLX90393_MEASURE_Z | MLX90393_MEASURE_Y | MLX90393_MEASURE_X, buf, 7); //TODO: read mesurement zyx

    if (!ack) {
        return false;
    }

    mag->magADCRaw[X] = (buf[1] << 8) + buf[2]; //TODO:
    mag->magADCRaw[Y] = (buf[3] << 8) + buf[4]; //TODO:
    mag->magADCRaw[Z] = (buf[5] << 8) + buf[6]; //TODO:

    return true;
}

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
