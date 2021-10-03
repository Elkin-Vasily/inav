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

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include <string.h>

#include "platform.h"

#ifdef USE_MAG_MLX90393
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/bus.h"

#include "drivers/sensor.h"
#include "drivers/compass/compass.h"

#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(magDev_t * mag)
{
    for (int retryCount = 0; retryCount < DETECTION_MAX_RETRY_COUNT; retryCount++) {
        delay(10);

        uint8_t sig = 0;
        bool ack = busRead(mag->busDev, 0b00111111, &sig); //TODO: Start Single Measurement Mode zyxt

        if (ack && ((sig & 0b00010000) == 0)) { //TODO: No error
            return true;
        }
    }

    return false;
}

static bool mlx90393Init(magDev_t * mag)
{
    bool ack;
    UNUSED(ack);

    ack = busWrite(mag->busDev, 0b00011110, 0); //TODO: burst mode zyx
    delay(20);

    return true;
}

static bool mlx90393Read(magDev_t * mag)
{
    bool ack = false;
    uint8_t buf[9];

    ack = busReadBuf(mag->busDev, 0b01001110, buf, 9); //TODO: read mesurement zyx

    if (!ack) {
        return false;
    }

    mag->magADCRaw[X] = (buf[3] << 8) + buf[4]; //TODO:
    mag->magADCRaw[Y] = (buf[5] << 8) + buf[6]; //TODO:
    mag->magADCRaw[Z] = (buf[7] << 8) + buf[8]; //TODO:

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
