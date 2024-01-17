/*
    AK09918.cpp
    A library for Grove - IMU 9DOF(ICM20600 + AK09918)

    Copyright (c) 2018 seeed technology inc.
    Website    : www.seeed.cc
    Author     : Jerry Yip
    Create Time: 2018-06
    Version    : 0.1
    Change Log :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/
// Refactored by Nils Kiefer and Annika Wong
#include "Wire.h"
#include <AK09918.h>
#include <Arduino.h>

AK09918::AK09918() {
    _addr = AK09918_I2C_ADDR;
}

AK09918_err_type_t AK09918::initialize(AK09918_mode_type_t mode) {
    if (mode == AK09918_SELF_TEST) {
        mode = AK09918_POWER_DOWN;
    }
    _mode = mode;

    if (mode == AK09918_NORMAL) {
        return AK09918_ERR_OK;
    } else {
        return AK09918::switchMode(_mode);
    }
}

AK09918_err_type_t AK09918::isDataReady() {
    Wire1.beginTransmission(_addr);
    Wire1.write(AK09918_ST1);
    if (Wire1.endTransmission(false) != 0) { // false to keep the connection active
        return AK09918_ERR_READ_FAILED;
    }

    Wire1.requestFrom((int)_addr, 1); // request 1 byte
    if (Wire1.available()) {
        _buffer[0] = Wire1.read();
        if (_buffer[0] & AK09918_DRDY_BIT) {
            return AK09918_ERR_OK;
        } else {
            return AK09918_ERR_NOT_RDY;
        }
    } else {
        return AK09918_ERR_READ_FAILED;
    }
}

AK09918_err_type_t AK09918::isDataSkip() {
    Wire1.beginTransmission(_addr);
    Wire1.write(AK09918_ST1);
    if (Wire1.endTransmission(false) != 0) { // false to keep the connection active
        return AK09918_ERR_READ_FAILED;
    }

    Wire1.requestFrom((int)_addr, 1); // request 1 byte
    if (Wire1.available()) {
        _buffer[0] = Wire1.read();
        if (_buffer[0] & AK09918_DOR_BIT) {
            return AK09918_ERR_DOR;
        } else {
            return AK09918_ERR_OK;
        }
    } else {
        return AK09918_ERR_READ_FAILED;
    }
}

AK09918_err_type_t AK09918::getData(int32_t *axis_x, int32_t *axis_y, int32_t *axis_z) {
    AK09918_err_type_t err = AK09918::getRawData(axis_x, axis_y, axis_z);
    (*axis_x) = (*axis_x) * 15 / 100;
    (*axis_y) = (*axis_y) * 15 / 100;
    (*axis_z) = (*axis_z) * 15 / 100;

    return err;
}

AK09918_err_type_t AK09918::getRawData(int32_t *axis_x, int32_t *axis_y, int32_t *axis_z) {
    if (_mode == AK09918_NORMAL) {
        AK09918::switchMode(AK09918_NORMAL);
        bool is_end = false;
        int count = 0;
        while (!is_end) {
            if (AK09918::_getRawMode() == 0x00) {
                is_end = true;
            }
            if (count >= 15) {
                return AK09918_ERR_TIMEOUT;
            }
            count++;
            delay(1);
        }
    }

    Wire1.beginTransmission(_addr);
    Wire1.write(AK09918_HXL);
    if (Wire1.endTransmission(false) != 0) { // false to keep the connection active
        return AK09918_ERR_READ_FAILED;
    }

    Wire1.requestFrom((int)_addr, 8); // request 8 bytes
    if (Wire1.available() >= 8) {
        for (int i = 0; i < 8; i++) {
            _buffer[i] = Wire1.read();
        }
        *axis_x = (int16_t)(_buffer[1] << 8 | _buffer[0]);
        *axis_y = (int16_t)(_buffer[3] << 8 | _buffer[2]);
        *axis_z = (int16_t)(_buffer[5] << 8 | _buffer[4]);
        if (_buffer[7] & AK09918_HOFL_BIT) {
            return AK09918_ERR_OVERFLOW;
        }
        return AK09918_ERR_OK;
    } else {
        return AK09918_ERR_READ_FAILED;
    }
}

AK09918_mode_type_t AK09918::getMode() {
    return _mode;
}

AK09918_err_type_t AK09918::switchMode(AK09918_mode_type_t mode) {
    if (mode == AK09918_SELF_TEST) {
        return AK09918_ERR_WRITE_FAILED;
    }
    _mode = mode;

    Wire1.beginTransmission(_addr);
    Wire1.write(AK09918_CNTL2);
    Wire1.write(mode);
    if (Wire1.endTransmission() != 0) {
        return AK09918_ERR_WRITE_FAILED;
    }
    return AK09918_ERR_OK;
}

// 1.Set Power-down mode. (MODE[4:0] bits = “00000”)
// 2.Set Self-test mode. (MODE[4:0] bits = “10000”)
// 3.Check Data Ready or not by polling DRDY bit of ST1 register.
// 4.When Data Ready, proceed to the next step. Read measurement data. (HXL to HZH)
AK09918_err_type_t AK09918::selfTest() {
    int32_t axis_x, axis_y, axis_z;
    bool is_end = false;
    AK09918_err_type_t err;

    Wire1.beginTransmission(_addr);
    Wire1.write(AK09918_CNTL2);
    Wire1.write(AK09918_POWER_DOWN);
    if (Wire1.endTransmission() != 0) {
        return AK09918_ERR_WRITE_FAILED;
    }
    delay(1);

    Wire1.beginTransmission(_addr);
    Wire1.write(AK09918_CNTL2);
    Wire1.write(AK09918_SELF_TEST);
    if (Wire1.endTransmission() != 0) {
        return AK09918_ERR_WRITE_FAILED;
    }

    while (!is_end) {
        err = AK09918::isDataReady();
        if (err == AK09918_ERR_OK) {
            is_end = true;
        } else if (err == AK09918_ERR_READ_FAILED) {
            return AK09918_ERR_READ_FAILED;
        }
        delay(1);
    }

    // Read data and check
    Wire1.beginTransmission(_addr);
    Wire1.write(AK09918_HXL);
    if (Wire1.endTransmission(false) != 0) {
        return AK09918_ERR_READ_FAILED;
    }

    Wire1.requestFrom((int)_addr, 8);
    if (Wire1.available() >= 8) {
        for (int i = 0; i < 8; i++) {
            _buffer[i] = Wire1.read();
        }

        axis_x = (int32_t)((((int16_t)_buffer[1]) << 8) | _buffer[0]);
        axis_y = (int32_t)((((int16_t)_buffer[3]) << 8) | _buffer[2]);
        axis_z = (int32_t)((((int16_t)_buffer[5]) << 8) | _buffer[4]);

        // Check if the self-test values are within the expected range
        if ((axis_x >= -200) && (axis_x <= 200) &&
            (axis_y >= -200) && (axis_y <= 200) &&
            (axis_z >= -1000) && (axis_z <= -150)) {
            return AK09918_ERR_OK;
        } else {
            return AK09918_ERR_SELFTEST_FAILED;
        }
    } else {
        return AK09918_ERR_READ_FAILED;
    }
}

AK09918_err_type_t AK09918::reset() {
    Wire1.beginTransmission(_addr);
    Wire1.write(AK09918_CNTL3);
    Wire1.write(AK09918_SRST_BIT);
    byte error = Wire1.endTransmission();

    if (error) {
        return AK09918_ERR_WRITE_FAILED;
    }
    return AK09918_ERR_OK;
}

String AK09918::strError(AK09918_err_type_t err) {
    String result;
    switch (err) {
    case AK09918_ERR_OK:
        result = "AK09918_ERR_OK: OK";
        break;

    case AK09918_ERR_DOR:
        result = "AK09918_ERR_DOR: Data skipped";
        break;

    case AK09918_ERR_NOT_RDY:
        result = "AK09918_ERR_NOT_RDY: Not ready";
        break;

    case AK09918_ERR_TIMEOUT:
        result = "AK09918_ERR_TIMEOUT: Timeout";
        break;

    case AK09918_ERR_SELFTEST_FAILED:
        result = "AK09918_ERR_SELFTEST_FAILED: Self test failed";
        break;

    case AK09918_ERR_OVERFLOW:
        result = "AK09918_ERR_OVERFLOW: Sensor overflow";
        break;

    case AK09918_ERR_WRITE_FAILED:
        result = "AK09918_ERR_WRITE_FAILED: Fail to write";
        break;

    case AK09918_ERR_READ_FAILED:
        result = "AK09918_ERR_READ_FAILED: Fail to read";
        break;

    default:
        result = "Unknown Error";
        break;
    }
    return result;
}

uint16_t AK09918::getDeviceID() {
    uint16_t deviceID = 0;

    Wire1.beginTransmission(_addr);
    Wire1.write(AK09918_WIA1);
    if (Wire1.endTransmission(false) == 0) {
        Wire1.requestFrom(_addr, (uint8_t)2);
        if (Wire1.available() == 2) {
            deviceID = Wire1.read() << 8;
            deviceID |= Wire1.read();
        }
    }
    return deviceID;
}

uint8_t AK09918::_getRawMode() {
    Wire1.beginTransmission(_addr);
    Wire1.write(AK09918_CNTL2);
    if (Wire1.endTransmission(false) != 0) {
        return 0xFF;
    } else {
        Wire1.requestFrom(_addr, (byte)1);
        if (Wire1.available()) {
            _buffer[0] = Wire1.read();
        }
        return _buffer[0];
    }
}
