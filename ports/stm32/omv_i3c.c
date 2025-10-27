/*
 * Copyright (C) 2023-2024 OpenMV, LLC.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Any redistribution, use, or modification in source or binary form
 *    is done solely for personal benefit and not for any commercial
 *    purpose or for monetary gain. For commercial licensing options,
 *    please contact openmv@openmv.io
 *
 * THIS SOFTWARE IS PROVIDED BY THE LICENSOR AND COPYRIGHT OWNER "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE LICENSOR OR COPYRIGHT
 * OWNER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * STM32 I3C driver.
 */
#if defined(STM32N6)

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "py/mphal.h"
#include "py/runtime.h"

#include "omv_portconfig.h"
#include "omv_boardconfig.h"
#include "omv_gpio.h"
#include "omv_common.h"
#include "omv_i3c.h"

#include "stm32n6xx_hal.h"
#include "stm32n6xx_util_i3c.h"

#define I3C_SCAN_TIMEOUT    (10)
#define I3C_XFER_TIMEOUT    (1000)

static I3C_HandleTypeDef i3c1_handle = {0};
static I3C_HandleTypeDef i3c2_handle = {0};

static I3C_XferTypeDef context_buffers[2];
/* Buffer used for transmission */
static uint8_t tx_buffer[0x0F];
/* Buffer used for data reception */
static uint8_t rx_buffer[400];
/* Buffer used by HAL to compute control data for the Private Communication */
static uint32_t control_buffer[0xF];

int omv_i3c_init(omv_i2c_t *i3c, uint32_t bus_id, uint32_t speed) {
    i3c->id = bus_id;
    i3c->initialized = false;

    I3C_FifoConfTypeDef fifo_config = {0};
    I3C_CtrlConfTypeDef ctrl_config = {0};

    I3C_HandleTypeDef *base;

    switch (bus_id) {
        #if defined(OMV_I3C1_ID)
        case 1: {
            base = &I2CHandle1;
            base->Instance = I3C1;
            i3c->scl_pin = OMV_I3C1_SCL_PIN;
            i3c->sda_pin = OMV_I3C1_SDA_PIN;
            break;
        }
        #endif
        #if defined(OMV_I3C2_ID)
        case 2: {
            base = &i3c2_handle;
            base->Instance = I3C2;
            i3c->scl_pin = OMV_I3C2_SCL_PIN;
            i3c->sda_pin = OMV_I3C2_SDA_PIN;
            break;
        }
        #endif
        default:
            return -1;
    }

    base->Mode = HAL_I3C_MODE_CONTROLLER;
    base->Init.CtrlBusCharacteristic.SDAHoldTime = HAL_I3C_SDA_HOLD_TIME_1_5;
    base->Init.CtrlBusCharacteristic.WaitTime = HAL_I3C_OWN_ACTIVITY_STATE_0;
    base->Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x7c;
    base->Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x7c;
    base->Init.CtrlBusCharacteristic.SCLODLowDuration = 0x7c;
    base->Init.CtrlBusCharacteristic.SCLI2CHighDuration = 0x00;
    base->Init.CtrlBusCharacteristic.BusFreeDuration = 0x0c;
    base->Init.CtrlBusCharacteristic.BusIdleDuration = 0xf8;

    HAL_I3C_DeInit(base);
    if (HAL_I3C_Init(base) != HAL_OK) {
        i3c->inst = NULL;
        i3c->scl_pin = NULL;
        i3c->sda_pin = NULL;
        return -1;
    }

    /** Configure FIFO
     */
    fifo_config.RxFifoThreshold = HAL_I3C_RXFIFO_THRESHOLD_1_4;
    fifo_config.TxFifoThreshold = HAL_I3C_TXFIFO_THRESHOLD_1_4;
    fifo_config.ControlFifo = HAL_I3C_CONTROLFIFO_DISABLE;
    fifo_config.StatusFifo = HAL_I3C_STATUSFIFO_DISABLE;
    if (HAL_I3C_SetConfigFifo(base, &fifo_config) != HAL_OK) {
        i3c->inst = NULL;
        i3c->scl_pin = NULL;
        i3c->sda_pin = NULL;
        return -1;
    }

    /** Configure controller
     */
    ctrl_config.DynamicAddr = 0;
    ctrl_config.StallTime = 0x00;
    ctrl_config.HotJoinAllowed = DISABLE;
    ctrl_config.ACKStallState = DISABLE;
    ctrl_config.CCCStallState = DISABLE;
    ctrl_config.TxStallState = DISABLE;
    ctrl_config.RxStallState = DISABLE;
    ctrl_config.HighKeeperSDA = DISABLE;
    if (HAL_I3C_Ctrl_Config(base, &ctrl_config) != HAL_OK) {
        i3c->inst = NULL;
        i3c->scl_pin = NULL;
        i3c->sda_pin = NULL;
        return -1;
    }

    i3c->inst = (omv_i2c_dev_t) base;

    if (omv_i3c_set_scl(i3c, speed) != 0) {
        return -1;
    }

    i3c->initialized = true;
    return 0;
}

int omv_i3c_deinit(omv_i2c_t *i3c) {
    I3C_HandleTypeDef *base = (I3C_HandleTypeDef *) i3c->inst;
    if (i3c->initialized) {
        HAL_I2C_DeInit(base);
        base->Instance = NULL;
    }
    i3c->inst = NULL;
    i3c->scl_pin = NULL;
    i3c->sda_pin = NULL;
    i3c->initialized = false;
    return 0;
}

int omv_i3c_set_scl(omv_i2c_t *i3c, uint32_t speed) {
    I3C_ctrl_timingTypeDef ctrl_timing;
    LL_I3C_ctrl_bus_configTypeDef ctrl_bus_config;
    I3C_HandleTypeDef *base = (I3C_HandleTypeDef *) i3c->inst;
    if (!i3c->initialized || speed != i3c->speed) {
        //change clock speed to 12.5MHz to read
        uint64_t periph_clock = (base == &i3c1_handle) ? RCC_PERIPHCLK_I3C1 : RCC_PERIPHCLK_I3C2;
        ctrl_timing.clockSrcFreq = HAL_RCCEx_GetPeriphCLKFreq(periph_clock);
        ctrl_timing.i3cPPFreq = speed;
        ctrl_timing.i2cODFreq = 0;
        ctrl_timing.dutyCycle = 50;
        ctrl_timing.busType = I3C_PURE_I3C_BUS;

        /* Calculate the new timing for Controller */
        I3C_ctrl_timingComputation(&ctrl_timing, &ctrl_bus_config);

        /* Update Controller Bus characteristic */
        HAL_I3C_Ctrl_BusCharacteristicConfig(base, &ctrl_bus_config);

        i3c->speed = speed;
    }

    return 0;
}

static int omv_i3c_gen_dyn_addr() {
    static devices_count = 0;

    uint8_t dyn_addr = devices_count + 0x09;

    devices_count++;

    return dyn_addr;
}

int omv_i3c_assign(omv_i2c_t *i3c, uint8_t static_addr, uint8_t *ref_dyn_addr) {
    I3C_HandleTypeDef *base = (I3C_HandleTypeDef *) i3c->inst;

    uint8_t dyn_addr = omv_i3c_gen_dyn_addr();
    uint8_t SETDASA_DATA[1] = { (dyn_addr << 1) };
    /* Descriptor for direct write SETDASA CCC */
    I3C_CCCTypeDef SETDSA_CCC[] = { {static_addr, I3C_CCC_SETDASA, {SETDASA_DATA, 1}, HAL_I3C_DIRECTION_WRITE} };
    /*Returns error if slave static address is invalid */
    if (!static_addr) {
        return -1;
    }

    /* Send a RSTDAA to reset previous dynamic address the target */
    context_buffers[0].CtrlBuf.pBuffer = control_buffer;
    context_buffers[0].CtrlBuf.Size = 2;
    context_buffers[0].TxBuf.pBuffer = tx_buffer;
    context_buffers[0].TxBuf.Size = 1;

    /* Add context buffer Set CCC frame in Frame context */
    if (HAL_I3C_AddDescToFrame(base, SETDSA_CCC, NULL, context_buffers, 1, I3C_DIRECT_WITHOUT_DEFBYTE_RESTART) != HAL_OK) {
        return -1;
    }

    if (HAL_I3C_Ctrl_TransmitCCC_IT(base, context_buffers) != HAL_OK) {
        return -1;
    }

    while (HAL_I3C_GetState(base) != HAL_I3C_STATE_READY) {
        ;
    }

    /* After a dynamic address has been assigned, the sensor is recognized as an I3C device */
    /* Check if the LSM6D sensor is ready to communicate in I3C */
    if (HAL_I3C_Ctrl_IsDeviceI3C_Ready(base, dyn_addr, 300, 1000) != HAL_OK) {
        return -1;
    }

    devices_count++;
    *ref_dyn_addr = dyn_addr;

    return 0;
}

int omv_i3c_scan_assign(omv_i2c_t *i3c, uint8_t *list, uint8_t size) {
    I3C_HandleTypeDef *base = (I3C_HandleTypeDef *) i3c->inst;

    if (HAL_I3C_Ctrl_DynAddrAssign_IT(base, I3C_RSTDAA_THEN_ENTDAA) != HAL_OK) {
        return -1;
    }

    return 0;
}

void HAL_I3C_TgtReqDynamicAddrCallback(I3C_HandleTypeDef *hi3c, uint64_t targetPayload) {
    HAL_I3C_Ctrl_SetDynAddr(hi3c, omv_i3c_gen_dyn_addr());
}

int omv_i3c_enable(omv_i2c_t *i3c, bool enable) {
    I3C_HandleTypeDef *base = (I3C_HandleTypeDef *) i3c->inst;
    if (enable) {
        if (HAL_I3C_Init(base) != HAL_OK) {
            return -1;
        }
    } else {
        if (HAL_I3C_DeInit(base) != HAL_OK) {
            return -1;
        }
    }

    return 0;
}

int omv_i3c_reset(omv_i2c_t *i3c, uint8_t tgt_addr) {
    uint8_t command = I3C_CCC_RSTDAA(true);
    uint8_t option = I3C_BROADCAST_WITHOUT_DEFBYTE_RESTART;
    I3C_HandleTypeDef *base = (I3C_HandleTypeDef *) i3c->inst;

    if (tgt_addr != 0x7E) {
        if (tgt_addr < 0x29 || tgt_addr >= 0x78) {
            return -1;
        }
        command = I3C_CCC_RSTDAA(false);
        option = I3C_DIRECT_WITHOUT_DEFBYTE_RESTART;
    }

    I3C_CCCTypeDef RST_CCC[] = { {tgt_addr, command, {NULL, 0}, HAL_I3C_DIRECTION_WRITE} };

    if (HAL_I3C_AddDescToFrame(base, RST_CCC, NULL, context_buffers, 1, option) != HAL_OK) {
        return -1;
    }

    if (HAL_I3C_Ctrl_TransmitCCC(base, context_buffers, 1000) != HAL_OK) {
        return -1;
    }

    while (HAL_I3C_GetState(base) != HAL_I3C_STATE_READY) {
        ;
    }

    // __HAL_I3C_RESET_HANDLE_STATE(base);

    return 0;
}

// static int omv_i3c_write_read_bytes(omv_i2c_t *i3c, uint8_t tgt_addr, uint8_t *reg_addr, int reg_size, uint8_t *data, int data_size) {
//     int32_t pos;
//     i3c_xfer_t xfer = {0};
//     I3C_HandleTypeDef *base = (I3C_HandleTypeDef *) i3c->inst;

//      uint8_t tx_buffer[2];
//      I3C_XferTypeDef context_buffers[2];
//      uint32_t control_buffer[2];

//      I3C_PrivateTypeDef aPrivateDescriptor[2] = \
//      {
//              {tgt_addr, {reg_addr, reg_size}, {NULL, 0}, HAL_I3C_DIRECTION_WRITE},
//              {tgt_addr, {NULL, 0}, {data, data_size}, HAL_I3C_DIRECTION_READ}
//      };

//      context_buffers[0].CtrlBuf.pBuffer = control_buffer;
//      context_buffers[0].CtrlBuf.Size    = 2;
//      context_buffers[0].TxBuf.pBuffer   = tx_buffer;
//      context_buffers[0].TxBuf.Size      = 2;
//      context_buffers[0].RxBuf.pBuffer   = data;
//      context_buffers[0].RxBuf.Size      = data_size;

//      if (HAL_I3C_AddDescToFrame(base,NULL, &aPrivateDescriptor[0], &context_buffers[0], context_buffers[0].CtrlBuf.Size, I3C_PRIVATE_WITH_ARB_RESTART) != HAL_OK) {
//              return -1;
//      }

//      if (HAL_I3C_Ctrl_MultipleTransfer_IT(base, &context_buffers[0]) != HAL_OK) {
//              /* Error_Handler() function is called when error occurs. */
//              return -1;
//      }

//      while (HAL_I3C_GetState(base) != HAL_I3C_STATE_READY);

//     return 0;
// }

int omv_i3c_readb(omv_i2c_t *i3c, uint8_t tgt_addr, uint8_t reg_addr,  uint8_t *reg_data) {
    int ret = 0;
    ret |= omv_i3c_write_bytes(i3c, tgt_addr, &reg_addr, 1, OMV_I2C_XFER_NO_STOP);
    ret |= omv_i3c_read_bytes(i3c, tgt_addr, reg_data, 1, OMV_I2C_XFER_NO_FLAGS);
    return ret;
}

int omv_i3c_writeb(omv_i2c_t *i3c, uint8_t tgt_addr, uint8_t reg_addr, uint8_t reg_data) {
    int ret = 0;
    uint8_t buf[] = {reg_addr, reg_data};
    ret |= omv_i3c_write_bytes(i3c, tgt_addr, buf, 2, OMV_I2C_XFER_NO_FLAGS);
    return ret;
}

int omv_i3c_readb2(omv_i2c_t *i3c, uint8_t tgt_addr, uint16_t reg_addr, uint8_t *reg_data) {
    int ret = 0;
    uint8_t buf[] = {(reg_addr >> 8), reg_addr};
    ret |= omv_i3c_write_bytes(i3c, tgt_addr, buf, 2, OMV_I2C_XFER_NO_STOP);
    ret |= omv_i3c_read_bytes(i3c, tgt_addr, reg_data, 1, OMV_I2C_XFER_NO_FLAGS);
    return ret;
}

int omv_i3c_writeb2(omv_i2c_t *i3c, uint8_t tgt_addr, uint16_t reg_addr, uint8_t reg_data) {
    int ret = 0;
    uint8_t buf[] = {(reg_addr >> 8), reg_addr, reg_data};
    ret |= omv_i3c_write_bytes(i3c, tgt_addr, buf, 3, OMV_I2C_XFER_NO_FLAGS);
    return ret;
}

int omv_i3c_readw(omv_i2c_t *i3c, uint8_t tgt_addr, uint8_t reg_addr, uint16_t *reg_data) {
    int ret = 0;
    ret |= omv_i3c_write_bytes(i3c, tgt_addr, &reg_addr, 1, OMV_I2C_XFER_NO_STOP);
    ret |= omv_i3c_read_bytes(i3c, tgt_addr, (uint8_t *) reg_data, 2, OMV_I2C_XFER_NO_FLAGS);
    *reg_data = (*reg_data << 8) | (*reg_data >> 8);
    return ret;
}

int omv_i3c_writew(omv_i2c_t *i3c, uint8_t tgt_addr, uint8_t reg_addr, uint16_t reg_data) {
    int ret = 0;
    uint8_t buf[] = {reg_addr, (reg_data >> 8), reg_data};
    ret |= omv_i3c_write_bytes(i3c, tgt_addr, buf, 3, OMV_I2C_XFER_NO_FLAGS);
    return ret;
}

int omv_i3c_readw2(omv_i2c_t *i3c, uint8_t tgt_addr, uint16_t reg_addr, uint16_t *reg_data) {
    int ret = 0;
    uint8_t buf[] = {(reg_addr >> 8), reg_addr};
    ret |= omv_i3c_write_bytes(i3c, tgt_addr, buf, 2, OMV_I2C_XFER_NO_STOP);
    ret |= omv_i3c_read_bytes(i3c, tgt_addr, (uint8_t *) reg_data, 2, OMV_I2C_XFER_NO_FLAGS);
    *reg_data = (*reg_data << 8) | (*reg_data >> 8);
    return ret;
}

int omv_i3c_writew2(omv_i2c_t *i3c, uint8_t tgt_addr, uint16_t reg_addr, uint16_t reg_data) {
    int ret = 0;
    uint8_t buf[] = {(reg_addr >> 8), reg_addr, (reg_data >> 8), reg_data};
    ret |= omv_i3c_write_bytes(i3c, tgt_addr, buf, 4, OMV_I2C_XFER_NO_FLAGS);
    return ret;
}

int omv_i3c_readdw(omv_i2c_t *i3c, uint8_t tgt_addr, uint8_t reg_addr, uint32_t *reg_data) {
    int ret = 0;
    ret |= omv_i3c_write_bytes(i3c, tgt_addr, &reg_addr, 1, OMV_I2C_XFER_NO_STOP);
    ret |= omv_i3c_read_bytes(i3c, tgt_addr, (uint8_t *) reg_data, 4, OMV_I2C_XFER_NO_FLAGS);
    data_1 = (*reg_data) & 0xFF;
    data_2 = (*reg_data) & 0xFF00;
    data_3 = (*reg_data) & 0xFF0000;
    data_4 = (*reg_data) & 0xFF000000;
    *reg_data = (data_1 << 24) | (data_2 << 8) | (data_3 >> 8) | (data_4 >> 24);
    return ret;
}

int omv_i3c_writedw(omv_i2c_t *i3c, uint8_t tgt_addr, uint8_t reg_addr, uint32_t reg_data) {
    int ret = 0;
    uint8_t buf[] = {reg_addr, (reg_data >> 24), (reg_data >> 16), (reg_data >> 8), reg_data};
    ret |= omv_i3c_write_bytes(i3c, tgt_addr, buf, 5, OMV_I2C_XFER_NO_FLAGS);
    return ret;
}

int omv_i3c_readdw2(omv_i2c_t *i3c, uint8_t tgt_addr, uint16_t reg_addr, uint32_t *reg_data) {
    int ret = 0;
    uint8_t buf[] = {(reg_addr >> 8), reg_addr};
    ret |= omv_i3c_write_bytes(i3c, tgt_addr, buf, 2, OMV_I2C_XFER_NO_STOP);
    ret |= omv_i3c_read_bytes(i3c, tgt_addr, (uint8_t *) reg_data, 4, OMV_I2C_XFER_NO_FLAGS);
    data_1 = (*reg_data) & 0xFF;
    data_2 = (*reg_data) & 0xFF00;
    data_3 = (*reg_data) & 0xFF0000;
    data_4 = (*reg_data) & 0xFF000000;
    *reg_data = (data_1 << 24) | (data_2 << 8) | (data_3 >> 8) | (data_4 >> 24);
    return ret;
}

int omv_i3c_writedw2(omv_i2c_t *i3c, uint8_t tgt_addr, uint16_t reg_addr, uint32_t reg_data) {
    int ret = 0;
    uint8_t buf[] = {(reg_addr >> 8), reg_addr, (reg_data >> 24), (reg_data >> 16), (reg_data >> 8), reg_data};
    ret |= omv_i3c_write_bytes(i3c, tgt_addr, buf, 6, OMV_I2C_XFER_NO_FLAGS);
    return ret;
}

int omv_i3c_read_bytes(omv_i2c_t *i3c, uint8_t tgt_addr, uint8_t *buf, int len, uint32_t flags) {
    int32_t pos;
    i3c_xfer_t xfer = {0};
    I3C_HandleTypeDef *base = (I3C_HandleTypeDef *) i3c->inst;

    I3C_XferTypeDef context_buffers[2];
    uint32_t control_buffer[2];

    I3C_PrivateTypeDef aPrivateDescriptor[1] = {
        {tgt_addr, {NULL, 0}, {buf, len}, HAL_I3C_DIRECTION_READ}
    };

    context_buffers[0].CtrlBuf.pBuffer = control_buffer;
    context_buffers[0].CtrlBuf.Size = 1;
    context_buffers[0].RxBuf.pBuffer = buf;
    context_buffers[0].RxBuf.Size = len;

    if (HAL_I3C_AddDescToFrame(&hi3c1,
                               NULL,
                               &aPrivateDescriptor[0],
                               &context_buffers[0],
                               context_buffers[0].CtrlBuf.Size,
                               I3C_PRIVATE_WITH_ARB_RESTART) != HAL_OK) {
        // Error_Handler() function is called when error occurs.
        return -1;
    }

    /* Invoke master receive api */
    if (flags & (OMV_I2C_XFER_NO_STOP | OMV_I2C_XFER_SUSPEND)) {
        if (HAL_I3C_Ctrl_Receive_IT(base, &context_buffers[0]) != HAL_OK) {
            return -1;
        }
    } else {
        if (HAL_I3C_Ctrl_Receive(base, &context_buffers[0], I3C_XFER_TIMEOUT) != HAL_OK) {
            return -1;
        }
    }

    while (HAL_I3C_GetState(base) != HAL_I3C_STATE_READY) {
        ;
    }

    return 0;
}

int omv_i3c_write_bytes(omv_i2c_t *i3c, uint8_t tgt_addr, uint8_t *buf, int len, uint32_t flags) {
    int32_t pos;
    i3c_xfer_t xfer = {0};
    I3C_HandleTypeDef *base = (I3C_HandleTypeDef *) i3c->inst;

    I3C_XferTypeDef context_buffers[2];
    uint32_t control_buffer[2];

    I3C_PrivateTypeDef aPrivateDescriptor[1] = {
        {tgt_addr, {buf, len}, {NULL, 0}, HAL_I3C_DIRECTION_WRITE},
    };

    context_buffers[0].CtrlBuf.pBuffer = control_buffer;
    context_buffers[0].CtrlBuf.Size = 1;
    context_buffers[0].TxBuf.pBuffer = buf;
    context_buffers[0].TxBuf.Size = len;

    if (HAL_I3C_AddDescToFrame(base,
                               NULL,
                               &aPrivateDescriptor[0],
                               &context_buffers[0],
                               context_buffers[0].CtrlBuf.Size,
                               I3C_PRIVATE_WITH_ARB_STOP) != HAL_OK) {
        // Error_Handler() function is called when error occurs.
        return -1;
    }

    if (flags & (OMV_I2C_XFER_NO_STOP | OMV_I2C_XFER_SUSPEND)) {
        if (HAL_I3C_Ctrl_Transmit_IT(base, &context_buffers[0]) != HAL_OK) {
            return -1;
        }
    } else {
        if (HAL_I3C_Ctrl_Transmit(base, &context_buffers[0], I3C_XFER_TIMEOUT) != HAL_OK) {
            return -1;
        }
    }

    while (HAL_I3C_GetState(base) != HAL_I3C_STATE_READY) {
        ;
    }

    return 0;
}

#endif // STM32N6