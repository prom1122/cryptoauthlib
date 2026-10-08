/**
 * \file
 * \brief  definitions for I2C for Arduino
 *
 * \copyright (c) 2018 Gabriel Notman.
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip software
 * and any derivatives exclusively with Microchip products. It is your
 * responsibility to comply with third party license terms applicable to your
 * use of third party software (including open source software) that may
 * accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
 * PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT,
 * SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE
 * OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF
 * MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE
 * FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL
 * LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED
 * THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR
 * THIS SOFTWARE.
 */

 #include "Arduino.h"
 #include "Wire.h"
 
 #include "arduino_hal_select.h"
 #include "atca_hal.h"
 #include "atca_iface.h"
 #include "atca_status.h"
 #include "i2c_arduino.h"
 
 #if defined(USE_ARDUINO_I2C)
 
 extern "C" {
 
 ATCA_STATUS hal_i2c_init(void *hal, ATCAIfaceCfg *cfg);
 ATCA_STATUS hal_i2c_post_init(ATCAIface iface);
 ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t *txdata, int txlength);
 ATCA_STATUS hal_i2c_receive(ATCAIface iface, uint8_t *rxdata, uint16_t *rxlength);
 ATCA_STATUS hal_i2c_wake(ATCAIface iface);
 ATCA_STATUS hal_i2c_idle(ATCAIface iface);
 ATCA_STATUS hal_i2c_sleep(ATCAIface iface);
 ATCA_STATUS hal_i2c_release(void *hal_data);
 ATCA_STATUS hal_i2c_discover_buses(int i2c_buses[], int max_buses);
 ATCA_STATUS hal_i2c_discover_devices(int busNum, ATCAIfaceCfg *cfg, int *found);
 
 };
 
 
 ATCA_STATUS hal_i2c_init(void *hal, ATCAIfaceCfg *cfg)
 {
     if (cfg->iface.atcai2c.bus >= MAX_I2C_BUSES) {
         return ATCA_COMM_FAIL;
     }

    if (cfg->iface.atcai2c.bus == 1) {
        Wire1.begin();
        Wire1.setClock(cfg->iface.atcai2c.baud);
    } else {
        Wire.begin();
        Wire.setClock(cfg->iface.atcai2c.baud);
    }
     return ATCA_SUCCESS;
 }
 
 ATCA_STATUS hal_i2c_post_init(ATCAIface iface)
 {
     return ATCA_SUCCESS;
 }
 
 
 ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t *txdata, int txlength)
 {
     ATCAIfaceCfg *cfg = atgetifacecfg(iface);
 
     txdata[0] = 0x03;
     txlength++;
 
     ATCA_STATUS result = ATCA_SUCCESS;
 
     if (cfg->iface.atcai2c.bus == 1) {
         Wire1.beginTransmission(cfg->iface.atcai2c.slave_address >> 1);
         if (Wire1.write(txdata, txlength) != (size_t)txlength) {
             result = ATCA_COMM_FAIL;
         }
         if (Wire1.endTransmission() != 0) {
             result = ATCA_COMM_FAIL;
         }
     } else {
         Wire.beginTransmission(cfg->iface.atcai2c.slave_address >> 1);
         if (Wire.write(txdata, txlength) != (size_t)txlength) {
             result = ATCA_COMM_FAIL;
         }
         if (Wire.endTransmission() != 0) {
             result = ATCA_COMM_FAIL;
         }
     }
 
     return result;
 }
 
 
 ATCA_STATUS hal_i2c_receive(ATCAIface iface, uint8_t *rxdata, uint16_t *rxlength)
 {
     ATCAIfaceCfg *cfg = atgetifacecfg(iface);
     int retries = cfg->rx_retries;
     int status = ATCA_COMM_FAIL;
     uint16_t rxdata_max_size = *rxlength;
 
     *rxlength = 0;
 
     if (rxdata_max_size < 1) {
         return ATCA_SMALL_BUFFER;
     }
 
     while (retries-- > 0 && status != ATCA_SUCCESS) {
         uint8_t readCount = 0;
         if (cfg->iface.atcai2c.bus == 1) {
             readCount = Wire1.requestFrom((int)(cfg->iface.atcai2c.slave_address >> 1), 1);
             if (readCount == 1) {
                 rxdata[0] = Wire1.read();
                 status = ATCA_SUCCESS;
             } else {
                 status = ATCA_COMM_FAIL;
             }
         } else {
             readCount = Wire.requestFrom((int)(cfg->iface.atcai2c.slave_address >> 1), 1);
             if (readCount == 1) {
                 rxdata[0] = Wire.read();
                 status = ATCA_SUCCESS;
             } else {
                 status = ATCA_COMM_FAIL;
             }
         }
     }
 
     if (status != ATCA_SUCCESS) {
         return (ATCA_STATUS)status;
     }
 
     if (rxdata[0] < ATCA_RSP_SIZE_MIN) {
         return ATCA_INVALID_SIZE;
     }
     if (rxdata[0] > rxdata_max_size) {
         return ATCA_SMALL_BUFFER;
     }
 
     uint8_t len = rxdata[0] - 1;
     int readCount = 0;
     if (cfg->iface.atcai2c.bus == 1) {
         readCount = Wire1.requestFrom((int)(cfg->iface.atcai2c.slave_address >> 1), (int)len);
         if (readCount == (int)len) {
             for (uint8_t i = 1; i <= len; i++) {
                 rxdata[i] = Wire1.read();
             }
             status = ATCA_SUCCESS;
         } else {
             status = ATCA_COMM_FAIL;
         }
     } else {
         readCount = Wire.requestFrom((int)(cfg->iface.atcai2c.slave_address >> 1), (int)len);
         if (readCount == (int)len) {
             for (uint8_t i = 1; i <= len; i++) {
                 rxdata[i] = Wire.read();
             }
             status = ATCA_SUCCESS;
         } else {
             status = ATCA_COMM_FAIL;
         }
     }
 
     if (status == ATCA_SUCCESS) {
         *rxlength = rxdata[0];
     }
 
     return (ATCA_STATUS)status;
 }
 
 
 ATCA_STATUS hal_i2c_wake(ATCAIface iface)
 {
     ATCAIfaceCfg *cfg = atgetifacecfg(iface);
     int retries = cfg->rx_retries;
     uint32_t bdrt = cfg->iface.atcai2c.baud;

     uint8_t data[4];
     memset(data, 0, sizeof(data));
 
     if (cfg->iface.atcai2c.bus == 1) {
         Wire1.setClock(100000);
         // send wake
         Wire1.beginTransmission(0x00);
         Wire1.endTransmission(false);
     } else {
         Wire.setClock(100000);
         Wire.beginTransmission(0x00);
         Wire.endTransmission(false);
     }
 
     atca_delay_us(cfg->wake_delay);
 
     bool success = false;
     while (retries-- > 0 && !success) {
         if (cfg->iface.atcai2c.bus == 1) {
             if (Wire1.requestFrom((int)(cfg->iface.atcai2c.slave_address >> 1), (int)sizeof(data)) == (int)sizeof(data)) {
                 for (uint8_t i = 0; i < sizeof(data); i++) {
                     data[i] = Wire1.read();
                 }
                 success = true;
             }
         } else {
             if (Wire.requestFrom((int)(cfg->iface.atcai2c.slave_address >> 1), (int)sizeof(data)) == (int)sizeof(data)) {
                 for (uint8_t i = 0; i < sizeof(data); i++) {
                     data[i] = Wire.read();
                 }
                 success = true;
             }
         }
     }
 
     if (cfg->iface.atcai2c.bus == 1) {
         Wire1.setClock(bdrt);
     } else {
         Wire.setClock(bdrt);
     }
 
     return hal_check_wake(data, 4);
 }
 
 
 ATCA_STATUS hal_i2c_idle(ATCAIface iface)
 {
     ATCAIfaceCfg *cfg = atgetifacecfg(iface);
     ATCA_STATUS result = ATCA_SUCCESS;

      
     if (cfg->iface.atcai2c.bus == 1) {
         Wire1.beginTransmission(cfg->iface.atcai2c.slave_address >> 1);
         if (Wire1.write(0x02) != 1) result = ATCA_COMM_FAIL;
         if (Wire1.endTransmission() != 0) result = ATCA_COMM_FAIL;
     } else {
         Wire.beginTransmission(cfg->iface.atcai2c.slave_address >> 1);
         if (Wire.write(0x02) != 1) result = ATCA_COMM_FAIL;
         if (Wire.endTransmission() != 0) result = ATCA_COMM_FAIL;
     }
     atca_delay_ms(1);
     return result;
 }
 
 
 ATCA_STATUS hal_i2c_sleep(ATCAIface iface)
 {
     ATCAIfaceCfg *cfg = atgetifacecfg(iface);
     ATCA_STATUS result = ATCA_SUCCESS;
     
 
     if (cfg->iface.atcai2c.bus == 1) {
         Wire1.beginTransmission(cfg->iface.atcai2c.slave_address >> 1);
         if (Wire1.write(0x01) != 1) result = ATCA_COMM_FAIL;
         if (Wire1.endTransmission() != 0) result = ATCA_COMM_FAIL;
     } else {
         Wire.beginTransmission(cfg->iface.atcai2c.slave_address >> 1);
         if (Wire.write(0x01) != 1) result = ATCA_COMM_FAIL;
         if (Wire.endTransmission() != 0) result = ATCA_COMM_FAIL;
     }
     atca_delay_ms(1);
     return result;
 }
 
 
 ATCA_STATUS hal_i2c_release(void *hal_data)
 {
     // If needed, we could end the I2C, but typically there's no tear-down
     return ATCA_SUCCESS;
 }
 
 ATCA_STATUS hal_i2c_discover_buses(int i2c_buses[], int max_buses)
 {
     return ATCA_UNIMPLEMENTED;
 }
 
 ATCA_STATUS hal_i2c_discover_devices(int busNum, ATCAIfaceCfg *cfg, int *found )
 {
     return ATCA_UNIMPLEMENTED;
 }
 
 #endif // USE_ARDUINO_I2C
 