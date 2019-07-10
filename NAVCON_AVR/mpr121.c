/*
 * mpr121.c
 *
 * Created: 2017-05-29 09:57:17
 *  Author: LUKE
 */ 
/**
 * Capacityive sensor library (MPR121)
 *
 * The library assumes that the i2c bus has been initialized already.
 * It will only communicate by issuing start/stop conditions and transmitting
 * command and data requests.
 *
 * @author Matthias L. Jugel
 *
 * == LICENSE ==
 * Copyright 2015 ubirch GmbH (http://www.ubirch.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mpr121.h"
#include "I2CBase.h"

#ifndef NDEBUG
#   ifdef i2c_assert
#      undef i2c_assert
#   endif
#   define i2c_assert(e, m)
#endif



void mpr_set(uint8_t reg, uint8_t data) {
    uint8_t ecr = MPR_read_reg(MPR_DEVICE_ADDRESS, MPR_R_ECR);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_ECR, ecr & 0xC0); // stop
   MPR_write_reg(MPR_DEVICE_ADDRESS, reg, data);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_ECR, ecr); // back to configured run mode
}

inline uint8_t mpr_get(uint8_t reg) {
    return MPR_read_reg(MPR_DEVICE_ADDRESS, reg);
}

inline uint16_t mpr_status(void) {
    return MPR_read_reg16bit(MPR_DEVICE_ADDRESS, MPR_R_TOUCHSTATUS) & 0x0FFF;
}

uint16_t mpr_status_filtered(uint8_t pin) {
    return pin > 12 ? 0 : MPR_read_reg16bit(MPR_DEVICE_ADDRESS, MPR_R_TOUCHSTATUS_F + 2 * pin);
}

uint16_t mpr_baseline(uint8_t pin) {
    uint16_t baseline = pin > 12 ? 0 :MPR_read_reg(MPR_DEVICE_ADDRESS, MPR_R_BASELINE + pin);
    // Although internally the baseline value is 10-bit, users can only access the
    // 8 MSB of the 10-bit baseline value through the baseline value registers. The
    // read out from the baseline register must be left shift two bits before
    // comparing it with the 10-bit electrode data.
    return baseline << 2;
}

inline void mpr_setup(uint16_t bitmask) {
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_ECR, bitmask);
}

void mpr_set_threshold(uint8_t pin, uint8_t touch, uint8_t release) {
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_TCH_THRESHOLD + 2 * pin, touch);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_REL_THRESHOLD + 2 * pin, release);
}

void mpr_set_debounce(uint8_t touch, uint8_t release) {
    mpr_set(MPR_R_DEBOUNCE, (release << 4) | touch);
}

uint8_t mpr_reset(void) {
   
	MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_ECR, 0x00);
	
   

  /*  // sensors
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_MHD_RISING, 0x01);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_NHD_RISING, 0x01);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_NCL_RISING, 0x10);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_FDL_RISING, 0x20);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_MHD_FALLING, 0x01);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_NHD_FALLING, 0x01);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_NCL_FALLING, 0x10);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_FDL_FALLING, 0x20);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_NHD_TOUCHED, 0x01);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_NCL_TOUCHED, 0x10);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_FDL_TOUCHED, 0xFF);
*/
  //each electrode configuration
     //MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_CDC_CONFIG, 0b01000001); 
    // MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_CDT_CONFIG, 0b00110100); 
	// MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_BASELINE, 20); 
	 //MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_BASELINE + 1, 20); 
	/// MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_BASELINE + 2, 20); 
	// MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_BASELINE + 3, 20); 

    // autoconfig
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_AUTOCONFIG, 0b01011101);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_AUTOCONFIG + 1, 0x00);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_USL, 196);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_LSL, 127);
   MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_TARGET_LEVEL, 176);

   
  uint8_t elenum = 4; //number of electrodes
  for(uint8_t i = 0; i<elenum; i++)
     {
	 mpr_set_threshold(i, 3, 3);   //by³o 6 i 5
	 }
	MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_ECR, 0x04); //pracuj¹ elektrody 0, 1, 2, 3.
    return 1;
}

uint8_t MPR_read_reg(uint8_t addr, uint8_t reg) 
{
	uint8_t r;
	I2C_SendStartAndSelect(addr<<1);
	I2C_SendByte(reg);
	I2C_SendStartAndSelect((addr<<1) | 0b00000001); //ustaw tryb do odczytu
	
	r = I2C_ReceiveData_NACK();
	
	I2C_Stop();
	I2C_WaitTillStopWasSent();
	
	return r;
}

uint16_t MPR_read_reg16bit(uint8_t addr, uint8_t reg)
{
	uint8_t datalow;
	uint16_t r;
	I2C_SendStartAndSelect(addr<<1);
	I2C_SendByte(reg);
	I2C_SendStartAndSelect((addr<<1) | 0x01); //ustaw tryb do odczytu
	datalow = I2C_ReceiveData_ACK();
	r = I2C_ReceiveData_NACK();
	I2C_Stop();
	I2C_WaitTillStopWasSent();
	r=(r<<8)+datalow;
	return r;
}

void MPR_write_reg(uint8_t addr, uint8_t reg, uint8_t data)
{
	I2C_SendStartAndSelect(addr<<1);
	I2C_SendByte(reg);
	I2C_SendByte(data);
	I2C_Stop();
	I2C_WaitTillStopWasSent();
}

void MPR_init(uint8_t addr)
{
	
	MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_ECR, 0x00);
	MPR_write_reg(MPR_DEVICE_ADDRESS, MPR_R_ECR, 0x04); //pracuj¹ elektrody 0, 1, 2, 3.
	
}