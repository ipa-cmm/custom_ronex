/*
 * Copyright (c) 2013, Shadow Robot Company, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */

/**
 * @file   spi_hardware_interface.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  The RoNeX SPI module data to be stored
 *         in the HardwareInterface.
 **/

#ifndef _SPI_HARDWARE_INTERFACE_H_
#define _SPI_HARDWARE_INTERFACE_H_

#define MAXCOMMAND 4000

#include <ros_ethercat_model/hardware_interface.hpp>
#include <custom_protocol/Custom_Ronex_Protocol_0x02000002_MYO_MOTOR.h>
#include <vector>
#include <sr_ronex_utilities/sr_ronex_utilities.hpp>

namespace ronex
{
struct sensorData {
        int position;
        short velocity;
        short displacement;
        short current;
        short sensor1;
        short sensor2;
        uint16_t analog[6];
};

class CustomRonex
        : public ros_ethercat_model::CustomHW
{
public:
sensorData sens;

CustomRonex()
{
        state_.reset(new RONEX_STATUS_02000002());
        command_.reset(new RONEX_COMMAND_02000002());
        command_->command_type = RONEX_COMMAND_02000002_COMMAND_TYPE_NORMAL;

        // Set everything to 0
        digitalIO = 0;
        command = 0;
        for(int i= 0; i<SPI_TRANSACTION_MAX_SIZE; ++i)
                spi_data[i] = 0;

        sens.position = 0;
        sens.velocity = 0;
        sens.displacement = 0;
        sens.current = 0;
        sens.sensor1 = 0;
        sens.sensor2 = 0;

        for(int i = 0; i <6; i++)
                sens.analog[i] = 0;


}

boost::shared_ptr<RONEX_STATUS_02000002> state_;
boost::shared_ptr<RONEX_COMMAND_02000002> command_;

int command;
int16u digitalIO;
int8u spi_data[SPI_TRANSACTION_MAX_SIZE];

//  @brief set output pin index to state
//  @param state new state of pins
//  @param index index of pin that should be changed
void setDigitalOut(bool state,int index){
        if(index < 6)
                if(state)
                        digitalIO |= 1 << index;
                else
                        digitalIO |= ~(1 << index);
}

void setCommand(int command_){
        if (command < MAXCOMMAND && command > -1*MAXCOMMAND)
                command = command_;
}

sensorData getSensorData() const {
        return sens;
}

double getAnalog(int index){
        return sens.analog[index];
}


};

}  // namespace ronex

#endif /* _SPI_HARDWARE_INTERFACE_H_ */
