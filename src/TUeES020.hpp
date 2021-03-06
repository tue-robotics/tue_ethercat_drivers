/***************************************************************************
 tag: Sava Marinkov , Ruud van den Bogaert, Max Baeten
 Driver for TUeES020

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/
 
#ifndef TUEES020_H
#define TUEES020_H

#include <soem_master/soem_driver_factory.h>
#include <soem_master/soem_driver.h>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <vector>
#include <math.h>
#include <iostream>
#include "COE_config.h"
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>

#include <rtt/os/Timer.hpp>

using namespace std;
typedef vector<double> doubles;

// WARNING, the bits are numbered in reversed order
typedef union PACKED {
    struct PACKED {
        uint8 spare_di_1:1;		// bit 0
        uint8 spare_di_2:1;
        uint8 spare_di_3:1;
        uint8 spare_di_4:1;
        uint8 reserved_1:1;
        uint8 reserved_2:1;
        uint8 reserved_3:1;
        uint8 power_status:1;  // bit 7
    }       line;
    uint8   port;
} digital_in_t;

typedef struct PACKED {
        digital_in_t digital_in;        // Digital inputs
        uint16 encoder_1;               // Encoder 1
        uint16 encoder_2;               // Encoder 2
        uint16 encoder_3;               // Encoder 3
		uint16 force_1; 				// Analog ADC value of force sensor input 1
        uint16 force_2; 				// Analog ADC value of force sensor input 2
        uint16 force_3; 				// Analog ADC value of force sensor input 3
        uint16 position_1; 				// Analog ADC value of position sensor 1
        uint16 position_2; 				// Analog ADC value of position sensor 2
        uint16 position_3; 				// Analog ADC value of position sensor 3
        uint16 spare_ai_1;              // Spare analog in 1
        uint16 spare_ai_2;              // Spare analog in 2
        uint16 time_stamp;              // Time stamp (1 bit equals 256 ns)
    } in_tueEthercatMemoryt;

// WARNING, the bits are numbered in reversed order
typedef union PACKED {
    struct PACKED {	
		uint8 enable_1:1;		// bit 0
        uint8 enable_2:1;	
        uint8 spare_do_3:1;
        uint8 spare_do_4:1;	
        uint8 reserved_1:1;
        uint8 reserved_2:1;
        uint8 reserved_3:1;
        uint8 reserved_4:1;		// bit 7
    }       line;
    uint8   port;
} digital_out_t;

typedef struct PACKED {
    digital_out_t digital_out;      // Digital outputs
    int16 pwm_duty_motor_1; 		// PWM duty cycle for motor 1 (limited from -1000 up to 1000, 0 is no motion)
    int16 pwm_duty_motor_2; 		// PWM duty cycle for motor 2
    int16 pwm_duty_motor_3; 		// PWM duty cycle for motor 3
} out_tueEthercatMemoryt;

using namespace RTT;

namespace soem_beckhoff_drivers {

    class TUeES020: public soem_master::SoemDriver {
    public:
        TUeES020(ec_slavet* mem_loc);
        ~TUeES020() {};

        void update();
        bool configure();
        bool start();
		void read_digital_ins();
        void read_encoders();
        void read_forces();
        void read_positions();
        void read_analog_ins();
        void read_time_stamp();
        void write_pwm(float val1,float val2,float val3);
        void stop();

    private:
        // Declaring of variables
        int printEnabled;
        int printDisabled;
        uint16 cntr;
        bool enablestatus;
        bool enable;
        bool port_enabled_was_connected;
        digital_out_t digitalout;
        digital_in_t digitalin;
        digital_in_t digitalin_prev;

        // Declaring of Messages
        DigitalMsg digitalIns_msg;
        EncoderMsg encoder1_msg;
        EncoderMsg encoder2_msg;
        EncoderMsg encoder3_msg;
        AnalogMsg  forceSensors_msg;
        AnalogMsg  positionSensors_msg;
        AnalogMsg  analogIns_msg;
        EncoderMsg timeStamp_msg;
        DigitalMsg digitalOuts_msg;
        AnalogMsg  pwmDutyMotors_msg;

        // Declaring of pointers to the ethercan memory
        in_tueEthercatMemoryt* m_in_tueEthercat;
        out_tueEthercatMemoryt* m_out_tueEthercat;

        // Declaring of In and Out ports
        OutputPort<DigitalMsg> port_out_digitalIns;
        OutputPort<EncoderMsg> port_out_encoder1; // Make a new message type for
        OutputPort<EncoderMsg> port_out_encoder2; // multiple encoder values.
        OutputPort<EncoderMsg> port_out_encoder3;
        OutputPort<AnalogMsg>  port_out_forceSensors;
        OutputPort<AnalogMsg>  port_out_positionSensors;
        OutputPort<AnalogMsg>  port_out_analogIns;
        OutputPort<EncoderMsg> port_out_timeStamp;
        InputPort<DigitalMsg>  port_in_digitalOuts;
        InputPort<AnalogMsg>   port_in_pwmDutyMotors;
        InputPort<bool>        port_in_enable;

		uint16 print_counter;
    };
}
#endif
