/***************************************************************************
 tag: Sava Marinkov, Ruud van den Bogaert,  Fri Mar 23 12:44:00 CET 2011  soem_sergioEthercat.h

 soem_sergioEthercat.h -  dedicated ethercat module TU/e
 -------------------
 begin                : Fri November 23 2012
 copyright            : (C) 2012 Sava Marinkov & Ruud van den Bogaert & Max Baeten
 email                : s.marinkov@student.tue.nl , r.v.d.bogaert@tue.nl

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

#ifndef TUEETHERCATSLAVE030_H
#define TUEETHERCATSLAVE030_H

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

typedef union PACKED {
    struct PACKED {
        uint8 power_status:1;
        uint8 reserved_3:1;
        uint8 reserved_2:1;
        uint8 reserved_1:1;
        uint8 spare_di_4:1;
        uint8 spare_di_3:1;
        uint8 spare_di_2:1;
        uint8 spare_di_1:1;
    }       line;
    uint8   port;
} digital_in_t;

typedef struct PACKED {
        digital_in_t digital_in;        // Digital inputs
        uint16 encoder_1;               // Encoder 1
        uint16 encoder_2;               // Encoder 2
        uint16 encoder_3;               // Encoder 3
        uint16 current_1;               // Current 1
        uint16 current_2;               // Current 2
        uint16 current_3;               // Current 3
        uint16 calipher_1;              // calipher 1 ??
        uint16 calipher_2;              // calipher 2 ??
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

typedef union PACKED {
    struct PACKED {
        uint8 reserved_4:1;
        uint8 reserved_3:1;
        uint8 reserved_2:1;
        uint8 reserved_1:1;
        uint8 spare_do_4:1;
        uint8 spare_do_3:1;
        uint8 enable_2:1;
        uint8 enable_1:1;
    }       line;
    uint8   port;
} digital_out_t;

typedef struct PACKED {
    digital_out_t digital_out;      // Digital outputs
    int16 pwm_duty_motor_1; 		// PWM duty cycle for motor 1 (limited from -1000 up to 1000, 0 is no motion)
    int16 pwm_duty_motor_2; 		// PWM duty cycle for motor 2
    int16 pwm_duty_motor_3; 		// PWM duty cycle for motor 3
    int16 analog_out_1;             // Analog output 1  (0V = -2048, 10V = 2047, 5V = 0 is no motion)
    int16 analog_out_2;             // Analog output 2
} out_tueEthercatMemoryt;

using namespace RTT;

namespace soem_beckhoff_drivers {

    class TueETHERCAT: public soem_master::SoemDriver {
    public:
        TueETHERCAT(ec_slavet* mem_loc);
        ~TueETHERCAT() {};

        void update();
        bool configure();
        void read_encoders();
        void read_currents();
        void read_caliphers();
        void read_forces();
        void read_positions();
        void read_analog_ins();
        void read_time_stamp();
        void write_pwm(float val1,float val2,float val3);
        void write_analog_out(float val1, float val2);
        void stop();

    private:
        // Declaring of variables
        int printEnabled;
        int printDisabled;
        uint16 cntr;
        bool enablestatus;
        bool enable;
        digital_out_t digitalout;
        digital_in_t digitalin;
        digital_in_t digitalin_prev;

        // Declaring of Messages
        DigitalMsg digitalIns_msg;
        EncoderMsg encoder1_msg;
        EncoderMsg encoder2_msg;
        EncoderMsg encoder3_msg;
        AnalogMsg  currents_msg;
        AnalogMsg  caliphers_msg;
        AnalogMsg  forceSensors_msg;
        AnalogMsg  positionSensors_msg;
        AnalogMsg  analogIns_msg;
        EncoderMsg timeStamp_msg;
        DigitalMsg digitalOuts_msg;
        AnalogMsg  pwmDutyMotors_msg;
        AnalogMsg  analogOuts_msg;

        // Declaring of pointers to the ethercan memory
        in_tueEthercatMemoryt* m_in_tueEthercat;
        out_tueEthercatMemoryt* m_out_tueEthercat;

        // Declaring of In and Out ports
        OutputPort<DigitalMsg> port_out_digitalIns;
        OutputPort<EncoderMsg> port_out_encoder1; // Make a new message type for
        OutputPort<EncoderMsg> port_out_encoder2; // multiple encoder values.
        OutputPort<EncoderMsg> port_out_encoder3;
        OutputPort<AnalogMsg>  port_out_currents;
        OutputPort<AnalogMsg>  port_out_caliphers;
        OutputPort<AnalogMsg>  port_out_forceSensors;
        OutputPort<AnalogMsg>  port_out_positionSensors;
        OutputPort<AnalogMsg>  port_out_analogIns;
        OutputPort<EncoderMsg> port_out_timeStamp;
        InputPort<DigitalMsg>  port_in_digitalOuts;
        InputPort<AnalogMsg>   port_in_pwmDutyMotors;
        InputPort<AnalogMsg>   port_in_analogOuts;
        InputPort<bool>        port_in_enable;

		uint16 print_counter;
    };
}
#endif
