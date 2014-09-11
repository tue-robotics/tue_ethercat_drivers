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

using namespace std;
typedef vector<double> doubles;

typedef struct PACKED {
		uint8 status_register; 			// General system status register
		uint16 encoder_angle_1;  		// Actual position of motor encoder 1
		uint16 encoder_angle_2;  		// Actual position of motor encoder 2
		uint16 encoder_angle_3;  		// Actual position of motor encoder 3
		uint16 force_1; 				// Analog ADC value of force sensor input 1
		uint16 position_1; 				// Analog ADC value of position sensor 1
		uint16 force_2; 				// Analog ADC value of force sensor input 2
		uint16 position_2; 				// Analog ADC value of position sensor 2
		uint16 force_3; 				// Analog ADC value of force sensor input 3
		uint16 position_3; 				// Analog ADC value of position sensor 3
		uint8 message_index; 			// Message index counter
	} in_armEthercatMemoryt;

	typedef struct PACKED {
		int16 pwm_duty_motor_1; 		// PWM duty cycle for motor 1
		int16 pwm_duty_motor_2; 		// PWM duty cycle for motor 2
		int16 pwm_duty_motor_3; 		// PWM duty cycle for motor 3
		uint8 control_register;			// Control register, used to disable heart beat and emergency button detection
		uint8 heart_beat;				// Heart beat 
		uint8 disable_motor_register;	// Register to enable/disable motors (Bit 0 = 1 disables all motors)
	} out_armEthercatMemoryt;

	using namespace RTT;

	namespace soem_beckhoff_drivers {

		class TUeES020: public soem_master::SoemDriver {
		public:
			TUeES020(ec_slavet* mem_loc);
			~TUeES020() {};

			void update();
			bool configure();
			void write_pwm(float val1,float val2,float val3);
			void read_encoders();
			void read_forces();
			void read_positions();
			void stop();

		private:
			// Declaring of 
			int printEnabled;
			int printDisabled;
			bool enablestatus;
			bool setOutputToZero;
			uint16 cntr;
        
			// declaring of local vectors and scalars
			uint16 enc1;
			uint16 enc2;
			uint16 enc3;
			uint8 heartbeat;
			uint8 controlregister;
			uint8 statusregister;
			uint8 disable_motor_register;
			std::vector<float> forceSensors;			
			std::vector<float> positionSensors;					
			std::vector<float> pwmDutyMotors;	
			bool enable;
			uint8 statusregister_prev;
			int j;
				
			// Declaring of Messages		
			EncoderMsg encoderAngle1_msg;
			EncoderMsg encoderAngle2_msg;
			EncoderMsg encoderAngle3_msg;
			AnalogMsg  positionSensors_msg;
			AnalogMsg  forceSensors_msg;
			AnalogMsg  pwmDutyMotors_msg;
			
			in_armEthercatMemoryt* m_in_armEthercat;
			out_armEthercatMemoryt* m_out_armEthercat;

			// Declaring of In and Out ports
			OutputPort<EncoderMsg> port_out_encoderAngle1;
			OutputPort<EncoderMsg> port_out_encoderAngle2;		
			OutputPort<EncoderMsg> port_out_encoderAngle3;
			OutputPort<AnalogMsg>  port_out_positionSensors;
			OutputPort<AnalogMsg>  port_out_forceSensors;
			InputPort<AnalogMsg>   port_in_pwmDutyMotors;
			InputPort<bool>  	   port_in_enable;
		};
	}
#endif