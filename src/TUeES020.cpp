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

#include "TUeES020.hpp"

using namespace soem_beckhoff_drivers;

TUeES020::TUeES020(ec_slavet* mem_loc) :
	soem_master::SoemDriver(mem_loc), 
			port_out_encoderAngle1("encoderAngle1"),
			port_out_encoderAngle2("encoderAngle2"),
			port_out_encoderAngle3("encoderAngle3"),
			port_out_positionSensors("positionSensors"),
  			port_out_forceSensors("forceSensors"),
            port_in_pwmDutyMotors("pwmDutyMotorsIn"),
            port_in_enable("enablePort"){

		m_service->doc(
			std::string("Services for custom EtherCat ") + std::string(
					m_datap->name) + std::string(" module"));

	m_service->addOperation("write_pwm", &TUeES020::write_pwm, this, RTT::OwnThread).doc("Write pwm duty values");
	m_service->addOperation("read_encoders", &TUeES020::read_encoders, this, RTT::OwnThread).doc("Read encoder values");
 	m_service->addOperation("read_forces", &TUeES020::read_forces, this, RTT::OwnThread).doc("Read force sensor values");
 	m_service->addOperation("read_positions", &TUeES020::read_positions, this, RTT::OwnThread).doc("Read position sensor values");

	m_service->addPort(port_out_encoderAngle1).doc("");
	m_service->addPort(port_out_encoderAngle2).doc("");
	m_service->addPort(port_out_encoderAngle3).doc("");
	m_service->addPort(port_out_positionSensors).doc("");
 	m_service->addPort(port_out_forceSensors).doc("");
	m_service->addPort(port_in_pwmDutyMotors).doc("");
    m_service->addPort(port_in_enable).doc("");

	positionSensors_msg.values.assign(3, 0.0);
 	forceSensors_msg.values.assign(3, 0.0);
	pwmDutyMotors_msg.values.assign(3, 0.0);
	encoderAngle1_msg.value = 0;
	encoderAngle2_msg.value = 0;
	encoderAngle3_msg.value = 0;
}

bool TUeES020::configure() {
	
	positionSensors.assign(3, 0.0);
  	forceSensors.assign(3, 0.0);
	pwmDutyMotors.assign(3, 0.0);
	enc1 = 0;
	enc2 = 0;
	enc3 = 0;
	
	setOutputToZero = false;
	enablestatus = true;
	
    cntr = 0;	
	heartbeat = 0;
    printEnabled = 0;
    printDisabled = 1;
    j=0;
    statusregister_prev = 0;
    
    // in the controlregister, the first bit toggles the heartbeat, the second bit the emergency button detection, and the third bit the ramp
    //controlregister = 0x00;	// heart beat on 		+ emergency button detection on  		+ ramp on
    //controlregister = 0x01;	// heart beat off 		+ emergency button detection on  		+ ramp on
	//controlregister = 0x02;   // heart beat on 		+ emergency button detection off  		+ ramp on
	//controlregister = 0x03;	// heart beat off 		+ emergency button detection off    	+ ramp on
	controlregister   = 0x04;	// heart beat on 		+ emergency button detection on   		+ ramp off
    //controlregister = 0x05;	// heart beat off 		+ emergency button detection on  		+ ramp off
	//controlregister = 0x06;   // heart beat on 		+ emergency button detection off  		+ ramp off
	//controlregister = 0x07;	// heart beat off 		+ emergency button detection off   		+ ramp off
   
	disable_motor_register = 255; // Start with all motors disabled

    // This is to avoid random outputs at startup
    if (!setOutputToZero) {
        log(Info) << "Doing extra output update to start with zero outputs" << endlog();
        m_out_armEthercat = ((out_armEthercatMemoryt*) (m_datap->outputs));
        log(Info) << "PWM Values before initialization are " << m_out_armEthercat->pwm_duty_motor_1 << "\t" << m_out_armEthercat->pwm_duty_motor_2 << "\t" << m_out_armEthercat->pwm_duty_motor_3 << endlog();
        write_pwm((float)(0.0),(float)(0.0),(float)(0.0));
        setOutputToZero = true;
    }
        
	return true;
}

void TUeES020::update() {
	
    if (port_in_enable.connected()) {
        port_in_enable.read(enable);
        if (enablestatus == true)
			log(Info)<<"TUeES020 Driver Port_in_enable is connected"<<endlog();
        enablestatus = false;
		}
		
    else if (!port_in_enable.connected()) {
		if (enablestatus == false)	{ 
			log(Warning)<<"TUeES020 Driver Waiting for Enable Signal, port_in_enable is not connected"<<endlog();
			enablestatus = true;
		}
    }
    
    // heartbeat
    heartbeat++;
    if (heartbeat > 250)
	{
		heartbeat = 0;  
	}	
    
    m_out_armEthercat->heart_beat = heartbeat;
	m_out_armEthercat->control_register = controlregister;
	m_out_armEthercat->disable_motor_register = disable_motor_register;														

	m_in_armEthercat = ((in_armEthercatMemoryt*) (m_datap->inputs));
	m_out_armEthercat = ((out_armEthercatMemoryt*) (m_datap->outputs));
	
	read_positions();
 	read_forces();
	read_encoders();
		
	if (port_in_pwmDutyMotors.connected()) {
		if (port_in_pwmDutyMotors.read(pwmDutyMotors_msg) == NewData) {
			write_pwm((pwmDutyMotors_msg.values[0]),(pwmDutyMotors_msg.values[1]),(pwmDutyMotors_msg.values[2]));
		}
    }

    // If enable is true no error occured in the system
    if(enable){
		disable_motor_register = 0; // Enable amplifiers
        if(printEnabled==0){
            printEnabled++;
            printDisabled=0;
            if(cntr != 0) {
				log(Info)<<"TUeES020 Driver enable = true -> PWM output enabled" <<endlog();
				cntr=0;
			}
        }
    }
    else if(!enable){
		disable_motor_register = 255; // Disable amplifiers
        if(printDisabled==0){
            log(Info)<<"TUeES020 Driver enable = false -> PWM output set to zero"<<endlog();
            printDisabled++;
            printEnabled=0;
            cntr++;
        }
        write_pwm((float)(0.0),(float)(0.0),(float)(0.0));
    }
    
    // status_register warnings
    statusregister = m_in_armEthercat->status_register;
    
    //j++;	
    if ((statusregister & 0x03) != (statusregister_prev & 0x03)) {		
		if ((statusregister & 0x03) == 0x00 ) { log(Info)<< "TUeES020 Driver Status:  E_OK" << endlog(); }
		if ((statusregister & 0x03) == 0x01 ) {
			if ((statusregister ^ 0X7C) & 0x04 ) { 	log(Info)<< "TUeES020 Driver Status:  E_POWER_DOWN: 5V" << endlog(); }
			if ((statusregister ^ 0X7C) & 0x08 ) { 	log(Info)<< "TUeES020 Driver Status:  E_POWER_DOWN: 12V" << endlog(); }
			if ((statusregister ^ 0X7C) & 0x10 ) { 	log(Info)<< "TUeES020 Driver Status:  E_POWER_DOWN: 24V" << endlog(); }
			if ((statusregister ^ 0X7C) & 0x20 ) { 	log(Info)<< "TUeES020 Driver Status:  E_POWER_DOWN: 1.2V" << endlog(); }
			if ((statusregister ^ 0X7C) & 0x40 ) { 	log(Info)<< "TUeES020 Driver Status:  E_POWER_DOWN: 1.65V" << endlog(); }
			}			
		if ((statusregister & 0x03) == 0x02 ) {	log(Info)<< "TUeES020 Driver Status:  E_COMM_DOWN" << endlog(); }	
		if ((statusregister & 0x03) == 0x03 ) { log(Error)<< "TUeES020 Driver Status:  E_NO_OP_STATE" << endlog(); }
	
		statusregister_prev = statusregister;
	}      
}

void TUeES020::read_forces(){
		
	float force1 = (float) m_in_armEthercat->force_1;
	float force2 = (float) m_in_armEthercat->force_2;
	float force3 = (float) m_in_armEthercat->force_3;
	
	 forceSensors_msg.values[0] = (force1/2047.0*3.3);   	 //11 bits over 3,3V or 12 bits over 6,6V
	 forceSensors_msg.values[1] = (force2/2047.0*3.3);   	 //11 bits over 3,3V or 12 bits over 6,6V
	 forceSensors_msg.values[2] = (force3/2047.0*3.3);   	 //11 bits over 3,3V or 12 bits over 6,6V

	 port_out_forceSensors.write(forceSensors_msg);
}

void TUeES020::read_encoders(){

    enc1 = m_in_armEthercat->encoder_angle_1;
    enc2 = m_in_armEthercat->encoder_angle_2;
    enc3 = m_in_armEthercat->encoder_angle_3;

    encoderAngle1_msg.value = (enc1);
    encoderAngle2_msg.value = (enc2);
    encoderAngle3_msg.value = (enc3);   

	port_out_encoderAngle3.write(encoderAngle3_msg); 
	port_out_encoderAngle2.write(encoderAngle2_msg);
	port_out_encoderAngle1.write(encoderAngle1_msg);
}

void TUeES020::read_positions(){
	float position1 = (float) m_in_armEthercat->position_1;
	float position2 = (float) m_in_armEthercat->position_2;
	float position3 = (float) m_in_armEthercat->position_3;

	positionSensors_msg.values[0] = position1;
	positionSensors_msg.values[1] = position2;
	positionSensors_msg.values[2] = position3;
	
	port_out_positionSensors.write(positionSensors_msg);
}

void TUeES020::write_pwm(float val1, float val2, float val3) {
    int16 tmp1 = (int16) val1;
	int16 tmp2 = (int16) val2;
	int16 tmp3 = (int16) val3;

	// limit the duty cycles
	int16 maxPWM = 1000;
	if (tmp1 > maxPWM)
		tmp1 = maxPWM;
	if (tmp1 < -maxPWM)
		tmp1 = -maxPWM;

	if (tmp2 > maxPWM)
		tmp2 = maxPWM;
	if (tmp2 < -maxPWM)
		tmp2 = -maxPWM;

	if (tmp3 > maxPWM)
		tmp3 = maxPWM;
	if (tmp3 < -maxPWM)
		tmp3 = -maxPWM;
		
	m_out_armEthercat->pwm_duty_motor_1 = tmp1;
	m_out_armEthercat->pwm_duty_motor_2 = tmp2;
	m_out_armEthercat->pwm_duty_motor_3 = tmp3;
}

namespace {
soem_master::SoemDriver* createTUeES020(ec_slavet* mem_loc) {
	return new TUeES020(mem_loc);
}

const bool registered0 =
		soem_master::SoemDriverFactory::Instance().registerDriver(
				"TUeES020", createTUeES020);
}