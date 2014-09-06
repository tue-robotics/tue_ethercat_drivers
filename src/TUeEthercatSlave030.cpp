/***************************************************************************
 tag: Sava Marinkov, Ruud van den Bogaert,  Fri Mar 23 12:44:00 CET 2011  soem_sergioEthercat.cpp

 soem_sergioEthercat.cpp -  dedicated ethercat module TU/e
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

#include "TUeEthercatSlave030.hpp"

using namespace soem_beckhoff_drivers;

TueETHERCAT::TueETHERCAT(ec_slavet* mem_loc) :
	soem_master::SoemDriver(mem_loc), 
            port_out_digitalIns("digitalIns"),
            port_out_encoder1("encoder1"),
            port_out_encoder2("encoder2"),
            port_out_encoder3("encoder3"),
            port_out_currents("currents"),
            port_out_caliphers("caliphers"),
            port_out_forceSensors("forceSensors"),
            port_out_positionSensors("positionSensors"),
            port_out_analogIns("analogIns"),
            port_out_timeStamp("timeStamp"),
            port_in_digitalOuts("digitalOuts"),
            port_in_pwmDutyMotors("pwmDutyMotorsIn"),
            port_in_analogOuts("analogOuts"),
            port_in_enable("enablePort"){

		m_service->doc(
			std::string("Services for custom EtherCat ") + std::string(
					m_datap->name) + std::string(" module"));

    m_service->addOperation("write_pwm", &TueETHERCAT::write_pwm, this, RTT::OwnThread).doc("Write pwm duty values");
    m_service->addOperation("write_analog_out", &TueETHERCAT::write_analog_out, this, RTT::OwnThread).doc("Write analog out values");
    m_service->addOperation("read_encoders", &TueETHERCAT::read_encoders, this, RTT::OwnThread).doc("Read encoder values");
    m_service->addOperation("read_currents", &TueETHERCAT::read_currents, this, RTT::OwnThread).doc("Read current values");
    m_service->addOperation("read_caliphers", &TueETHERCAT::read_caliphers, this, RTT::OwnThread).doc("Read calipher values");
    m_service->addOperation("read_forces", &TueETHERCAT::read_forces, this, RTT::OwnThread).doc("Read force sensor values");
    m_service->addOperation("read_positions", &TueETHERCAT::read_positions, this, RTT::OwnThread).doc("Read position sensor values");
    m_service->addOperation("read_time_stamp", &TueETHERCAT::read_time_stamp, this, RTT::OwnThread).doc("Read time stamp");

    m_service->addPort(port_out_digitalIns).doc("");
    m_service->addPort(port_out_encoder1).doc("");
    m_service->addPort(port_out_encoder2).doc("");
    m_service->addPort(port_out_encoder3).doc("");
    m_service->addPort(port_out_currents).doc("");
    m_service->addPort(port_out_caliphers).doc("");
    m_service->addPort(port_out_forceSensors).doc("");
	m_service->addPort(port_out_positionSensors).doc("");
    m_service->addPort(port_out_analogIns).doc("");
    m_service->addPort(port_out_timeStamp).doc("");
    m_service->addPort(port_in_digitalOuts).doc("");
    m_service->addPort(port_in_pwmDutyMotors).doc("");
    m_service->addPort(port_in_analogOuts).doc("");
    m_service->addPort(port_in_enable).doc("");

    digitalIns_msg.values.assign(4,0);
    encoder1_msg.value = 0;
    encoder2_msg.value = 0;
    encoder3_msg.value = 0;
    currents_msg.values.assign(3,0.0);
    caliphers_msg.values.assign(2,0.0);
    forceSensors_msg.values.assign(3, 0.0);
    positionSensors_msg.values.assign(3, 0.0);
    analogIns_msg.values.assign(2,0.0);
    timeStamp_msg.value = 0;
    digitalOuts_msg.values.assign(2,0.0);
    analogOuts_msg.values.assign(2,0.0);
    pwmDutyMotors_msg.values.assign(3,0.0);
    
}

bool TueETHERCAT::start() {
	log(Warning) << "Start TUeES030" << endlog();
	return true;
}

bool TueETHERCAT::configure() {
	
    log(Warning) << "Configuring TUeES030" << endlog();
	
    // initialize variables
    //enablestatus = false;
	//enable = false;
	
    //cntr = 1;
    //printEnabled = 0;
    //printDisabled = 1;

    //digitalin.port = 0;
    //digitalin_prev.port = 0;
    //digitalin_prev.line.power_status = 1;
    //digitalout.port = 0;
    
    // Properly initialize output struct (set all to zero)
    //m_out_tueEthercat = ((out_tueEthercatMemoryt*) (m_datap->outputs));
    //m_out_tueEthercat->digital_out = digitalout;
    //write_pwm((float)(0.0),(float)(0.0),(float)(0.0));
    //write_analog_out((float)(0.0),(float)(0.0));

	//print_counter = 0;
	return true;
}

void TueETHERCAT::update() {
	
    // check if port_in_enable is connected and read the value
    if (!port_in_enable.connected()) {
        enable = false;
        if (enablestatus == false)	{
            log(Warning)<<"TueEthercat: Waiting for Enable Signal, port_in_enable is not connected"<<endlog();
            enablestatus = true;
        }
    }
    else if (port_in_enable.connected()) {
        port_in_enable.read(enable);
        if (enablestatus == true) {
            log(Warning)<<"TueEthercat: Port_in_enable is connected"<<endlog();
			enablestatus = false;
        }
    }

    // get the pointers to the communication structs
    m_in_tueEthercat = ((in_tueEthercatMemoryt*) (m_datap->inputs));
    	
    // read the data from the ethercat memory input and send to orocos
    digitalin = m_in_tueEthercat->digital_in;
    read_encoders();
    read_currents();
    read_caliphers();
    read_forces();
    read_positions();
    read_time_stamp();

    // get data from orocos and send to ehtercat memory output
    // digital outputs
    if (port_in_digitalOuts.connected()) {
        if (port_in_digitalOuts.read(digitalOuts_msg) == NewData) {
            digitalout.line.spare_do_3 = digitalIns_msg.values[0];
            digitalout.line.spare_do_4 = digitalIns_msg.values[1];
        }
    }
    // pwm duty motors
    if (port_in_pwmDutyMotors.connected()) {
		if (port_in_pwmDutyMotors.read(pwmDutyMotors_msg) == NewData) {
            write_pwm((pwmDutyMotors_msg.values[0]),(pwmDutyMotors_msg.values[1]),(pwmDutyMotors_msg.values[2]));
		}
    }
    // analog outputs
    if (port_in_analogOuts.connected()) {
        if (port_in_analogOuts.read(analogOuts_msg) == NewData) {
            write_analog_out(analogOuts_msg.values[0],analogOuts_msg.values[2]);
        }
    }

    // Shut down amplifiers if enable signal from orocos is false or connection is lost
    if(enable){
        digitalout.line.enable_1 = 1; // Enable amplifiers
        digitalout.line.enable_2 = 1;
        digitalout.port = 1;
        if(printEnabled==0){
            printEnabled++;
            printDisabled=0;
			if(cntr != 0) {
                log(Warning)<<"TueEthercat: enable = true -> PWM output enabled" <<endlog();
			cntr=0;
			}
        }
    }
    else if(!enable){
        digitalout.line.enable_1 = 0; // Disable amplifiers
        digitalout.line.enable_2 = 0;
        digitalout.port = 0;
        if(printDisabled==0){
            log(Warning)<<"TueEthercat: enable = false -> PWM output set to zero"<<endlog();
            printDisabled++;
            printEnabled=0;
            cntr++;
        }
        write_pwm((float)(0.0),(float)(0.0),(float)(0.0));
        write_analog_out((float)(0.0),(float)(0.0));
    }
	
    // set digital outputs to ethercat memory output
    m_out_tueEthercat->digital_out = digitalout;

    // check for powerchanges
    if (digitalin.line.power_status != digitalin_prev.line.power_status) {
        if (digitalin.line.power_status) {
            log(Warning)<< "TueEthercat Status:  Power back up" << endlog();
        } else {
            log(Warning)<< "TueEthercat Status:  Power down" << endlog();
        }
    }
    digitalin_prev.port = digitalin.port;
    
    m_out_tueEthercat = ((out_tueEthercatMemoryt*) (m_datap->outputs));
    
}

void TueETHERCAT::read_encoders(){

    encoder1_msg.value = m_in_tueEthercat->encoder_1;
    encoder2_msg.value = m_in_tueEthercat->encoder_2;
    encoder3_msg.value = m_in_tueEthercat->encoder_3;

    port_out_encoder3.write(encoder3_msg);
    port_out_encoder2.write(encoder2_msg);
    port_out_encoder1.write(encoder1_msg);

    //log(Info) << "Angles: ["<< enc1 << ", "<< enc2 << ", " << enc3 << "]" << endlog();
}

void TueETHERCAT::read_currents(){

    float current1 = (float) m_in_tueEthercat->current_1;
    float current2 = (float) m_in_tueEthercat->current_2;
    float current3 = (float) m_in_tueEthercat->current_3;

    // TODO: add a conversion from bit to current?
    currents_msg.values[0] = (current1);
    currents_msg.values[1] = (current2);
    currents_msg.values[2] = (current3);

    port_out_currents.write(currents_msg);
}

void TueETHERCAT::read_caliphers(){

    caliphers_msg.values[0] = (float) m_in_tueEthercat->calipher_1;
    caliphers_msg.values[1] = (float) m_in_tueEthercat->calipher_2;

    port_out_caliphers.write(caliphers_msg);
}

void TueETHERCAT::read_forces(){
		
    float force1 = (float) m_in_tueEthercat->force_1;
    float force2 = (float) m_in_tueEthercat->force_2;
    float force3 = (float) m_in_tueEthercat->force_3;
	
	 forceSensors_msg.values[0] = (force1/2047.0*3.3);   	 //11 bits over 3,3V or 12 bits over 6,6V
	 forceSensors_msg.values[1] = (force2/2047.0*3.3);   	 //11 bits over 3,3V or 12 bits over 6,6V
	 forceSensors_msg.values[2] = (force3/2047.0*3.3);   	 //11 bits over 3,3V or 12 bits over 6,6V

	 port_out_forceSensors.write(forceSensors_msg);

	//log(Warning) << "Forces: ["<< force1 << ", "<< force2 << ", " << force3 << "]" << endlog();
}

void TueETHERCAT::read_positions(){

    positionSensors_msg.values[0] = (float) m_in_tueEthercat->position_1;
    positionSensors_msg.values[1] = (float) m_in_tueEthercat->position_2;
    positionSensors_msg.values[2] = (float) m_in_tueEthercat->position_3;

    port_out_positionSensors.write(positionSensors_msg);

    // log(Info) << "Positions: ["<< position1 << ", "<< position2 << ", " << position3 << "]" << endlog();
}

void TueETHERCAT::read_time_stamp(){

    timeStamp_msg.value = m_in_tueEthercat->time_stamp;
    port_out_timeStamp.write(timeStamp_msg);
    //log(Warning) << "Time Stamp: " << timeSt << endlog();
}

void TueETHERCAT::write_pwm(float val1, float val2, float val3) {
    int16 tmp1 = (int16) val1;
	int16 tmp2 = (int16) val2;
	int16 tmp3 = (int16) val3;

    // saturate the pwm duty cycles
    if (tmp1 > 1000)
        tmp1 = 1000;
    if (tmp1 < -1000)
        tmp1 = -1000;

    if (tmp2 > 1000)
        tmp2 = 1000;
    if (tmp2 < -1000)
        tmp2 = -1000;

    if (tmp3 > 1000)
        tmp3 = 1000;
    if (tmp3 < -1000)
        tmp3 = -1000;
		
    m_out_tueEthercat->pwm_duty_motor_1 = tmp1;
    m_out_tueEthercat->pwm_duty_motor_2 = tmp2;
    m_out_tueEthercat->pwm_duty_motor_3 = tmp3;
    
    //if (print_counter>1000) {
	//	log(Warning) << "output pwms are " << tmp1 <<" and " << tmp2 << endlog();
	//	print_counter = 0;
	//}
	//print_counter++;
}

void TueETHERCAT::write_analog_out(float val1, float val2) {
    int16 tmp1 = (int16) val1;
    int16 tmp2 = (int16) val2;

    // saturate analog outs
    if (tmp1 > 2000)
        tmp1 = 2000;
    if (tmp1 < -2000)
        tmp1 = -2000;

    if (tmp2 > 2000)
        tmp2 = 2000;
    if (tmp2 < -2000)
        tmp2 = -2000;

    m_out_tueEthercat->analog_out_1 = tmp1;
    m_out_tueEthercat->analog_out_2 = tmp2;

}

void TueETHERCAT::stop() {
	return;
}

namespace {
soem_master::SoemDriver* createTueETHERCAT(ec_slavet* mem_loc) {
    return new TueETHERCAT(mem_loc);
}

const bool registered0 =
		soem_master::SoemDriverFactory::Instance().registerDriver(
                "EPC ESRA FGPA Ethercat Slave", createTueETHERCAT);
}
