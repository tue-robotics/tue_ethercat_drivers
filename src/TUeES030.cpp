/***************************************************************************
 tag:
 V2: Matthijs van der Burgh
 V1: Ton Peters, Max Baeten, Ruud van den Bogaert
 Driver for TUeES030

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

#include "TUeES030.hpp"

using namespace soem_beckhoff_drivers;

TUeES030::TUeES030(ec_slavet* mem_loc) :
	soem_master::SoemDriver(mem_loc), 
            port_out_digitalIns("digitalIns"),
            port_out_encoder1("encoder1"),
            port_out_encoder2("encoder2"),
            port_out_encoder3("encoder3"),
            port_out_enc_time1("encoder_timestamp1"),
            port_out_enc_time2("encoder_timestamp2"),
            port_out_enc_time3("encoder_timestamp3"),
            port_out_enc_vels("encoder_velocities"),
            port_out_currents("currents"),
            port_out_caliphers("caliphers"),
            port_out_forceSensors("forceSensors"),
            port_out_positionSensors("positionSensors"),
            port_out_analogIns("analogIns"),
            port_out_linevoltage("linevoltage"),
            port_out_timeStamp("timeStamp"),
            port_in_digitalOuts("digitalOuts"),
            port_in_pwmDutyMotors("pwmDutyMotors"),
            port_in_ffmotors("ffmotors"),
            port_in_mcom1("mcom1"),
            port_in_mcom2("mcom2"),
            port_in_mcom3("mcom3"),
            port_in_analogOuts("analogOuts"),
            port_in_enable("enablePort"){

                m_service->doc(
                        std::string("Services for custom EtherCat ") + std::string(
					m_datap->name) + std::string(" module"));

    m_service->addOperation("write_pwm", &TUeES030::write_pwm, this, RTT::OwnThread).doc("Write motor current values");
    m_service->addOperation("write_analog_out", &TUeES030::write_analog_out, this, RTT::OwnThread).doc("Write analog out values");
    m_service->addOperation("write_mcommands", &TUeES030::write_mcommands, this, RTT::OwnThread).doc("Write motor commands");
    m_service->addOperation("write_ffmotors", &TUeES030::write_ffmotors, this, RTT::OwnThread).doc("Write motor feedforward values");
    m_service->addOperation("read_digital_ins", &TUeES030::read_digital_ins, this, RTT::OwnThread).doc("Read digital inputs");
    m_service->addOperation("read_encoders", &TUeES030::read_encoders, this, RTT::OwnThread).doc("Read encoder values");
    m_service->addOperation("read_encoder_times", &TUeES030::read_encoder_times, this, RTT::OwnThread).doc("Read encoder timestamps");
    m_service->addOperation("read_encoder_vels", &TUeES030::read_encoder_vels, this, RTT::OwnThread).doc("Read encoder velocities");
    m_service->addOperation("read_currents", &TUeES030::read_currents, this, RTT::OwnThread).doc("Read current values");
    m_service->addOperation("read_caliphers", &TUeES030::read_caliphers, this, RTT::OwnThread).doc("Read calipher values");
    m_service->addOperation("read_forces", &TUeES030::read_forces, this, RTT::OwnThread).doc("Read force sensor values");
    m_service->addOperation("read_positions", &TUeES030::read_positions, this, RTT::OwnThread).doc("Read position sensor values");
    m_service->addOperation("read_linevoltage", &TUeES030::read_linevoltage, this, RTT::OwnThread).doc("Read linevoltage");
    m_service->addOperation("read_time_stamp", &TUeES030::read_time_stamp, this, RTT::OwnThread).doc("Read time stamp");

    m_service->addPort(port_out_digitalIns).doc("");
    m_service->addPort(port_out_encoder1).doc("");
    m_service->addPort(port_out_encoder2).doc("");
    m_service->addPort(port_out_encoder3).doc("");
    m_service->addPort(port_out_enc_time1).doc("");
    m_service->addPort(port_out_enc_time2).doc("");
    m_service->addPort(port_out_enc_time3).doc("");
    m_service->addPort(port_out_enc_vels).doc("");
    m_service->addPort(port_out_currents).doc("");
    m_service->addPort(port_out_caliphers).doc("");
    m_service->addPort(port_out_forceSensors).doc("");
    m_service->addPort(port_out_positionSensors).doc("");
    m_service->addPort(port_out_analogIns).doc("");
    m_service->addPort(port_out_linevoltage).doc("");
    m_service->addPort(port_out_timeStamp).doc("");
    m_service->addPort(port_in_digitalOuts).doc("");
    m_service->addPort(port_in_pwmDutyMotors).doc("");
    m_service->addPort(port_in_analogOuts).doc("");
    m_service->addPort(port_in_enable).doc("");

    // EL6022
    m_service->addPort("data_rx", port_out).doc("Msg containing the received data from serial device");
    m_service->addPort("data_tx", port_in).doc("Msg containing the data to send to the serial device");
    m_service->addPort("ready_rx", port_rx_ready).doc("Signal specifying that the serial device is ready to receive the data");
    m_service->addPort("running", port_running).doc("Signal specifying that the serial device is ready to transmit the data");

    #if 0
    parameter temp;

    temp.description = "Send handshake Ch1";
    temp.index = 0x8000;
    temp.subindex = 0x02;
    temp.name = "S_Handshake1";
    temp.size = 1;
    temp.param = XON_XOFF_DISABLE;
    m_params.push_back(temp);

    temp.description = "Receive handshake Ch1";
    temp.index = 0x8000;
    temp.subindex = 0x03;
    temp.name = "R_Handshake1";
    temp.size = 1;
    temp.param = XON_XOFF_DISABLE;
    m_params.push_back(temp);

    temp.description = "Send handshake Ch2";
    temp.index = 0x8010;
    temp.subindex = 0x02;
    temp.name = "S_Handshake2";
    temp.size = 1;
    temp.param = XON_XOFF_DISABLE;
    m_params.push_back(temp);

    temp.description = "Receive handshake Ch2";
    temp.index = 0x8010;
    temp.subindex = 0x03;
    temp.name = "R_Handshake2";
    temp.size = 1;
    temp.param = XON_XOFF_DISABLE;
    m_params.push_back(temp);

    temp.description = "Baudrate Ch1";
    temp.index = 0x8000;
    temp.subindex = 0x11;
    temp.name = "Baud1";
    temp.size = 1;
    temp.param = RS485_57600_BAUD;
    m_params.push_back(temp);

    temp.description = "Baudrate Ch2";
    temp.index = 0x8010;
    temp.subindex = 0x11;
    temp.name = "Baud2";
    temp.size = 1;
    temp.param = RS485_57600_BAUD;
    m_params.push_back(temp);

    temp.description = "Data frame Ch1";
    temp.index = 0x8000;
    temp.subindex = 0x15;
    temp.name = "Data_frame1";
    temp.size = 1;
    temp.param = RS485_8B_NP_1S;
    m_params.push_back(temp);

    temp.description = "Data frame Ch2";
    temp.index = 0x8010;
    temp.subindex = 0x15;
    temp.name = "Data_frame2";
    temp.size = 1;
    temp.param = RS485_8B_NP_1S;
    m_params.push_back(temp);

    temp.description = "Enable half duplex Ch1";
    temp.index = 0x8000;
    temp.subindex = 0x06;
    temp.name = "E_half_duplex1";
    temp.size = 1;
    temp.param = RS485_HALF_DUPLEX;
    m_params.push_back(temp);

    temp.description = "Enable half duplex Ch2";
    temp.index = 0x8010;
    temp.subindex = 0x06;
    temp.name = "E_half_duplex2";
    temp.size = 1;
    temp.param = RS485_HALF_DUPLEX;
    m_params.push_back(temp);

    for (unsigned int i = 0; i < m_params.size(); i++) {
        m_service->addProperty(m_params[i].name, m_params[i].param).doc(m_params[i].description);
    }
    #endif

    digitalIns_msg.values.assign(4,0);
    encoder1_msg.value = 0;
    encoder2_msg.value = 0;
    encoder3_msg.value = 0;
    enc_time1_msg.value=0;
    enc_time2_msg.value=0;
    enc_time3_msg.value=0;
    enc_vels_msg.values.assign(3, 0.0);
    currents_msg.values.assign(3, 0.0);
    caliphers_msg.values.assign(2, 0.0);
    forceSensors_msg.values.assign(3, 0.0);
    positionSensors_msg.values.assign(3, 0.0);
    analogIns_msg.values.assign(2, 0.0);
    linevoltage_msg.value=0;
    timeStamp_msg.value = 0;
    digitalOuts_msg.values.assign(2, 0.0);
    analogOuts_msg.values.assign(2, 0.0);
    pwmDutyMotors_msg.values.assign(3, 0.0);
    mcom1_msg.value=0; // braked
    mcom2_msg.value=0; // braked
    mcom3_msg.value=0; // braked
    ffmotors_msg.values.assign(3, 0.0);

    analogconverter=4095.0/3.3;
}



bool TUeES030::configure() {
	
    // initialize variables
    enablestatus = false;
    enable = false;
    port_enabled_was_connected = false;

    cntr = 1;
    printEnabled = 0;
    printDisabled = 1;

    digitalin.port = 0;
    digitalin_prev.port = 0;
    digitalout.port = 0;
    
    print_counter = 0;

    // EL6022
    #if 0
    for (unsigned int i = 0; i < m_params.size(); i++) {

        while (EcatError)
            log(RTT::Error) << ec_elist2string() << RTT::endlog();

        ec_SDOwrite(((m_datap->configadr) & 0x0F), m_params[i].index, m_params[i].subindex, FALSE, m_params[i].size,
                &(m_params[i].param), 
                EC_TIMEOUTRXM);
    }
    #endif

    msg_in.channels.resize(CHANNEL_NUM);
    state = START;
    old_status=0x00;
	count = 0;
    
    log(Debug) << "TUeES030::configure()"<< endlog();
    return true;
}

bool TUeES030::start() {

    // Properly initialize output struct (set all to zero)
    m_out_tueEthercat = ((out_tueEthercatMemoryt*) (m_datap->outputs));
    m_out_tueEthercat->digital_out = digitalout;
    write_pwm((float)(0.0),(float)(0.0),(float)(0.0));
    write_analog_out((float)(0.0),(float)(0.0));
    write_mcommands(0, 0, 0); //braked

    // get the pointers to the communication structs
    m_in_tueEthercat = ((in_tueEthercatMemoryt*) (m_datap->inputs));
    m_out_tueEthercat = ((out_tueEthercatMemoryt*) (m_datap->outputs));

    return true;
}

void TUeES030::update() {
	
    // check if port_in_enable is connected and read the value
    if (!port_in_enable.connected() && port_enabled_was_connected) {
        enable = false;
        if (enablestatus == false) {
            log(Warning)<<"TUeES030 Driver: Waiting for Enable Signal, port_in_enable is not connected"<<endlog();
            enablestatus = true;
        }
    }
    else if (port_in_enable.connected()) {
        port_enabled_was_connected = true;
        port_in_enable.read(enable);
        if (enablestatus == true) {
            log(Warning)<<"TUeES030 Driver: Port_in_enable is connected"<<endlog();
            enablestatus = false;
        }
    }

    // EL6022
    executeStateActions();
    updateState();
    old_status=m_in_tueEthercat->in_el6022.status;
    old_control=m_out_tueEthercat->out_el6022.control;
    
    if (count % 1 == 0)
    {
		status = m_in_tueEthercat->in_el6022.status;
		log(Warning) << "RS485 status: " << status <<endlog();
		control = m_out_tueEthercat->out_el6022.control;
		log(Warning) << "RS485 control: " << control <<endlog();
		//buffer = m_in_tueEthercat->in_el6022.buffer_in[0];
		//log(Warning) << "RS485 buffer_in: " << buffer <<endlog();
	} 
	count=count+1;
	//log(Warning) << count << endlog();
	
    // read the data from the ethercat memory input and send to orocos
    read_digital_ins();
    read_encoders();
    read_encoder_times();
    read_encoder_vels();
    read_linevoltage();
    read_time_stamp();
    read_currents();
    read_caliphers();
    read_forces();
    read_positions();

    // enable = true, all outputs are send to Slave
    // enable = false, all outputs set to zero
    if (enable) {
        // digital outputs
        if (port_in_digitalOuts.connected()) {
            if (port_in_digitalOuts.read(digitalOuts_msg) == NewData) {
                digitalout.line.enable_1 = digitalOuts_msg.values[0];
                digitalout.line.enable_2 = digitalOuts_msg.values[1];
                digitalout.line.spare_do_3 = digitalOuts_msg.values[2];
                digitalout.line.spare_do_4 = digitalOuts_msg.values[3];
            }
        }
        // motor commands
        if (port_in_mcom1.connected() || port_in_mcom2.connected() || port_in_mcom3.connected()) {
            if (port_in_mcom1.read(mcom1_msg) == NewData || port_in_mcom2.read(mcom2_msg) == NewData || port_in_mcom3.read(mcom3_msg) == NewData) {
                write_mcommands(mcom1_msg.value, mcom2_msg.value, mcom3_msg.value);
                }
        }
        // motor feedforward
        if (port_in_ffmotors.connected()) {
            if (port_in_ffmotors.read(ffmotors_msg) == NewData) {
                write_ffmotors((ffmotors_msg.values[0]),(ffmotors_msg.values[1]),(ffmotors_msg.values[2]));
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
                    write_analog_out(analogOuts_msg.values[0],analogOuts_msg.values[1]);
                }
        }
    }
    else {
        digitalout.port = 0;
        write_pwm((float)(0.0),(float)(0.0),(float)(0.0));
        write_analog_out((float)(0.0),(float)(0.0));
        write_mcommands(0, 0, 0);
        write_ffmotors((float)(0.0),(float)(0.0),(float)(0.0));
    }
    // set digital outputs to ethercat memory output
    m_out_tueEthercat->digital_out = digitalout;

    // check for powerchanges
    if (digitalin.line.power_status != digitalin_prev.line.power_status) {
        if (!digitalin.line.power_status) {
            log(Warning)<< "TUeES030 Driver Status:  Power OK" << endlog();
        }
        else {
            log(Warning)<< "TUeES030 Driver Status:  Power down" << endlog();
        }
    }
    digitalin_prev.port = digitalin.port;
}

void TUeES030::read_digital_ins(){

    digitalin = m_in_tueEthercat->digital_in;

    digitalIns_msg.values[0] = digitalin.line.spare_di_1;
    digitalIns_msg.values[1] = digitalin.line.spare_di_2;
    digitalIns_msg.values[2] = digitalin.line.spare_di_3;
    digitalIns_msg.values[3] = digitalin.line.spare_di_4;

    port_out_digitalIns.write(digitalIns_msg);
}

void TUeES030::read_encoders(){

    encoder1_msg.value = m_in_tueEthercat->encoder_1;
    encoder2_msg.value = m_in_tueEthercat->encoder_2;
    encoder3_msg.value = m_in_tueEthercat->encoder_3;

    port_out_encoder3.write(encoder3_msg);
    port_out_encoder2.write(encoder2_msg);
    port_out_encoder1.write(encoder1_msg);

    //log(Info) << "Angles: ["<< enc1 << ", "<< enc2 << ", " << enc3 << "]" << endlog();
}

void TUeES030::read_encoder_times(){

    enc_time1_msg.value = m_in_tueEthercat->timestamp1;
    enc_time2_msg.value = m_in_tueEthercat->timestamp2;
    enc_time3_msg.value = m_in_tueEthercat->timestamp3;

    port_out_enc_time3.write(enc_time3_msg);
    port_out_enc_time2.write(enc_time2_msg);
    port_out_enc_time1.write(enc_time1_msg);

    //log(Info) << "Timestamps: ["<< enc1 << ", "<< enc2 << ", " << enc3 << "]" << endlog();
}

void TUeES030::read_encoder_vels(){

    float velocity1 = (float) m_in_tueEthercat->velocity1;
    float velocity2 = (float) m_in_tueEthercat->velocity2;
    float velocity3 = (float) m_in_tueEthercat->velocity3;

    // conversion from bit to rad/s
    enc_vels_msg.values[0] = (velocity1*0.1);
    enc_vels_msg.values[1] = (velocity2*0.1);
    enc_vels_msg.values[2] = (velocity3*0.1);

    port_out_enc_vels.write(enc_vels_msg);
    //log(Info) << "Velocities: ["<< velocity1 << ", "<< velocity2 << ", " << velocity3 << "]" << endlog();
}

void TUeES030::read_currents(){

    float current1 = (float) m_in_tueEthercat->current_1;
    float current2 = (float) m_in_tueEthercat->current_2;
    float current3 = (float) m_in_tueEthercat->current_3;

    // TODO: add a conversion from bit to current?
    currents_msg.values[0] = (current1);
    currents_msg.values[1] = (current2);
    currents_msg.values[2] = (current3);

    port_out_currents.write(currents_msg);
}

void TUeES030::read_caliphers(){

    // Note, the caliphers are connected wrong
    // calipher 2 is attached to the legs and calipher 1 to the trunk
    float calipher2 = (float)m_in_tueEthercat->calipher_1;
    float calipher1 = (float)m_in_tueEthercat->calipher_2;
	
    // conversion  1 bit = 0.01mm
    caliphers_msg.values[0] = (calipher1*0.00001);
    caliphers_msg.values[1] = (calipher2*0.00001);
    
    port_out_caliphers.write(caliphers_msg);
}

void TUeES030::read_forces(){
		
    float force1 = (float) m_in_tueEthercat->force_1;
    float force2 = (float) m_in_tueEthercat->force_2;
    float force3 = (float) m_in_tueEthercat->force_3;
	
    forceSensors_msg.values[0] = (force1/analogconverter);   	 //12 bits over 3.3V
    forceSensors_msg.values[1] = (force2/analogconverter);   	 //12 bits over 3.3V
    forceSensors_msg.values[2] = (force3/analogconverter);   	 //12 bits over 3.3V

    port_out_forceSensors.write(forceSensors_msg);

    //log(Warning) << "Forces: ["<< force1 << ", "<< force2 << ", " << force3 << "]" << endlog();
}

void TUeES030::read_positions(){

    float position1 = (float) m_in_tueEthercat->position_1;
    float position2 = (float) m_in_tueEthercat->position_2;
    float position3 = (float) m_in_tueEthercat->position_3;
    
    positionSensors_msg.values[0] = (position1/analogconverter);   	 //12 bits over 3.3V
    positionSensors_msg.values[1] = (position2/analogconverter);   	 //12 bits over 3.3V
    positionSensors_msg.values[2] = (position3/analogconverter);   	 //12 bits over 3.3V

    port_out_positionSensors.write(positionSensors_msg);

    // log(Info) << "Positions: ["<< position1 << ", "<< position2 << ", " << position3 << "]" << endlog();
}

void TUeES030::read_linevoltage(){

    linevoltage_msg.value = m_in_tueEthercat->linevoltage;
    port_out_linevoltage.write(linevoltage_msg);
}

void TUeES030::read_time_stamp(){

    timeStamp_msg.value = m_in_tueEthercat->time_stamp;
    port_out_timeStamp.write(timeStamp_msg);
    //log(Warning) << "Time Stamp: " << timeSt << endlog();
}

void TUeES030::write_pwm(float val1, float val2, float val3) {

    int16 tmp1 = (int16) val1;
    int16 tmp2 = (int16) val2;
    int16 tmp3 = (int16) val3;

    // saturate the pwm currents
    // PWM 1: 6A, PWM 2,3: 3A
    if (tmp1 > 6000)
        tmp1 = 6000;
    if (tmp1 < -6000)
        tmp1 = -6000;

    if (tmp2 > 3000)
        tmp2 = 3000;
    if (tmp2 < -3000)
        tmp2 = -3000;

    if (tmp3 > 3000)
        tmp3 = 3000;
    if (tmp3 < -3000)
        tmp3 = -3000;
		
    m_out_tueEthercat->pwm_duty_motor_1 = tmp1;
    m_out_tueEthercat->pwm_duty_motor_2 = tmp2;
    m_out_tueEthercat->pwm_duty_motor_3 = tmp3;
    
    //if (print_counter>1000) {
    //	log(Warning) << "output pwms are " << tmp1 <<" and " << tmp2 << endlog();
    //	print_counter = 0;
    //}
    //print_counter++;
}

void TUeES030::write_ffmotors(float val1, float val2, float val3) {

    int16 tmp1 = (int16) val1;
    int16 tmp2 = (int16) val2;
    int16 tmp3 = (int16) val3;

    // saturate the ff currents
    // PWM 1: 6A, PWM 2,3: 3A
    // TODO, what are the limits???
    if (tmp1 > 6000)
        tmp1 = 6000;
    if (tmp1 < -6000)
        tmp1 = -6000;

    if (tmp2 > 3000)
        tmp2 = 3000;
    if (tmp2 < -3000)
        tmp2 = -3000;

    if (tmp3 > 3000)
        tmp3 = 3000;
    if (tmp3 < -3000)
        tmp3 = -3000;

    m_out_tueEthercat->ff1 = tmp1;
    m_out_tueEthercat->ff2 = tmp2;
    m_out_tueEthercat->ff3 = tmp3;
}

void TUeES030::write_mcommands(uint8 val1, uint8 val2, uint8 val3) {

    if (val1 > 3)
        val1 = 0;

    if (val2 > 3)
        val2 = 0;

    if (val3 > 3)
        val3 = 0;

    m_out_tueEthercat->mcom1 = (uint16) val1;
    m_out_tueEthercat->mcom2 = (uint16) val2;
    m_out_tueEthercat->mcom3 = (uint16) val3;
}

void TUeES030::write_analog_out(float val1, float val2) {

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

// EL6022
void TUeES030::executeStateActions() {
    switch (state) {
        case START:
            
            trial = 0;
            m_out_tueEthercat->out_el6022.control = 0x00;
            m_out_tueEthercat->out_el6022.output_length = 0x00;
            
            for (unsigned int j = 0; j < RS485_MAX_DATA_LENGTH; j++) {
                m_out_tueEthercat->out_el6022.buffer_out[j] = 0x00;
            }
            break;
        case INIT_REQ:
            m_out_tueEthercat->out_el6022.control = INIT_REQUEST;
            break;
        case INIT_WAIT:
            trial++;
            break;
        case PREP_REQ:
            trial=0;
            m_out_tueEthercat->out_el6022.control = 0x00;
            break;
        case PREP_WAIT:
            trial++;
            break;
        case RUN:
            port_running.write(true);
            
            //Writing incoming data into controller buffer waiting to be written to terminal.
            if (port_in.read(msg_out) == NewData) {
                for (unsigned int i = 0; i < msg_out.channels[0].datasize; i++) {
                    if ((uint8)bytesOut.size() >= MAX_OUT_QUEUE_SIZE) {
                        //log(Warning) << "Max out queue size reached on RS485 channel. Throwing away old bytes..." << endlog();
                        bytesOut.pop();
                    } 
                    bytesOut.push(msg_out.channels[0].datapacket[i]);
                }
            }
            
            if (TUeES030::read_rx()) {
                port_out.write(msg_in);
                port_rx_ready.write(true);
            } else {
                port_rx_ready.write(false);                 
            }
            
            TUeES030::write_tx(); // write after read
            
        break;
    }
}

void TUeES030::updateState() {
    switch (state) {
        case START:
            state=INIT_REQ;
            log(Debug) << "The state machine (re)started on RS485 channel " << endlog();
            break;
        case INIT_REQ:
            state=INIT_WAIT;
            log(Debug) << "The controller requests terminal for initialisation on RS485 channel " << endlog();
            break;
        case INIT_WAIT:
            if (readSB(INIT_ACCEPTED)) {
                state=PREP_REQ;
                log(Debug) << "Initialisation was completed by the terminal on RS485 channel " << endlog();
                break;
            }
            if (trial>MAX_TRIALS) {
                state=START;
                log(Debug) << "Max num of terminal initialization trials reached on RS485 channel " << endlog();
                break;
            } else {
                state=INIT_WAIT;
                break;
            }
        case PREP_REQ:
            state=PREP_WAIT;
            log(Debug) << "The controller requests terminal to prepare for serial data exchange on RS485 channel " << endlog();
            break;
        case PREP_WAIT:
            if (!readSB(INIT_ACCEPTED)) {
                state=RUN;
                log(Debug) << "The terminal is ready for serial data exchange on RS485 channel " << endlog();
                break;
            }
            if (trial>MAX_TRIALS) {
                state=START;
                log(Debug) << "Max num of terminal preparation trials reached on RS485 channel " << endlog();
                break;
            } else {
                state=PREP_WAIT;
                break;
            }
        case RUN:
            if (readSB(PARITY_ERROR)) {
                log(Warning) << "Parity error on RS485 channel!" << endlog();
            }
            if (readSB(FRAMING_ERROR)) {
                log(Warning) << "Framing error on RS485 channel!" << endlog();
            }
            if (readSB(OVERRUN_ERROR)) {
                log(Warning) << "Overrun error on RS485 channel!" << endlog();
            }
            state=RUN;
            break;
        default:
            state=START;
    }
}

bool TUeES030::read_rx() {
	log(Warning) << "Receive request:  " << readSB(RECEIVE_REQUEST) << endlog();
	log(Warning) << "Receive accepted: " << readCB(RECEIVE_ACCEPTED) << endlog();
	log(Warning) << "Input length:     " << m_in_tueEthercat->in_el6022.input_length << endlog();
    if (readSB(RECEIVE_REQUEST)!=readCB(RECEIVE_ACCEPTED)) {
        uint8 length = m_in_tueEthercat->in_el6022.input_length;
        
        msg_in.channels[0].datapacket.clear();
        msg_in.channels[0].datapacket.resize(length);
        
        for (unsigned int i = 0; i < length; i++) {
            msg_in.channels[0].datapacket[i] = m_in_tueEthercat->in_el6022.buffer_in[i];
        }
        msg_in.channels[0].datasize = length;
        
        log(Warning) << "Read " << (uint16) length << " bytes on RS485 channel: ";
        for (unsigned int i = 0; i < length; i++) {
            log(Debug) << (unsigned int) m_in_tueEthercat->in_el6022.buffer_in[i] << " ";
        }
        log(Warning) << endlog();
        control = m_out_tueEthercat->out_el6022.control;
        log(Warning) << "Control_before: " << control << endlog();
        m_out_tueEthercat->out_el6022.control = m_out_tueEthercat->out_el6022.control ^ RECEIVE_ACCEPTED; // acknowledge that the new data is received; ^ = toggle
        control = m_out_tueEthercat->out_el6022.control;
        log(Warning) << "Control_after:  " << control << endlog();
        return true;
    }
    return false;
}

bool TUeES030::write_tx() {
    if (readCB(TRANSMIT_REQUEST)==readSB(TRANSMIT_ACCEPTED)) {
        uint8 length = 0;
        
        while ((!bytesOut.empty()) && (length < RS485_MAX_DATA_LENGTH)) {
            m_out_tueEthercat->out_el6022.buffer_out[length] = bytesOut.front();
            bytesOut.pop();
            log(Warning) << "Writing " << endlog();
            length++;
        }
        
        if (length == 0) return false;

        m_out_tueEthercat->out_el6022.output_length = length;
        
        log(Warning) << "Written " << (uint16) length << " bytes on RS485 channel: ";
        for (unsigned int i = 0; i < length; i++) {
            log(Debug) << (unsigned int) m_out_tueEthercat->out_el6022.buffer_out[i] << " ";
        }
        log(Debug) << endlog();
        
        m_out_tueEthercat->out_el6022.control = m_out_tueEthercat->out_el6022.control ^ TRANSMIT_REQUEST; // request transmitting the new data; ^ = toggle
        
        return true;
    }
    return false;
}

bool TUeES030::readSB(uint8 bitmask) {
    return ((m_in_tueEthercat->in_el6022.status ^ old_status) & bitmask)==bitmask;
}

bool TUeES030::readCB(uint8 bitmask) {
    return ((m_out_tueEthercat->out_el6022.control /*^ old_control*/) & bitmask)==bitmask;
}

void TUeES030::stop() {
	return;
}

namespace {
soem_master::SoemDriver* createTUeES030(ec_slavet* mem_loc) {
    return new TUeES030(mem_loc);
}

const bool registered0 =
		soem_master::SoemDriverFactory::Instance().registerDriver(
                "TUeES030", createTUeES030);
}
