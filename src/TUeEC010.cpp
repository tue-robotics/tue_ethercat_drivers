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

#include "TUeEC010.hpp"

using namespace soem_beckhoff_drivers;

bool Dummy::start() {
	log(Warning) << "Starting DummyDriver" << endlog();
	return true;
}

bool Dummy::configure() {	
    log(Warning) << "Configuring DummyDriver" << endlog();
	return true;
}

void Dummy::update() {
	log(Warning) << "Updating DummyDriver" << endlog();
}

namespace {
soem_master::SoemDriver* createDummy(ec_slavet* mem_loc) {
    return new Dummy(mem_loc);
}

const bool registered0 =
		soem_master::SoemDriverFactory::Instance().registerDriver(
                "TUeEC010", createDummy);
}
