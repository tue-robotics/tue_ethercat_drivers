/***************************************************************************
 tag: Max Baeten
 Dummy driver for TUeES010

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

#include "TUeEC010_dummy.hpp"

using namespace soem_beckhoff_drivers;

TUeEC010::TUeEC010(ec_slavet* mem_loc) :
	soem_master::SoemDriver(mem_loc) { 
		m_service->doc(std::string(m_datap->name));
		}

bool TUeEC010::start() {
	return true;
}

bool TUeEC010::configure() {
	return true;
}

void TUeEC010::update() {
}

void TUeEC010::stop() {
}

namespace {
soem_master::SoemDriver* createTUeEC010(ec_slavet* mem_loc) {
    return new TUeEC010(mem_loc);
}

const bool registered0 =
		soem_master::SoemDriverFactory::Instance().registerDriver(
                "TUeEC010", createTUeEC010);
}
