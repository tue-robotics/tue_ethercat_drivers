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

#ifndef DUMMYDRIVERS_H
#define DUMMYDRIVERS_H

#include <soem_master/soem_driver_factory.h>
#include <soem_master/soem_driver.h>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <iostream>
#include "COE_config.h"

using namespace std;

using namespace RTT;

namespace soem_beckhoff_drivers {

    class Dummy: public soem_master::SoemDriver {
    public:
        Dummy(ec_slavet* mem_loc);
        ~Dummy() {};

        void update();
        bool configure();
        bool start();
        void stop();

    private:
    
    };
}
#endif
