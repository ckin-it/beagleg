// -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// (c) 2013, 2014 Henner Zeller <h.zeller@acm.org>
//
// This file is part of BeagleG. http://github.com/hzeller/beagleg
//
// BeagleG is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// BeagleG is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.
//
// see motor-interface-constants.h for available PINs

// This contains the defines the GPIO mappings for BUMPS
// https://github.com/hzeller/bumps


#define MOTOR_ENABLE_GPIO  PIN_P9_25  // ENn
#define MOTOR_ENABLE_IS_ACTIVE_HIGH 1  // 1 if EN, 0 if ~EN

#define MOTOR_1_STEP_GPIO  PIN_P8_43  // motor 1

#define MOTOR_1_DIR_GPIO   PIN_P8_12  // motor 1

#define IN_1_GPIO          PIN_P9_41  // END_DOWN
#define IN_2_GPIO          PIN_P8_16  // END_UP

#define AUX_1_GPIO         PIN_P8_7  // AUX_2 "Aux, Open Collector"
#define AUX_2_GPIO         PIN_P8_8  // AUX_2 "Aux, Open Collector"
