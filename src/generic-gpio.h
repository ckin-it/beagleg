/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2014 Henner Zeller <h.zeller@acm.org>
 *
 * This file is part of BeagleG. http://github.com/hzeller/beagleg
 *
 * BeagleG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BeagleG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __GENERIC_GPIO_H
#define __GENERIC_GPIO_H

#include "motor-interface-constants.h"

int get_gpio(uint32_t gpio_def);

void set_gpio(uint32_t gpio_def);
void clr_gpio(uint32_t gpio_def);

bool map_gpio();
void unmap_gpio();

class FSGpio {
public:

  typedef enum direction {
    INPUT,
    OUTPUT
  } Direction;

  typedef enum edge {
    NONE,
    RISING,
    FALLING,
    BOTH
  } Edge;

  FSGpio(uint32_t gpio_def, Direction dir, bool active_high);
  ~FSGpio();

  int GetValue();
  int GetFd();
  int TriggerOnActive();
  int TriggerOnInactive();

private:

  int SetDir(Direction dir);
  int SetEdge(Edge edge);

  typedef struct gpio_desc {
    char *path;
    int idx;
    int base;
    int fd;
    bool active_high;
    gpio_desc() {}
    gpio_desc(const gpio_desc& source) : idx(source.idx), base(source.base),
                                         fd(source.fd) {
      asprintf(&path, source.path);
    }
  } GpioDesc;

  GpioDesc gpio_;

};

#endif // __GENERIC_GPIO_H
