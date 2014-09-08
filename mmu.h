/*
 * Copyright (C) 2014 Frederic Meyer
 * 
 * This file is part of gnes.
 *
 * gnes is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *   
 * gnes is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gnes.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _MMU_H_
#define _MMU_H_

#include <stdint.h>

uint8_t mmu_read(uint16_t addr);
uint16_t mmu_read16(uint16_t addr);
void mmu_write(uint16_t addr, uint8_t value);
void mmu_write16(uint16_t addr, uint16_t value);

#endif
