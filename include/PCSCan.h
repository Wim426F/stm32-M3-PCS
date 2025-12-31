/*
 * This file is part of the Model 3 PCS Controller project.
 *
 * Copyright (C)  2022 Damien Maquire
 *                2025 Wim Boone
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PCSCan_h
#define PCSCan_h

#include <stdint.h>
#include "stm32_can.h"
#include "params.h"
#include "digio.h"

class PCSCan
{
public:
    static void Msg13D();
    static void Msg20A();
    static void Msg212();
    static void Msg21D();
    static void Msg22A();
    static void Msg232();
    static void Msg23D();
    static void Msg25D();
    static void Msg2B2(uint16_t Charger_Power);
    static void Msg321();
    static void Msg333();
    static void Msg3A1();
    static void Msg3B2();
    static void Msg545();

    static void handle204(uint32_t data[2]);
    static void handle224(uint32_t data[2]);
    static void handle264(uint32_t data[2]);
    static void handle2A4(uint32_t data[2]);
    static void handle2C4(uint32_t data[2]);
    static void handle3A4(uint32_t data[2]);
    static void handle424(uint32_t data[2]);
    static void handle504(uint32_t data[2]);
    static void handle76C(uint32_t data[2]);
    static void AlertHandler();

private:

};

static uint8_t CalcPCSChecksum(uint8_t *data, uint16_t id);
static int16_t ProcessTemps(uint16_t InVal);
static void ProcessCANRat(uint16_t AlertCANId,uint8_t AlertRxError);

#endif /* PCSCan_h */
