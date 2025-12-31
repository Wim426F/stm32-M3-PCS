/*
 * This file is part of the Model 3 PCS Controller project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2025 Wim Boone
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
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/crc.h>
#include "stm32_can.h"
#include "canmap.h"
#include "cansdo.h"
#include "sdocommands.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "terminalcommands.h"
#include "PCSCan.h"

#define PRINT_JSON 0

extern "C" void __cxa_pure_virtual() { while (1); }

struct CanMsg {
   uint32_t id;
   uint32_t data[2];
   uint8_t dlc;
};
static CanMsg rxQueue[20];  // Circular buffer (size 20 for bursts)
static volatile int rxWrite = 0;  // ISR writes
static volatile int rxRead = 0;   // Task reads

static Stm32Scheduler *scheduler;
static CanHardware* can;
static CanMap* canMap;
static CanSdo* canSdo;
static Terminal* terminal;

static uint32_t startTime;
static bool CAN_Enable = false;
static uint16_t ChgPower = 0;
static uint8_t pwrDntmr = 10;
static bool ZeroPower = false;


static void delay_ms(int64_t FLASH_DELAY)
{
    int64_t i;
    for (i = 0; i < (FLASH_DELAY * 2000); i++)
    __asm__("nop");
}

static bool CheckDelay()
{
   uint32_t now = rtc_get_counter_val();
   uint32_t start = Param::GetInt(Param::timedly) * 60;
   if (start < 60)
   {
      return 1;
   }
   else
   {
      return start > 0 && (now - startTime) > start;
   }
}

void handle109(uint32_t data[2])
{
   uint8_t* bytes = (uint8_t*)data;//Mux id in byte 0.

   static bool vcuEn = false;

   Param::SetInt(Param::opmode, bytes[0]); // opmode from VCU

   uint16_t vcuHVvolts=(bytes[2]<<8 |bytes[1]); //hv voltage from vcu

   Param::SetInt(Param::udcspnt, (bytes[4]<<8 | bytes[3])); // HV voltage setpoint from vcu
   Param::SetInt(Param::pacspnt, (bytes[6]<<8 | bytes[5])); // max charger power from vcu
   
   // Set iaclim from (bits 56–59, encoded as 0–15 for 1–16A)
   uint8_t currentLimit = bytes[7] & 0xF; // Extract 4-bit CurrentLimit
   Param::SetInt(Param::iaclim, currentLimit + 1); // Map 0–15 to 1–16A

   if((bytes[7]>>4)==0xA) Param::SetInt(Param::chargerEnable, 1); // enable/disable request from vcu
   if((bytes[7]>>4)==0xC) Param::SetInt(Param::chargerEnable, 0);
}


static void ChargerStateMachine()
{
   uint8_t opmode = Param::GetInt(Param::opmode);

   switch (opmode)
   {
   case MOD_OFF:
      ZeroPower = true; // charger power =0 in off.
      CAN_Enable = false;
      pwrDntmr = 10;    // reset powerdown timer

      DigIo::pcsena_out.Clear();
      DigIo::dcdcena_out.Set();
      DigIo::chena_out.Set();
      Param::SetInt(Param::activate, EN_NONE);
      break;

   case MOD_PRECHARGE:
      break;

   case MOD_RUN:
      ZeroPower = true; // charger power=0 in drive.
      CAN_Enable = true;

      DigIo::pcsena_out.Set();
      DigIo::chena_out.Set(); // charger off
      DigIo::dcdcena_out.Clear(); // DC-DC on   
      Param::SetInt(Param::activate, EN_DCDC);   
      break;

   case MOD_CHARGE:
      startTime = rtc_get_counter_val();
      ZeroPower = false;
      CAN_Enable = true;

      if (!ZeroPower)  DigIo::chena_out.Clear(); // charger on
      DigIo::pcsena_out.Set();
      DigIo::dcdcena_out.Clear();   // DC-DC on
      Param::SetInt(Param::activate, EN_BOTH);

      break;
   
   default:
      break;
   }
}

uint16_t ChgPwrRamp()
{
   uint8_t Charger_state = Param::GetInt(Param::CHG_STAT);
   uint16_t Charger_Pwr_Max = Param::GetInt(Param::pacspnt);
   if (Charger_state != chargerStates::ENABLE)
      ChgPower = 0; // Set power =0 unless charger is enabled.
   if (ZeroPower)
      ChgPower = 0;
   else
   {
      if (ChgPower < Charger_Pwr_Max)
         ChgPower += 10;
      if (ChgPower > Charger_Pwr_Max)
         ChgPower -= 10;
   }

   return ChgPower;
}

static void Ms1Task(void) // actually 10 and 100ms task effectively
{
   // Process RX queue here instead of in ISR CanCallback, this avoids ISR interrupting too long
   while (rxRead != rxWrite) {
      CanMsg m = rxQueue[rxRead];
      rxRead = (rxRead + 1) % 20;
      switch (m.id) {
         case 0x204: PCSCan::handle204(m.data); break;
         case 0x224: PCSCan::handle224(m.data); break;
         case 0x264: PCSCan::handle264(m.data); break;
         case 0x2A4: PCSCan::handle2A4(m.data); break;
         case 0x2C4: PCSCan::handle2C4(m.data); break;
         case 0x3A4: PCSCan::handle3A4(m.data); break;
         case 0x424: PCSCan::handle424(m.data); break;
         case 0x504: PCSCan::handle504(m.data); break;
         case 0x76C: PCSCan::handle76C(m.data); break;
         case 0x109: handle109(m.data); break;
         default: break;
      }
   }


   /* * FIXME: The CAN driver currently struggles with TX Mailbox congestion
      * when firing many messages back-to-back. Instead of using blocking delays,
      * we stagger the messages across a 100ms window using the 1ms base task.
      * This ensures the hardware mailboxes have time to clear between bursts.
   */
   static uint8_t tick_count = 0; // 0-99 (100ms cycle)

   if (!CAN_Enable)
   {
      tick_count = 0;
      return;
   }

   // Handle the 10ms messages (Total: 3 messages)
   // We fire these at 5ms, 15ms, 25ms... 95ms
   if ((tick_count % 10) == 5)
   {
      PCSCan::Msg13D();
      PCSCan::Msg22A();
      PCSCan::Msg3B2();
   }

   // Handle the 100ms messages (Total: 11 messages)
   // We spread these out so we never hit the driver too hard
   switch (tick_count)
   {
      case 0:
         PCSCan::Msg20A();
         PCSCan::Msg212();
         break;
      case 10:
         PCSCan::Msg21D();
         PCSCan::Msg232();
         break;
      case 20:
         PCSCan::Msg23D();
         PCSCan::Msg25D();
         break;
      case 30:
         PCSCan::Msg2B2(ChgPwrRamp());
         PCSCan::Msg321();
         break;
      case 40:
         PCSCan::Msg333();
         PCSCan::Msg3A1();
         break;
   }

   // Increment and wrap
   tick_count++;
   if (tick_count >= 100)
   {
      tick_count = 0;
   }
}

static void Ms50Task(void)
{
   if (CAN_Enable)
   {
      // Send 50ms PCS CAN when enabled.
      PCSCan::Msg545();
   }
}

// sample 100ms task
static void Ms100Task(void)
{
   DigIo::led_out.Toggle();
   // The boot loader enables the watchdog, we have to reset it
   // at least every 2s or otherwise the controller is hard reset.
   iwdg_reset();
   // Calculate CPU load. Don't be surprised if it is zero.
   float cpuLoad = scheduler->GetCpuLoad() / 10.0f;
   // This sets a fixed point value WITHOUT calling the parm_Change() function
   Param::SetFloat(Param::cpuload, cpuLoad);
   // Set timestamp of error message
   ErrorMessage::SetTime(rtc_get_counter_val());
   Param::SetInt(Param::uptime, rtc_get_counter_val());
   Param::SetFloat(Param::uaux, AnaIn::uaux.Get() / 223.418f);

   ChargerStateMachine();
   PCSCan::AlertHandler();
}


//Whenever the user clears mapped can messages or changes the
//CAN interface of a device, this will be called by the CanHardware module
static void SetCanFilters()
{
   // Set up CAN  callback and messages to listen for
   can->RegisterUserMessage(0x204); // PCS Charge Status
   can->RegisterUserMessage(0x224); // PCS DCDC Status
   can->RegisterUserMessage(0x264); // PCS Chg Line Status
   can->RegisterUserMessage(0x2A4); // PCS Temps
   can->RegisterUserMessage(0x2C4); // PCS Logging
   can->RegisterUserMessage(0x3A4); // PCS Alert Matrix
   can->RegisterUserMessage(0x424); // PCS Alert Log
   can->RegisterUserMessage(0x504); // PCS Boot ID
   can->RegisterUserMessage(0x76C); // PCS Debug output
   can->RegisterUserMessage(0x109); // VCU charge request
}

/** This function is called when the user changes a parameter */
void Param::Change(Param::PARAM_NUM paramNum)
{
   switch (paramNum)
   {
   case Param::canspeed: 
      can->SetBaudrate((CanHardware::baudrates)Param::GetInt(Param::canspeed));
      break;

   case Param::nodeid:
      canSdo->SetNodeId(Param::GetInt(Param::nodeid)); //Set node ID for SDO access
      //can->RegisterUserMessage(0x600 + Param::GetInt(Param::nodeid)); // Dynamic CanSDO request COB-ID (0x600 + Node-ID)
      break;

   default:
      // Handle general parameter changes here. Add paramNum labels for handling specific parameters
      break;
   }
}

static bool CanCallback(uint32_t id, uint32_t data[2], uint8_t dlc) {
   // queue all messages instead of processing the handlers directly, this avoids ISR interrupting too long
   int nextWrite = (rxWrite + 1) % 20;
   if (nextWrite != rxRead) {
      rxQueue[rxWrite].id = id;
      rxQueue[rxWrite].data[0] = data[0];
      rxQueue[rxWrite].data[1] = data[1];
      rxQueue[rxWrite].dlc = dlc;
      rxWrite = nextWrite;
      return true;
   }
   return false;
}

// Whichever timer(s) you use for the scheduler, you have to
// implement their ISRs here and call into the respective scheduler
extern "C" void tim2_isr(void)
{
   scheduler->Run();
}

extern "C" int main(void)
{
   extern const TERM_CMD termCmds[];

   clock_setup(); // Must always come first
   rtc_setup();
   ANA_IN_CONFIGURE(ANA_IN_LIST);
   DIG_IO_CONFIGURE(DIG_IO_LIST);
   AnaIn::Start();             // Starts background ADC conversion via DMA
   write_bootloader_pininit(); // Instructs boot loader to initialize certain pins
   gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN1_REMAP_PORTB);

   tim_setup();                  // Use timer3 for sampling pilot PWM
   nvic_setup();                 // Set up some interrupts
   parm_load();                  // Load stored parameters

   //store a pointer for easier access
   FunctionPointerCallback canCb(CanCallback, SetCanFilters);

   Stm32Can c(CAN1, (CanHardware::baudrates)Param::GetInt(Param::canspeed), true);
   can = &c;
   can->AddCallback(&canCb);
   SetCanFilters();

   CanMap cm(&c);
   canMap = &cm;
   TerminalCommands::SetCanMap(canMap);

   CanSdo sdo(&c, &cm);
   canSdo = &sdo;
   canSdo->SetNodeId(Param::GetInt(Param::nodeid)); //Set node ID for SDO access e.g. by wifi module
   SdoCommands::SetCanMap(canMap);

   Stm32Scheduler s(TIM2); // We never exit main so it's ok to put it on stack
   scheduler = &s;

   Terminal t(USART3, termCmds);
   terminal = &t;

   // Up to four tasks can be added to each timer scheduler
   // AddTask takes a function pointer and a calling interval in milliseconds.
   // The longest interval is 655ms due to hardware restrictions
   // You have to enable the interrupt (int this case for TIM2) in nvic_setup()
   // There you can also configure the priority of the scheduler over other interrupts
   s.AddTask(Ms100Task, 100);
   s.AddTask(Ms50Task, 50);
   s.AddTask(Ms1Task, 1);

   // backward compatibility, version 4 was the first to support the "stream" command
   Param::SetInt(Param::version, 4);

   // Now all our main() does is running the terminal
   // All other processing takes place in the scheduler or other interrupt service routines
   // The terminal has lowest priority, so even loading it down heavily will not disturb
   // our more important processing routines.
   while(1)
   {
      char c = 0;
      CanSdo::SdoFrame* sdoFrame = sdo.GetPendingUserspaceSdo();
      terminal->Run();

      if (canSdo->GetPrintRequest() == PRINT_JSON)
      {
         TerminalCommands::PrintParamsJson(canSdo, &c);
      }
      if (0 != sdoFrame)
      {
         CanSdo::SdoFrame sdoOrig = *sdoFrame;
         SdoCommands::ProcessStandardCommands(sdoFrame);

         sdo.SendSdoReply(sdoFrame);
      }
   }

   return 0;
}
