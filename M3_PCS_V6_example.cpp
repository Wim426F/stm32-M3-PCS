/*
Tesla Model 3 Power Conversion System Controller Software. Alpha version.

V6 features auto enable and disable of PCS and DCDC function when car reports on.
also experimantal version for LIM testing.

Copyright 2021 
Damien Maguire

What works : 
Enable/disable and set output for dcdc converter on both US and EU version.Tested on bench and in vehicle.
12v derived precharge tested on bench and seems to work.

Charge function tested on US version to 1.4kw AC.

Using V1 PCS controller hardware :
https://github.com/damienmaguire/Tesla-Model-3-Charger

Messages are sent over IPC can only! The PCS does not seem to require any messages over its CP can.
PWM lines are not PWM lines. Just logic level enables.
DCDC must be at 5v to enable.
CHG must be at 5v to enable.
*/
#include <Metro.h>
#include <due_can.h>  
#include <due_wire.h> 
#include <DueTimer.h>  
#include <Wire_EEPROM.h> 
#include <AverageValue.h>


#define SerialDEBUG SerialUSB
 template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Allow streaming
 
#define PCS_ENABLE 50
#define PWM_DCDC 8
#define PWM_CHG 9
#define EVSE_PILOT 2
#define EVSE_ACT 44
#define EVSE_PROX A0
#define OUT1 48
#define OUT2 49
#define IN1 6
#define IN2 7
#define led 13

Metro timer_diag = Metro(1100);
Metro timer_Frames100 = Metro(100);
Metro timer_Frames10 = Metro(10);
Metro timer_Frames50 = Metro(50);

float AClim,ACamps,ACpwr,DCDCvolts,DCDCamps,HVvolts,LVvolts;
float ACvolts=0;
uint16_t CHGpwr=0;
uint16_t voltsSetPnt = 395; //set point for charger HV.
uint16_t vcuHVvolts=0;
uint16_t chgPwrSetPnt = 0;
uint16_t maxChgPwr = 3500;
bool vcuEn=true;
bool mux545=true;
bool mux3b2=true;
bool DCDCact=false;
bool CHGact=false;
bool PCSact=false;
bool USpcs=true;
bool Menudisp=false;
byte Count545=0;
bool ACState=false;
bool CANState=false;
bool autoDCDC=true;
bool autoCHG=true;
bool CHGhvreq=false;
bool CHGpwror=false;
byte vcuMode=0x00;
byte limMode=0x00;

CAN_FRAME outFrame;  //A structured variable according to due_can library for transmitting CAN data.
CAN_FRAME inFrame;    //structure to keep inbound inFrames

////////////////////////////////////////////////////////////////////////
//*********EVSE VARIABLE   DATA ******************
byte Proximity = 0;
//proximity status values for type 1
#define Unconnected 0 // 3.3V
#define Buttonpress 1 // 2.3V
#define Connected 2 // 1.35V

volatile uint32_t pilottimer = 0;
volatile uint16_t timehigh, duration = 0;
volatile uint16_t accurlim = 0;
volatile uint16_t accurlimTMP = 0;
volatile int dutycycle = 0;
byte CPtype=2;  //set type 2 charge socket

uint16_t cablelim =32; // Type 2 cable current limit
//////////////////////////////////////////////////////////////////////

// Number of values to calculate with. Prevents memory problems
const long MAX_VALUES_NUM = 40;//using this to smooth out the pilot current reading.

AverageValue<long> averageValue(MAX_VALUES_NUM);

void setup() 
  {
  Can0.begin(CAN_BPS_500K);   // IPC CAN To PCS
  Can1.begin(CAN_BPS_500K);   // External CAN
  Can0.watchFor();
  Can1.watchForRange(0x109, 0x10f); //only receive the VCU msg on can 1.
  Serial.begin(115200);  //Initialize our USB port which will always be redefined as SerialUSB to use the Native USB port tied directly to the SAM3X processor.
  Serial2.begin(19200); //setup serial 2 for wifi access
  pinMode(PCS_ENABLE,OUTPUT);
  pinMode(PWM_DCDC,OUTPUT);
  pinMode(PWM_CHG,OUTPUT);
  pinMode(EVSE_ACT,OUTPUT);
  pinMode(OUT1,OUTPUT);
  pinMode(OUT2,OUTPUT);
  pinMode(led,OUTPUT);
  pinMode(IN1,INPUT);
  pinMode(IN2,INPUT);
  pinMode(EVSE_PILOT,INPUT);
  pinMode(EVSE_PROX,INPUT);
  

  digitalWrite(EVSE_ACT,LOW); //turn off evse at start
    

digitalWrite(PCS_ENABLE,LOW);  //pcs off at startup
//digitalWrite(PWM_DCDC,LOW); //set dcdc pwm line high
//digitalWrite(PWM_CHG,HIGH); //set charger pwm line high
  
  }

  
void loop()
{ 
checkCAN();
  if(timer_diag.check())
  {
  if(autoDCDC)
  {
    if(vcuMode==0x01)//if autodcdc is enabled and vcu reports run state then fire up pcs and dcdc
    {
      PCSact=true;
      DCDCact=true;
    }
    if(vcuMode==0x00)//if autodcdc is enabled and vcu reports off state then shutdown dcdc and pcs
    {
      DCDCact=false;
      PCSact=false;
    }


    
  }
  }

  if(timer_Frames10.check())
  {
  if(PCSact) Msgs10ms();
  if(vcuMode==0x04) CHGramp();  //call charge ramp routine
 }

  if(timer_Frames50.check())
  {
  if(PCSact) Msgs50ms();
 }

  if(timer_Frames100.check())
  {
  externCAN();  //always send eternal can
  if(PCSact) Msgs100ms();
  if(ACState) digitalWrite(EVSE_ACT,HIGH); //turn on evse
  if(!ACState) digitalWrite(EVSE_ACT,LOW); //turn off evse
  if(DCDCact) digitalWrite(PWM_DCDC,LOW); //set dcdc pwm line high
  if(!DCDCact) digitalWrite(PWM_DCDC,HIGH); //set dcdc pwm line low
  if(CHGact) digitalWrite(PWM_CHG,LOW); //set charger pwm line high
  if(!CHGact) digitalWrite(PWM_CHG,HIGH); //set charger pwm line low
  if(PCSact) digitalWrite(PCS_ENABLE,HIGH); //set PCS enable line high
  if(!PCSact) digitalWrite(PCS_ENABLE,LOW); //set PCS enable line low
  if(autoCHG) //if we are in auto charge mode

}

if((vcuMode==0x04)&&(vcuEn==true))  //if we have a evse plug detected and are allowed to enable by the vcu ...
{
  PCSact=true;  //wake up the pcs
  CHGhvreq=true; //request hv on from the vcu

}




  if(vcuMode==0x04) //wait for the vcu to enter charge mode (precharge complete, contactors closed, cooling pump on)
  {
    DCDCact=true; //fire off the dcdc converter
    //digitalWrite(EVSE_ACT,HIGH); //turn on evse power
    CHGact=true;  //fire off the charger
    CHGhvreq=true; //request hv on from the vcu
    
  }


if(vcuMode==0x04) //only disable charge mode if we are in charge mode !
{
if(vcuEn==false)  //if we are disabled by the vcu
{
    chgPwrSetPnt=0; //set charger power at 0W
    if(ACamps==0)   //wait for ac current to hit 0 then disable
    {
    DCDCact=false; //turn off the dcdc converter
    CHGact=false;  //turn off the charger
    CHGhvreq=false; //request hv off from vcu
    PCSact=false;  //turn off the pcs
    }
}

}

}



void Msgs10ms()                       //10ms messages here
{
                                    //This message is the heart of the beast. controls dcdc modes and en/dis of dcdc and charger.
        outFrame.id = 0x22A;       //HVS PCS control. kills hvp mia alert
        outFrame.length = 4;            
        outFrame.extended = 0;          
        outFrame.rtr=1;                 
        outFrame.data.bytes[0]=0x00; //precharge request voltage. 16 bit signed int. scale 0.1. Bytes 0 and 1. 
        outFrame.data.bytes[1]=0x00; 
       if(DCDCact && CHGact)   outFrame.data.bytes[2]=((lowByte(vcuHVvolts)<<4)|0xD);  //7D = charge and dcdc enabled.
       if(DCDCact && !CHGact)  outFrame.data.bytes[2]=((lowByte(vcuHVvolts)<<4)|0x9);  //79 = support (dcdc enabled,chg disabled)
                                      //78 = shutdown(dcdc hw enabled)
                                      //7A = precharge
                                      //7B = discharge
                                      
       if(!DCDCact &&  CHGact) outFrame.data.bytes[2]=((lowByte(vcuHVvolts)<<4)|0x4); //74 = dcdc off chg enabled                             
       if(!DCDCact && !CHGact) outFrame.data.bytes[2]=((lowByte(vcuHVvolts)<<4)|0x0);//0x70 = shutdown and both dcdc and chg hw disabled.
                                                  //391vdc. I think this will need to reflect actual dc link voltage.   
                                                  //needs to rise in accordance with precharge or precharge mode will fault out.    
                                                  //dc link in bits 20-30 as an 11bit signed int. scale of 1. 
                                                                       
        outFrame.data.bytes[3]=((highByte(vcuHVvolts)<<4)|lowByte(vcuHVvolts)>>4);              //0x187 = 391 dec.
        Can0.sendFrame(outFrame); //send to pcs IPC can
 
        outFrame.id = 0x3B2;       //BMS log message
        outFrame.length = 8;            
        outFrame.extended = 0;          
        outFrame.rtr=1;       

        if(mux3b2)
        {
        outFrame.data.bytes[0]=0x5E;  //kills bms mia
        outFrame.data.bytes[1]=0x0F; 
        outFrame.data.bytes[2]=0xF9;  
        outFrame.data.bytes[3]=0xFF;   
        outFrame.data.bytes[4]=0x00;
        outFrame.data.bytes[5]=0xCB; 
        outFrame.data.bytes[6]=0xB6;
        outFrame.data.bytes[7]=0x04;
        mux3b2=false;
        Can0.sendFrame(outFrame); //send to pcs IPC can
        }
        else
        {
        outFrame.data.bytes[0]=0x5D;  
        outFrame.data.bytes[1]=0x0F; 
        outFrame.data.bytes[2]=0xF9;  
        outFrame.data.bytes[3]=0xFF;   
        outFrame.data.bytes[4]=0x00;
        outFrame.data.bytes[5]=0xCB; 
        outFrame.data.bytes[6]=0xB6;
        outFrame.data.bytes[7]=0x04;
        mux3b2=true;
        Can0.sendFrame(outFrame); //send to pcs IPC can
        }
}
    
void Msgs50ms()                       //50ms messages here
{
        outFrame.id = 0x545;       //VCFront unknown message
        outFrame.length = 8;            
        outFrame.extended = 0;          
        outFrame.rtr=1;       

        if(mux545)
        {
        outFrame.data.bytes[0]=0x14;  //kills vcfront mia
        outFrame.data.bytes[1]=0x00; 
        outFrame.data.bytes[2]=0x3f;  
        outFrame.data.bytes[3]=0x70;   
        outFrame.data.bytes[4]=0x9f;
        outFrame.data.bytes[5]=0x01; 
        outFrame.data.bytes[6]=(Count545<<4)|0xA;
        PCS_cksum(outFrame.data.bytes, 0x545);
        mux545=false;
        Can0.sendFrame(outFrame); //send to pcs IPC can
        }
        else
        {
        outFrame.data.bytes[0]=0x03;  //kills vcfront mia
        outFrame.data.bytes[1]=0x19; 
        outFrame.data.bytes[2]=0x64;  
        outFrame.data.bytes[3]=0x32;   
        outFrame.data.bytes[4]=0x19;
        outFrame.data.bytes[5]=0x00; 
        outFrame.data.bytes[6]=(Count545<<4);
        PCS_cksum(outFrame.data.bytes, 0x545);
        mux545=true;
        Can0.sendFrame(outFrame); //send to pcs IPC can
        }
        Count545++;
        if(Count545>0x0F) Count545=0;



}


void Msgs100ms()                      ////100ms messages here
{

        outFrame.id = 0x333;       //UI charge request message
        outFrame.length = 4;            
        outFrame.extended = 0;          
        outFrame.rtr=1;                 
        outFrame.data.bytes[0]=0x04;  //kills ui mia
        outFrame.data.bytes[1]=0x30; //48A limit
        outFrame.data.bytes[2]=0x29;  
        outFrame.data.bytes[3]=0x07;   
        Can0.sendFrame(outFrame); //send to pcs IPC can


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Needed to kill bms mia alert but will set can irrational alert if present on US version. May need a friend....
///////////////////////////////////////////////////////////////////////////////////////////////////////
              outFrame.id = 0x2B2;       //Possible PCS charge power request message.kills bms mia.
if(USpcs)     outFrame.length = 3;        //This message is used to request charge power from the pcs. 
if(!USpcs)     outFrame.length = 5;        //This message is used to request charge power from the pcs.  
              outFrame.extended = 0;      //kills bms mia.  
              outFrame.rtr=1;              //US hvcon sends this as dlc=3. EU sends as dlc=5. A missmatch here will trigger a can rationality error.
              outFrame.data.bytes[0]=lowByte(chgPwrSetPnt);  //0x0578 = 1400 dec = 1400Watts. So 16 bit unsigned scale of 1 ... ? or possible kW with a scale of 0.001
              outFrame.data.bytes[1]=highByte(chgPwrSetPnt);  //byte 2 bit 1 may be an ac charge enable.
if(CHGact)   outFrame.data.bytes[2]=0x02; 
if(!CHGact)   outFrame.data.bytes[2]=0x00;   
if(!USpcs)    outFrame.data.bytes[3]=0x00;
if(!USpcs)    outFrame.data.bytes[4]=0x00;
              Can0.sendFrame(outFrame); //send to pcs IPC can
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        outFrame.id = 0x3A1;       //VCFront vehicle status
        outFrame.length = 8;       //This message contains the 12v dcdc target setpoint.  
        outFrame.extended = 0;     //bits 16-26 as an 11bit unsigned int. scale 0.01    
        outFrame.rtr=1;                 
        outFrame.data.bytes[0]=0x09;  //kills vcfront mia
        outFrame.data.bytes[1]=0x62; 
        outFrame.data.bytes[2]=0x78; //78 , 9 gives us a 14v target. 
        outFrame.data.bytes[3]=0x9d;  
        outFrame.data.bytes[4]=0x08;
        outFrame.data.bytes[5]=0x2C; 
        outFrame.data.bytes[6]=0x12;
        outFrame.data.bytes[7]=0x5A;
        Can0.sendFrame(outFrame); //send to pcs IPC can

        outFrame.id = 0x321;       //VCFront sensors
        outFrame.length = 8;            
        outFrame.extended = 0;          
        outFrame.rtr=1;                 
        outFrame.data.bytes[0]=0x2C;  //kills vcfront mia
        outFrame.data.bytes[1]=0xB6; 
        outFrame.data.bytes[2]=0xA8;  
        outFrame.data.bytes[3]=0x7F;   
        outFrame.data.bytes[4]=0x02;
        outFrame.data.bytes[5]=0x7F; 
        outFrame.data.bytes[6]=0x00;
        outFrame.data.bytes[7]=0x00;
        Can0.sendFrame(outFrame); //send to pcs IPC can

        outFrame.id = 0x25D;       //CP unknown static msg     
        outFrame.length = 8;         //kills cp mia   
        outFrame.extended = 0;          
        outFrame.rtr=1;                 
        outFrame.data.bytes[0]=0xd8;  
        outFrame.data.bytes[1]=0x8c; 
        outFrame.data.bytes[2]=0x01;  
        outFrame.data.bytes[3]=0xb5;   
        outFrame.data.bytes[4]=0x4a;
        outFrame.data.bytes[5]=0xc1; 
        outFrame.data.bytes[6]=0x0a;
        outFrame.data.bytes[7]=0xe0;
        Can0.sendFrame(outFrame); //send to pcs IPC can


            outFrame.id = 0x23D;       //CP AC charge current limit    
if(!USpcs)  outFrame.length = 4;       //  US pcs gives a long msg alert when 4 bytes so lets try 3 , nope lets go for 1...
if(USpcs)  outFrame.length = 2;  
            outFrame.extended = 0;          
            outFrame.rtr=1;                 
            outFrame.data.bytes[0]=0x78;  //AC charge current limit in byte 0 as 8 bit unsigned int scale 0.5. 0x40 = 64dec = 32A.0x78=60A.
            outFrame.data.bytes[1]=0x00; //kills cp mia
if(!USpcs)  outFrame.data.bytes[2]=0x00;  
if(!USpcs)  outFrame.data.bytes[3]=0x00;   
            Can0.sendFrame(outFrame); //send to pcs IPC can

        outFrame.id = 0x21D;       //CP evse status. Connected? how much amps available? from cp ecu.     
        outFrame.length = 8;       //Pilot current in byte 1 as an 8 bit unsigned int scale 0.5     
        outFrame.extended = 0;     //Cable current limit in byte 3 bits 24-30 as a 7 bit unsigned int scale 1.     
        outFrame.rtr=1;                 
        outFrame.data.bytes[0]=0x5d;  //2d for EU , 5d for US
        outFrame.data.bytes[1]=0x20; 
        outFrame.data.bytes[2]=0x00;  
        outFrame.data.bytes[3]=0x20;   
        outFrame.data.bytes[4]=0x80;
        outFrame.data.bytes[5]=0x00; 
        outFrame.data.bytes[6]=0x60;
        outFrame.data.bytes[7]=0x10;
        Can0.sendFrame(outFrame); //send to pcs IPC can
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        outFrame.id = 0x20A;       //HVP contactor state     
        outFrame.length = 8;            
        outFrame.extended = 0;          
        outFrame.rtr=1;                 
        outFrame.data.bytes[0]=0xf6;  //basically telling the pcs we have closed main contactors.
        outFrame.data.bytes[1]=0x15;  //and are ready to party.
        outFrame.data.bytes[2]=0x09;  //may need to change if we are using dcdc precharge mode....
        outFrame.data.bytes[3]=0x82;   //CP CAN
        outFrame.data.bytes[4]=0x18;
        outFrame.data.bytes[5]=0x01; 
        outFrame.data.bytes[6]=0x00;
        outFrame.data.bytes[7]=0x00;
        Can0.sendFrame(outFrame); //send to pcs IPC can

        outFrame.id = 0x212;            //BMS ready message
        outFrame.length = 8;            
        outFrame.extended = 0;          
        outFrame.rtr=1;                 
        outFrame.data.bytes[0]=0xb9;  
        outFrame.data.bytes[1]=0x1c; 
        outFrame.data.bytes[2]=0x94;  
        outFrame.data.bytes[3]=0xad;   
        outFrame.data.bytes[4]=0xc1;
        outFrame.data.bytes[5]=0x15; 
        outFrame.data.bytes[6]=0x36;
        outFrame.data.bytes[7]=0x6b;
        Can0.sendFrame(outFrame); //send to pcs
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


        
digitalWrite(led,!digitalRead(led));//blink led every time we fire this interrrupt.

}

void externCAN()
{
          outFrame.id = 0x108;            //VCU HV request
        outFrame.length = 8;            
        outFrame.extended = 0;          
        outFrame.rtr=1;                 
if(CHGhvreq)  outFrame.data.bytes[0]=0xAA;   //hv on request
if(!CHGhvreq) outFrame.data.bytes[0]=0xCC;  //hv off request  
        outFrame.data.bytes[1]=0x00; 
        outFrame.data.bytes[2]=0x00;  
        outFrame.data.bytes[3]=0x00;   
        outFrame.data.bytes[4]=0x00;
        outFrame.data.bytes[5]=0x00; 
        outFrame.data.bytes[6]=0x00;
        outFrame.data.bytes[7]=0x00;
        Can1.sendFrame(outFrame); //send to vcu
}


void checkCAN()
{

  if(Can0.available())
  {
    Can0.read(inFrame);
  if(inFrame.id == 0x264)//AC input data
  {
  AClim = (uint16_t)(((inFrame.data.bytes[5]<<8 | inFrame.data.bytes[4])&0x3ff)*0.1);
  ACpwr = ((inFrame.data.bytes[3])*.1);
  ACvolts = (((inFrame.data.bytes[1]<<8 | inFrame.data.bytes[0])&0x3FFF)*0.033);
  ACamps = (((inFrame.data.bytes[2]<<9 | inFrame.data.bytes[1])>>7)*0.1);
  }
  if(inFrame.id == 0x224)//dcdc data
  {
   DCDCamps = (((inFrame.data.bytes[3]<<8 | inFrame.data.bytes[2])&0xFFF)*0.1); //dcdc actual current. 12 bit unsigned int in bits 16-27. scale 0.1.
  }

  if(inFrame.id == 0x2C4)//dcdc data from  log messages. This is a big muxed mess. We only want mux 6 for our needs.
  {
    byte MuxID = (inFrame.data.bytes[0]);
    if((MuxID==0xE6) || (MuxID==0xC6))//if in mux 6 grab the info...
    {
     HVvolts = (((inFrame.data.bytes[3]<<8 | inFrame.data.bytes[2])&0xFFF)*0.146484); //measured hv voltage. 12 bit unsigned int in bits 16-27. scale 0.146484. 
     LVvolts = ((((inFrame.data.bytes[1]<<9 | inFrame.data.bytes[0])>>6))*0.0390625); //measured lv voltage. 10 bit unsigned int in bits 5-14. scale 0.0390626. 
    }
    
   
  }
  
}

  if(Can1.available())
  {
    Can1.read(inFrame);
  if(inFrame.id == 0x109)//VCU input data
  {
    vcuMode=inFrame.data.bytes[0];
    vcuHVvolts=(inFrame.data.bytes[2]<<8 |inFrame.data.bytes[1]); //hv voltage from vcu
    voltsSetPnt=(inFrame.data.bytes[4]<<8 |inFrame.data.bytes[3]); //hv voltage setpoint from vcu
    maxChgPwr=(inFrame.data.bytes[6]<<8 |inFrame.data.bytes[5]); //max charger power from vcu
    if((inFrame.data.bytes[7]>>4)==0xA) vcuEn=true;               //enable/disable request from vcu
    if((inFrame.data.bytes[7]>>4)==0xC) vcuEn=false;
  }

  if(inFrame.id == 0x110)//LIM controller msg
  {
    limMode=inFrame.data.bytes[0];

  }

  
  }
}


void CHGramp()
{
  if(vcuHVvolts<voltsSetPnt)
  {
    chgPwrSetPnt++;
  }

   if(vcuHVvolts>=voltsSetPnt)
  {
    chgPwrSetPnt--;
  }
  if (chgPwrSetPnt>11000) chgPwrSetPnt=0; //error catch in case of a roll over to 65535
  if (chgPwrSetPnt<0) chgPwrSetPnt=0;
  if (chgPwrSetPnt>10000) chgPwrSetPnt=10000; //max 10 kw
  if (chgPwrSetPnt>maxChgPwr) chgPwrSetPnt=maxChgPwr;
  if (CHGpwror) chgPwrSetPnt=0;
  if (!vcuEn) chgPwrSetPnt=0;
}


void PCS_cksum(uint8_t *data, uint16_t id)
{
  data[7] = 0;
  uint16_t checksum_calc=0;
    for(int b=0; b<8; b++)
    {
        checksum_calc = checksum_calc + data[b];
    }
    checksum_calc += id + (id >> 8);
    checksum_calc &= 0xFF;
   (uint8_t)checksum_calc;
    
  data[7] = checksum_calc;
}
