/*
 * gcc -o stop stop.c
 * set initialzed data
 */
#include <stdio.h>
#include <stdlib.h>
#include "car_lib.h"

// MAIN __________________________________________________________
//----------------------------------------------------------------
void main(void)
{
    CarControlInit();

// servo init
	SteeringServoControl_Write(1478);   //Servo Center Position, default : 1478
	CameraXServoControl_Write(1470);	//default : 1500
	CameraYServoControl_Write(1500);	//default : 1500

// speed init
    SpeedPIDProportional_Write(20);     //speed -> P gain : default:10, range:10~50
    SpeedPIDIntegral_Write(20);         //speed -> I gain : default:10, range:10~50
    SpeedPIDDifferential_Write(20);     //speed -> D gain : default:10, range:10~50
    PositionProportionPoint_Write(10);  //Encoder -> P gain : default:10, range:10~50
    
	DesireSpeed_Write(0);
    EncoderCounter_Write(0);
    
	PositionControlOnOff_Write(UNCONTROL);  // positionControl function OFF 
    SpeedControlOnOff_Write(UNCONTROL);     // speedControl OFF

// etc	
	CarLight_Write(ALL_ON);
	Winker_Write(ALL_OFF);
	Alarm_Write(ON);

// led on/off
	sleep(2);//wait for 200ms
	CarLight_Write(ALL_OFF);
	Alarm_Write(OFF);
    sleep(1);
}
