// gcc main.c -o main

#include <stdio.h>
#include <stdlib.h>
#include "car_lib.h"


// USER DEFINE ___________________________________________________
//----------------------------------------------------------------
/*
#define LIGHT_BEEP		// to test light and beep
#define SPEED_CONTROL		// to test speed control
#define POSITION_CONTROL	// to test postion control
#define SERVO_CONTROL		// to test servo control(steering & camera position)
#define LINE_TRACE		// to test line trace sensor
#define DISTANCE_SENSOR	// to test distance sensor
*/
#define PARAM_PI	3.14159//
#define PARAM_CAR_WIDTH		190 //mm
#define PARAM_CAR_LENGTH	330 //mm
#define PARAM_CAR_HEIGHT	160 //mm
#define PARAM_CAR_DIAMETER	65  //mm		
#define PARAM_CAR_ROTATE	300 //mm
#define PARAM_CAR_STEP	195     //step per 1 rotate
#define PARAM_CAR_DISTANCE_SENSOR_MAX	300//mm
#define PARAM_CAR_DISTANCE_SENSOR_MIN	40//mm

#define TRUE	1
#define FALSE	0

//#define SPEED_CONTROL		// to test speed control
//#define POSITION_CONTROL	// to test postion control

// Initialize Function ___________________________________________
//----------------------------------------------------------------
void UserInit(void);
void UserMove(int direction, int steer, int distance);
void horizon_parking(void);
void virtical_parking(void);
int CheckParkingArea(void);


// MAIN __________________________________________________________
//----------------------------------------------------------------
void main(void)
{
/*
	6--1--2
	 |   |
	 |   |
    5--4--3
*/
	unsigned char status;
	short speed;
	unsigned char gain;
	int position, position_now;
	short angle;
	int channel;
	int data;
	char sensor;
	int i, j;
	char byte = 0x80;
	int obstacle_flag = 0;
	int temp;
	int avg;
	CarControlInit();
	UserInit();
	/*
	   DesireSpeed_Write(0);   //speed = 0;
	   SpeedControlOnOff_Write(UNCONTROL); //wheel off
	*/
	//DesireSpeed_Write(50);   //speed = 0;
	//SpeedControlOnOff_Write(CONTROL); //wheel off



	while(1)
	{
		avg = DistanceSensor(3);
		tempdistance = DesireEncoderCount_Read();
		printf("avg : %d, distance : %d\n",avg, 1500-tempdistance);
		usleep(200);
		
		/*
		temp = CheckParkingArea();
		if(temp>200) 
		{
			horizon_parking();
			printf("----------------------end\n");
			break;
		}
		*/
	}
	sleep(3);
	SteeringServoControl_Write(1460);
	DesireSpeed_Write(0);   //speed = 0;
	SpeedControlOnOff_Write(UNCONTROL); //wheel off
	sleep(1);
}

// User Function _________________________________________________
//----------------------------------------------------------------
void UserInit(void)
{
	SteeringServoControl_Write(1460);   //initialize Servo Center Position

	SpeedPIDProportional_Write(20);     //speed -> P gain : default:10, range:10~50
	SpeedPIDIntegral_Write(20);         //speed -> I gain : default:10, range:10~50
	SpeedPIDDifferential_Write(20);     //speed -> D gain : default:10, range:10~50

	PositionProportionPoint_Write(10);  //Encoder -> P gain : default:10, range:10~50
	DesireSpeed_Write(0);
	EncoderCounter_Write(0);

	PositionControlOnOff_Write(UNCONTROL);  // positionControl function OFF 
	SpeedControlOnOff_Write(UNCONTROL);     // speedControl OFF

	CameraXServoControl_Write(1470);    
	CameraYServoControl_Write(1500);    

	Winker_Write(ALL_OFF);
	usleep(100000);//wait for 100ms
}
void UserMove(int direction, int steer, int distance)//
{
	int temp;
	int state;

	state = SpeedControlOnOff_Read();
	if(state==0)
	{
		DesireSpeed_Write(50);
		SpeedControlOnOff_Write(CONTROL);   // speed Control ON
	}

	state = PositionControlOnOff_Read();
	if(state==0)       
		PositionControlOnOff_Write(CONTROL);    // positionControl function ON 

	state = DesireSpeed_Read();
	if(state!=150)   DesireSpeed_Write(150);

	SteeringServoControl_Write(steer);
	EncoderCounter_Write(0);
	distance=distance*18/10;

	if(direction==1)
	{
		DesireEncoderCount_Write(distance);
		temp = distance;
		while(temp>30)
		{
			temp = DesireEncoderCount_Read();
			printf("temp:%d\n",temp);
			usleep(1000);
		}
	}
	else
	{
		DesireEncoderCount_Write(-distance);
		//temp = DesireEncoderCount_Read();
		temp = -distance;
		while(temp<-30)
		{
			temp = DesireEncoderCount_Read();
			usleep(1000);
		}
	}

	PositionControlOnOff_Write(UNCONTROL);
	DesireSpeed_Write(state);
}
//----------------------------------------------------------------------------
void virtical_parking(void)
{
	int temp;
	int state;
	int front_dis;
	int back_dis;
	int distance = 1100;
	int winker_cnt = 0;
	state = SpeedControlOnOff_Read();
	if(state==0)
	{
		DesireSpeed_Write(50);
		SpeedControlOnOff_Write(CONTROL);   // speed Control ON
	}

	state = PositionControlOnOff_Read();
	if(state==0)       
		PositionControlOnOff_Write(CONTROL);    // positionControl function ON 

	state = DesireSpeed_Read();
	if(state!=50)   DesireSpeed_Write(50);

	EncoderCounter_Write(0);

	distance=1000*18/10;
	front_dis = distance+30;
	back_dis = -distance-30;

	DesireEncoderCount_Write(back_dis);
	temp = back_dis;

	while(temp<-30)
	{
		temp = DesireEncoderCount_Read();
		if(temp<-800)	
		{
			SteeringServoControl_Write(1000);
			Winker_Write(ALL_ON);
		}
		else		
		{
			SteeringServoControl_Write(1460);
		}
		usleep(500);
	}

	DesireEncoderCount_Write(front_dis);
	temp = front_dis;

	while(temp>31)
	{
		temp = DesireEncoderCount_Read();
		if(temp>1450)	SteeringServoControl_Write(1460);
		else			SteeringServoControl_Write(1000);
		usleep(500);
	}

	PositionControlOnOff_Write(UNCONTROL);
	DesireSpeed_Write(state);
}

//----------------------------------------------------------------------------
void horizon_parking(void)
{
	int temp;
	int state;
	int distance = 1100;
	int winker_cnt = 0;

	distance=distance*18/10;
	
	state = SpeedControlOnOff_Read();
	if(state==0)
	{
		DesireSpeed_Write(50);
		SpeedControlOnOff_Write(CONTROL);   // speed Control ON
	}
	state = PositionControlOnOff_Read();
	if(state==0)       
		PositionControlOnOff_Write(CONTROL);    // positionControl function ON 

	state = DesireSpeed_Read();
	if(state!=100)   
		DesireSpeed_Write(100);

	EncoderCounter_Write(0);

	// turn small left
/*
	DesireEncoderCount_Write(350);
	temp = 350;
	while(temp>30)
	{
		temp = DesireEncoderCount_Read();
		usleep(50);
	}
	usleep(100000);
*/
	EncoderCounter_Write(0);
	CarLight_Write(REAR_ON);
	
	// into the parking lot
	DesireEncoderCount_Write(-distance);
	temp = -distance;
	while(temp<-30)
	{
		temp = DesireEncoderCount_Read();
		if(temp<-1100)	SteeringServoControl_Write(1000);
		else			SteeringServoControl_Write(1950);
		usleep(500);
	}
	usleep(100000);

	CarLight_Write(ALL_OFF);

	EncoderCounter_Write(0);

	//out of parking lot
	DesireEncoderCount_Write(distance);
	temp = distance;
	while(temp>30)
	{
		temp = DesireEncoderCount_Read();
		if(temp>1100)	SteeringServoControl_Write(1950);
		else			SteeringServoControl_Write(1000);
		usleep(500);
	}

	EncoderCounter_Write(0);
	// turn small left
/*
	DesireEncoderCount_Write(200);
	temp = 200;
	while(temp>30)
	{
		temp = DesireEncoderCount_Read();
		usleep(50);
	}
*/
	PositionControlOnOff_Write(UNCONTROL);
	DesireSpeed_Write(state);
}
//-------------------------------------------------------------------------
//check parking area
int CheckParkingArea(void)
{
	int state;
	int tempdistance;
	int ParkingAreaDistance;
	static int obstacle_flag = 0;
	static int parkingarea_flag = 0;
	static int avg=0;
	int wink_cnt=0;

	avg = DistanceSensor(3);
	printf("_______data = %d\n",avg);
	if(avg<500){
		parkingarea_flag++;
	}
	else{
		parkingarea_flag=0;
	}

	if(parkingarea_flag>2 && obstacle_flag>50)  //tempdata
	{    
		PositionControlOnOff_Write(CONTROL);    // positionControl function ON 
		state = DesireSpeed_Read();
		if(state!=30)  
			DesireSpeed_Write(30);
		EncoderCounter_Write(0);
		DesireEncoderCount_Write(1500);
		tempdistance = 1500;

		Winker_Write(ALL_ON);
		while(tempdistance>30)
		{
			if(avg>1000)
			{
				ParkingAreaDistance = tempdistance;
				obstacle_flag=0;
				parkingarea_flag = 0;
				Winker_Write(ALL_OFF);
				printf("parking distance : %d\n",ParkingAreaDistance);
				break;
			}
			else
			{
				avg = DistanceSensor(3);
				tempdistance = DesireEncoderCount_Read();
				printf("avg : %d, distance : %d\n",avg, 1500-tempdistance);
				usleep(200);
			}

		}

		PositionControlOnOff_Write(UNCONTROL);
		DesireSpeed_Write(state);
	}

	if(avg>800 && avg<4096)
	{
		obstacle_flag++;
	}

	return ParkingAreaDistance;
}







/*
   int checkParkingArea(void)
   {
   int temp;
   temp = DistanceSensor(6);
   if(temp > PARAM_CAR_WIDTH/2)
   {
   if(IsObstacleArea == TRUE && temp > PARAM_CAR_WIDTH+ObstacleDistance)
   {
   IsParkingArea = TRUE;
   IsObstacleArea = FALSE;
   start_p = EncoderCounter_Read();
   }
   else
   {	
   ObstacleDistance += temp;
   ObstacleDistance/=2;
   IsObstacleArea = TRUE;
   }
   }
   int temp;
   int err = 0;
   ObstacleDistance = 0;
   ParkingSpace = 0;
   static volatile int start_p, end_p;

   if(IsParkingArea == FALSE)
   {
   temp = DistanceSensor(6);
   if(temp > PARAM_CAR_WIDTH/2)
   {
   if(IsObstacleArea == TRUE && temp > PARAM_CAR_WIDTH+ObstacleDistance)
   {
   IsParkingArea = TRUE;
   IsObstacleArea = FALSE;
   start_p = EncoderCounter_Read();
   }
   else
   {	
   ObstacleDistance += temp;
   ObstacleDistance/=2;
   IsObstacleArea = TRUE;
   }
   }
   else
   {
   ObstacleDIstance = 0;
   }

   }
   }
   */
/*
   int Parking(void)
   {
   int temp;
   int err = 0;
   static int ObstacleDistance = 0;
   static int ParkingSpace = 0;
   static volatile int start_p, end_p;

   if(IsParkingArea == FALSE)
   {
   temp = DistanceSensor(6);
   if(temp > PARAM_CAR_WIDTH/2)
   {
   if(IsObstacleArea == TRUE && temp > PARAM_CAR_WIDTH+ObstacleDistance)
   {
   IsParkingArea = TRUE;
   IsObstacleArea = FALSE;
   start_p = EncoderCounter_Read();
   }
   else
   {	
   ObstacleDistance += temp;
   ObstacleDistance/=2;
   IsObstacleArea = TRUE;
   }
   }
   else
   {
   ObstacleDIstance = 0;
   }

   }
   else if(IsParkingArea == TRUE)
   {
   if(temp == ObstacleDistance)
   {
   end_p = Encoder_Counter_Read();
   ParingSpace = (end_p - start_p)*5/10;
//ParkingStart();
}
}
else
{

}
return err;
}
*/

/*
#ifdef LIGHT_BEEP
//0. light and beep Control --------------------------------------------------
printf("\n\n 0. light and beep control\n");
CarLight_Write(ALL_ON);
usleep(1000000);
CarLight_Write(ALL_OFF);

Alarm_Write(ON);
usleep(100000);
Alarm_Write(OFF);

CarLight_Write(FRONT_ON);
usleep(1000000);
CarLight_Write(ALL_OFF);
CarLight_Write(REAR_ON);
usleep(1000000);
CarLight_Write(ALL_OFF);

Alarm_Write(ON);
usleep(100000);
Alarm_Write(OFF);

Winker_Write(ALL_ON);
usleep(1000000);
Winker_Write(ALL_OFF);
Winker_Write(LEFT_ON);
usleep(1000000);
Winker_Write(ALL_OFF);
Winker_Write(RIGHT_ON);
usleep(1000000);
Winker_Write(ALL_OFF);
#endif

#ifdef POSITION_CONTROL
// 1. position control -------------------------------------------------------
printf("\n\n 1. position control\n");

//jobs to be done beforehand;
SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
speed = 50; // speed set     --> speed must be set when using position controller
DesireSpeed_Write(speed);

//control on/off
status = PositionControlOnOff_Read();
printf("PositionControlOnOff_Read() = %d\n", status);
PositionControlOnOff_Write(CONTROL);

//position controller gain set
gain = PositionProportionPoint_Read();    // default value = 10, range : 1~50
printf("PositionProportionPoint_Read() = %d\n", gain);
gain = 20;
PositionProportionPoint_Write(gain);

//position write
position_now = 0;  //initialize
EncoderCounter_Write(position_now);

//position set
position=DesireEncoderCount_Read();
printf("DesireEncoderCount_Read() = %d\n", position);
position = 300;
DesireEncoderCount_Write(position);

position=DesireEncoderCount_Read();
printf("DesireEncoderCount_Read() = %d\n", position);

tol = 10;    // tolerance
while(abs(position_now-position)>tol)
{
position_now=EncoderCounter_Read();
printf("EncoderCounter_Read() = %d\n", position_now);
}
sleep(1);
#endif

#ifdef SPEED_CONTROL
// 2. speed control ----------------------------------------------------------
printf("\n\n 2. speed control\n");

//jobs to be done beforehand;
PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!

//control on/off
status=SpeedControlOnOff_Read();
printf("SpeedControlOnOff_Read() = %d\n", status);
SpeedControlOnOff_Write(CONTROL);

//speed controller gain set
//P-gain
gain = SpeedPIDProportional_Read();        // default value = 10, range : 1~50
printf("SpeedPIDProportional_Read() = %d \n", gain);
gain = 20;
SpeedPIDProportional_Write(gain);

//I-gain
gain = SpeedPIDIntegral_Read();        // default value = 10, range : 1~50
printf("SpeedPIDIntegral_Read() = %d \n", gain);
gain = 20;
SpeedPIDIntegral_Write(gain);

//D-gain
gain = SpeedPIDDifferential_Read();        // default value = 10, range : 1~50
printf("SpeedPIDDefferential_Read() = %d \n", gain);
gain = 20;
SpeedPIDDifferential_Write(gain);

//speed set    
speed = DesireSpeed_Read();
printf("DesireSpeed_Read() = %d \n", speed);
speed = -10;
DesireSpeed_Write(speed);

sleep(2);  //run time 

speed = DesireSpeed_Read();
printf("DesireSpeed_Read() = %d \n", speed);

speeVd = 0;
DesireSpeed_Write(speed);
sleep(1);
#endif

#ifdef SERVO_CONTROL
// 3. servo control ----------------------------------------------------------
printf("\n\n 3. servo control\n");
//steer servo set
angle = SteeringServoControl_Read();
printf("SteeringServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

angle = 1200;
SteeringServoControl_Write(angle);

//camera x servo set
angle = CameraXServoControl_Read();
printf("CameraXServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

angle = 1400;
CameraXServoControl_Write(angle);

//camera y servo set
angle = CameraYServoControl_Read();
printf("CameraYServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

angle = 1400;
CameraYServoControl_Write(angle);    

sleep(1);
angle = 1500;
SteeringServoControl_Write(angle);
CameraXServoControl_Write(angle);
CameraYServoControl_Write(angle); 
#endif  

#ifdef LINE_TRACE
// 4. line trace sensor --------------------------------------------------------
sensor = LineSensor_Read();        // black:1, white:0
printf("LineSensor_Read() = ");
for(i=0; i<8; i++)
{
	if((i % 4) ==0) printf(" ");
	if((sensor & byte)) printf("1");
	else printf("0");
	sensor = sensor << 1;
}
printf("\n");
printf("LineSensor_Read() = %d \n", sensor);
#endif

#ifdef DISTANCE_SENSOR
// 5. distance sensor --------------------------------------------------------
printf("\n\n 4. distance sensor\n");
for(i=0; i<1000; i++)
{
	printf("Please input ADC channel number\n");
	scanf("%d", &channel);
	for(j=0; j<50; j++)
	{
		data = DistanceSensor(channel);
		printf("channel = %d, distance = 0x%04X(%d) \n", channel, data, data);
		usleep(100000);
	}
}
#endif
}
*/

