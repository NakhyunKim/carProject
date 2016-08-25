void OverTaking(int direction)
{
	    if(direction==1) //we need to turn right
        {
                PositionControlOnOff_Write(CONTROL); // Again using position control
                Winker_Write(LEFT_ON); //Right winker
                SteeringServoControl_Write(1000); //Right
                DesireSpeed_Write(30); 
                position_now = 0; 
                EncoderCounter_Write(position_now);//initialize
                position = 390*12;
                DesireEncoderCount_Write(position);
             
                printf("%d\n", position);
                printf("%d\n", position_now);

                tol = 5;    // tolerance
                while(abs(position_now-position)>tol)
                {
                    position_now=EncoderCounter_Read();
                    //printf("EncoderCounter_Read() = %d\n", position_now);
                    if(position_now>position)   break;
                }
                sleep(1);

                SteeringServoControl_Write(2000); //left
                position_now = 0;
                EncoderCounter_Write(position_now);
                position = 390*14;
                DesireEncoderCount_Write(position);
                tol = 5;    // tolerance
                while(abs(position_now-position)>tol)
                {
                    position_now=EncoderCounter_Read();
                    //printf("EncoderCounter_Read() = %d\n", position_now);
                   if(position_now>position)    break;
                }
                sleep(1);
                Winker_Write(ALL_OFF);

                PositionControlOnOff_Write(UNCONTROL); // At first position controller must be OFF !!!
                angle = 1500;
                SteeringServoControl_Write(angle); //straight
                DesireSpeed_Write(30);
                sleep(1);	

						 for(k=0; k<1000; k++)
						 {
						 for(j=0; j<50; j++)
						 {
							data = DistanceSensor(3); //channel num3
							printf("channel = %d, distance = 0x%04X(%d) \n", channel, data, data);
							if(data<700) //If distance happen
							{
								flag=1; //if object is detected in num 1 sensor
							}
						 }
							if(flag==1)
							{
							DesireSpeed_Write(0);
							sleep(1); //stop during 1 sec      
							break;
							}
						  }

                PositionControlOnOff_Write(CONTROL); // Again using position control
                Winker_Write(RIGHT_ON); //left winker
         		  SteeringServoControl_Write(2000); //left
                position_now = 0;
                EncoderCounter_Write(position_now);
                position = 390*14;
                DesireEncoderCount_Write(position);
                tol = 5;    // tolerance
                while(abs(position_now-position)>tol)
                {
                    position_now=EncoderCounter_Read();
                    //printf("EncoderCounter_Read() = %d\n", position_now);
                   if(position_now>position)    break;
                }
                sleep(1);

                SteeringServoControl_Write(1000); //Right
                DesireSpeed_Write(30); 
                position_now = 0; 
                EncoderCounter_Write(position_now);//initialize
                position = 390*12;
                DesireEncoderCount_Write(position);
             
                printf("%d\n", position);
                printf("%d\n", position_now);

                tol = 5;    // tolerance
                while(abs(position_now-position)>tol)
                {
                    position_now=EncoderCounter_Read();
                    //printf("EncoderCounter_Read() = %d\n", position_now);
                    if(position_now>position)   break;
                }
                sleep(1);	
                Winker_Write(ALL_OFF);

                PositionControlOnOff_Write(UNCONTROL); // At first position controller must be OFF !!!
                angle = 1460;
                SteeringServoControl_Write(angle); //straight
                DesireSpeed_Write(30);
                sleep(1);
				
        				for(q=0; q<500; q++)
        	      {
        	        sensor = LineSensor_Read();
        					if(sensor==1  ) 
        					{
        						DesireSpeed_Write(0); //정지 
        					}
        				}
				
        				/*
        				if( 밑에 흰색 선이 검출되면 )
        				{
        					정지 //이 말은 즉슨 앞에 장애물이 없다는 의미.
        				}
        				*/

        }
        else //we need to turn left
        {
                PositionControlOnOff_Write(CONTROL); // Again using position control
                Winker_Write(RIGHT_ON); //left winker
	            SteeringServoControl_Write(2000); //left
	            DesireSpeed_Write(30); 
	            position_now = 0; 
	            EncoderCounter_Write(position_now);//initialize
	            position = 390*12;
	            DesireEncoderCount_Write(position);
	             
	            printf("%d\n", position);
	            printf("%d\n", position_now);

	            tol = 5;    // tolerance
	            while(abs(position_now-position)>tol)
	            {
	                position_now=EncoderCounter_Read();
	                //printf("EncoderCounter_Read() = %d\n", position_now);
	                if(position_now>position)   break;
	            }
	            sleep(1);

	            SteeringServoControl_Write(1000); //right
	            position_now = 0;
	            EncoderCounter_Write(position_now);
	            position = 390*14;
	            DesireEncoderCount_Write(position);
	            tol = 5;    // tolerance
	            while(abs(position_now-position)>tol)
	            {
	                position_now=EncoderCounter_Read();
	                //printf("EncoderCounter_Read() = %d\n", position_now);
	                if(position_now>position)    break;
	            }
	            sleep(1);
	            Winker_Write(ALL_OFF);

                PositionControlOnOff_Write(UNCONTROL); // At first position controller must be OFF !!!
                angle = 1500;
                SteeringServoControl_Write(angle); //straight
                DesireSpeed_Write(30);
                sleep(1);	
                
							for(k=0; k<1000; k++)
							{
							for(j=0; j<50; j++)
							{
								data = DistanceSensor(2); //channel num2
								printf("channel = %d, distance = 0x%04X(%d) \n", channel, data, data);
								if(data<700) //If distance happen
								{
									flag=1; //if object is detected in num 1 sensor
								}
							}
								if(flag==1)
								{
								DesireSpeed_Write(0);
								sleep(1); //stop during 1 sec      
								break;
								}
							 }

                PositionControlOnOff_Write(CONTROL); // Again using position control
                Winker_Write(LEFT_ON); //right winker
                SteeringServoControl_Write(1000); //right
                position_now = 0;
                EncoderCounter_Write(position_now);
                position = 390*14;
                DesireEncoderCount_Write(position);
                tol = 5;    // tolerance
                while(abs(position_now-position)>tol)
                {
                    position_now=EncoderCounter_Read();
                    //printf("EncoderCounter_Read() = %d\n", position_now);
                   if(position_now>position)    break;
                }
                sleep(1);

            	SteeringServoControl_Write(2000); //left
                DesireSpeed_Write(30); 
                position_now = 0; 
                EncoderCounter_Write(position_now);//initialize
                position = 390*12;
                DesireEncoderCount_Write(position);
             
                printf("%d\n", position);
                printf("%d\n", position_now);

                tol = 5;    // tolerance
                while(abs(position_now-position)>tol)
                {
                    position_now=EncoderCounter_Read();
                    //printf("EncoderCounter_Read() = %d\n", position_now);
                    if(position_now>position)   break;
                }
                sleep(1);
                Winker_Write(ALL_OFF);

                PositionControlOnOff_Write(UNCONTROL); // At first position controller must be OFF !!!
                angle = 1500;
                SteeringServoControl_Write(angle); //straight
                DesireSpeed_Write(30);
                sleep(1);
				
        				for(q=0; q<500; q++)
        	      {
        					sensor = LineSensor_Read();
        					if(sensor==1  ) //
        					{
        						DesireSpeed_Write(0); //정지 
        					}
                }
          				/*
          				if( 밑에 흰색 선이 검출되면 )
          				{
          					정지 //이 말은 즉슨 앞에 장애물이 없다는 의미.
          				}
          				*/
        }	



}
