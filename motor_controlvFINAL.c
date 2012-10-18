//Programmers: Brandon Stuart
//             Bojan Stefanovic
//             Jarid Warren
//Last Updated: March 29, 2012

#include "C:\Users\Bojan\Desktop\Applied Sciences 1299\FLYNN\sumovore.h"
#include "C:\Users\Bojan\Desktop\Applied Sciences 1299\FLYNN\motor_control.h"
#include "C:\Users\Bojan\Desktop\Applied Sciences 1299\FLYNN\interrupts.h"
#include <delays.h>
#include <timers.h>


//FUNCTIONS!

void spin_left(void);
void turn_left(void);
void straight_fwd(void);
void turn_right(void);
void spin_right(void);
void reverse_straight(void);
void center_right(void);
void center_left(void);
void stop_all(void);
void sharp_turn_right(void);
void sharp_turn_left(void);
void right_turn_right(void);
void right_turn_left(void);
void obtuse_turn_left(void);
void obtuse_turn_right(void);
void obtuse_right(void);
void obtuse_left(void);

void motor_control(void)		
{

	OpenTimer0(TIMER_INT_OFF & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256); //Open timer 1, 16 bit (max number: 65535), and prescalar 256. Max time interval: 2.09712 seconds
	INTCONbits.TMR0IF = 0; //Clear flag to get timer ready for use
	switch(SeeLine.B)
		{
		case(0b00100u):	//Only middle sensor
     		straight_fwd();
			check_sensors();
		break;
	
		case(0b10000u):	//Only far left sensor
			spin_left();
			check_sensors();
		break;
	
		case(0b01000u):	 //Only mid-left sensor
     		center_left();
			check_sensors();
		break;
	
		case(0b00001u):	//Only far right sensor
    		turn_right();
			check_sensors();
		break;
	
		case(0b00010u):	//Only mid-right sensor
   			center_right();
			check_sensors();
		break;

		case(0b00011u):	//Indicates a sharp corner if the robot enters slightly not straight		
   			right_turn_right();
			check_sensors();
		break;
	
		case(0b11000u):	//Indicates a sharp corner if the robot enters slightly not straight	
   			right_turn_left();
			check_sensors();
		break;

		case(0b01010u):	//Odd case, very rarely occurs, but included to ensure robot does not stop 
   			straight_fwd();
			check_sensors();
		break;

		case(0b00000u):	//When the robot reads nothing, could mean a gap, dead end, or slight deviation from the track
			WriteTimer0(50000);
			while(SeeLine.B == 0b00000u && !INTCONbits.TMR0IF) //While still reading nothing and timer hasn't expired
				{
					straight_fwd();
					check_sensors();
				}
			if(SeeLine.B == 0b00100u) //If the middle sensor is read, the robot has crossed a gap and should continue normally
				{
					INTCONbits.TMR0IF = 0;
					straight_fwd();
					check_sensors();
				}
			else if(SeeLine.B == 0b00111u) //Robot has hit a right turn after a gap
				{
					INTCONbits.TMR0IF = 0;
					right_turn_right();
					check_sensors();
				}
			else if(SeeLine.B == 0b11100u) //Robot has hit a left turn after a gap
				{
					INTCONbits.TMR0IF = 0;
					sharp_turn_left();
					check_sensors();
				}
			else if(SeeLine.B == 0b10000u) //Robot has hit a left turn after a gap
				{
					INTCONbits.TMR0IF = 0;
					while(SeeLine.B != 0b00000u && SeeLine.B != 0b01000u && SeeLine.B != 0b01100u && SeeLine.B != 0b11000u)
						{
							straight_fwd();
							check_sensors();
						}
					if(SeeLine.B == 0b00000u || SeeLine.B == 0b11000u)
						{
							while(SeeLine.B != 0b00100u)
							{
								spin_left();
								check_sensors();
							}
						}
					else if(SeeLine.B == 0b01000u)
						{
							while(SeeLine.B != 0b00100u)
								{
									turn_right();
									check_sensors();
								}
						}
					else if(SeeLine.B == 0b01100u)
						{
							while(SeeLine.B != 0b00100u)
								{
									turn_right();
									check_sensors();
								}
						}	
				}
			else if(SeeLine.B == 0b00001u) //Robot has hit a right turn after a gap
				{
					INTCONbits.TMR0IF = 0;
					while(SeeLine.B != 0b00000u && SeeLine.B != 0b00010u && SeeLine.B != 0b00110u && SeeLine.B != 0b00011u)
						{
							straight_fwd();
							check_sensors();
						}
					if(SeeLine.B == 0b00000u || SeeLine.B == 0b00011u)
						{
							while(SeeLine.B != 0b00100u)
							{
								spin_right();
								check_sensors();
							}
						}
					else if(SeeLine.B == 0b00010u)
						{
							while(SeeLine.B != 0b00100u)
								{
									turn_left();
									check_sensors();
								}
						}
					else if(SeeLine.B == 0b00110u)
						{
							while(SeeLine.B != 0b00100u)
								{
									turn_left();
									check_sensors();
								}
						}	
				}						
			else if(INTCONbits.TMR0IF) //Timer has expired, robot has reached a dead end, must reverse back onto track
				{
					INTCONbits.TMR0IF = 0;
					WriteTimer0(45000);
					while(SeeLine.B != 0b00100u && !INTCONbits.TMR0IF)
						{
							reverse_straight();
							check_sensors();
						}
					spin_right();
					while(SeeLine.B != 0b00100u)
						{
							spin_right();
							check_sensors();
						}		
				}
			else if(SeeLine.B == 0b00011u) //Right turn
				{
					INTCONbits.TMR0IF = 0;
					while(SeeLine.B != 0b00100u)
						{
							spin_right();	
							check_sensors();
						}
				}
			else if(SeeLine.B == 0b11000u) //Left turn
				{	
					INTCONbits.TMR0IF = 0;
					while(SeeLine.B != 0b00100u)
						{
							spin_left();
							check_sensors();
						}
				}
			check_sensors();
		break;

		case(0b00111u): // 90deg turn right
			WriteTimer0(47500);	
			right_turn_right();
		break;

		case(0b01111u): // 90deg turn right
			WriteTimer0(47500);	
			right_turn_right();
		break;

		case(0b11100u): // 90deg turn left
			WriteTimer0(47500);	
			right_turn_left();
		break;

		case(0b11110u): // 90deg turn left
			WriteTimer0(47500);	
			right_turn_left();
		break;

		case(0b00101u):	//acute right turn
			WriteTimer0(47500);	
			sharp_turn_right();
		break;
		
		case(0b10100u):	//acute left turn
			WriteTimer0(47500);	
			sharp_turn_left();
		break;
		
		case(0b01001u): //acute right turn
			WriteTimer0(40000);	
			sharp_turn_right();
		break;
		
		case(0b10010u): //Acute left turn
			WriteTimer0(40000);	
			sharp_turn_left();
		break;

		case(0b01100u): //Obtuse left turn
			obtuse_turn_left();		
		break;

		case(0b00110u): //Obtuse right turn
			obtuse_turn_right();		
		break;

	}
}


void spin_left(void) //Sets robot wheels to spin at the same speed in opposite directions, allowing the robot to spin on the spot in the left direction
{
  set_motor_speed(left, rev_medium, 0); 
  set_motor_speed(right, medium, 0); 
}

void turn_left(void) //Sets right wheel to full and left to off, so the robot pivots to the left
{
  set_motor_speed(left, stop, 0); 
  set_motor_speed(right, medium, 0); 
}
void straight_fwd(void) //Robot continues going straight
{
  set_motor_speed(left, medium, 0); 
  set_motor_speed(right, medium, 25); //to even out wheels, fast is 35 PS and medium is around 30.
}
void spin_right(void) //Sets robot wheels to spin at the same speed in opposite directions, allowing the robot to spin on the spot in the right direction
{
  set_motor_speed(left, medium, 0); 
  set_motor_speed(right, rev_medium, 0); 
}
void turn_right(void)//Sets left wheel to full and right to off, so the robot pivots to the right
{
  set_motor_speed(left, medium, 0); 
  set_motor_speed(right, stop, 0); 
}
void center_right(void) //Centers robot if it goes slightly off course to the left
{
  set_motor_speed(left, medium, 100);
  set_motor_speed(right, slow, 0);
}
void center_left(void) //Centers robot if it goes slightly off course to the right
{
  set_motor_speed(left, slow, 0);
  set_motor_speed(right, medium, 100);
}
void obtuse_right(void) //Robot wheels set to perform an obtuse turn right
{
  set_motor_speed(left, medium, 0);
  set_motor_speed(right, rev_slow, 0);
}
void obtuse_left(void) //Robot wheels set to perform an obtuse turn left
{
  set_motor_speed(left, rev_slow, 0);
  set_motor_speed(right, medium, 0);
}
void reverse_straight(void) //Reverse
{
	set_motor_speed(left, rev_medium, 30);
	set_motor_speed(right, rev_medium, 0);
}
void stop_all(void) //All wheels stop spinning
{
	set_motor_speed(left, stop, 0);
	set_motor_speed(right, stop, 0);
}

void sharp_turn_right(void) 
{		
	WriteTimer0(45000);			
	while(SeeLine.B != 0b00000u && !INTCONbits.TMR0IF && SeeLine.B != 0b11111u && SeeLine.B != 0b01000u) //While the sensors don't read any of the previous settings, the robot goes straight forward
		{
			straight_fwd();
	   		check_sensors();
		}

	if(SeeLine.B == 0b00000u) //If the robot reads nothing, it has gone over the turn, indicating that it is a right angled corner
	    {
			INTCONbits.TMR0IF = 0;
			WriteTimer0(65500);
			while(!INTCONbits.TMR0IF)
				{
					straight_fwd();
					check_sensors();
				}
			while(SeeLine.B != 0b00100u) //Spin right until only the middle sensor is read and the robot is then centred on the line
		     	{
			     	spin_right();			
			     	check_sensors();
				}						
			INTCONbits.TMR0IF = 0;
			check_sensors();
		}
	else if(INTCONbits.TMR0IF)
		{
			INTCONbits.TMR0IF = 0;
			straight_fwd();
			check_sensors();
		}
	else if (SeeLine.B == 0b11111u) //Indicates end of track or crossover
		{
			INTCONbits.TMR0IF = 0;
			WriteTimer0(59000);
			while(!INTCONbits.TMR0IF && SeeLine.B == 0b11111u)
				{
					straight_fwd();
					check_sensors();
				}
			if(INTCONbits.TMR0IF) //End of track scenario, all wheels stop
				{
					Delay10KTCYx(125);
					while(SeeLine.B == 0b11111u)
						{
							motors_brake_all();
							INTCONbits.TMR0IF = 0;
							check_sensors();
						}
				}
			else if(SeeLine.B != 0b11111u) //Crossover, robot continues forward
				{
					straight_fwd();
					INTCONbits.TMR0IF = 0;
				}
		}		
	check_sensors();
}
void sharp_turn_left(void)
{		
	WriteTimer0(45000);			
	while(SeeLine.B != 0b00000u && !INTCONbits.TMR0IF && SeeLine.B != 0b11111u && SeeLine.B != 0b00010u) //While the sensors don't read any of the previous settings, the robot goes straight forward
		{
			straight_fwd();
	   		check_sensors();
		}

	if(SeeLine.B == 0b00000u) //If the robot reads nothing, it has gone over the turn, indicating that it is a right angled corner
	    {
			INTCONbits.TMR0IF = 0;
			WriteTimer0(65500);
			while(!INTCONbits.TMR0IF)
				{
					straight_fwd();
					check_sensors();
				}
			while(SeeLine.B != 0b00100u) //Spin left until only the middle sensor is read and the robot is then centred on the line
		     	{
			     	spin_left();			
			     	check_sensors();
				}						
			INTCONbits.TMR0IF = 0;
			check_sensors();
		}
	else if(INTCONbits.TMR0IF)
		{
			INTCONbits.TMR0IF = 0;
			straight_fwd();
			check_sensors();
		}
	else if (SeeLine.B == 0b11111u) //Indicates end of track or crossover
		{
			INTCONbits.TMR0IF = 0;
			WriteTimer0(59000);
			while(!INTCONbits.TMR0IF && SeeLine.B == 0b11111u)
				{
					straight_fwd();
					check_sensors();
				}
			if(INTCONbits.TMR0IF) //End of track scenario, all wheels stop
				{
					Delay10KTCYx(125);
					while(SeeLine.B == 0b11111u) 
						{
							motors_brake_all();
							INTCONbits.TMR0IF = 0;
							check_sensors();
						}
				}
			else if(SeeLine.B != 0b11111u) //Crossover, robot continues forward
				{
					straight_fwd();
					INTCONbits.TMR0IF = 0;
				}
		}
	check_sensors();
}
void right_turn_right(void) 
{		
	WriteTimer0(45000);			
	while(SeeLine.B != 0b00000u && !INTCONbits.TMR0IF && SeeLine.B != 0b11111u) //While robot doesn't read these settings, drive forward
		{
			straight_fwd();
	   		check_sensors();
		}

	if(SeeLine.B == 0b00000u) //Reads nothing, robot is at a right angled turn
	    {
			INTCONbits.TMR0IF = 0;
			WriteTimer0(63000);
			while(!INTCONbits.TMR0IF)
				{
					check_sensors();
				}
			while(SeeLine.B != 0b00100u)
		     	{
			     	spin_right();
			     	check_sensors();
			    }
			INTCONbits.TMR0IF = 0;	
			check_sensors();
		}
	else if(INTCONbits.TMR0IF)
		{
			INTCONbits.TMR0IF = 0;
			straight_fwd();
			check_sensors();
		}
	else if (SeeLine.B == 0b11111u) //Indicates end of track or crossover
		{
			INTCONbits.TMR0IF = 0;
			WriteTimer0(59000);
			while(!INTCONbits.TMR0IF && SeeLine.B == 0b11111u)
				{
					straight_fwd();
					check_sensors();
				}
			if(INTCONbits.TMR0IF) //end of track
				{
					Delay10KTCYx(125);
					while(SeeLine.B == 0b11111u)
						{
							motors_brake_all();
							INTCONbits.TMR0IF = 0;
							check_sensors();
						}
				}
			else if(SeeLine.B != 0b11111u) //crossover
				{
					straight_fwd();
					INTCONbits.TMR0IF = 0;
				}
		}
	
					  	
	check_sensors();
}
void right_turn_left(void)
{		
	WriteTimer0(45000);			
	while(SeeLine.B != 0b00000u && !INTCONbits.TMR0IF && SeeLine.B != 0b11111u) //While robot doesn't read these settings, drive forward
		{
			straight_fwd();
	   		check_sensors();
		}

	if(SeeLine.B == 0b00000u) //Reads nothing, robot is at a right angled turn
	    {
			INTCONbits.TMR0IF = 0;
			WriteTimer0(63000);
			while(!INTCONbits.TMR0IF)
				{
					check_sensors();
				}
			while(SeeLine.B != 0b00100u)
		     	{
			     	spin_left();
			     	check_sensors();
			    }
			INTCONbits.TMR0IF = 0;	
			check_sensors();
		}
	else if(INTCONbits.TMR0IF)
		{
			INTCONbits.TMR0IF = 0;
			straight_fwd();
			check_sensors();
		}
	else if (SeeLine.B == 0b11111u) //Indicates end of track or crossover
		{
			INTCONbits.TMR0IF = 0;
			WriteTimer0(59000);
			while(!INTCONbits.TMR0IF && SeeLine.B == 0b11111u)
				{
					straight_fwd();
					check_sensors();
				}
			if(INTCONbits.TMR0IF) //End of track
				{
					Delay10KTCYx(125);
					while(SeeLine.B == 0b11111u)
						{
							motors_brake_all();
							INTCONbits.TMR0IF = 0;
							check_sensors();
						}
				}
			else if(SeeLine.B != 0b11111u) //Crossover
				{
					straight_fwd();
					INTCONbits.TMR0IF = 0;
				}
		}
	
					  	
	check_sensors();
}
void obtuse_turn_right()
{
	WriteTimer0(45000);
	while(SeeLine.B != 0b00100u && SeeLine.B != 0b11110u && SeeLine.B != 0b01111u && SeeLine.B != 0b00111u && SeeLine.B != 0b00101u && SeeLine.B != 0b11111u && !INTCONbits.TMR0IF) //While the robot doesn't read these settings, it slowly turns to the right
		{
			obtuse_right();
			check_sensors();
		}
	if(SeeLine.B == 0b00010u) //Keeps turning right
		{
			INTCONbits.TMR0IF = 0;
			obtuse_right();
			check_sensors();
		}
	if(SeeLine.B == 0b00100u) //Robot is centred on track, goes straight
		{
			INTCONbits.TMR0IF = 0;
			straight_fwd();
			check_sensors();
		}
	//if(SeeLine.B == 0b11110u);
	//	{
			//while(SeeLine.B != 0b00100u)
			//	{
			//		right_turn_left();
			//		check_sensors();
			//	}
			//check_sensors();
	//	}
	else if (SeeLine.B == 0b01111u) //Robot has entered a sharp turn at an angle, performs sharp right turn
		{
			INTCONbits.TMR0IF = 0;
			right_turn_right();
			check_sensors();
		}
	else if (SeeLine.B == 0b00111u) //Robot has entered a sharp turn at an angle, performs sharp right turn
		{
			INTCONbits.TMR0IF = 0;
			right_turn_right();
			check_sensors();
		}
	else if (SeeLine.B == 0b00101)
		{
			INTCONbits.TMR0IF = 0;
			straight_fwd();
			check_sensors();
		}
	else if (SeeLine.B == 0b11111u) //End of track or crossover
		{
			INTCONbits.TMR0IF = 0;
			WriteTimer0(59000);
			while(!INTCONbits.TMR0IF && SeeLine.B == 0b11111u)
				{
					straight_fwd();
					check_sensors();
				}
			if(INTCONbits.TMR0IF) //End of track
				{
					Delay10KTCYx(125);
					while(SeeLine.B == 0b11111u)
						{
							motors_brake_all();
							INTCONbits.TMR0IF = 0;
							check_sensors();
						}
				}
			else if(SeeLine.B != 0b11111u) //Crossover
				{
					straight_fwd();
					INTCONbits.TMR0IF = 0;
				}
		}
	else if(INTCONbits.TMR0IF)
		{
			INTCONbits.TMR0IF = 0;
			straight_fwd();
			check_sensors();
		}
	check_sensors();
}
void obtuse_turn_left()
{
	while(SeeLine.B != 0b00100u && SeeLine.B != 0b01111u && SeeLine.B != 0b11110u && SeeLine.B != 0b11100u && SeeLine.B != 0b10100u && SeeLine.B !=0b11111u) //While the robot doesn't read these settings, it slowly turns to the left
		{
			obtuse_left();
			check_sensors();
		}
	if(SeeLine.B == 0b01000u) //Keeps turning left
		{
			obtuse_left();
			check_sensors();
		}
	else if(SeeLine.B == 0b00100u) //Robot is centred on track, goes straight
		{
			INTCONbits.TMR0IF = 0;
			straight_fwd();
			check_sensors();
		}
	//if(SeeLine.B == 0b01111u);
	//	{
	//		while(SeeLine.B != 0b00100u)
	//			{
	//				right_turn_right();
	//				check_sensors();
	//			}
	//		check_sensors();
	//	}
	else if (SeeLine.B == 0b11110u) //Robot has entered a sharp turn at an angle, performs sharp left turn
		{
			INTCONbits.TMR0IF = 0;
			right_turn_left();
			check_sensors();
		}
	else if (SeeLine.B == 0b11100u) //Robot has entered a sharp turn at an angle, performs sharp left turn
		{
			INTCONbits.TMR0IF = 0;
			right_turn_left();
			check_sensors();
		}
	else if (SeeLine.B == 0b10100)
		{
			INTCONbits.TMR0IF = 0;
			straight_fwd();
			check_sensors();
		}
	else if (SeeLine.B == 0b11111u) //Indeicates end of track or crossover
		{
			INTCONbits.TMR0IF = 0;
			WriteTimer0(59000);
			while(!INTCONbits.TMR0IF && SeeLine.B == 0b11111u)
				{
					straight_fwd();
					check_sensors();
				}
			if(INTCONbits.TMR0IF) //End of track
				{
					Delay10KTCYx(125);
					while(SeeLine.B == 0b11111u)
						{
							motors_brake_all();
							INTCONbits.TMR0IF = 0;
							check_sensors();
						}
				}
			else if(SeeLine.B != 0b11111u) //Crossover
				{
					straight_fwd();
					INTCONbits.TMR0IF = 0;
				}
		}
	else if(INTCONbits.TMR0IF)
		{
			INTCONbits.TMR0IF = 0;
			straight_fwd();
			check_sensors();
		}
	check_sensors();
}