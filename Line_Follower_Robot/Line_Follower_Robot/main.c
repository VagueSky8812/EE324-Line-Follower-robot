
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"

//global variables and functions
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char l = 0;
unsigned char c = 0;
unsigned char r = 0;
int l_norm = 0;
int c_norm = 0;
int r_norm = 0;
int l_max = 0;
int c_max = 0;
int r_max = 0;
float angle_error_norm = 0;
float angle_error_guess = 0;
float angle_error_prev = 0;
unsigned char PortBRestore = 0;
float relative_error_l, relative_error_r;
float final_angle_error;
float actuation_CM;
float actuation_DM;
float motor_l;
float motor_r;
float kp = 0;
float ki =0;
float kd = 0;
float angle_error_diff;
float angle_error_sum;
float mod = 0;
unsigned char direction_command=0b00000000; 

void motion_pin_config (void)
{
 //setup of	PORTB0, PORTB1, PORTB2, PORTB3
 DDRB = DDRB | 0b00001111;   //set direction of the PORTB3 to PORTB0 pins as OUTPUT
 //initialisation of PORTB0, PORTB1, PORTB2, PORTB3 to 0
 PORTB = PORTB & 0b11110000; // set initial value of the PORTB3 to PORTB0 pins to logic 0
 
 //setup of PD4 and PD5
 DDRD = DDRD | 0b00110000;   //Setting PD4 and PD5 pins as OUTPUT for PWM generation
 //initialisation
 PORTD = PORTD | 0b00110000; //PD4 and PD5 pins are for VELOCITY CONTROL using PWM
}

//Function used to set the motor's direction
void motion_set (unsigned char DirectionCommand)
{
 unsigned char PortBRestore = 0;

 DirectionCommand &= 0b00001111; 		// removing upper nibble from Direction_Command as it is not needed
 PortBRestore = PORTB; 			// reading the PORTB's original status
 PortBRestore &= 0b11110000; 			// setting lower direction nibble to 0; resetting PB0,1,2,3 to '0' momentarilty
 PortBRestore |= DirectionCommand; 	// adding lower nibble for direction command and restoring the PORTB status
 PORTB = PortBRestore; 			// setting the command to the port
}

void forward (void)         //both wheels forward
{
  motion_set(0b00000110);
}

void back (void)            //both wheels backward
{
  motion_set(0b00001001);
}

void left (void)            //Left wheel backward, Right wheel forward
{
  motion_set(0b00000101);
}

void right (void)           //Left wheel forward, Right wheel backward
{   
  motion_set(0b00001010);
}

void soft_left (void)       //Left wheel stationary, Right wheel forward
{
 motion_set(0b00000100);
}

void soft_right (void)      //Left wheel forward, Right wheel is stationary
{ 
 motion_set(0b00000010);
}

void soft_left_2 (void)     //Left wheel backward, right wheel stationary
{
 motion_set(0b00000001);
}

void soft_right_2 (void)    //Left wheel stationary, Right wheel backward
{
 motion_set(0b00001000);
}

void hard_stop (void)       //hard stop(stop suddenly)
{
  motion_set(0b00000000);
}

void soft_stop (void)       //soft stop(stops slowly)
{
  motion_set(0b00000000);
}

//Function to Initialize ADC
void adc_init()
{
 ADCSRA = 0x00;
 ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}


void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 adc_init();
 sei(); //Enables the global interrupts
}


//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRA = 0x00;   //set PORTF direction as input
 PORTA = 0x00;  //set PORTF pins floating
}

//Function to Initialize PORTS
void port_init()
{
 lcd_port_config();
 adc_pin_config();
 motion_pin_config();
}

//TIMER1 initialize - prescale:64
// WGM: 5) PWM 8bit fast, TOP=0x00FF
// desired value: 450Hz
// actual value: 450.000Hz (0.0%)
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFF; //setup
 TCNT1L = 0x01;
 OCR1AH = 0x00;
 OCR1AL = 0xFF;//left motor pwm
 OCR1BH = 0x00;
 OCR1BL = 0xFF;//right motor pwm
 ICR1H  = 0x00;
 ICR1L  = 0xFF;
 TCCR1A = 0xA1;
 TCCR1B = 0x0D; //start Timer
}

//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
 unsigned char a;
 Ch = Ch & 0x07;
 ADMUX= 0x20| Ch;
 ADCSRA = ADCSRA | 0x40;	//Set start conversion bit
 while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10;      //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 return a;
}

//Main Function
int main(void)
{
 //initialisatons
 init_devices();
 timer1_init();

 lcd_set_4bit();
 lcd_init();
/*
while(1)
{
	l=ADC_Conversion(3);
	c=ADC_Conversion(4);
	r=ADC_Conversion(5);
	lcd_print(1, 1, l, 3);
	lcd_print(1, 5, c, 3);
	lcd_print(1, 9, r, 3);
	_delay_ms(300);
}

*/
while(1)
{
	//sensing part
	l=ADC_Conversion(3);
	c=ADC_Conversion(4);
	r=ADC_Conversion(5);
	lcd_print(1, 1, l, 3);
	lcd_print(1, 5, c, 3);
	lcd_print(1, 9, r, 3);
	//********************************************************************************************************************
	//Angle error Guess part
	l_max = 90;
	c_max = 96;
	r_max = 105;
	l_norm = (l-6)*100/l_max + 0.0001;//varies from 0 to 100
	c_norm = (c-6)*100/c_max + 0.0001;//varies from 0 to 100
	r_norm = (r-6)*100/r_max + 0.0001;//varies from 0 to 100
	
	if ((l_norm>=2) && (r_norm>=2)){//case of a crossing or 2 nearby tracks
		angle_error_norm = 0;//in this case move the bot forward to get out of dilemma
		angle_error_prev = 0;
	}
	else if ((c_norm<=2.9) && (r_norm>=0.1)){//turned too left from track
		angle_error_norm = 200-r_norm;
	}
	else if ((c_norm>=2) && (r_norm>=l_norm)){//turned little left from track
		angle_error_norm = 100-c_norm;
	}
	else if ((c_norm>=2) && (l_norm>=r_norm)){//turned little right from track
		angle_error_norm = c_norm-100;
	}
	else if ((c_norm<=2) && (l_norm>=0.1)){//turned too right from track
		angle_error_norm = l_norm-200;
	}
	else{
		angle_error_norm = angle_error_prev*5;
	}
	
	angle_error_guess = angle_error_norm/5.0;//Here we make a Rough Guess of The Angle
	//**************************************************************************************************************************
	/*relative_error_l = (l_norm-c_norm)*200/(l_norm+c_norm);//varies from 0 to 100
	relative_error_r = (r_norm-c_norm)*200/(r_norm+c_norm);//varies from 0 to 100
	final_angle_error = (relative_error_l-relative_error_r)*180/200;*/
	
	//control part 
	kp = 12.0;
	ki = 0.1;
	kd = 1;
	angle_error_sum = angle_error_sum + angle_error_guess;
	angle_error_diff = angle_error_guess - angle_error_prev;
	actuation_DM = (kp*angle_error_guess) + ki*angle_error_sum + kd*(angle_error_diff);//final_angle_error;	//floors off
	
	
	//forward(); back(); left(); right(); soft_left(); soft_right(); soft_left_2(); soft_right_2();
	//hard_stop(); soft_stop();
	//***********************************************************************************************************************************
	//actuation part
	//_delay_ms(100);
	//speed part
	
	//absolute value of actuation_DM
	if(actuation_DM<0){mod = -actuation_DM;}
	else {mod = actuation_DM;}
	
	//Common Mode Speed Part
	actuation_CM = 255 - mod/2.0;//speed of centre of mass of robot; lower speed for sharper turns
	motor_l = actuation_CM + actuation_DM/2.0;
	motor_r = actuation_CM - actuation_DM/2.0;
	
	//clipping
	if(motor_l>255){motor_l=255;}
	if(motor_l<-255){motor_l=-255;}
	if(motor_r>255){motor_r=255;}
	if(motor_r<-255){motor_r=-255;}
	//*******************************************************************************************************************************************
	//direction part
	if(motor_l<0)
	{
		left();
	}
	else if(motor_r<0)
	{
		right();
	}
	else{
		forward();
	}
	
	//absolute values of motor_l and motor_r
	if(motor_l<0){motor_l = -motor_l;}
	if(motor_r<0){motor_r = -motor_r;}
	OCR1AL = floor(motor_l);//left motor; adjust the sign
	OCR1BL = floor(motor_r);//right motor; adjust the complement sign
	//*****************************************************************************************************************************************
	//preparation for next step
	angle_error_prev = angle_error_guess;
}

}
