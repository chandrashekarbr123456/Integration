#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <LiquidCrystal.h>
#include<stdint.h>
#define ONE 1
#define one 1

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 6, 4, 3, 2);
void LCD_Command (char);				/* LCD command write function */
void LCD_Char (char);					/* LCD data write function */
void LCD_Init (void);					/* LCD Initialize function */
void LCD_String (char*);				/* Send string to LCD function */
void LCD_String_xy (char,char,char*);	/* Send row, position and string to LCD function */
void LCD_Clear();						/* Clear LCD*/
void ADC_Init();
void voltage_read();
void current_read();
void init_func();//Initialization of LED and switch pins
void sw_intr();//Activating pin change interrupt for sw_on and sw_off
void Lane_departure();

#define CLR_BIT(PORT,PIN) PORT&=~(1<<PIN)
#define SET_BIT(PORT,PIN) PORT|=(1<<PIN)
float input_voltage = 0.0;
float celsius;
float cel,c;
int charge;
int tilt;
int vol;
int i=0,j=0;

float obstacle_dist=0;  //   integer for access obstacle distance
int pulse = 0; //   interger  to access all though the program
int edge = 0;
int hill_flag1=0;
int hill_flag2=0;
int IGN=0x00;

void LCD_Init()
{
  lcd.begin(16, 2);// set up the LCD's number of columns and rows
  _delay_ms(100);
  lcd.clear();
  //lcd.print("ADAS & BMS !");
  _delay_ms(100);
  lcd.clear();
  //lcd.print("Loading...");
  //_delay_ms(200);
  lcd.clear();
}

void ADC_Init()
{									
	 ADMUX|=(1<<REFS0);//AVCC with external capacitor at AREF pin
     ADCSRA|=(1<<ADEN);//Enabling the ADC									
}

int ADC_Read(char channel)							
{
    ADMUX = 0x40 | (channel & 0x07);//Channel selection
    ADCSRA|=(1<<ADSC);//Set ADSC and Start Analog to digital conversion
    while((ADCSRA  & 1<<ADSC));//wait till ADSC becomes 0						
    _delay_ms(1);									
    return ADC;									
}


void voltage_read()
{
  lcd.setCursor(0, 0);
  int analog_vlaue=ADC_Read(0);
  input_voltage = (analog_vlaue * 5.0)/1023.0;//convert it show it shows in terms of 12V
  input_voltage *= 3.0;
  lcd.print(input_voltage);
  lcd.print("V");
}
int charge_read()
{
  //lcd.setCursor(11, 0); 
      //Voltage estimation in volts.
      int analog_value1 = ADC_Read(1);
  	  int charge=analog_value1/10;
  		return charge;
  /*
  		lcd.print(charge);
 		 lcd.print("%");
  		if(charge>30&&charge<90)
        {
          CLR_BIT(PORTD,PD0);
          //SET_BIT(PORTD,PD0);
          lcd.setCursor(0, 1);
          lcd.print("Battery: Stable");
          
        }
  		else if(charge>15&&charge<30)
        {
          //CLR_BIT(PORTD,PD0);
          SET_BIT(PORTD,PD0);
           lcd.setCursor(0, 1);
          lcd.print("Charge the Battery");
          
        }
        else
        {
          pwm_00();
          lcd.clear();
          PORTB&=~((ONE<<PB1)|(ONE<<PB2));
          CLR_BIT(PORTB,PB0);//LED off
          CLR_BIT(PORTD,PD1);//Buzzer off
          CLR_BIT(PORTD,PD0);//Buzzer off 
        }
        
          
  
  //    input_voltage = (analog_value1 * 5.0) / (1023.0 * 1501.0);
   //  input_current = input_voltage;
      //print to screen
     // lcd.print(input_voltage*1000);//conversion to mAmps
  		//lcd.setCursor(3, 1);
  // lcd.clear();
  */
      
}

void temp_read()
{
  lcd.setCursor(7, 0);
  celsius = ADC_Read(4);
  cel=celsius*5000/1024;
  c = ((cel/10)+(-50));
  int t=c;
  //lcd.print("T=");
  lcd.print(t);
  lcd.print("D");
  if(c>50)
  {
    SET_BIT(PORTD,PD0);
  }
  else
  {
    CLR_BIT(PORTD,PD0);
  }
}
void hill_init()
{
  DDRB |= ((ONE<<PB2)|(ONE<<PB1));// Motor output
  DDRB &=~(ONE<<PB5);// Tilt sensor 1
  DDRD &=~(ONE<<PD7);// Tilt sensor 2
  PORTB &=~((ONE<< PB2)|(ONE<<PB1));//Motor output
}
//Activating pin change interrupt for sw_on and sw_off
void hill_asst()
{
  PCICR |= (ONE<<PCIE2);// Activating PCIIE2 pin in PCICR
   PCICR |= (ONE<<PCIE0);//Activating PCIIE2 pin in PCICR
   PCMSK2 |= (ONE<<PCINT23);//Enabling PIN change interrupt at PCINT18
   PCMSK0 |= (ONE<<PCINT5);//Enabling PIN change interrupt at PCINT18
}  

void Lane_departure()
{
  SET_BIT(DDRD,PD1); //Buzzer  OUTPUT    
  SET_BIT(DDRB,PB0); //Green LED OUTPUT
  CLR_BIT(PORTD,PD1);        
  CLR_BIT(PORTB,PB0); 
  int value =ADC_Read(3);
  if((value<300 || value>500))
  {
    
      SET_BIT(PORTD,PD1);//Buzzer on
      CLR_BIT(PORTB,PB0);//LED off
     
   }
    else 
    {
      CLR_BIT(PORTD,PD1);//Buzzer off
      SET_BIT(PORTB,PB0);//LED on
      
    }
  
  
}

ISR(PCINT1_vect)//interrupt service routine when there is a change in logic level
{
   if (edge==1) //when logic from HIGH to LOW
{
TCCR1B=0; //disabling counter
pulse=TCNT1; //count memory is updated to integer
obstacle_dist=pulse/2.18543;
//Serial.println(obstacle_dist);
PORTC &=~(1<<PINC5);
TCNT1=0; //resetting the counter memory
edge=0;
}

if (edge==0)       //when logic change from LOW to HIGH
     {
TCCR1B|=((1<<CS12)|(1<<CS10));//enabling counter PRESCALAR 1024
edge=1;
     }
}

void Stop_motor()
{
  
  TIMSK0 &= ~(one<<OCIE0A);//Enable Compare Interrupt
  TIMSK0 &= ~(one<<OCIE0B);//Enable Compare Interrupt
  PORTD &= ~(one << PD5);//Reset DC motor pin to 0
  //PORTD |= (one << PD7);//Set Buzzer pin to 1
}

void pwm_75()
{
 
 OCR0A = 255;//100% duty cycle count updated for comparator
 OCR0B =255;
 TCCR0B |= (one<<CS02) | (one<<CS00);//for prescalar 1024
 TIMSK0 |= (one<<OCIE0A);//Enable Compare Interrupt
 TIMSK0 |= (one<<OCIE0B);//Enable Compare Interrupt
}
void pwm_50()
{
 
 OCR0A = 255;//100% duty cycle count updated for comparator
 OCR0B = 128;//50% duty cycle count updated for comparator
 TCCR0B |= (one<<CS02) | (one<<CS00);//for prescalar 1024
 TIMSK0 |= (one<<OCIE0A);//Enable Compare Interrupt
 TIMSK0 |= (one<<OCIE0B);//Enable Compare Interrupt
}
void pwm_00()
{
 PORTD&=~(one<<PD5);
 TCCR0B &= ~(one<<CS02) | (one<<CS00);//for prescalar 1024
 TIMSK0 &= ~(one<<OCIE0A);//Enable Compare Interrupt
 TIMSK0 &= ~(one<<OCIE0B);//Enable Compare Interrupt
}

void timer0_enable()
{
  TCNT0 = 0;
  TCCR0A |= (one<<WGM01);//for CTC mode WMG01 is 1 
  TCCR0A &=~(one<<WGM00);//for CTC mode WMG00 is 0
  TCCR0B &=~(one<<WGM02);//for CTC mode WMG02 is 0
}


void init1()
{
  DDRC &=~(one<<PC5);//Port for ultrasonic  sensor
  DDRD|=(one<<PD5);//port otor
  DDRD|=(one<<PD1);//port for buzzer
   DDRD|=(one<<PD0);
  PORTC&=~(one<<PC5);
  PORTD&=~(one<<PD5);
  PORTD&=~(one<<PD1);
	PCICR |= (ONE<<PCIE1);// Activating PCIIE2 pin in PCICR
    PCMSK1 |= (ONE<<PCINT13);
}


void compare_distance()
{
  if(obstacle_dist>60 && obstacle_dist<120)
  { 
    PORTD&=~(one<<PD1);//buzzer off
    PORTD&=~(one<<PD0);//MIL off
    pwm_75();
  }
  if(obstacle_dist<80 )
  { 
    PORTD|=(one<<PD0);//led on 
  }
  if(obstacle_dist>=30 && obstacle_dist<=50)
  {
    PORTD|=(one<<PD1);
    pwm_50();
   
  }
  if(obstacle_dist<30 )
  {
   PORTD&=~(one<<PD1);
    pwm_00();
  }
}


int main()
{
  //if(Ign==1){
  SET_BIT(DDRD,0);
  SET_BIT(DDRB,0);         //PD as output
    CLR_BIT(DDRC,2);         //keep all LEDs off
 CLR_BIT(PORTC,2);
  
  	LCD_Init();
    ADC_Init();
    SREG |= (ONE<<PD7);// Activating the global Interrupt Pin in status register
    hill_init(); 
    hill_asst();
    //Serial.begin(9600);
    init1();
    timer0_enable();
  int c;
	while(1)
	{
      c=charge_read();
      //while()
      //LCD_Init();
      
      while(c>15)
      {
      uint8_t pin_read=0x00;
		pin_read=PINC;
        
        if((pin_read&(1<<2)))
         {
          lcd.setCursor(11, 0);
            lcd.print(c);
            lcd.print("%");
            if(c>30&&c<90)
            {
              CLR_BIT(PORTD,PD0);
              //SET_BIT(PORTD,PD0);
              lcd.setCursor(0, 1);
              lcd.print("Battery: Stable");

            }
            else if(c>15&&c<30)
            {
              //CLR_BIT(PORTD,PD0);
              SET_BIT(PORTD,PD0);
              lcd.setCursor(0, 1);
              lcd.print("Charge the Battery");

            }
          
          voltage_read();
          c=charge_read();
          
          
          temp_read();
          Lane_departure();
      	 if(hill_flag1==0&&hill_flag2==0)//checking weather HAC is not interrupted
          {
            DDRC|=(1<<PC5);//output port     
            PORTC|=(1<<PINC5);//set it to one
            _delay_us(15);      ///triggering the sensor for 15usec
            PORTC &=~(1<<PINC5);
            DDRC&=~(1<<PC5);//input port
            _delay_ms(50);
            compare_distance();
     	  }
            else
              {
               pwm_00();
              }
        }
          else
          {
            pwm_00();
            lcd.clear();
            PORTB&=~((ONE<<PB1)|(ONE<<PB2));
            CLR_BIT(PORTB,PB0);//LED off
            CLR_BIT(PORTD,PD1);//
            CLR_BIT(PORTD,PD0);//
            
          }
            
      }
       
      pwm_00();
            lcd.clear();
            PORTB&=~((ONE<<PB1)|(ONE<<PB2));
            CLR_BIT(PORTB,PB0);//LED off
            CLR_BIT(PORTD,PD1);//Buzzer off
            CLR_BIT(PORTD,PD0);//Buzzer off
	}
	return 0;
}


ISR(PCINT0_vect)
{
  //pwm_00();
  hill_flag1=!(hill_flag1);
  if(i==0)
  {
  PORTB&=~(ONE<<PB1);
  PORTB|=(ONE<<PB2);
  }
  if(i==1)
  {
    PORTB&=~((ONE<<PB1)|(ONE<<PB2));
    i=-1;
  }
  i++;
}
ISR(PCINT2_vect)
{
  hill_flag2=!(hill_flag2);
  //pwm_00();
    if(j==0)
  {
  PORTB|=(ONE<<PB1);
  PORTB&=~(ONE<<PB2);
  }
  if(j==1)
  {
    PORTB&=~((ONE<<PB1)|(ONE<<PB2));
    j=-1;
  }
  j++;
}


ISR(TIMER0_COMPA_vect)
{
  PORTD&=~(one<<PD5);
}

ISR(TIMER0_COMPB_vect)
{
  PORTD|=(one<<PD5);
}



  
    
   
  

