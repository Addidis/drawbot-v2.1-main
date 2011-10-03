/////////////////////////////////////////////////////////////////////////////
// Include files
// These lines allow us to use code routines (libraries) from other files,
// so that we don't have to write everything by ourselves.
/////////////////////////////////////////////////////////////////////////////

#include <p18lf25k22.h>        // This file includes definitions for all of the
                                     // registers on our chip
#include <usart.h>
#include <math.h>
#include "delays.h"
#include "AN991.h"
#include "i2c.h"

#pragma config FOSC = INTIO67				//use INTOSC
#pragma config PLLCFG = OFF					//clocked at 16MHz
#pragma config WDTEN = OFF					//WDT off
#pragma config PBADEN = OFF
// Fix a compiler bug
#undef INTCON
#undef INTCONbits

#define numSteps 400
#define sw1    PORTBbits.RB0
#define sw2    PORTBbits.RB4
#define sw3    PORTBbits.RB3
#define sw4    PORTBbits.RB2
#define sw5    PORTBbits.RB1

//led macros
#define mLED_1 				LATCbits.LATC2
#define mLED_1_On()     	mLED_1 = 1;
#define mLED_1_Off()    	mLED_1 = 0;
#define mLED_1_Toggle()     mLED_1 = !mLED_1;

/////////////////////////////////////////////////////////////////////////////
// Function declarations
// Declare any user functions that you want to use here
/////////////////////////////////////////////////////////////////////////////
void main (void);
void setup(void);
void loop(void);
void trimMotors(void);
void low_isr(void);
void high_isr(void);
void putc1USART( char data );
void puts1USART( char *data );
void putrs1USART ( const MEM_MODEL rom char *data);
void OpenI2C1( unsigned char sync_mode, unsigned char slew );
void drawcircle(float a, float b, int r); 
void setHome(void);

// Bit masks for the motor state tables
int A_C1states[4] = {0x00, 0x01, 0x01, 0x00};
int A_C2states[4] = {0x02, 0x02, 0x00, 0x00};
int B_C1states[4] = {0x00, 0x04, 0x04, 0x00};
int B_C2states[4] = {0x08, 0x08, 0x00, 0x00};
//Step variables for tracking 
int Astate = 0;       // state of left motor
int Bstate = 0;       // state of right motor
int stepUnit = 100;   // calculated from spool diameter
int w;                // width of drawing field
int h;                // height
int a1;               // length of left string
int b1;               // length of right string
int x1;               // current x position
int y1;               // current y position
int sethome=1;		
volatile int pause = 1;	

//eprom declarations 
//See AN991 PDF
volatile unsigned char PageString[128];             	//Holds the device page data to/from EEPROM
unsigned int  PageSize = 0x80;							//Page size in bytes 128
volatile unsigned short pageadd = 0;					// variable for page address
volatile unsigned char ControlByte=0b10100110;          //Control Byte
volatile unsigned char HighAdd=0;                    	//High Order Address Byte
volatile unsigned char LowAdd=0;                     	//Low Order Address Byte
//unsigned char Data;                       				//Data Byte
//unsigned char Length;                     				//Length of bytes to read
              			

volatile unsigned char receiveCharacter = 0;    		// Stores the last character that was

//Drawing variables
								//paper size (not drawing area)
static int leftW = -800;			//left side of the paper in grid units (inches)
static int rightW = +800;			//right side of the paper 
static int widthT = 1600;			//total width 
static int height = 2300;			//total height 
static int offset = 100; 			// start the bottom of page at -100
static int numofpixWide = 128;		//This should stay the same
volatile int numofpixHigh = 128;	//Set this to picture settings 
volatile unsigned int speed=10000;				//speed of drawing 10000
volatile int caddystablizingdelay=1;		//delay after a move command
//pixels


void main (void)
{
    // Call the setup function once to put everything in order
    setup();

    // Then call the loop() function over and over
    while (1)
    {
      loop();
    }
}

// Map function, from:
// http://www.arduino.cc/en/Reference/Map
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setSpeed(void)
	{
 	PIE1bits.RC1IE=0;					//Turn off interrupts 
	while (!DataRdy1USART());		//wait for byte
	speed = map(RCREG, 1,127, 20000,500);
	PIE1bits.RC1IE=1;					//Turn off interrupts 
	}


void turnMotors(int stepDirA, int stepDirB,int delay)
  {

    // turn the left and right motors one step in the given direction
    unsigned int count;
    a1 += stepDirA;
    b1 += stepDirB;
    Astate-=stepDirA;
    Bstate+=stepDirB;
    if (Astate == -1) { Astate = 3; }
    if (Bstate == -1) { Bstate = 3; }
    Astate = Astate %4;
    Bstate = Bstate %4;

    PORTA = A_C1states[Astate] | A_C2states[Astate] |
          B_C1states[Bstate] | B_C2states[Bstate] ;

    for( count = 0; count < delay; count++) {}   // delay
}

void moveTo(int x2, int y2, int delay)
  {
    // move from the current point (x1, y1) to (x2, y2)
    // implement Breshenham's algorithm to straighten this out
    int dirA =0;
    int dirB=0;
    int a2 = sqrt(pow(x2, 2)+pow(y2, 2));
    int b2 = sqrt(pow((w-x2), 2)+pow(y2,2));
 
    if (a2 > a1) { dirA = 1; }
    if (a2 < a1) { dirA = -1; }
    if (b2 > b1) { dirB = 1; }
    if (b2 < b1) { dirB = -1; }
    while ((a1!=a2) || (b1!=b2)) {
    if (a1 == a2) { dirA = 0; }
    if (b1 == b2) { dirB = 0; }
    turnMotors(dirA, dirB, delay);
	Delay10KTCYx(caddystablizingdelay);//delay to stabilize caddy
    }
    x1=x2;
    y1=y2;

}


void trimMotors(void)
	{
	int i;
	int t;

	if (receiveCharacter != 0)
		{
		// See if we got a command
		switch (receiveCharacter)
			{
			case 'g':			//Move caddy up 4 steps
				for(i=0;i<4;i++){
					turnMotors(-1,-1,500);
					}		
			break;

			case 'h':		//move caddy  down 4 steps
				for(i=0;i<4;i++){
					turnMotors(1,1,500);
					}	
			break;

			case 'j':		//move caddy left 100 steps
				for(i=0;i<4;i++){
					turnMotors(-1,1,500);
					}	
			break;

			case 'k':		//move caddy right
				for(i=0;i<4;i++){
					turnMotors(1,-1,500);
					}
			break;

			case 'a':            //left stepper up 4 steps
				for(i=0; i<=4; i++)
					{                
					turnMotors(-1,0,500);
					}
			break;

			case 'q':            //Left stepper up  one turn
				for(i=0; i<400; i++)
					{
					turnMotors(-1,0,500);
					}
				break;

			case 's':            //left stepper down 4 steps
				for(i=0; i<=4; i++)
					{                
					turnMotors(1,0,500);
					}
			break;

			case 'w':            //stepper one direction b one turn
				for(i=0; i<400; i++)
					{                
					turnMotors(1,0,500);
					}
			break;

			case 'f':            //Right stepper  down 4 steps
				for(i=0; i<=4; i++)
					{                
					turnMotors(0,1,500);
					}
			break;

			case 'r':            //stepper down one turn
				for(i=0; i<400; i++)
					{                
					turnMotors(0,1,500);
					}
			break;

			case 'd':            //stepper two up 4 steps
				for(i=0; i<=4; i++)
					{                
					turnMotors(0,-1,500);
					}
			break;

			case 'e':            //stepper one direction b one turn
				for(i=0; i<400; i++)
					{                
					turnMotors(0,-1,500);
					}
			break;

			case '.':            //stepper one direction b one turn
 				setHome();
			break;

			case ']':            //stepper one direction b one turn
				setSpeed();
			break;

			case 'u':			//upload drawdata
				PIE1bits.RC1IE=0;					//Turn off interrupts 
				LATCbits.LATC2 = 0;					//Turn off led to tell user mode entered 
				for (t=0; t<(numofpixHigh*PageSize); t+=PageSize)			//Loop threw  pages 
					{					
					for(i=0; i<128; i++)      			//Get 128 bytes Stick them in the PageString				
						{
						while (!DataRdy1USART());		//wait for byte
						PageString[i] = RCREG;			//Stick it in PageString
						}
 					pageadd = t;						//t = page address
					ControlByte = 0b10100110;			//not needed unless you change blocks ..
					HighAdd = ((pageadd>>8)&0xff);		//Makes it easy to deal with pageaddress
					LowAdd = (pageadd&0xff);
					HDPageWriteI2C(ControlByte, HighAdd, LowAdd, PageString );                  //High Density Page Write
					//ACK for next page here 
					Delay1KTCYx( 60 );
					Write1USART((char)PageString[0]);		//send first byte of the page to the gui for an ack 
					}
					LATCbits.LATC2 = 1;				//turn on led to tell user its finished
					PIE1bits.RC1IE=1;				//Turn back on interrupts 
			break;

			case 'm':			//Clear Memory
				for(i=0; i<128; i++)      						//Initialize array to 255 (blank)
					{
					PageString[i] = 255;     						
					}
				for (t=0; t<=16256; t+=128)			//Loop threw 128 pages ((128-1)  *128)
					{
					pageadd = 0;
					ControlByte = 0b10100110;
					HighAdd = ((pageadd>>8)&0xff);
					LowAdd = (pageadd&0xff);
					HDPageWriteI2C(ControlByte, HighAdd, LowAdd, PageString );                  //High Density Page Write
					LATCbits.LATC2 = 0;
					Delay1KTCYx( 60 );
					}
					LATCbits.LATC2 = 1;
			break;

		/*		case 't':				//This will be adapted to verify the eprom before drawing.
				ControlByte = 0b10100110;
				pageadd =0;					
				HighAdd = ((pageadd>>8)&0xff);
				LowAdd = (pageadd&0xff);
				HDSequentialReadI2C(ControlByte, HighAdd, LowAdd, PageString, PageSize );   //High Density Page Read
				if (PageString[0]==255)
					{
					LATCbits.LATC2 = 0;
					Delay1KTCYx( 120 );
					LATCbits.LATC2 = 1;
					}
				if (PageString[0]==1)
					{
					LATCbits.LATC2 = 0;
					Delay1KTCYx( 220 );
					Delay1KTCYx( 220 );
					LATCbits.LATC2 = 1;
					Delay1KTCYx( 220 );
					Delay1KTCYx( 220 );
					LATCbits.LATC2 = 0;
					Delay1KTCYx( 220 );
					Delay1KTCYx( 220 );
					LATCbits.LATC2 = 1;
					}
			break;

		case 'c':	//Erase current page					
									for(i=0; i<128; i++)      													//Loop through full array
    									{
        								PageString[i] = 255;     												//Initialize array to 255 (blank)
    									}
									pageadd =0;
									ControlByte = 0b10100110;
									HighAdd = ((pageadd>>8)&0xff);
									LowAdd = (pageadd&0xff);
						 			HDPageWriteI2C(ControlByte, HighAdd, LowAdd, PageString );                  //High Density Page Write
									Delay1KTCYx( 60 );
									LATCbits.LATC2 = 0;
									Delay1KTCYx( 60 );
									LATCbits.LATC2 = 1;
									for(i=0; i<128; i++)      													//Loop through full array
    									{
        								PageString[i] = 0;     													//Initialize array to 0 
    									}
								break;
		*/
	}
	receiveCharacter=0;
}
           
       
// Switch handeling code. 
//***********************
//sw2
	if (sw2==0)				
		{
		Delay1KTCYx(10);
		if(sw2==0)
          	{
			receiveCharacter='a';		//move motor one direction A 4 steps
          	Delay10KTCYx(120);          //Delay to see if its held down or not
          	if(sw2==0)                  //Its held down
				{              
				receiveCharacter='q';			//move motor one direction A untill released
              	}
         	 }
      }//end sw2
//sw3
	if (sw3==0)				
		{
		Delay1KTCYx(10);
		if(sw3==0)
          	{
			receiveCharacter='s';				//move motor one direction B 4 steps
         	Delay10KTCYx(120);          //Delay to see if its held down or not
          	if(sw3==0)                   		//Its held down
				{              
				receiveCharacter='w';			//move motor one direction B untill released
              	}
         	 }
      }//end sw3
//sw4
	if (sw4==0)				
		{
		Delay1KTCYx(10);
		if(sw4==0)
          	{
			receiveCharacter='d';		//move motor Two direction A 4 steps
          	Delay10KTCYx(120);          //Delay to see if its held down or not
          	if(sw4==0)                   //Its held down
				{              
				receiveCharacter='e';			//move motor Two direction A untill released
              	}
         	 }
      }//end sw4
//sw5
	if (sw5==0)				
		{
		Delay1KTCYx(10);
		if(sw5==0)
          	{
			receiveCharacter='f';		//move motor two direction B 4 steps
          	Delay10KTCYx(120);          //Delay to see if its held down or not
          	if(sw5==0)                   //Its held down
				{              
				receiveCharacter='r';			//move motor one direction B untill released
              	}
         	 }
      }//end sw5
  }	//end trim motors


// This function is called once, when the microcontroller is turned on.

void setup(void)
  {
   OSCCONbits.IRCF=111;

    // Set all of the pins on Port A to be outputs,
    // Cleared bits are outputs.
    TRISA = 0x00;
    PORTA = 0x00;

	// Set up buttons as inputs RB0-START&\PAUSE\RESUME RB1 RB2 RB3 RB4
	TRISB = 0B00011111;

	//led
   	TRISCbits.TRISC2=0;
   	LATCbits.LATC2=1;

	//I2C pin config
	TRISCbits.TRISC3 = 1;		//Must be set as inputs for an991 to take over
	TRISCbits.TRISC4 = 1;

	//I2C setup
    ANSELCbits.ANSC3 = 0;     	 // Disable analog mode for SCA
    ANSELCbits.ANSC4 = 0;    	 // Disable analog mode for SDA
	SSP1STAT=0b10000000;
	SSP1CON1=0b00101000;
	SSP1ADD=0x27;				//16mhz fosc/4 , 100Khz
	SSP1CON2=0x00;
	SSP1CON1bits.SSPEN=1;		//Turn on MSSP

	//Set up int0 for start &| pause resume
	INTCONbits.GIE=1;
	INTCON2bits.INTEDG0=1;
	INTCONbits.INT0IF=0;
	INTCONbits.INT0IE=1;

	//serial port setup 9600 baud   
	BAUDCON1= 0b00000000;
	TXSTA1bits.BRGH = 0;      
	BAUD1CONbits.BRG16 = 0;
	SPBRG1 = 0x19; 
	SPBRGH1 = 0x00; 
	// Serial port configuration
    TRISCbits.TRISC6 = 1;          // Make TX pin an input (p264 ds)
    TRISCbits.TRISC7 = 1;          // and RX pin an input
    ANSELCbits.ANSC7 = 0;          // Specifically, an analog input
	RCONbits.IPEN = 0;             //Interrupt priority off
	INTCONbits.PEIE = 1;           //Peripheral Interrupt Enable bit
	PIR1bits.RC1IF=0;				//clear Interupt flag
    PIE1bits.RC1IE = 1;            // Turn on interrupts on serial receive
//    BAUDCON1bits.WUE = 1;          // Wake up the processor when a serial

	//Baud rate select Configure the serial port to run at 9600 baud
	//for 16 MHz clock (see manual, page 275)

    //turn on serial port
    RCSTA1bits.CREN = 1;           // Enable receive mode on the serial port
    TXSTA1bits.TXEN = 1;           // Enable transmitter
    RCSTA1bits.SPEN = 1;           // Enable receiver
	receiveCharacter = RCREG;
	receiveCharacter = RCREG;

    // calculate width of drawing area in steps

    w = 72*stepUnit;
    h = 36*stepUnit;
	setHome();


}

void setHome()
	{
    // set home position
    x1 = w/2;
    y1 = h;
    // calculate starting lengths

    a1 = sqrt(pow(x1, 2)+pow(y1, 2));
    b1 = sqrt(pow((w-x1), 2)+pow(y1,2));
	}


//not used atm saving space
/*
void drawcircle(float a, float b, int r)
	{
	float t;
	float x;
	float y;
	int x2;
	int y2;

	for(t=0;t<=8; t+=.25){
		x = (a + (r * cos(t)));
		y = (b + (r * sin(t)));
		x2 = x;
		y2=y;
		moveTo(x2, y2,6500);
		}
	}
*/

void drawpixel(int startx, int starty, char shade, int direction)
	{
	int widthofpixel = widthT/numofpixWide;
	int heightofpixel = (height-offset)/numofpixHigh;
	volatile int rows = numofpixHigh/2;
	int newh = h-offset;


	Delay10KTCYx( 120 );
		if(shade== '0'){ //Black
       		if(direction == 2){
           		widthofpixel = widthofpixel - widthofpixel - widthofpixel;
       		}
			
      		moveTo(startx,starty-heightofpixel,speed);          		//move up heightofpixel
      		moveTo(startx,starty,speed);                               	//move back


      		moveTo(startx+widthofpixel/4,starty,speed);         			//move to 1/4 width of pixel
      		moveTo(startx+widthofpixel/4,starty-heightofpixel,speed);//move to 1/4 width of pixel top
      		moveTo(startx+widthofpixel/4,starty,speed);         			//move back 

     		moveTo(startx+widthofpixel/2,starty,speed);         			//move to middle of pixel
     		moveTo(startx+widthofpixel/2,starty-heightofpixel,speed);         	//move to middle of pixel up to top
     		moveTo(startx+widthofpixel/2,starty,speed);         			//move back

     		moveTo(startx+(widthofpixel/4)*3,starty,speed);         			//move to middle of pixel
     		moveTo(startx+(widthofpixel/4)*3,starty-heightofpixel,speed);         	//move to middle of pixel up to top
     		moveTo(startx+(widthofpixel/4)*3,starty,speed);         			//move back

     		moveTo(startx+widthofpixel,starty,speed);         			//move to end of pixel
      		moveTo(startx+widthofpixel,starty-heightofpixel,speed);           	//moveto end of pixel top
      		moveTo(startx+widthofpixel,starty,speed);         			//move to end of pixel
		}		

		if(shade== '1'){	//black--1					
       		if(direction == 2){
           		widthofpixel = widthofpixel - widthofpixel - widthofpixel;
       		}
      		moveTo(startx,starty-heightofpixel/4*3,speed);          		//move up heightofpixel
      		moveTo(startx,starty,speed);                               	//move back

      		moveTo(startx+widthofpixel/4,starty,speed);         			//move to 1/4 width of pixel
      		moveTo(startx+widthofpixel/4,starty-heightofpixel,speed);//move to 1/4 width of pixel top
      		moveTo(startx+widthofpixel/4,starty,speed);         			//move back 

     		moveTo(startx+widthofpixel/2,starty,speed);         			//move to middle of pixel
     		moveTo(startx+widthofpixel/2,starty-heightofpixel,speed);         	//move to middle of pixel up to top
     		moveTo(startx+widthofpixel/2,starty,speed);         			//move back

     		moveTo(startx+(widthofpixel/4)*3,starty,speed);         			//move to middle of pixel
     		moveTo(startx+(widthofpixel/4)*3,starty-heightofpixel,speed);         	//move to middle of pixel up to top
     		moveTo(startx+(widthofpixel/4)*3,starty,speed);         			//move back

     		moveTo(startx+widthofpixel-1,starty,speed);         			//move to end of pixel
      		moveTo(startx+widthofpixel-1,starty-(heightofpixel/4)*3,speed);           	//moveto end of pixel top
      		moveTo(startx+widthofpixel-1,starty,speed);         			//move to end of pixel
		}	
					

		if(shade== '2'){
       		if(direction == 2){
           		widthofpixel = widthofpixel - widthofpixel - widthofpixel;
       		}
      		moveTo(startx,starty-(heightofpixel/4)*2,speed);          		//move up heightofpixel
      		moveTo(startx,starty,speed);                               	//move back

      		moveTo(startx+widthofpixel/4,starty,speed);         			//move to 1/4 width of pixel
      		moveTo(startx+widthofpixel/4,starty-heightofpixel,speed);//move to 1/4 width of pixel top
      		moveTo(startx+widthofpixel/4,starty,speed);         			//move back 

     		moveTo(startx+widthofpixel/2,starty,speed);         			//move to middle of pixel
     		moveTo(startx+widthofpixel/2,starty-heightofpixel,speed);         	//move to middle of pixel up to top
     		moveTo(startx+widthofpixel/2,starty,speed);         			//move back

     		moveTo(startx+(widthofpixel/4)*3,starty,speed);         			//move to middle of pixel
     		moveTo(startx+(widthofpixel/4)*3,starty-heightofpixel,speed);         	//move to middle of pixel up to top
     		moveTo(startx+(widthofpixel/4)*3,starty,speed);         			//move back

     		moveTo(startx+widthofpixel-1,starty,speed);         			//move to end of pixel
      		moveTo(startx+widthofpixel-1,starty-(heightofpixel/4)*2,speed);           	//moveto end of pixel top
      		moveTo(startx+widthofpixel-1,starty,speed);         			//move to end of pixel
		}
	
		if(shade=='3'){ 			
       		if(direction == 2){
           		widthofpixel = widthofpixel - widthofpixel - widthofpixel;
       		}
      		moveTo(startx,starty-(heightofpixel/4)*2,speed);          		//move up heightofpixel
      		moveTo(startx,starty,speed);                               	//move back

      		moveTo(startx+widthofpixel/4,starty,speed);         			//move to 1/4 width of pixel
      		moveTo(startx+widthofpixel/4,starty-(heightofpixel/4)*3,speed);//move to 1/4 width of pixel top
      		moveTo(startx+widthofpixel/4,starty,speed);         			//move back 

     		moveTo(startx+widthofpixel/2,starty,speed);         			//move to middle of pixel
     		moveTo(startx+widthofpixel/2,starty-heightofpixel,speed);         	//move to middle of pixel up to top
     		moveTo(startx+widthofpixel/2,starty,speed);         			//move back
	
     		moveTo(startx+(widthofpixel/4)*3,starty,speed);         			//move to middle of pixel
     		moveTo(startx+(widthofpixel/4)*3,starty-(heightofpixel/4)*3,speed);         	//move to middle of pixel up to top
     		moveTo(startx+(widthofpixel/4)*3,starty,speed);         			//move back

     		moveTo(startx+widthofpixel-1,starty,speed);         			//move to end of pixel
      		moveTo(startx+widthofpixel-1,starty-(heightofpixel/4)*2,speed);           	//moveto end of pixel top
      		moveTo(startx+widthofpixel-1,starty,speed);         			//move to end of pixel
		}			

		if(shade=='4'){			
       		if(direction == 2){
           		widthofpixel = widthofpixel - widthofpixel - widthofpixel;
       		}
      		moveTo(startx+widthofpixel/4,starty,speed);         			//move to 1/4 width of pixel
      		moveTo(startx+widthofpixel/4,starty-(heightofpixel/4)*3,speed);//move to 1/4 width of pixel top
      		moveTo(startx+widthofpixel/4,starty,speed);         			//move back 

     		moveTo(startx+widthofpixel/2,starty,speed);         			//move to middle of pixel
     		moveTo(startx+widthofpixel/2,starty-heightofpixel,speed);         	//move to middle of pixel up to top
     		moveTo(startx+widthofpixel/2,starty,speed);         			//move back

     		moveTo(startx+(widthofpixel/4)*3,starty,speed);         			//move to middle of pixel
     		moveTo(startx+(widthofpixel/4)*3,starty-(heightofpixel/4)*3,speed);         	//move to middle of pixel up to top
     		moveTo(startx+(widthofpixel/4)*3,starty,speed);         			//move back
		}			

		if(shade=='5'){			
       		if(direction == 2){
           		widthofpixel = widthofpixel - widthofpixel - widthofpixel;
       		}
      		moveTo(startx+widthofpixel/4,starty,speed);         			//move to 1/4 width of pixel
      		moveTo(startx+widthofpixel/4,starty-(heightofpixel/4)*2,speed);//move to 1/4 width of pixel top
      		moveTo(startx+widthofpixel/4,starty,speed);         			//move back 

     		moveTo(startx+widthofpixel/2,starty,speed);         			//move to middle of pixel
     		moveTo(startx+widthofpixel/2,starty-heightofpixel,speed);         	//move to middle of pixel up to top
     		moveTo(startx+widthofpixel/2,starty,speed);         			//move back

     		moveTo(startx+(widthofpixel/4)*3,starty,speed);         			//move to middle of pixel
     		moveTo(startx+(widthofpixel/4)*3,starty-(heightofpixel/4)*2,speed);         	//move to middle of pixel up to top
     		moveTo(startx+(widthofpixel/4)*3,starty,speed);         			//move back
		}			
		

		if(shade=='6'){
       		if(direction == 2){
           		widthofpixel = widthofpixel - widthofpixel - widthofpixel;
       		}
     		moveTo(startx+widthofpixel/2,starty,speed);         			//move to middle of pixel
     		moveTo(startx+widthofpixel/2,starty-heightofpixel,speed);         	//move to middle of pixel up to top
     		moveTo(startx+widthofpixel/2,starty,speed);         			//move back
			Delay10KTCYx(caddystablizingdelay);//delay to stabilize caddy
		}		

		if(shade=='7'){
       		if(direction == 2){
           		widthofpixel = widthofpixel - widthofpixel - widthofpixel;
       		}
     		moveTo(startx+widthofpixel/2,starty,speed);         			//move to middle of pixel
     		moveTo(startx+widthofpixel/2,starty-(heightofpixel/4)*3,speed);         	//move to middle of pixel up to top
     		moveTo(startx+widthofpixel/2,starty,speed);         			//move back
			Delay10KTCYx(caddystablizingdelay);//delay to stabilize caddy
		}		

		if(shade=='8'){		//near white
       		if(direction == 2){
           		widthofpixel = widthofpixel - widthofpixel - widthofpixel;
       		}
     		moveTo(startx+widthofpixel/2,starty,speed);         			//move to middle of pixel
     		moveTo(startx+widthofpixel/2,starty-(heightofpixel/4)*2,speed);         	//move to middle of pixel up to top
     		moveTo(startx+widthofpixel/2,starty,speed);         			//move back
			Delay10KTCYx(caddystablizingdelay);//delay to stabilize caddy
		}
	} 

// This function is called repeatedly
void loop(void)
	{	
	LATCbits.LATC2 = pause;

	if(pause==0)            // This should be done every line below or it finishes                        
        {          			//before pausing
		int widthofpixel = widthT/numofpixWide;
		int heightofpixel = (height-offset)/numofpixHigh;
		int rows = numofpixHigh/2;
		int newh = h-offset;
		int shade=0;
		int memblock;
		int curentpixel;
		int halfofpixelswide = numofpixWide/2;
		pageadd = (numofpixHigh-1)*PageSize;	

		setHome();
/*
		//code to test paper size (with marker cap on ;)
		moveTo(w/2,h-height,800);		//test top of paper 
		while (sw2==1){};
		moveTo(w/2,newh,800);			//moveback
		moveTo(w/2+leftW,newh,800);	//test left of paper 
		while (sw2==1){};
		moveTo(w/2+rightW,newh,800);	//test right of paper
		while (sw2==1){};
*/
		//Draw code 
		//loop to draw number of pixelsH /2 left to right, up one right to left to start , looped number of pixels high divided by 2   
		for(rows = 0; rows<numofpixHigh*heightofpixel; rows+=heightofpixel*2)
			{
			memblock =0;

			//read data from eeprom
			ControlByte = 0b10100110;
			HighAdd = ((pageadd>>8)&0xff);
			LowAdd = (pageadd&0xff);
			HDSequentialReadI2C(ControlByte, HighAdd, LowAdd, PageString, PageSize );   //High Density Page Read
			pageadd-=PageSize;

			//draws from far left to middle of the first line in the row (direction 1)
			for(curentpixel = halfofpixelswide*widthofpixel; curentpixel>0; curentpixel-=widthofpixel)
				{

				shade = PageString[memblock];
				memblock++;
				moveTo(w/2-curentpixel,newh-rows ,speed);
				drawpixel(w/2-curentpixel,newh-rows,shade,1);
				}

			//draws from middle to far right of the first line in the row (direction 1)
		 	for(curentpixel=0; curentpixel< halfofpixelswide*widthofpixel; curentpixel+=widthofpixel)
				{
				shade = PageString[memblock];
				memblock++;
				moveTo(w/2+curentpixel, newh-rows,speed);
				drawpixel(w/2+curentpixel,newh-rows,shade,1);
				}

			//read data from eeprom
			HighAdd = ((pageadd>>8)&0xff);
			LowAdd = (pageadd&0xff);
			memblock=PageSize;
			HDSequentialReadI2C(ControlByte, HighAdd, LowAdd, PageString, PageSize );   //High Density Page Read
			pageadd-=PageSize;

			//draws from far right, to middle second line in the row	(direction  )
			for(curentpixel= (widthofpixel*halfofpixelswide)+widthofpixel; curentpixel>0; curentpixel-=widthofpixel)
				{

				shade = PageString[memblock];
				memblock--;
				moveTo(w/2+curentpixel, newh-rows-heightofpixel,speed);
				drawpixel(w/2+curentpixel,newh-rows-heightofpixel,shade,2);
				}
			//draws from middle to left second line in the row	(direction  )
			for(curentpixel= 0; curentpixel<=widthofpixel*halfofpixelswide; curentpixel+=widthofpixel)
				{
				shade = PageString[memblock];
				memblock--;
				moveTo(w/2-curentpixel, newh-rows-heightofpixel,speed);
				drawpixel(w/2-curentpixel, newh-rows-heightofpixel,shade,2);
				} 
			}
				
			while (sw2==1){{Delay1KTCYx(500);Delay1KTCYx(500);mLED_1_Toggle();}};	// Only print once
		}
	else if (pause==1)        //allow trimming on first power on, and any time drawing is paused
		{
		trimMotors();
		}
	}


//Interrupt vectors
//REFEREnce DS51288a-page 27
#pragma code high_vector=0x08

void interrupt_at_high_vector(void)
{
_asm GOTO high_isr _endasm
}
#pragma code /* return to the default code section */
#pragma interrupt high_isr

void high_isr (void)
{

if(PIR1bits.RC1IF==1)
	{
	receiveCharacter = RCREG;
	PIR1bits.RCIF=0;
	switch (receiveCharacter)
		{
		case 'p':            //This is so we can pause by remote
			if(pause==0)
				{
				while(RCREG!='q'){
					if(RCREG==']')
						{
						setSpeed();
						}				
                  	}
				}
             else pause=0;
          // LATCbits.LATC2=pause;
			receiveCharacter =0;
		break;
       }
   }

if(INTCONbits.INT0IF==1)			//int0 sw1
  { 
  while(sw2!=0){Delay10KTCYx(200);mLED_1_Toggle();}        //delay to debounce
  if(pause==1)
      {
      pause =0;
      }
  INTCONbits.INT0IF=0;
  }
// high Interrupt goes here check which interrupt caused the interrupt , do stuff , clear int flag etc
}


/*  //REFEREnce DS51288a-page 27

#pragma code low_vector=0x18
void interrupt_at_low_vector(void)
{
_asm GOTO low_isr _endasm
}
#pragma code // return to the default code section 
#pragma interruptlow low_isr
void low_isr (void)
{

// low Interrupt goes here check which interrupt caused the interrupt , do stuff , clear int flag etc
}
*/