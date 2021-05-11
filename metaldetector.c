#include <XC.h>
#include <math.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Secondary Oscillator Enable (Disabled)

// Defines
#define SYSCLK 40000000L
#define DEF_FREQ 22050L
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)

#define LCD_RS LATBbits.LATB15		//Pin 26 = RB15
#define LCD_RW LATBbits.LATB13 		// Pin 24 = RB13
#define LCD_E  LATBbits.LATB12		// Pin 23 = RB12
#define LCD_D4 LATAbits.LATA3    //Pin 10 = RA3
#define LCD_D5 LATAbits.LATA2	//Pin 9 = RA2
#define LCD_D6 LATBbits.LATB3	//Pin 7 = RB3
#define LCD_D7 LATBbits.LATB2   //Pin 6 = RB2
#define CHARS_PER_LINE 16

#define PWM_FREQ    200000L
#define DUTY_CYCLE  50

#define SET_CS LATBbits.LATB0=1
#define CLR_CS LATBbits.LATB0=0

#define WRITE_ENABLE     0x06  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define WRITE_DISABLE    0x04  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define READ_STATUS      0x05  // Address:0 Dummy:0 Num:1 to infinite fMax: 32MHz
#define READ_BYTES       0x03  // Address:3 Dummy:0 Num:1 to infinite fMax: 20MHz
#define READ_SILICON_ID  0xab  // Address:0 Dummy:3 Num:1 to infinite fMax: 32MHz
#define FAST_READ        0x0b  // Address:3 Dummy:1 Num:1 to infinite fMax: 40MHz
#define WRITE_STATUS     0x01  // Address:0 Dummy:0 Num:1 fMax: 25MHz
#define WRITE_BYTES      0x02  // Address:3 Dummy:0 Num:1 to 256 fMax: 25MHz
#define ERASE_ALL        0xc7  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define ERASE_BLOCK      0xd8  // Address:3 Dummy:0 Num:0 fMax: 25MHz
#define READ_DEVICE_ID   0x9f  // Address:0 Dummy:2 Num:1 to infinite fMax: 25MHz
#define PI 3.14159265358979323846       // pi
#define cap 0.0000000091                // capacitor values 

volatile unsigned long int playcnt=0;
volatile unsigned char play_flag=0;

// Initially from here:
// http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/pwm
void Init_pwm (void)
{
    // OC1 can be assigned to PA0, PB3, PB4, PB15, and PB7(in use).
    // Check TABLE 11-2: OUTPUT PIN SELECTION in datasheet.
    // Set OC1 to pin PA0 (pin 2 of DIP 28) with peripheral pin select
    RPA0Rbits.RPA0R = 0x0005;
 
    // Configure standard PWM mode for output compare module 1
    OC1CON = 0x0006; 
 
    // A write to PRy configures the PWM frequency
    // PR = [FPB / (PWM Frequency * TMR Prescale Value)] � 1
    PR2 = (SYSCLK / (PWM_FREQ*1)) - 1;
 
    // A write to OCxRS configures the duty cycle
    // : OCxRS / PRy = duty cycle
    OC1RS = (PR2 + 1) * ((float)DUTY_CYCLE / 100);

 	T2CON = 0x0;
    T2CONSET = 0x8000;      // Enable Timer2, prescaler 1:1
	T2CONbits.TCKPS=0x0; // Set pre-scaler to 1
    OC1CONSET = 0x8000;     // Enable Output Compare Module 1
}

void Set_pwm (unsigned char val)
{
	OC1RS = (PR2 + 1) * ((float)val / 256.0);
}

void uart_putc (unsigned char c)
{
    while( U2STAbits.UTXBF); // wait while TX buffer full
    U2TXREG = c; // send single character to transmit buffer
}

void uart_puts (char * buff)
{
	while (*buff)
	{
		uart_putc(*buff);
		buff++;
	}
}

//returns a character that was sent to the microcontroller from PIC32
unsigned char uart_getc (void)
{
	unsigned char c;
	
	while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	c=U2RXREG; //value sent to micro from PC
	return c;
}

// SPI Flash Memory connections:
// RA1  (pin 3)   (MISO) -> Pin 5 of 25Q32
// RB1  (pin 5)   (MOSI) -> Pin 2 of 25Q32
// RB14 (pin 25)  (SCLK) -> Pin 6 of 25Q32
// RB0  (pin 4)   (CSn)  -> Pin 1 of 25Q32
// 3.3V: connected to pins 3, 7, and 8
// GND:  connected to pin 4

void config_SPI(void)
{
	int rData;

	// SDI1 can be assigned to any of these pins (table TABLE 11-1: INPUT PIN SELECTION):
	//0000 = RPA1; 0001 = RPB5; 0010 = RPB1; 0011 = RPB11; 0100 = RPB8
	SDI1Rbits.SDI1R=0b0010; //SET SDI1 to RB1, pin 5 of DIP28
    ANSELB &= ~(1<<1); // Set RB1 as a digital I/O
    TRISB |= (1<<1);   // configure pin RB1 as output
	
	// SDO1 can be configured to any of these pins by writting 0b0011 to the corresponding register.
	// Check TABLE 11-2: OUTPUT PIN SELECTION (assuming the pin exists in the dip28 package): 
	// RPA1, RPB5, RPB1, RPB11, RPB8, RPA8, RPC8, RPA9, RPA2, RPB6, RPA4
	// RPB13, RPB2, RPC6, RPC1, RPC3
	RPA1Rbits.RPA1R=0b0011; // config RA1 (pin 3) for SD01
	
	// SCK1 is assigned to pin 25 and can not be changed, but it MUST be configured as digital I/O
	// because it is configured as analog input by default.
    ANSELB &= ~(1<<14); // Set RB14 as a digital I/O
    TRISB |= (1<<14);   // configure RB14 as output
    
    // CSn is assigned to RB0, pin 4.  Also onfigure as digital output pin.
    ANSELB &= ~(1<<0); // Set RB0 as a digital I/O
	TRISBbits.TRISB0 = 0;
	LATBbits.LATB0 = 1;	

	SPI1CON = 0; // Stops and resets the SPI1.
	rData=SPI1BUF; // clears the receive buffer
	SPI1STATCLR=0x40; // clear the Overflow
	SPI1CON=0x10008120; // SPI ON, 8 bits transfer, SMP=1, Master,  SPI mode unknown (looks like 0,0)
	SPI1BRG=8; // About 2.4MHz clock frequency
}

unsigned char SPIWrite(unsigned char a)
{
	SPI1BUF = a; // write to buffer for TX
	while(SPI1STATbits.SPIRBF==0); // wait for transfer complete
	return SPI1BUF; // read the received value
}

void SetupTimer1 (void)
{
	// Explanation here:
	// https://www.youtube.com/watch?v=bu6TTZHnMPY
	__builtin_disable_interrupts();
	PR1 =(SYSCLK/DEF_FREQ)-1; // since SYSCLK/FREQ = PS*(PR1+1)
	TMR1 = 0;
	T1CONbits.TCKPS = 0; // Pre-scaler: 1
	T1CONbits.TCS = 0; // Clock source
	T1CONbits.ON = 1;
	IPC1bits.T1IP = 5;
	IPC1bits.T1IS = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	
	INTCONbits.MVEC = 1; //Int multi-vector
	__builtin_enable_interrupts();
}

void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
	unsigned char c;
	
	LATBbits.LATB6 = !LATBbits.LATB6; // Toggle pin RB6 (used to check the right frequency)
	IFS0CLR=_IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0
	
	if(play_flag!=0)
	{  
		if(playcnt==0)
		{
			SET_CS; // Done playing: Disable 25Q32 SPI flash memory
			play_flag=0;
		}
		else
		{
			c=SPIWrite(0x00);
			Set_pwm(c); // Output value to PWM (used as DAC)
			playcnt--;
		}
	}
}

void Start_Playback (unsigned long int address, unsigned long int numb)
{
    CLR_CS; // Enable 25Q32 SPI flash memory.
    SPIWrite(READ_BYTES);
    SPIWrite((unsigned char)((address>>16)&0xff));
    SPIWrite((unsigned char)((address>>8)&0xff));
    SPIWrite((unsigned char)(address&0xff));
    playcnt=numb;
    play_flag=1;
}

void Enable_Write (void)
{
    CLR_CS; // Enable 25Q32 SPI flash memory.
    SPIWrite(WRITE_ENABLE);
	SET_CS; // Disable 25Q32 SPI flash memory
}

void Check_WIP (void)
{
	unsigned char c;
	do
	{
    	CLR_CS; // Enable 25Q32 SPI flash memory.
	    SPIWrite(READ_STATUS);
	    c=SPIWrite(0x55);
		SET_CS; // Disable 25Q32 SPI flash memory
	} while (c&0x01);
}

static const unsigned short crc16_ccitt_table[256] = {
    0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
    0x8108U, 0x9129U, 0xA14AU, 0xB16BU, 0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
    0x1231U, 0x0210U, 0x3273U, 0x2252U, 0x52B5U, 0x4294U, 0x72F7U, 0x62D6U,
    0x9339U, 0x8318U, 0xB37BU, 0xA35AU, 0xD3BDU, 0xC39CU, 0xF3FFU, 0xE3DEU,
    0x2462U, 0x3443U, 0x0420U, 0x1401U, 0x64E6U, 0x74C7U, 0x44A4U, 0x5485U,
    0xA56AU, 0xB54BU, 0x8528U, 0x9509U, 0xE5EEU, 0xF5CFU, 0xC5ACU, 0xD58DU,
    0x3653U, 0x2672U, 0x1611U, 0x0630U, 0x76D7U, 0x66F6U, 0x5695U, 0x46B4U,
    0xB75BU, 0xA77AU, 0x9719U, 0x8738U, 0xF7DFU, 0xE7FEU, 0xD79DU, 0xC7BCU,
    0x48C4U, 0x58E5U, 0x6886U, 0x78A7U, 0x0840U, 0x1861U, 0x2802U, 0x3823U,
    0xC9CCU, 0xD9EDU, 0xE98EU, 0xF9AFU, 0x8948U, 0x9969U, 0xA90AU, 0xB92BU,
    0x5AF5U, 0x4AD4U, 0x7AB7U, 0x6A96U, 0x1A71U, 0x0A50U, 0x3A33U, 0x2A12U,
    0xDBFDU, 0xCBDCU, 0xFBBFU, 0xEB9EU, 0x9B79U, 0x8B58U, 0xBB3BU, 0xAB1AU,
    0x6CA6U, 0x7C87U, 0x4CE4U, 0x5CC5U, 0x2C22U, 0x3C03U, 0x0C60U, 0x1C41U,
    0xEDAEU, 0xFD8FU, 0xCDECU, 0xDDCDU, 0xAD2AU, 0xBD0BU, 0x8D68U, 0x9D49U,
    0x7E97U, 0x6EB6U, 0x5ED5U, 0x4EF4U, 0x3E13U, 0x2E32U, 0x1E51U, 0x0E70U,
    0xFF9FU, 0xEFBEU, 0xDFDDU, 0xCFFCU, 0xBF1BU, 0xAF3AU, 0x9F59U, 0x8F78U,
    0x9188U, 0x81A9U, 0xB1CAU, 0xA1EBU, 0xD10CU, 0xC12DU, 0xF14EU, 0xE16FU,
    0x1080U, 0x00A1U, 0x30C2U, 0x20E3U, 0x5004U, 0x4025U, 0x7046U, 0x6067U,
    0x83B9U, 0x9398U, 0xA3FBU, 0xB3DAU, 0xC33DU, 0xD31CU, 0xE37FU, 0xF35EU,
    0x02B1U, 0x1290U, 0x22F3U, 0x32D2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U,
    0xB5EAU, 0xA5CBU, 0x95A8U, 0x8589U, 0xF56EU, 0xE54FU, 0xD52CU, 0xC50DU,
    0x34E2U, 0x24C3U, 0x14A0U, 0x0481U, 0x7466U, 0x6447U, 0x5424U, 0x4405U,
    0xA7DBU, 0xB7FAU, 0x8799U, 0x97B8U, 0xE75FU, 0xF77EU, 0xC71DU, 0xD73CU,
    0x26D3U, 0x36F2U, 0x0691U, 0x16B0U, 0x6657U, 0x7676U, 0x4615U, 0x5634U,
    0xD94CU, 0xC96DU, 0xF90EU, 0xE92FU, 0x99C8U, 0x89E9U, 0xB98AU, 0xA9ABU,
    0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18C0U, 0x08E1U, 0x3882U, 0x28A3U,
    0xCB7DU, 0xDB5CU, 0xEB3FU, 0xFB1EU, 0x8BF9U, 0x9BD8U, 0xABBBU, 0xBB9AU,
    0x4A75U, 0x5A54U, 0x6A37U, 0x7A16U, 0x0AF1U, 0x1AD0U, 0x2AB3U, 0x3A92U,
    0xFD2EU, 0xED0FU, 0xDD6CU, 0xCD4DU, 0xBDAAU, 0xAD8BU, 0x9DE8U, 0x8DC9U,
    0x7C26U, 0x6C07U, 0x5C64U, 0x4C45U, 0x3CA2U, 0x2C83U, 0x1CE0U, 0x0CC1U,
    0xEF1FU, 0xFF3EU, 0xCF5DU, 0xDF7CU, 0xAF9BU, 0xBFBAU, 0x8FD9U, 0x9FF8U,
    0x6E17U, 0x7E36U, 0x4E55U, 0x5E74U, 0x2E93U, 0x3EB2U, 0x0ED1U, 0x1EF0U
};

unsigned short crc16_ccitt(unsigned char val, unsigned short crc)
{
    unsigned short tmp;

    tmp = (crc >> 8) ^ val;
    crc = ((unsigned short)(crc << 8U)) ^ crc16_ccitt_table[tmp];
    return crc;
}

// Get a 24-bit number from the serial port and store it into a unsigned long
void get_ulong(unsigned long * lptr)
{
    unsigned char * bytes;
    bytes=(unsigned char *) lptr;
	bytes[3]=0;
	bytes[2]=uart_getc();
	bytes[1]=uart_getc();
	bytes[0]=uart_getc();
}

// Uses Timer4 to delay <us> microseconds
void Timer4us(unsigned char t) 
{
     T4CON = 0x8000; // enable Timer4, source PBCLK, 1:1 prescaler
 
    // delay 100us per loop until less than 100us remain
    while( t >= 100)
    {
        t-=100;
        TMR4 = 0;
        while( TMR4 < SYSCLK/10000);
    }
 
    // delay 10us per loop until less than 10us remain
    while( t >= 10)
    {
        t-=10;
        TMR4 = 0;
        while( TMR4 < SYSCLK/100000);
    }
 
    // delay 1us per loop until finished
    while( t > 0)
    {
        t--;
        TMR4 = 0;
        while( TMR4 < SYSCLK/1000000);
    }
    // turn off Timer4 so function is self-contained
    T4CONCLR = 0x8000;
}

void Init_LCD_Pins (void)
{
    TRISBbits.TRISB15=0;			//Pin 26 = RB15
    TRISBbits.TRISB13=0;			//Pin 24 = RB13
    TRISBbits.TRISB12=0;			//Pin 23 = RB12
    TRISAbits.TRISA3=0;			//Pin 10 = RA3
    TRISAbits.TRISA2=0;			//Pin 9 = RA2
    TRISBbits.TRISB3=0;			//Pin 7 = RB3
    TRISBbits.TRISB2=0;			//Pin 6 = RB2
}

void LCD_pulse (void)
{
	LCD_E=1;
	Timer4us(40);
	LCD_E=0;
}

void LCD_byte (unsigned char x)
{
    LCD_D7=(x & 0x80)?1:0;
    LCD_D6=(x & 0x40)?1:0;
    LCD_D5=(x & 0x20)?1:0;
    LCD_D4=(x & 0x10)?1:0;
    LCD_pulse();
    Timer4us(40); // Or whatever the name of your us function is
    LCD_D7=(x & 0x08)?1:0;
    LCD_D6=(x & 0x04)?1:0;
    LCD_D5=(x & 0x02)?1:0;
    LCD_D4=(x & 0x01)?1:0;
    LCD_pulse();
}

void waitms_2 (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer4us(250);
}

void WriteCommand (unsigned char x)
{
	LCD_RS=0;
	LCD_byte(x);
	waitms_2(5);
}

void WriteData (unsigned char x)
{
	LCD_RS=1;
	LCD_byte(x);
	waitms_2(2);
}

void LCD_4BIT (void)
{
	LCD_E=0; // Resting state of LCD's enable is zero
	LCD_RW=0; // We are only writing to the LCD in this program
	waitms_2(20);
	// First make sure the LCD is in 8-bit mode and then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode

	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	waitms_2(20); // Wait for clear screen command to finsih.
}

void LCDprint(char * string, unsigned char line, bool clear)
{
	int j;

	WriteCommand(line==2?0xc0:0x80);
	waitms_2(5);
	for(j=0; string[j]!=0; j++)	WriteData(string[j]);// Write the message
	if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); // Clear the rest of the line
}

void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

// Needed to by scanf() and gets()
int _mon_getc(int canblock)
{
	char c;
	
    if (canblock)
    {
	    while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	    c=U2RXREG;
	    if(c=='\r') c='\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
		return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA) // if data available in RX buffer
        {
		    c=U2RXREG;
		    if(c=='\r') c='\n';
			return (int)c;
        }
        else
        {
            return -1; // no characters to return
        }
    }
}

// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

#define PIN_PERIOD (PORTB&(1<<5))

// GetPeriod() seems to work fine for frequencies between 200Hz and 700kHz.
long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}

    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}
	
    _CP0_SET_COUNT(0); // resets the core timer count
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
	}

	return  _CP0_GET_COUNT();
}

// Information here:
// http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/1-basic-digital-io-220
void main(void)
{
	//waitms(500);
	long int count;
    

    float inductance;
    long int change_value;

	float T, f;

	char buff[16];
    unsigned char c;
    unsigned int j, n;
    unsigned long start, nbytes;
    unsigned short crc;

	DDPCON = 0;
	CFGCON = 0;
  
    TRISBbits.TRISB6 = 0;
	LATBbits.LATB6 = 0;	
	INTCONbits.MVEC = 1;

    // LED pins 
    

    TRISBbits.TRISB10 = 0;  // red 
	LATBbits.LATB10 = 0;

    TRISBbits.TRISB4 = 0;   // green
	LATBbits.LATB4 = 0;

    TRISAbits.TRISA4 = 0;   // yellow 
	LATAbits.LATA4 = 0;

    Init_pwm(); // pwm output used to implement DAC
	SetupTimer1(); // The ISR for this timer playsback the sound
	Init_LCD_Pins();
    LCD_4BIT();
    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    config_SPI(); // Configure hardware SPI module
    
    ANSELB &= ~(1<<5); // Set RB5 as a digital I/O
    TRISB |= (1<<5);   // configure pin RB5 as input
    CNPUB |= (1<<5);   // Enable pull-up resistor for RB5

 
	waitms(500);
	printf("Period measurement using the core timer free running counter.\r\n"
	      "Connect signal to RB5 (pin 14).\r\n");
    

    waitms_2(1000);
    LCDprint("Metal",1,1);
    waitms_2(500);
    LCDprint("Detector",2,1);
    waitms_2(1000);
    LCDprint("Using PIC32!",1,1);
    LCDprint("",2,1);
    waitms_2(2000);

    LCDprint("  Loading Up!",1,1);
    int i =0; 
    while (i<101)
        {
            sprintf(buff, "      %d", i);
            LCDprint(buff,2,1);
            waitms_2(50);
            i++; 
        }
    
    waitms_2(2000); 
     //       0123456789abcdef
    LCDprint("",2,1);  
    LCDprint("   IT'S.",1,1);
    waitms_2(1000);
    LCDprint("   IT'S. GO.",1,1);
    waitms_2(1000);
    LCDprint("     TIME.",2,1);
    waitms_2(2000); 
    


    playcnt=0;
	play_flag=0;
	SET_CS; // Disable 25Q32 SPI flash memory
    long int initial_value = GetPeriod(100); // the initial value upon starting the program



    while(1)
    {
        
        
		count=GetPeriod(100);
        change_value = initial_value - count; 
		if(count>0)
		{
			T=(count*2.0)/(SYSCLK*100.0);
			f=1/T;
            inductance = (1)/(4*PI*PI*f*f*cap); 
            inductance = inductance*1000.0;                                  
            sprintf(buff, "L=%.5f mH", inductance);

			LCDprint(buff,1,1);
            // check if large non-ferrous 
            if (change_value >= 588) {
                
                // red 
	            LATBbits.LATB10 = 1;
                // green
	            LATBbits.LATB4 = 0;
                // yellow 
	            LATAbits.LATA4 = 1;
                printf("\n Large Non-Ferrous   \r");
				LCDprint("Large NF",2,1);
                Start_Playback(0x01c120, 0x8000);
                while(play_flag);
                
            }

            // check if large ferrous 
            else if (change_value >= 84){
                
                // red 
	            LATBbits.LATB10 = 1;
                // green
	            LATBbits.LATB4 = 0;
                // yellow 
	            LATAbits.LATA4 = 0;
                printf("\n Large Ferrous   \r");
				LCDprint("Large F",2,1);
                Start_Playback(0x01546a, 0x8000);
                while(play_flag);
                
            }

            // check if small ferrous 
            else if (change_value >= 36) {
                
                // red 
	            LATBbits.LATB10 = 0;
                // green
	            LATBbits.LATB4 = 1;
                // yellow 
	            LATAbits.LATA4 = 0;
                printf("\n Small Ferrous    \r"); 
				LCDprint("Small F",2,1);
                Start_Playback(0x0045f9, 0x8000);
                while(play_flag);
                
            }

            // check if small non-ferrous 
            else if (change_value >= 12) {
                
                // red 
	            LATBbits.LATB10 = 0;
                // green
	            LATBbits.LATB4 = 0;
                // yellow 
	            LATAbits.LATA4 = 1;
                printf("\n Small Non-Ferrous    \r");
				LCDprint("Small NF",2,1);
                Start_Playback(0x00cc9c, 0x8000);
                while(play_flag);
                
            }

            // check if nothing 
            else {
                
                // red 
	            LATBbits.LATB10 = 0;
                // green
	            LATBbits.LATB4 = 0;
                // yellow 
	            LATAbits.LATA4 = 0;
                printf("\n Nothing Detected \r");
				LCDprint("Nothing",2,1);
                Start_Playback(0x00002d, 0x8000);
                while(play_flag);
                
            }
		}
		else
		{
			printf("NO SIGNAL                     \r");
			LCDprint("NO SIGNAL",2,1);
		}
		waitms(1000);
    }
}