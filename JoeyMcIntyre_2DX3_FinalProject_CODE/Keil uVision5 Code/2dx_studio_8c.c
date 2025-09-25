/**********************************************************************************
* 2DX3 Final Project Code 
*
* This project implements an embedded spatial mapping system using:
* - A VL53L1X Time-of-Flight (ToF) sensor (via I2C) to measure distances.
* - A stepper motor (controlled with port H) to rotate the sensor for a 360degree scan.
* - UART communication to transmit data to my PC.
* - Pushbutton input (port J) to trigger the scanning process.
* - Onboard LEDs for status indication
*
* My student-specific requirements (student number: 400520473)
* - Bus speed set to 22MHz (PLL configuration)
* - LED D1 for measurements, D2 for scanning process, D3 for additional status (motor turning)
*
* This code inclides: 
* - Initialization of ports and peripherals (motor, pushbutton, I2C, UART, LEDS).
* - Functions for motor controll
* - Functions for sensor intiialization and aquiring data
* - An interrupt service routine for handlling pushbutton events
*
* IMPORTANT: Alot of the code is reused / inspired by posted studio solutions from the 2DX3 course.
* All credit goes to the professors, IAIs and TAs who worked on these example solutions.
*
* Joey McIntyre
* Tuesday, April 1st, 2025
**********************************************************************************/

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

uint16_t	dev = 0x29;					// I2C address of the VL53L1X sensor.
int status = 0;								// Global variable to hold status return codes from sensor functions


//============================================================//
// Port Initilizations
//============================================================//

//Initialize port H for the motor
void PortH_Init(void)
{
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 	// Activate the clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};        	// Wait for clock to stabalize
	GPIO_PORTH_DIR_R = 0b00001111;       								   		// Make H (0-3) outputs
  GPIO_PORTH_DEN_R = 0b00001111;
	return;
}	
	
//============================================================//
// Interupts Initalization
//============================================================//

// Enable interrupts
void EnableInt(void)
{    
	__asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    
	__asm("    cpsid   i\n");
}

// Low power mode until interrupt occurs
void WaitForInt(void)
{    
	__asm("    wfi\n");
}

// Global variable for counting falling edge interrupts (for debugging)
volatile unsigned long FallingEdges = 0;


//============================================================//
// Pushbutton Initialization (Port J)
//============================================================//


// Initializes port J for pushbutton input
void PortJ_Init(void)
{
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;							// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};			// Wait for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    												// Set PJ1 as input 
  GPIO_PORTJ_DEN_R |= 0x02;     												// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 										//  Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;													//  Disable analog function on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;															//	Enable weak pull up resistor on PJ1
}


// Configure an enable the interrupt for PJ1
void PortJ_Interrupt_Init(void)
{
	FallingEdges = 0;             			// Initializes counter

	GPIO_PORTJ_IS_R = 0;     						// Set PJ1 to be edge-sesnitive
	GPIO_PORTJ_IBE_R = 0;   						// Dont trigger on both edges
	GPIO_PORTJ_IEV_R = 0;  							// Set to trigger on the falling edge
	GPIO_PORTJ_ICR_R = 2;      					// Clear any existing interrupt flag for PJ1
	GPIO_PORTJ_IM_R = 2;      					// Arm interrupt on PJ1 by setting proper bit in IM register
    
	NVIC_EN1_R = 0x00080000;            // Enable interrupt number 51 in the NVIC (for port J)
	
	NVIC_PRI12_R = 0xA000000; 					// Set the interrupt priority to level 5
}

//============================================================//
// I2C Initilization (VL53L1X sensor)
//============================================================//

#define I2C_MCS_ACK             0x00000008  	// Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  	// Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  	// Acknowledge Address
#define I2C_MCS_STOP            0x00000004  	// Generate STOP
#define I2C_MCS_START           0x00000002  	// Generate START
#define I2C_MCS_ERROR           0x00000002  	// Error
#define I2C_MCS_RUN             0x00000001  	// I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  	// I2C Busy
#define I2C_MCR_MFE             0x00000010  	// I2C Master Function Enable
#define MAXRETRIES              5           	// number of receive attempts before giving up

// Initialize I2C0 for communication with the VL53L1X sensor
void I2C_Init(void)
{
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// Activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// Acticate clock for port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// Wait for port B to be ready

  GPIO_PORTB_AFSEL_R |= 0x0C;           																		// Enable alternate function on PB2 and PB3 (I2C pins) - 0b00001100 (bits 2,3)
  GPIO_PORTB_ODR_R |= 0x08;             																		// Enable open drain on PB3 (SDA line)
  GPIO_PORTB_DEN_R |= 0x0C;             																		// Enable digital I/O on PB2 and PB3

  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF) + 0x00002200;    			// Configure PB2 and PB3 for I2C by setting proper values in port control register
  I2C0_MCR_R = I2C_MCR_MFE;                      														// Enable I2C master function
  I2C0_MTPR_R = 0b0000000000000101000000000111011;                       		// Configure I2C master timer for 100kpbs operation     
}

//============================================================//
// ToF Sensor XSHUT and Sesnor Initialization
//============================================================//

// Initializes port G for controlling the sensors XSHUT pin
void PortG_Init(void)
{
  // Uses PG0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                			// Activates clock for port G
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    					// Waits for port G clock to stabalize
  GPIO_PORTG_DIR_R &= 0x00;                                   	// Configure PG0 as input (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                  // Disable alt functions on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                     // Enables digital I/O on PG0
  GPIO_PORTG_AMSEL_R &= ~0x01;                                  // Disables analog functionality on PG0

  return;
}

// Control the XSHUT (shutdown) pin for the sensor
void VL53L1X_XSHUT(void)
{
  GPIO_PORTG_DIR_R |= 0x01;                                     // Set PG0 as an output to control the shutdown pin
  GPIO_PORTG_DATA_R &= 0b11111110;                              // Drive PG0 low (active-low shutdown)
  FlashAllLEDs();																								// Flash all LEDs to indicate this action
  SysTick_Wait10ms(10);
  GPIO_PORTG_DIR_R &= ~0x01;                                    // Sets PG0 back to input (HiZ) to re-enable sensor
}

//============================================================//
// Global Variables for SAensor Data Processing
//============================================================//

// Array for temp. data storage; initialized to 0xFF
uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;

uint16_t wordData;
uint16_t Distance;								// Measured distance from ToF sensor
uint16_t SignalRate;
uint16_t AmbientRate;
uint16_t SpadNum; 
uint8_t RangeStatus;							// Status code for distance measurement
uint8_t dataReady;								// Flag indicating id new data from sensor is ready
	
//============================================================//
// Motor Controll Functions
//============================================================//

int motor_pos = 0;														// Tracks motors current position

// Stops the motor
void stop_Init(void)
{
	GPIO_PORTH_DATA_R = 0b00000000;							// Clears the outputs on port H
}


// Rotates the stepper motor to move to the next scanning location
void spin()
{
	int angle = 512/32;													// Number of steps to rotate for one scanning segment
	
	for(int i=0; i < angle; i++)
	{
		//Step sequence for the motor
		GPIO_PORTH_DATA_R = 0b00001001;						
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);
		FlashLED3(1);															// Flash additional status LED to indicate a motor step
	}
	
	motor_pos += 1;															// Increments the global motor position counter
}

// Rotates the motor in reverse to its home position
void home()
{
	int val = motor_pos * 512 / 32;							// Total steps needed to return home
	for(int j=0; j<val; j++)
	{
		// Reverses step sequence to move motor other direction
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(1);
	}
	
	motor_pos = 0;															// Resets motor position to home
}

//============================================================//
// ToF Sensor Functions
//============================================================//

// Initializes the ToF Sensor
void Run_ToF_Init(void)
{	
	// Sends startup messages over UART
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX ToF deliverable %d\r\n",mynumber);
	UART_printf(printf_buffer);

	// Retrieves sensor ID to verify sensor connection
	status = VL53L1X_GetSensorId(dev, &wordData);
	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Boot the sensor (Loop until the sensor indicates that it is ready)
	while(sensorState==0)
	{
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();																									// Flash all LEDs to indicate sensor boot is a success.
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	// Clear the sensor interrupt flag
	status = VL53L1X_ClearInterrupt(dev); 
	
  // Initialize sensor with default settings
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	// Start sensor ranging process
  status = VL53L1X_StartRanging(dev);
}



//Get batch of 32 distance measurements from the sensor
void Get_Distances()
{
	Run_ToF_Init();																									// Ensures sensor is initialized and ready
	
	FlashLED2(1);																										// Flash LED2 to indicate the start of the scanning process and data transmission
	
	for(int i = 0; i < 32; i++) 
	{
		int test = 0;
		
		// Wait until the ToF sensor's data is ready
	  while (dataReady == 0)
		{
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
      VL53L1_WaitMs(dev, 5);																			// Wait 5ms between checks
			test++;
			
			// Timeout error message meaning sensor didnt become ready
			if(test == 100)
			{
				UART_printf("scan failed\r\n");
				i = (100); 																								// Force exit from loop
				dataReady = 1;
			}			
	  }
		
		dataReady = 0;																								// Resets data ready flag
		
		// No error occured: 
	  if(i != (100))
		{ 
			// Retrieve rang status and measured distance from sensor
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);	
			FlashLED1(1);																								// Flash the measurement status LED (meaning valid measurement was made)
		}
		
		// Error condition occured:
		else
		{ 
			RangeStatus = '0'; 																					// Set range status to indicate error
			status = VL53L1X_GetDistance(dev, &Distance);
		}
		
	  status = VL53L1X_ClearInterrupt(dev); 												// Clear sensor interrupt for next measurement
		
		// Transmit range status and measured distance over UART
		sprintf(printf_buffer,"%u\r\n", RangeStatus); 								// Print range status to UART
		UART_printf(printf_buffer);
		sprintf(printf_buffer,"%u\r\n", Distance); 										// Print distance data to UART
		UART_printf(printf_buffer);
		
		// If nonzero range status is detected, reinitialize the sensor
		if( RangeStatus != 0)
		{
			Run_ToF_Init();
		}
		
	  SysTick_Wait10ms(25);																					// Wait 250ms between measurements
		
		// If not the final measurement, rotate the motor to the next scan location
		if(i != 31)
		{
			spin();
		}
		
		stop_Init(); 																									// Stop motor betweeen scan positions
		SysTick_Wait10ms(25);																					// Delay for stability
	} 
	
	// Notify via UART that scanning is complete and the motor will return home if at last location
	UART_printf("rotating to home position\r\n"); 
	home(); 																												// Return motor to home position
	stop_Init(); 																										// Stop the motor

	VL53L1X_StopRanging(dev);																				// Stop sensors ranging process
}


// Interrupt service routing for port J
void GPIOJ_IRQHandler(void)
{
  //FallingEdges = FallingEdges + 1;															// OPTIONAL: increment falling edge counter for debugging
	
	SysTick_Wait10ms(1);																						// Brief delay for debounce
	Get_Distances();																								// Begin distance measurement process

	GPIO_PORTJ_ICR_R = 0x02;     																		// Clear interrupt flag for PJ1 by writing a 1 to the corresponding bit
}


//============================================================//
// Main Function
//============================================================//

int main(void) 
{
	PortH_Init(); 									// Initialize motor control port (port H)
	
	PortJ_Init();										// Initialize pushbutton port (port J)
	PortJ_Interrupt_Init();					// Configure port J interrupt
	
	PLL_Init();	 										// Initialize system clock vial PLL
	SysTick_Init(); 								// Initialize SysTick timer for delay functions
	onboardLEDs_Init(); 						// Initialize onboard LEDs for status indication
	I2C_Init(); 										// Initialize I2C communication for the sensor
	UART_Init(); 										// Initialize UART for communication with the PC
	
	Run_ToF_Init(); 								// Initialize the ToF sensor
	
	// Wait for an interrupt to be triggerd (PJ1 is pressed)
  while(1) 
	{
		WaitForInt();									// Enter low-power wait mode until an interrupt occurs
		SysTick_Wait10ms(1);					// Small delay after waking up
	}
}
