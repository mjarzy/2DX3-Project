// 2DX3 - Deliverable (2) - Final Project
// Matthew Jarzynowski

// PN0 - Measurement
// PN1 - Status
// Bus Speed - 60 Mhz

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
#include <math.h> 

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

#define PI 3.14159
#define readings 128
#define rotations 1

int motorState = 0;
int zstep = 0;
int Y = 0;
int ycounter = 0;
double thetaRadians = 0.0;
int readingCount = 0;
int status = 0;

int totalSteps = 0;

uint16_t dev = 0x29;

// Initialize onboard LEDs
void PortN_Init(void){
	
	//Use PortN onboard LEDs
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				// Activate clock for Port N
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};	// Allow time for clock to stabilize
	GPIO_PORTN_DIR_R |= 0x03;        								// Make PN0 and PN1 output (Built-in LEDs: D1 (PN1) and D2 (PN0))
  GPIO_PORTN_AFSEL_R &= ~0x03;     								// Disable alt funct on PN0 and PN1
  GPIO_PORTN_DEN_R |= 0x03;        								// Enable digital I/O on PN0 and PN1
																									
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTN_AMSEL_R &= ~0x03;     								// Disable analog functionality on PN0 and PN1
	FlashLED1(2);																	// Flash LED D1 (Hello World)
	return;
}

// Port M init, output for the motor
void PortM_Init(void){
	
	//Use PortM pins (PM0-PM3) for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x0F;        								// configure Port M pins (PM0-PM3) as output
  GPIO_PORTM_AFSEL_R &= ~0x0F;     								// disable alt funct on Port M pins (PM0-PM3)
  GPIO_PORTM_DEN_R |= 0x0F;        								// enable digital I/O on Port M pins (PM0-PM3)
																									// configure Port M as GPIO
  GPIO_PORTM_AMSEL_R &= ~0x0F;     								// disable analog functionality on Port M	pins (PM0-PM3)	
	FlashLED2(2);	
		return;
}

void PortH_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;          // Activate the clock for Port H
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0); // Allow time for clock to stabilize
    GPIO_PORTH_DIR_R = 0b00001111;                    // Enable PH0-PH3 as outputs
    GPIO_PORTH_DEN_R = 0b00001111;                    // Enable PH0-PH3 as digital pins
}



void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void takeMeasurement(void) {
    uint16_t distance;
    uint8_t dataReady = 0;
    int status = VL53L1X_StartRanging(dev); // Ensure sensor is ranging
	
		//status = VL53L1X_SetTimingBudgetInMs(dev, 20); /* in ms possible values [20, 50, 100, 200, 500] */
		//status = VL53L1X_SetInterMeasurementInMs(dev, 25); /* in ms, IM must be > = TB */

    // Wait until the ToF sensor's data is ready
    while (dataReady == 0) {
        status = VL53L1X_CheckForDataReady(dev, &dataReady);
        SysTick_Wait10ms(5);
    }
		
    // Read the distance measurement from the ToF sensor
    status = VL53L1X_GetDistance(dev, &distance);
    if (status == 0) { // If measurement was successful
        
				int X = -distance*cos(thetaRadians);
				int Z = -distance*sin(thetaRadians);
			
				thetaRadians += (2*PI)/ readings;
				if (ycounter % 128 == 0){
					Y += 100; 
				}
				sprintf(printf_buffer, "%d , %d, %d\r\n", X,Y,Z);
        UART_printf(printf_buffer);
				ycounter++;
    }
    
    VL53L1X_ClearInterrupt(dev); // Clear interrupt to enable next measurement
    FlashLED2(1); // PN0 - D2 - Measurement
}

// Motor Spin
void spinForward(int totalMeasurements) {
    uint32_t delay = 1; // Delay between steps
		
		int stepsPerMeasure = 512 / readings;
    // Start ranging before entering the loop
    VL53L1X_StartRanging(dev);

    for (int measurement = 0; measurement < totalMeasurements; measurement++) {
        for (int step = 0; step < stepsPerMeasure; step++) {
            
						// Perform the stepping sequence
            GPIO_PORTM_DATA_R = 0b00000011;
            SysTick_Wait10ms(delay);
            GPIO_PORTM_DATA_R = 0b00000110;
            SysTick_Wait10ms(delay);
            GPIO_PORTM_DATA_R = 0b00001100;
            SysTick_Wait10ms(delay);
            GPIO_PORTM_DATA_R = 0b00001001;
            SysTick_Wait10ms(delay);
        
						totalSteps += 1;
				}

        takeMeasurement(); // Take a measurement after every 2 deg step, 4 steps
    }
		if(totalSteps >= 512) {
        totalSteps = 0; // Reset totalSteps after one full rotation
    }

    VL53L1X_StopRanging(dev); // Stop ranging after completing all measurements
}

// Return Home - Returns home based on the number of steps, 512
void returnHome(int totalSteps) {
  uint32_t delay = 1; // Delay between steps  
	
	for (int step = 0; step < totalSteps; step++) {
        // Reverse the stepping sequence for one step
        GPIO_PORTM_DATA_R = 0b00001001; // Reverse fourth energization phase
        SysTick_Wait10ms(delay);
        GPIO_PORTM_DATA_R = 0b00001100; // Reverse third energization phase
        SysTick_Wait10ms(delay);
        GPIO_PORTM_DATA_R = 0b00000110; // Reverse second energization phase
        SysTick_Wait10ms(delay);
        GPIO_PORTM_DATA_R = 0b00000011; // Reverse first energization phase
        SysTick_Wait10ms(delay);
    }
}


// "Helper" Functions for On-Board Button

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

// Global variable visible in Watch window of debugger
// Increments at least once per button press
volatile unsigned long FallingEdges = 0;

// Give clock to Port J and initalize PJ1 as Digital Input GPIO
void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;          // Activate clock for Port J
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0); // Wait for clock to stabilize
    GPIO_PORTJ_DIR_R &= ~0x03;                        // Make PJ0 and PJ1 inputs
    GPIO_PORTJ_DEN_R |= 0x03;                         // Enable digital I/O on PJ0 and PJ1
    GPIO_PORTJ_PUR_R |= 0x03;                         // Enable pull-up on PJ0 and PJ1
}



// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
    GPIO_PORTJ_IS_R &= ~0x03;       // PJ0 and PJ1 are edge-sensitive
    GPIO_PORTJ_IBE_R &= ~0x03;      // Trigger is controlled by IEV
    GPIO_PORTJ_IEV_R &= ~0x03;      // Falling edge trigger for PJ0 and PJ1
    GPIO_PORTJ_ICR_R = 0x03;        // Clear any prior interrupt
    GPIO_PORTJ_IM_R |= 0x03;        // Enable interrupt on PJ0 and PJ1

    NVIC_EN1_R = 0x80000;           // Enable interrupt 51 in NVIC
    NVIC_PRI12_R = (NVIC_PRI12_R & 0xFF0FFFFF) | 0xA000000; // Priority 5
    EnableInt();                    // Enable global Interrupt
}


// Port J IRQ Handler
void GPIOJ_IRQHandler(void){
    if(GPIO_PORTJ_RIS_R & 0x01){ // Check if PJ0 was pressed
        GPIO_PORTJ_ICR_R = 0x01; // Clear interrupt flag for PJ0
        
				//motorState = 0; // Reset motor state, turn it off
				//returnHome(totalSteps); // Return home based on the relative position
				//totalSteps = 0; // Reset totalSteps counter
				//motorState = 0; // Reset motor state, turn it off
    }
    if(GPIO_PORTJ_RIS_R & 0x02){ // Check if PJ1 was pressed
        GPIO_PORTJ_ICR_R = 0x02; // Clear interrupt flag for PJ1
        motorState = !motorState; // Toggle motor state
        if(motorState){
            FlashLED3(1); // Additional status LED, PN1, D3, to see if button is pressed 
        }else{
            FlashLED3(1); // Additional status LED, PN1, D3, to see if button is pressed 
        }
    }
}
// Signal - Sends a pulse on PH3 and can be measured using the AD2 to determine frequency
void signal(void) {
    while(1) {  // Infinite loop to continuously toggle the signal
        GPIO_PORTH_DATA_R ^= 0b00001000;  // Toggle PH3 using XOR
        SysTick_Wait(1000000);            // Delay to control toggle speed, adjust as needed
    }
}


// Main Function - Main loop
int main(void) {
    
	// Constants
	uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint8_t dataReady;
	
	// Initializations
  PLL_Init();
  SysTick_Init();
  onboardLEDs_Init();
  I2C_Init();
  UART_Init();
    
  PortM_Init(); // Motor Port initialization
  PortN_Init(); // Onboard LEDs Port initialization
  PortJ_Init(); // Onboard button port initialization
  PortJ_Interrupt_Init(); // Onboard button interrupt initialization
	PortH_Init();
	//signal(); //Uncomment to measure frequency on PortH, PIN3, PH3

	// Introduction
	UART_printf("--------------------------------------\r\n");
	UART_printf("2DX3 - Deliverable (2) - Final Project\r\n");
	UART_printf("Matthew Jarzynowski - 400455803\r\n");
	UART_printf("--------------------------------------\r\n\n");
	
	status = VL53L1X_GetSensorId(dev, &wordData);

	UART_printf("--------------------------------------\r\n");
	sprintf(printf_buffer,"Model ID: 0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// TOF booting
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("TOF Chip Booted\r\n");
	status = VL53L1X_ClearInterrupt(dev);
	
  // Initialize defaults
  status = VL53L1X_SensorInit(dev);
	Status_Check("Sensor Initization:", status);
	UART_printf("--------------------------------------\r\n\n");
	
	UART_printf("--------------------------------------\r\n");
	sprintf(printf_buffer, "Left button (PJ1), acquire data at Z = %u\r\n", zstep);
	UART_printf(printf_buffer);	
	UART_printf("--------------------------------------\r\n");
	
	status = VL53L1X_SetDistanceMode(dev, 2); // 1, short; 2, long //
    while(1) {
        if(motorState == 1) {
            
					spinForward(readings);
					returnHome(512);
          motorState = 0; // Reset motorState to wait for next button press
        }
        WaitForInt(); // Wait for an interrupt (button press)
    }
}

