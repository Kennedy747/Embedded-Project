
/************************************************************************
*			STM32F439 Main (C Startup File)  																	*
*			Developed for the STM32																						*
*			Author: Dr. Glenn Matthews, Matthew Kennedy, Alec Harbis					*
*			Source File																												*
************************************************************************/

#include <stdint.h>
#include "boardSupport.h"
#include "main.h"

#include "stm32f439xx.h"
#include "gpioControl.h"

//Function Declarations ********************************************************//
void init(void);
void configureGPIOPins(void);
void configureUSART(void);
void configureADC1(void);

void count1Second(void);

void incrementClock(void);
void transmitCharacter(uint8_t character);
void transmitUSARTOutput(void);
int8_t getCharacter(void);
void decipherUSARTInput(int);

void USARTFanOn(void);
void USARTFanOff(void);
void USARTLightOn(void);
void USARTLightOff(void);

void triggerADCConversion(void);
int percentageHumidity(void);

void buttonCONTROL(void);
void configureTIM7(void);
void buttonCHECK(void);
void setTIM7(int);
void latchFAN(void);
void latchLIGHT(void);

void readLIGHTintensity(void);
void checkHUMIDITY(void);
void triggerOUTPUTS(void);

void incrementSimTimer7(void);

// Global Variables ************************************************************//
int fanStatus = 0;
int lightStatus = 0;

int16_t buttonState = -1;

int8_t USARTLightOffStatus = -1;

int8_t FanPressed = -1;
int8_t LightPressed = -1;
int8_t BothPressed = -1;
int8_t countVALUE = 0;
int16_t threeSecondCount = 0;

int8_t lightIntensity = -1;
int8_t lightIntensityFlag = -1;

int receivedCharacterCount = 0;
int receivedCharacters[2];

int percentageHumidityValue = 0;
int8_t HumidityFlag = -1;
uint16_t adcValue = 0;

int8_t timer7Flag = 0;
int8_t buttonValue = 0;


//******************************************************************************//
// Function: main()
// Input : None
// Return : None
// Description : Entry point into the application.
// *****************************************************************************//
int main(void)
{

	// Bring up the GPIO for the power regulators.
	boardSupport_init();
	
	init();
	
	//count1Second();
	
  while (1)
  {
		//int receivedCharacter = -1;
		
		//Reads intensity value. Sets flag if pressed
//	readLIGHTintensity();

		
		//Always Checks Buttons
		buttonCONTROL();
		
		//Checks the humidity value and determines the humidity %
//		checkHUMIDITY();
		
		triggerOUTPUTS();
		incrementSimTimer7();
	} 
}

//******************************************************************************//
// Function: incrementSimTimer6()
// Input : None
// Return : None
// Description : Increment the simulated.
// *****************************************************************************//
void incrementSimTimer7(void)
{
	
	// Check if the count is enabled.
	if((TIM7->CR1 & TIM_CR1_CEN) == TIM_CR1_CEN)
	{
		if(TIM7->CNT == TIM7->ARR)
		{
				// Timer has expired - 
				// Set the count complete flag in
				TIM7->SR |= TIM_SR_UIF;
			
				// Check to see if running in one pulse mode 
				// If so, stop the timer.
				if(TIM7->CR1 & TIM_CR1_OPM)
				{
					TIM7->CR1 &= ~(TIM_CR1_CEN);
				}
				
				// Clear the counter.
				TIM7->CNT &= ~(TIM_CNT_CNT_Msk);
				
				if((TIM7->DIER & TIM_DIER_UIE) == TIM_DIER_UIE)
				{
					// Trigger interrupt handler.
					//TIM6_DAC_IRQHandler();
				}
			
		}
		else
		{
			// Increment the timer.
			TIM7->CNT = TIM7->CNT + 1;
		}
	}
}
//******************************************************************************//
// Function: triggerOUTPUTS()
// Input : None
// Return : None
// Description : Reads the flags and outputs based on them
// *****************************************************************************//
void triggerOUTPUTS()
{
	//Checks Light Status Flag and Writes Output
	if(lightStatus == 1)
	{
		//And to turn off because active low outputs
		GPIOB->ODR &= ~GPIO_ODR_OD0;
	}
	else if(lightStatus != 1)
	{
		GPIOB->ODR |= GPIO_ODR_OD0;
	}
		
	//Checks Fan Status Flag and Writes Output
	if(fanStatus == 1 || HumidityFlag == 1)
	{
		//And to turn of cause active low outputs
		GPIOA->ODR &= ~GPIO_ODR_OD10;
	}
	else if(fanStatus != 1)
	{
		GPIOA->ODR |= GPIO_ODR_OD10;
	}
	
}

//******************************************************************************//
// Function: buttonCONTROL()
// Input : None
// Return : None
// Description : Checks the status of the buttons and checks if they have been pushed for 1 second
// *****************************************************************************//
void buttonCONTROL()
{
	//Read Button Inputs (only PA8,PA9)
	buttonState = (GPIOA->IDR & 0x300); 
		
	//If Light Button Is Pressed
	if(buttonState == 256)
	{
	LightPressed = 1;
	buttonValue = 1;
	}
	else {LightPressed = 0;}
	
	//If Fan Button Is Pressed
	if(buttonState == 512)
	{
	FanPressed = 1;
	buttonValue = 2;
	}	
	else {FanPressed = 0;}
	
	//If Both Buttons ARE Pressed
	if(buttonState == 0)
	{
	BothPressed = 1;
	buttonValue = 3;
	}
	else {BothPressed = 0;}
	
	buttonCHECK();
}


//******************************************************************************//
// Function: buttonCHECK()
// Input : None
// Return : None
// Description : Latches the FAN button
// *****************************************************************************//
void buttonCHECK()
{
	//Checks to see if a button has been pressed
	if((FanPressed || LightPressed || BothPressed) == 1 || (TIM7->SR & TIM_SR_UIF))
	{
		
		
		//If the timer hasn't started then start it and UIF flag hasn't been set
		//Otherwise do nothing
		
		//Starts 1s timer
		if(timer7Flag == 0)
		{
		setTIM7(0x4C2C0);
		timer7Flag = 1;
		}

		//If the timer has expired and one of the buttons is still pressed then set outputs
		if((TIM7->SR & TIM_SR_UIF) && (timer7Flag == 1))
		{

			//Checks to only latch outputs one button is released
			if((FanPressed || LightPressed || BothPressed) == 0)
			{
				//Clears timer 7 flag
				timer7Flag = 0;
				
				//Clears Status Flag
				TIM7->SR &= ~(TIM_SR_UIF);
				
				
				//Determines which output needs to be set
				
				//Fan needs to be set
				if((buttonValue == 2 || buttonValue == 3))
				{
						latchFAN();
						/*
						if(HumidityFlag == 1)
						{
							//Starts 10ms timer
							setTIM7(0x4C2C0);
							//Clear Status Flag
							//TIM7->SR &= ~(TIM_SR_UIF);
							
							//Counts to 30seconds (30000 10ms chunks)
							threeSecondCount = threeSecondCount + 1;
							if(((TIM7->SR & TIM_SR_UIF) == 1) && threeSecondCount == 30000)
							{
								latchFAN();
							}
						
						}
						*/
					}
					
					
					//Light needs to be set
					//Also checks the lightIntensityFlag. If not set then don't effect light
					
					//if((LightPressed == 1 || BothPressed == 1) && lightIntensityFlag == 0)
					if((buttonValue == 1 || buttonValue == 0))
					{
						if(USARTLightOffStatus == -1){
						latchLIGHT();
						}
					}
				USARTLightOffStatus = -1;
			}
		}
	}
	
	//Else if buttons not pressed 
	if((FanPressed || LightPressed || BothPressed) == 0)
	{
		//timer7Flag = 0;
		//if((LightPressed && lightStatus) || (FanPressed && fanStatus)) != 0){
		//latchLIGHT();
		//latchFAN();
	}
}
//}

//******************************************************************************//
// Function: latchFAN()
// Input : None
// Return : None
// Description : Latches the FAN button
// *****************************************************************************//
void latchFAN()
{
	if(fanStatus == 1) {fanStatus = 0;}
	else if(fanStatus == 0) {fanStatus = 1;}
}


//******************************************************************************//
// Function: latchLIGHT()
// Input : None
// Return : None
// Description : Latches the FAN button
// *****************************************************************************//
void latchLIGHT()
{
	if(lightStatus == 1) {lightStatus = 0;}
	else if(lightStatus == 0) {lightStatus = 1;}
}

//******************************************************************************//
// Function: setTIM7()
// Input : None
// Return : None
// Description : Resets the timer to 10ms and starts it
// *****************************************************************************//
void setTIM7(int count)
{
	//Clear the reload register
	TIM7->ARR &= ~(TIM_ARR_ARR_Msk);
	
	//Set Count value
	TIM7->ARR |= count; //Counts to this value to give
	//This is where I'm gonna change the duty cycle (<2^16-1)
	
	//Clear Status Flag
	TIM7->SR &= ~(TIM_SR_UIF);
	
	//Start Timer
	TIM7->CR1 |= TIM_CR1_CEN;	
		
}


//******************************************************************************//
// Function: configureGPIOPorts()
// Input : None
// Return : None
// Description : Set up the GPIO Port Pins.
// *****************************************************************************//
void configureGPIOPins(void) {
	
	//PB0, PA10, PA9, PA8, PA3, PF10
	
	GPIO_Config portConfig;
	
	portConfig.port = GPIOB;
	portConfig.pin = Pin0;
	portConfig.mode = GPIO_Output;
	portConfig.pullUpDown = GPIO_No_Pull;
	portConfig.outputType = GPIO_Output_PushPull;
	portConfig.speed = GPIO_2MHz;
	
	// PB0
	gpio_configureGPIO(&portConfig);

	// PA10
	portConfig.port = GPIOA;
	portConfig.pin = Pin10;
	gpio_configureGPIO(&portConfig);
	
	// PA9
	portConfig.pin = Pin9;
	portConfig.mode = GPIO_Input;
	gpio_configureGPIO(&portConfig);
	
	// PA8
	portConfig.pin = Pin8;
	gpio_configureGPIO(&portConfig);

	// PA3
	portConfig.pin = Pin3;
	gpio_configureGPIO(&portConfig);
	
	// PF10
	portConfig.port = GPIOF;
	portConfig.pin = Pin10;
	portConfig.mode = GPIO_Analog;
	gpio_configureGPIO(&portConfig);
}

//******************************************************************************//
// Function: configureADC1()
// Input : None
// Return : None
// Description : Configure the ADC.
// *****************************************************************************//
void configureADC1(void){
	//Turn off battery sensing
	ADC123_COMMON->CCR &= ~(ADC_CCR_VBATE);\
	//Disable Temp sensor 
	ADC123_COMMON->CCR &= ~(ADC_CCR_TSVREFE);
	//Sets ADC Prescaler to divide by 2 (11b)
	ADC123_COMMON->CCR &= ~(ADC_CCR_ADCPRE);
	ADC123_COMMON->CCR |= (0x03 << ADC_CCR_ADCPRE_Pos);
	//Turn off scan and set resolution to 12bit (00b)
	ADC3->CR1 &= ~((ADC_CR1_SCAN) | (0x03 << ADC_CR1_RES_Pos));
	//Continuous mode, and sofeware start is off. Allignment set to right
	ADC3->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_ALIGN | ADC_CR2_SWSTART);
	
	//Set to channel 8 which is where ADC3 is
	ADC3->SQR3 &= ~(ADC_SQR3_SQ1_Msk);
	ADC3->SQR3 |= 0x08;
	//Set for only One conversion
	ADC3->SQR1 &= ~(ADC_SQR1_L);
	//Set sample time to 56 cycles (11b = 0x03Hex)
	ADC3->SMPR2 &= ~(ADC_SMPR2_SMP0_Msk);
	ADC3->SMPR2 |= 0x03 << (ADC_SMPR2_SMP0_Pos);
	
	ADC3->CR2 |= ADC_CR2_ADON;
}

//******************************************************************************//
// Function: configureTIM7()
// Input : None
// Return : None
// Description : Configures the timer 7 for button press checking
// *****************************************************************************//
void configureTIM7()
{
//Turning off timer
	TIM7->CR1 &= ~(TIM_CR1_CEN);
	
	//Clear Prescaler
	TIM7->PSC &= ~(TIM_PSC_PSC_Msk);
	
	//Set Prescaler to 25+1 to make a 10ms Timer
	TIM7->PSC |= 26; //make 26
	
	//Clear the reload register
	TIM7->ARR &= ~(TIM_ARR_ARR_Msk);
	
	//Set Count value
	//TIM7->ARR |= 0x1; //Counts to this value
	
	//Set Timer to count be poled after each count is reached
	TIM7->CR1 |= TIM_CR1_OPM;
	
	//Start Timer
	//TIM7->CR1 |= TIM_CR1_CEN;	
}

//******************************************************************************//
// Function: initRCC()
// Input : None
// Return : None
// Description : Configure the RCC for USART3, TIM6, ADC1 and GPIO Ports.
// *****************************************************************************//
void init(void){
	//TIM6 & TIM7 & USART3 RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_USART3EN;
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST | RCC_APB1RSTR_USART3RST;
	__ASM("NOP");
	__ASM("NOP");
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM6RST) & ~(RCC_APB1RSTR_TIM7RST) & ~(RCC_APB1RSTR_USART3RST);
	__ASM("NOP");
	__ASM("NOP");
	
	//GPIO RCC
	RCC->AHB1ENR = RCC->AHB1ENR | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | 
	RCC_AHB1ENR_GPIOFEN;
	RCC->AHB1RSTR = RCC->AHB1RSTR | RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOBRST | 
	RCC_AHB1RSTR_GPIOFRST;
	__ASM("NOP");
	__ASM("NOP");
	RCC->AHB1RSTR = RCC->AHB1RSTR & ~(RCC_AHB1RSTR_GPIOARST) 
	& ~(RCC_AHB1RSTR_GPIOBRST) & ~(RCC_AHB1RSTR_GPIOFRST);
	__ASM("NOP");
	__ASM("NOP");
	
	//ADC RCC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
	__ASM("NOP");
	__ASM("NOP");
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADCRST);
	__ASM("NOP");
	__ASM("NOP");
	
	configureGPIOPins();
	//configureUSART();
	configureADC1();
	configureTIM7();
}
