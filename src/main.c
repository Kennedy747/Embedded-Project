
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
void count1second(void);

void transmitCharacter(uint8_t character);
void transmitUSARTOutput(void);
void usartReceivingControl(void);
int8_t getCharacter(void);
void decipherUSARTInput(void);

void USARTFanOn(void);
void USARTFanOff(void);
void USARTLightOn(void);
void USARTLightOff(void);

void buttonCONTROL(void);
void configureTIM7(void);
void buttonCHECK(void);
void setTIM7(int);
void latchFAN(void);
void latchLIGHT(void);

void checkHUMIDITY(void);
void triggerOUTPUTS(void);

void incrementSimTimer5(void);
void incrementSimTimer6(void);
void incrementSimTimer7(void);

// Global Variables ************************************************************//
int fanStatus = 0;
int lightStatus = 0;

int16_t buttonState = -1;

int8_t USARTLightOffStatus = -1;

int8_t FanPressed = -1;
int8_t LightPressed = -1;
int8_t BothPressed = -1;
//int8_t countVALUE = 0;
//int16_t threeSecondCount = 0;

int8_t lightIntensity = -1;

int receivedCharacterCount = 0;
int receivedCharacters[2];
int receivedCharacter = -1;

int percentageHumidityValue = 0;
int8_t HumidityFlag = -1;
int8_t HumidityFlagPast = -1;
uint16_t adcValue = 0;

int8_t timer7Flag = 0;
int8_t buttonValue = 0;
int8_t timeOutFlag = 0;

int ButtonBlock = -1;
int8_t Humidity30secOFF = 0;

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
	
  while (1)
  {
		
		// Button Blocking Timer
		if((TIM5->SR & TIM_SR_UIF) == 1){
			TIM5->CR1 &= ~(TIM_CR1_CEN);
			TIM5->SR &= ~(TIM_SR_UIF);
			ButtonBlock = -1;
		}
		
		// Checks for new usart3 input and ensures the header is received first.
		usartReceivingControl();
		
		//buttonCONTROL & triggerOUTPUTS are copied from the working Just buttons project//
		
		//Checks the humidity value and determines the humidity %
		checkHUMIDITY();
		
		//Always Checks Buttons
		buttonCONTROL();
		
		//Latches outputs
		triggerOUTPUTS();
				
		if((TIM6->SR & TIM_SR_UIF) == 1)
		{
			//Transmit Data 
			transmitUSARTOutput();
				
			// Restart timer
			TIM6->SR &= ~(TIM_SR_UIF);
			TIM6->CR1 |= TIM_CR1_CEN;
		} 
		
		//Sim Timers
		incrementSimTimer6();
		incrementSimTimer7();
		incrementSimTimer5();
  }
}

//******************************************************************************//
// Function: uartReceivingControl()
// Input : None
// Return : None
// Description : Assembles the received characters into an array and also recognises the header
// *****************************************************************************//
void usartReceivingControl() {
	receivedCharacter = -1;
	receivedCharacter = getCharacter();
		
		if (receivedCharacter != -1){
			if(receivedCharacterCount == 0 && receivedCharacter != '!'){
				//do nothing
			}else{
				receivedCharacters[receivedCharacterCount] = receivedCharacter;
				receivedCharacterCount++;
			}
		}
		
		if (receivedCharacterCount == 2){
			
			if (receivedCharacters[1] != '!'){
				decipherUSARTInput();
			}
			
			receivedCharacterCount = 0;		
		}
}

//******************************************************************************//
// Function: getCharacter()
// Input : None
// Return : int8_t
// Description : Determines the validity of a character
// *****************************************************************************//
int8_t getCharacter(void){
	int8_t receivedCharacter = -1;
	int8_t validCharacter = -1;
	
	if(USART3->SR & USART_SR_RXNE){
		receivedCharacter = USART3->DR;
		
		// Light On, Fan On = 12
		// Light On, Fan Off = 8
		// Light Off, Fan On = 4
		// Light Off, Fan Off = 0
		
		if(receivedCharacter == 12 || receivedCharacter == 8 || receivedCharacter == 4 || receivedCharacter == 0 || receivedCharacter == '!'){
			validCharacter = 1;
		}
//		else if ((receivedCharacter == 0x0D) || (receivedCharacter == 0x0A)){
//			validCharacter = 1;
//		}
		
		if (validCharacter == -1){
			receivedCharacter = -1;
		}
	}
	
	return receivedCharacter;
}

//******************************************************************************//
// Function: decipherUSARTInput()
// Input : None
// Return : None
// Description : Transmits HMS information through USART.
// *****************************************************************************//
void decipherUSARTInput(){
	
		// Light On, Fan On = 12
		// Light On, Fan Off = 8
		// Light Off, Fan On = 4
		// Light Off, Fan Off = 0
	
	switch(receivedCharacters[1]){
		case 12:
			USARTLightOn();
			USARTFanOn();
			break;
		case 8:
			USARTLightOn();
			USARTFanOff();
			break;
		case 4:
			USARTLightOff();
			USARTFanOn();
			break;
		case 0:
			USARTLightOff();
			USARTFanOff();
			break;
	}
}
void USARTFanOn(void){
	if(fanStatus == 1){return;}
	
	if(HumidityFlag == 1){return;} 
	else {
		fanStatus = 1;
	}
}
void USARTFanOff(void){
	if(fanStatus == 0){return;} 
	
	if(HumidityFlag == 1){return;} 
	else {
		fanStatus = 0;
	}
}
void USARTLightOn(void){
	if(lightStatus == 1){return;}
	
	lightStatus = 1;
	
	ButtonBlock = 1;
	TIM5->CR1 |= TIM_CR1_CEN;
}
void USARTLightOff(void){
	if(lightStatus == 0){return;}
	
	lightStatus = 0;
	
	ButtonBlock = 1;
	TIM5->CR1 |= TIM_CR1_CEN;
}

//******************************************************************************//
// Function: transmitUSARTOutput()
// Input : None
// Return : None
// Description : Transmits HMS information through USART.
// *****************************************************************************//
void transmitUSARTOutput(void){
	//Transmits '!' ASCII
	transmitCharacter(0x21);
	//Transmits Humidity Percentage
	transmitCharacter((int)percentageHumidityValue);
	//Transmits Light status and fan status as two seperate bits
	transmitCharacter((lightStatus<<1)|fanStatus);
	
	
	
	/* 
	SIMtransmitCharacter(0x21);
	SIMtransmitCharacter((int)percentageHumidityValue);
	SIMtransmitCharacter((lightStatus<<1)|fanStatus);
	*/
	}

	
//******************************************************************************//
// Function: transmitCharacter()
// Input : None
// Return : None
// Description : Transmits a character.
// *****************************************************************************//
void transmitCharacter(uint8_t character){
	while((USART3->SR & USART_SR_TXE) == 0x00);
	USART3->DR = character;
	while((USART3->SR & USART_SR_TC) == 0x00);
}


//******************************************************************************//
// Function: checkHUMIDITY
// Input : None
// Return : None
// Description : Checks of humidity is above 75% if so then the fan switch only turns fan off for 30 seeconds
// *****************************************************************************//
void checkHUMIDITY()
{

	/*
	//else if(((HumidityFlag == 0) && (timeOutFlag == 1) && Humidity30secOFF == 0) || (timer7Flag == 0))
	else if(HumidityFlag == 0 && buttonValue != 2)
	//else if((HumidityFlag == 0) && ((timer7Flag == 0) || (timeOutFlag == 0)))
	{
		//Turn off fan
		fanStatus = 0;
		//triggerOUTPUTS();
	}
	*/
							
	//start ADC

	ADC3->CR2 |= ADC_CR2_SWSTART;
	
	//Wait for conversion to complete
	//Is cleared by reading the data register
	//Thus will only get stuck her for a short period
	while((ADC3->SR & ADC_SR_EOC) == 0x00);
	
	//Read Data register (Only bottom 16bits)
	adcValue = (ADC3->DR & 0xFFFF);
	
	percentageHumidityValue = ((adcValue*100)/4095);
	if(percentageHumidityValue > 75)
	{
		HumidityFlagPast = HumidityFlag;
		HumidityFlag = 1;
	}
	else
	{
		HumidityFlagPast = HumidityFlag;
		HumidityFlag = 0;
	}
	
		//if(HumidityFlag == 1 && Humidity30secOFF != 1 && (timer7Flag == 1) && (timeOutFlag == 1) )
	//if(HumidityFlag == 1 && ((timer7Flag == 1) || (timeOutFlag == 1)))
	if(HumidityFlag == 1)	
	{
		//Turn on fan
		fanStatus = 1;
		//triggerOUTPUTS();
	}
	if(HumidityFlag == 0)
	{
		if(HumidityFlagPast == 1)
		{
			fanStatus = 0;
		}
	}
}


//******************************************************************************//
// Function: sampleSimADC()
// Input : Simulated ADC value.
// Return : Simulated ADC value.
// Description : Perform a simulated ADC read.
// *****************************************************************************//
uint16_t sampleSimADC(uint16_t simulatedValue)
{
	uint16_t currentSample = 0;
	
	// Trigger and ADC conversion
	ADC1->CR2 |= ADC_CR2_SWSTART;
	
	// In simulation, set the conversion complete flag.
	ADC1->SR |= ADC_SR_EOC;
	
	// Wait for the conversion to complete.
	while((ADC1->SR & ADC_SR_EOC) == 0x00);
	
	// Get the value from the ADC
	currentSample = (ADC1->DR & 0x0000FFFF);
	
	// If we are running in the simulator, then just returned the passed
	// in value.
	currentSample = simulatedValue;
	
	// Return the raw ADC value.
	return currentSample;
	
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
	if(fanStatus == 1)
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
	//Sets Humidity Lock
	
	
	//Read Button Inputs (only PA8,PA9)
	buttonState = (GPIOA->IDR & 0x300);
	//Reads PA3 which is Light Intensity Sensor
	lightIntensity = (GPIOA->IDR & 8);
	
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
	if(((TIM7->CNT & TIM_CNT_CNT) < 0x4C2C0) && ((TIM7->CNT & TIM_CNT_CNT) != 0)
		&& ((FanPressed || LightPressed || BothPressed) == 0) )
	{
		//Clears Flag
		timeOutFlag = 0;		
	}
	else{timeOutFlag = 1;}
		
	
	//Checks to see if a button has been pressed
	if((FanPressed || LightPressed || BothPressed) == 1 || (TIM7->SR & TIM_SR_UIF))
	{
		//timeOutFlag = 1;
		
		//If the timer hasn't started then start it and UIF flag hasn't been set
		//Otherwise do nothing
		
		//Starts 1s timer
		if(timer7Flag == 0)
		{
		setTIM7(10000);
		timer7Flag = 1;
		}
		
		
		//If the timer has expired and the timer was started
		if((TIM7->SR & TIM_SR_UIF) && (timer7Flag == 1) && timeOutFlag == 1)
		{
			
			//Checks to only latch outputs once button is released
			//if(((FanPressed || LightPressed || BothPressed) == 0) && (TIM7->CNT & TIM_CNT_CNT) == 0)
			if(((FanPressed || LightPressed || BothPressed) == 0))
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
						
						if(HumidityFlag == 1)
						{
							//Set Humidity30secOFF Flag for output blocking
							Humidity30secOFF = 1;
							//Trigger the Fan to compensate for latch above given it 
							// will wait for the 30 seconds to expire
							triggerOUTPUTS();
							
							//Starts 30s timer (30*10000)
							setTIM7(30000);
							
							//Hults here and waits for 30seonds to finish
							while((TIM7->SR & TIM_SR_UIF) == 0);
							//Turn off fan
							latchFAN();
							//Clear Humidity30secOFF Flag for output blocking
							Humidity30secOFF = 0;
							
						}
						
				}
				//Light needs to be set
				//Also checks the lightIntensityFlag. If not set then don't effect light
				
				//if((LightPressed == 1 || BothPressed == 1) && lightIntensityFlag == 0)
				if((buttonValue == 1 || buttonValue == 0) && (lightIntensity == 8))
				{
					if (ButtonBlock == -1){latchLIGHT();}
				}
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

	if(timeOutFlag == 0)
	{
	//Turn off timer
	TIM7->CR1 &= ~TIM_CR1_CEN;
		
	//Clear the reload register
	TIM7->ARR &= ~(TIM_ARR_ARR_Msk);

	//Clear the count register
	TIM7->CNT &= ~TIM_CNT_CNT;
		
	//Set Count value
	TIM7->ARR |= 0x4C2C0; //Counts to this value to give
	//This is where I'm gonna change the duty cycle (<2^16-1)
	
	//Clear Status Flag
	TIM7->SR &= ~(TIM_SR_UIF);
	
	//Sets Flags
	timer7Flag = 0;
	timeOutFlag = 1;
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
// Function: incrementSimTimer5()
// Input : None
// Return : None
// Description : Increment the simulated.
// *****************************************************************************//
void incrementSimTimer5(void)
{
	
	// Check if the count is enabled.
	if((TIM5->CR1 & TIM_CR1_CEN) == TIM_CR1_CEN)
	{
		if(TIM5->CNT == TIM5->ARR)
		{
				// Timer has expired - 
				// Set the count complete flag in
				TIM5->SR |= TIM_SR_UIF;
			
				// Check to see if running in one pulse mode 
				// If so, stop the timer.
				if(TIM5->CR1 & TIM_CR1_OPM)
				{
					TIM5->CR1 &= ~(TIM_CR1_CEN);
				}
				
				// Clear the counter.
				TIM5->CNT &= ~(TIM_CNT_CNT_Msk);
				
				if((TIM5->DIER & TIM_DIER_UIE) == TIM_DIER_UIE)
				{
					// Trigger interrupt handler.
					//TIM6_DAC_IRQHandler();
				}
			
		}
		else
		{
			// Increment the timer.
			TIM5->CNT = TIM5->CNT + 1;
		}
	}
}

//******************************************************************************//
// Function: incrementSimTimer6()
// Input : None
// Return : None
// Description : Increment the simulated.
// *****************************************************************************//
void incrementSimTimer6(void)
{
	
	// Check if the count is enabled.
	if((TIM6->CR1 & TIM_CR1_CEN) == TIM_CR1_CEN)
	{
		if(TIM6->CNT == TIM6->ARR)
		{
				// Timer has expired - 
				// Set the count complete flag in
				TIM6->SR |= TIM_SR_UIF;
			
				// Check to see if running in one pulse mode 
				// If so, stop the timer.
				if(TIM6->CR1 & TIM_CR1_OPM)
				{
					TIM6->CR1 &= ~(TIM_CR1_CEN);
				}
				
				// Clear the counter.
				TIM6->CNT &= ~(TIM_CNT_CNT_Msk);
				
				if((TIM6->DIER & TIM_DIER_UIE) == TIM_DIER_UIE)
				{
					// Trigger interrupt handler.
					//TIM6_DAC_IRQHandler();
				}
			
		}
		else
		{
			// Increment the timer.
			TIM6->CNT = TIM6->CNT + 1;
		}
	}
}


//******************************************************************************//
// Function: incrementSimTimer7()
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

//*********************************************************************************************************************************************//
//*********************************************************************************************************************************************//

//CONFIG SECTION//

//******************************************************************************//
// Function: configureUSART()
// Input : None
// Return : None
// Description : Configures the alternate function register and sets up Usart.
// *****************************************************************************//
void configureUSART(void) {
	//==Setup Alternate Function==================================================//
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL11_Msk | GPIO_AFRH_AFSEL10_Msk);
	GPIOB->AFR[1] |= (0x07 << GPIO_AFRH_AFSEL11_Pos) | (0x07 << GPIO_AFRH_AFSEL10_Pos);
	
	USART3->CR1 &= ~(USART_CR1_OVER8);
	USART3->BRR &= 0xFFFF0000;
	
	//Set Baud Rate 9600
	USART3->BRR |= (0x16 << USART_BRR_DIV_Mantissa_Pos) | (0x4E << USART_BRR_DIV_Fraction_Pos);

	USART3->CR1 &= ~(USART_CR1_M);
	
	USART3->CR2 &= ~(USART_CR2_STOP_Msk);
	USART3->CR2 |= (0x00 << USART_CR2_STOP_Pos);
	
	USART3->CR1 &= ~(USART_CR1_PCE);
	
	USART3->CR2 &= ~(USART_CR2_CLKEN | USART_CR2_CPOL | USART_CR2_CPHA);
	USART3->CR2 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
	
	USART3->CR1 |= (USART_CR1_TE | USART_CR1_UE | USART_CR1_RE);
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
	
	GPIOB->MODER &= ~(GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE10);
	GPIOB->MODER |= (0x02 << GPIO_MODER_MODE11_Pos) | (0x02 << GPIO_MODER_MODE10_Pos);
	GPIOB->AFR[1] &=~(GPIO_AFRH_AFSEL11_Msk | GPIO_AFRH_AFSEL10_Msk);
	GPIOB->AFR[1] |= (0x07 << GPIO_AFRH_AFSEL11_Pos | 0x07 << GPIO_AFRH_AFSEL10_Pos);
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
// Function: count1Second()
// Input : None
// Return : None
// Description : Counts 1 second.
// *****************************************************************************//
void count1Second() {
	TIM6->CR1 &= ~TIM_CR1_CEN;
	TIM6->PSC &= ~(TIM_PSC_PSC_Msk);
	TIM6->PSC |= 4199; 
	
	TIM6->ARR &= ~(TIM_ARR_ARR_Msk);
	TIM6->ARR |= 10000;
	TIM6->CR1 |= TIM_CR1_OPM;
	TIM6->CR1 |= TIM_CR1_CEN;
}

//******************************************************************************//
// Function: configureTIM5()
// Input : None
// Return : None
// Description : Configures the timer 5 
// *****************************************************************************//
void configureTIM5(){
	TIM5->CR1 &= ~(TIM_CR1_CEN);
	TIM5->CR1 |= TIM_CR1_OPM;
	TIM5->CR1 &= ~(TIM_CR1_DIR); // set to up counter
	TIM5->PSC &= ~(TIM_PSC_PSC_Msk);
	TIM5->PSC |= 4199;
	
	//Clear the reload register
	TIM5->ARR &= ~(TIM_ARR_ARR_Msk);
	TIM5->ARR |= 10000;
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
	
	//Set Prescaler to 4199 to make a 1s Timer
	TIM7->PSC |= 4199;
	
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
	//TIM5 & TIM6 & TIM7 & USART3 RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_USART3EN;
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM5RST | RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST | RCC_APB1RSTR_USART3RST;
	__ASM("NOP");
	__ASM("NOP");
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM5RST) & ~(RCC_APB1RSTR_TIM6RST) & ~(RCC_APB1RSTR_TIM7RST) & ~(RCC_APB1RSTR_USART3RST);
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
	configureUSART();
	configureADC1();
	configureTIM7();
	configureTIM5();
	count1Second();
}
