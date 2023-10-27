// DragRaceStarter.c
// Starter file for CECS346 Project 2
#include <stdint.h> // C99 data types
//#include "tm4c123gh6pm.h"

//Port A initialization
#define GPIO_PORTA_DIR_R    	(*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_IS_R     	(*((volatile unsigned long *)0x40004404))
#define GPIO_PORTA_IBE_R  	  (*((volatile unsigned long *)0x40004408))
#define GPIO_PORTA_IEV_R    	(*((volatile unsigned long *)0x4000440C))
#define GPIO_PORTA_IM_R     	(*((volatile unsigned long *)0x40004410))
#define GPIO_PORTA_RIS_R    	(*((volatile unsigned long *)0x40004414))
#define GPIO_PORTA_ICR_R    	(*((volatile unsigned long *)0x4000441C))
#define GPIO_PORTA_AFSEL_R  	(*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_PUR_R    	(*((volatile unsigned long *)0x40004510))
#define GPIO_PORTA_PDR_R    	(*((volatile unsigned long *)0x40004514))
#define GPIO_PORTA_DEN_R    	(*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AMSEL_R  	(*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R   	(*((volatile unsigned long *)0x4000452C))
#define GPIO_PORTA_DATA_R   	(*((volatile unsigned long *)0x400043FC))
#define GPIO_PORTA_CR_R     	(*((volatile unsigned long *)0x40004524))
#define NVIC_PRI0_R           (*((volatile unsigned long *)0xE000E400))



//initialization of registers for port B
#define GPIO_PORTB_DATA_BITS_R  ((volatile unsigned long *)0x40005000)
#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_IS_R         (*((volatile unsigned long *)0x40005404))
#define GPIO_PORTB_IBE_R        (*((volatile unsigned long *)0x40005408))
#define GPIO_PORTB_IEV_R        (*((volatile unsigned long *)0x4000540C))
#define GPIO_PORTB_IM_R         (*((volatile unsigned long *)0x40005410))
#define GPIO_PORTB_RIS_R        (*((volatile unsigned long *)0x40005414))
#define GPIO_PORTB_MIS_R        (*((volatile unsigned long *)0x40005418))
#define GPIO_PORTB_ICR_R        (*((volatile unsigned long *)0x4000541C))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DR2R_R       (*((volatile unsigned long *)0x40005500))
#define GPIO_PORTB_DR4R_R       (*((volatile unsigned long *)0x40005504))
#define GPIO_PORTB_DR8R_R       (*((volatile unsigned long *)0x40005508))
#define GPIO_PORTB_ODR_R        (*((volatile unsigned long *)0x4000550C))
#define GPIO_PORTB_PUR_R        (*((volatile unsigned long *)0x40005510))
#define GPIO_PORTB_PDR_R        (*((volatile unsigned long *)0x40005514))
#define GPIO_PORTB_SLR_R        (*((volatile unsigned long *)0x40005518))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_LOCK_R       (*((volatile unsigned long *)0x40005520))
#define GPIO_PORTB_CR_R         (*((volatile unsigned long *)0x40005524))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define GPIO_PORTB_ADCCTL_R     (*((volatile unsigned long *)0x40005530))
#define GPIO_PORTB_DMACTL_R     (*((volatile unsigned long *)0x40005534))
	

//port D initialization
#define GPIO_PORTD_DATA_BITS_R  ((volatile unsigned long *)0x40007000)
#define GPIO_PORTD_DATA_R       (*((volatile unsigned long *)0x400073FC))
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_IS_R         (*((volatile unsigned long *)0x40007404))
#define GPIO_PORTD_IBE_R        (*((volatile unsigned long *)0x40007408))
#define GPIO_PORTD_IEV_R        (*((volatile unsigned long *)0x4000740C))
#define GPIO_PORTD_IM_R         (*((volatile unsigned long *)0x40007410))
#define GPIO_PORTD_RIS_R        (*((volatile unsigned long *)0x40007414))
#define GPIO_PORTD_MIS_R        (*((volatile unsigned long *)0x40007418))
#define GPIO_PORTD_ICR_R        (*((volatile unsigned long *)0x4000741C))
#define GPIO_PORTD_AFSEL_R      (*((volatile unsigned long *)0x40007420))
#define GPIO_PORTD_DR2R_R       (*((volatile unsigned long *)0x40007500))
#define GPIO_PORTD_DR4R_R       (*((volatile unsigned long *)0x40007504))
#define GPIO_PORTD_DR8R_R       (*((volatile unsigned long *)0x40007508))
#define GPIO_PORTD_ODR_R        (*((volatile unsigned long *)0x4000750C))
#define GPIO_PORTD_PUR_R        (*((volatile unsigned long *)0x40007510))
#define GPIO_PORTD_PDR_R        (*((volatile unsigned long *)0x40007514))
#define GPIO_PORTD_SLR_R        (*((volatile unsigned long *)0x40007518))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))
#define GPIO_PORTD_LOCK_R       (*((volatile unsigned long *)0x40007520))
#define GPIO_PORTD_CR_R         (*((volatile unsigned long *)0x40007524))
#define GPIO_PORTD_AMSEL_R      (*((volatile unsigned long *)0x40007528))
#define GPIO_PORTD_PCTL_R       (*((volatile unsigned long *)0x4000752C))
#define GPIO_PORTD_ADCCTL_R     (*((volatile unsigned long *)0x40007530))
#define GPIO_PORTD_DMACTL_R     (*((volatile unsigned long *)0x40007534))
	
#define GPIOS_PORTD     				(*((volatile unsigned long *)0x40007404))
#define GPIOBE_PORTD     				(*((volatile unsigned long *)0x40007408))
#define GPIOEV_PORTD     				(*((volatile unsigned long *)0x4000740C))
	

#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))

#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
	
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))

#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))

//check this
#define SYSCTL_RCC2_R           (*((volatile unsigned long *)0x400FE070))
	
#define NVIC_EN0_R							(*((volatile unsigned long *)0xE000E100))

// Function Prototypes (external functions from startup.s)
extern void DisableInterrupts(void); // Disable interrupts
extern void EnableInterrupts(void);  // Enable interrupts
extern void WaitForInterrupt(void);  // Go to low power mode while waiting for the next interrupt

// Function Prototypes
void Sensors_Init(void); // Initialize the left and right push buttons
void Reset_Init(void);   // Initialize reset button
void Lights_Init(void);  // Initialize 8 race lights 
void SysTick_Init(void); // Initialize SysTick timer with interrupt enabled 
void SysTick_Start(uint32_t reload); // Start systick timer with specified reload value.
void System_Init(void);
void GPIOPortD_Handler(void);
void GPIOPortA_Handler(void);
void SysTick_Handler(void);

//TODO: Bit addresses for two sensors, 8 color lights, and one reset button 
#define SENSORS 								(*((volatile unsigned long *) 0x4000700C)) // bit addresses for  2 sensors on PD0,1 
#define LIGHTS              		(*((volatile unsigned long *) 0x400053FC)) // bit addresses for 8 Race Lights PB0-7
#define RESET                   (*((volatile unsigned long *) 0x40004008)) // bit address for one reset button PA2
 
// FSM 
struct State { 
	uint8_t Out;
	uint8_t Time;     // multiples of 0.5 second
	uint8_t Next[4];
};

typedef const struct State STyp;

// Constant definitions

// TODO: define reload value for half second
#define HALF_SEC = 39999999; 	//40 million ticks per half second

//TODO: assign a value to all states in Drag Race FSM

//enum DragRace_states {};
// ^ not needed? 

// Just copying the set up from the last project

#define Initialize 0
#define Waiting 1
#define Countdown_Yellow1 2
#define Countdown_Yellow2 3
#define Go 4
#define False_Start_Left 5
#define False_Start_Right 6
#define False_Start_Both 7
#define Win_Left 8
#define Win_Right 9
#define Win_Both 10

//TODO: Define Drag Race FSM
STyp DragRace_FSM[11] = {
	//output, delay (in multiples of 0.5s), next
	
		// Initialize
		{0xFF, 2, {Waiting, Waiting, Waiting, Waiting}}, 
 
		// Waiting
		{0x00,1,{Waiting, Waiting, Waiting, Countdown_Yellow1}},
		
		// Countdown Yellow 1
    {0x03, 1, {False_Start_Both, False_Start_Left, False_Start_Right, Countdown_Yellow2}},
 
    // Countdown Yellow 2
    {0x0F, 1, {False_Start_Both, False_Start_Left, False_Start_Right, Go}},
 
    // Go
    {0x30, 1, {Win_Both, Win_Left, Win_Right, Go}},
 
    // False Start Left
    {0x80, 2, {Waiting, Waiting, Waiting, Waiting}},
		
    // False Start Right
    {0x40, 2, {Waiting, Waiting, Waiting, Waiting}},
 
    // False Start Both
    {0xC0, 2, {Waiting, Waiting, Waiting, Waiting}},
 
    // Win Left
    {0x20, 2, {Waiting, Waiting, Waiting, Waiting}},
 
    // Win Right
    {0x10, 2, {Waiting, Waiting, Waiting, Waiting}},
		
		// Win Both
    {0x30, 2, {Waiting, Waiting, Waiting, Waiting}},
};


uint8_t State_Index;  // current state index
uint8_t timesup;
uint8_t Input;
uint8_t reset;  // flag to reset the system, set by the reset button located at breadboard, not the launchpad reset button.
	
	
int main(void){
	System_Init();
	uint32_t	Half_Sec = 3999999;
	State_Index = Initialize ;
	LIGHTS = DragRace_FSM[State_Index].Out;
  while(1){
		
    // TODO: assign Initial state: all lights on
    State_Index = Initialize ; 
		while (!reset) {
			LIGHTS = DragRace_FSM[State_Index].Out;
			SysTick_Start(DragRace_FSM[State_Index].Time * Half_Sec);
			while((!timesup)&&(!reset)){
			  WaitForInterrupt();
			}
			
			timesup=0;
			
			State_Index = DragRace_FSM[State_Index].Next[Input];
			reset = 0;
		}
  }
}

void System_Init(void) {
	DisableInterrupts();
  Sensors_Init(); 
	Reset_Init(); 
	Lights_Init();
	SysTick_Init(); 
	EnableInterrupts();
	timesup = 0;
}

// Initialize the two sensors, enable both edge edge-triggered interrupt for both sensors
void Sensors_Init(void){
	SYSCTL_RCGC2_R |= 0x00000008;     // D clock (double check this)
	GPIO_PORTD_CR_R = 0xFF;           // allow changes to all of port D       
	GPIO_PORTD_AMSEL_R = 0x00;        // disable analog function
	GPIO_PORTD_PCTL_R = 0x00000000;   // GPIO clear bit PCTL  
	GPIO_PORTD_DIR_R = 0x00;          // all inputs   
	GPIO_PORTD_AFSEL_R = 0x00;        // no alternate function
	GPIO_PORTD_PUR_R = 0x00;					// disable pull up resistors
	GPIO_PORTD_DEN_R |= 0xFF;					// digital enable on all
	
	SYSCTL_RCC2_R |= 0x08;		//activate port D clock
	
	NVIC_EN0_R |= 0x00080000;		//enable port D interrupts DOUBLE CHECK VALUE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
	GPIO_PORTD_PCTL_R &= ~0x0000000F;	//PD as GPIO
	GPIO_PORTD_AMSEL_R &= ~0x01;		//disable analog on PF0
	GPIO_PORTD_IS_R &= ~0x07; 			// PD0,1 are edge sensitive
	GPIO_PORTD_IBE_R &= ~0x0F;			//PD0,1 not both edges
	GPIO_PORTD_IEV_R &= ~0x07; 		//PD0, 1 is a falling edge event
	
	GPIO_PORTD_ICR_R = 0xFF;					//clear flag 0
	GPIO_PORTD_IM_R |= 0x07;					//arm interrupt on PF0
	
	//NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00A00000?;		//priority number set to 5 NEEDS TO BE 2
	
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF) | 0x40000000; 	//priority 2
	//^double check

	EnableInterrupts();
}

// Initialize the reset button
void Reset_Init(void){

		SYSCTL_RCGC2_R |= 0x00000001;
		//while ((SYSCTL_RCGC2_R&0x00000001)!=0x00000001);
		
	  GPIO_PORTA_CR_R |= 0x04;
		GPIO_PORTA_AMSEL_R &= ~0x04;
		GPIO_PORTA_DIR_R &= ~0x04;
		
		GPIO_PORTA_PCTL_R &= ~0x00000F00;  //PortA 2
		//^ double check this
		
		GPIO_PORTA_AFSEL_R &= ~0x04;
		GPIO_PORTA_DEN_R |= 0x04;
		GPIO_PORTA_IS_R &= ~0x04; //edge sensitive
		GPIO_PORTA_IBE_R &= ~0x04; // not on both edges
		GPIO_PORTA_ICR_R |= 0x04; // clear flag
		GPIO_PORTA_IM_R |= 0x04; // arm interrupts on PortA 2
		
		
		NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFFFF00) | 0x00000020; //priority 1
		//^ double check this
		
		NVIC_EN0_R |= 0x00000001;

}

// Initialize 8 LEDs
void Lights_Init(void){
	SYSCTL_RCGC2_R |= 0x00000002;     // B clock (double check this)
	GPIO_PORTB_CR_R = 0xFF;           // allow changes to all of port B       
	GPIO_PORTB_AMSEL_R = 0x00;        // disable analog function
	GPIO_PORTB_PCTL_R = 0x00000000;   // GPIO clear bit PCTL  
	GPIO_PORTB_DIR_R = 0xFF;          // all outputs   
	GPIO_PORTB_AFSEL_R = 0x00;        // no alternate function
	GPIO_PORTB_PUR_R = 0x00;					// disable pull up resistors
	GPIO_PORTB_DEN_R |= 0xFF;					// digital enable on all
}

// Initialize systick timer: take care of systick interrupt priority,
// set systick timer to use core clock and enable Interrupt
// You don't need to setup the reload value in this function.
// Reload value will be taken care of in SysTick_Start().
void SysTick_Init(void) {
    NVIC_ST_CTRL_R = 0;                 // disable
    NVIC_ST_RELOAD_R = 39999999;      		// reload value for 0.5s
    NVIC_ST_CURRENT_R = 0;              // clear
    NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x60000000;  //priority 3
    NVIC_ST_CTRL_R = 0x00000007;              //enable
	  EnableInterrupts();
}

// Interrupt handler for the two sensors: take care of Input here 
void GPIOPortD_Handler(void){
	//handles the buttons
	// no conditionals
	
	GPIO_PORTD_ICR_R |= 0x03;
	//...
	
	Input = SENSORS;
}

// Interrupt handler for reset button: Resets the board to the initalization state: take care of global variable reset here 
void GPIOPortA_Handler(void) {
	//chose port A 
	//NEEDS TO BE A DIFFERENT PORT FOR RESET
	NVIC_ST_CTRL_R &= 0; 
	GPIO_PORTA_ICR_R |= 0x00000100; // clear flags
	
	reset = 1; 
}

// Take care of timesup here
void SysTick_Handler(void) {
		timesup = 1;
}

// Start systick timer with specified reload value.
void SysTick_Start(uint32_t reload){
	//used to reset the timer to 0.5s 
	NVIC_ST_CTRL_R = 0;                 // disable
  NVIC_ST_RELOAD_R = reload;      		// reload value for 0.5s
  NVIC_ST_CURRENT_R = 0;              // clear
	NVIC_ST_CTRL_R = 0x00000007;              //enable
}
