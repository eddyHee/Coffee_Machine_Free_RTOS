//******************************************************************************
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"
//******************************************************************************

//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "croutine.h"
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
//******************************************************************************

//*******

GPIO_InitTypeDef GPIO_Initstructure;
TIM_TimeBaseInitTypeDef timer_InitStructure;
TaskHandle_t xMasterThreadHandler = NULL;

// use for setup the clock speed for TIM2
// this timer is used for general purpose timing
const uint16_t TIMER_2_PRESCALER = 232;
const uint16_t TIMER_2_PERIOD = 2999;
const uint16_t TIMER_2_FREQUENCY = 120;

// setup the clock speed for TIM3
// this timer is used for blinking LED
const uint16_t TIMER_3_PRESCALER = 2100 - 1;
const uint16_t TIMER_3_PERIOD = 10000 - 1;
const uint16_t TIMER_3_FREQUENCY = 4;

// LED names
const uint16_t GREEN = GPIO_Pin_12;
const uint16_t ORANGE = GPIO_Pin_13;
const uint16_t RED = GPIO_Pin_14;
const uint16_t BLUE = GPIO_Pin_15;

// size option for coffee
const uint16_t MOCHA = GPIO_Pin_13; // orange
const uint16_t ESPRESSO = GPIO_Pin_14; // red
const uint16_t LATTE = GPIO_Pin_15; // blue

// define few timing for events
const uint16_t LONG_PRESS_TIME = 2; // 3 seconds holding for long press.
const float MIN_PRESS_TIME = 0.05; // the min single press should need 0.05 second.
const float DOUBLE_CLICK_TIME = 0.5; // double press should be with in 0.5 second.
const uint16_t IDLE_TIME = 5;
const float SOUND_OUTPUT = 0.3; // output the sound for 1 second

// used to help calculate the time interval for some event
unsigned int timer_for_button_hold = 0;
unsigned int timer_for_button_released = 0;
unsigned int timer_for_idle = 0;

// keep track button states
bool is_button_up = true;

// variables used for checking clicks
bool within_double_click_period = false;
bool button_clicked = false;
bool is_single_click = false;
bool is_double_click = false;
bool is_long_click = false; // need to hold the button for more than 3 seconds

// the state for the machine
typedef enum MODE {programming, programming_update_timing, brewing, neutral, making} mode;
mode curMode;
bool countdown_timer_has_started = false;
//bool is_programming_state = false; // this state allows user to change the timing for different size of coffee
//bool is_selecting = false;

uint16_t error_LED_1 = 0;
uint16_t error_LED_2 = 0;
uint16_t display_LED_1 = 0;
int num_blink = 0;

int coffee_type = 0;
const int MAX_TYPE_OPTION = 3;

int ingredient_type = 0;
const int MAX_INGREDIENT_OPTION = 3;


typedef struct {
	int espresso;
	int milk;
	int choco;
} ingredients;

// predefine timing for coffee machine
int time_milk = 4;
int time_espresso = 4;
int time_choc = 4;
int new_num_click = 0;

uint16_t coffee_LED = 0;

bool display_ingredient_predef_timing = false;

fir_8 filt;
bool output_sound = false;
int timer_for_sound = 0;
bool start_sound_timer = false;
bool sound_init = false;


void UpdateMachineStatus(void);
void DisplayCountdown(void);


#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS (4 bytes).*/
void vDispenseLatte(void *pvParameters);
void vDispenseEspresso(void *pvParameters);
void vDispenseMocha(void *pvParameters);


void InitLEDs() {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_Initstructure.GPIO_Pin = GREEN | ORANGE | RED | BLUE;
  GPIO_Initstructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Initstructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, & GPIO_Initstructure);
}

void InitButton() {// initialize user button
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Initstructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Initstructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, & GPIO_Initstructure);
}

void InitTimer_2() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  timer_InitStructure.TIM_Prescaler = TIMER_2_PRESCALER;
  timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timer_InitStructure.TIM_Period = TIMER_2_PERIOD;
  timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timer_InitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, & timer_InitStructure);
  TIM_Cmd(TIM2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void InitTimer_3() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	timer_InitStructure.TIM_Prescaler = TIMER_3_PRESCALER;
	timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timer_InitStructure.TIM_Period = TIMER_3_PERIOD;
	timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timer_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &timer_InitStructure);
	TIM_Cmd(TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

void EnableTimer2Interrupt() {
  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( & nvicStructure);
}

void EnableTimer3Interrupt() {
	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
}


void InitSound() {
	SystemInit();

	//enables GPIO clock for PortD
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	codec_init();
	codec_ctrl_init();

	I2S_Cmd(CODEC_I2S, ENABLE);

	initFilter(&filt);
}
void LEDOn(uint16_t GPIO_Pin) {
  GPIO_SetBits(GPIOD, GPIO_Pin);
}

void LEDOff(uint16_t GPIO_Pin) {
  GPIO_ResetBits(GPIOD, GPIO_Pin);
}

void TIM2_IRQHandler() {
	// tick = idle time
  //Checks whether the TIM2 interrupt has occurred or not
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    
		timer_for_button_hold++;
    timer_for_button_released++;
	
    if (!is_button_up && timer_for_button_hold >= LONG_PRESS_TIME * TIMER_2_FREQUENCY) {
      is_long_click = true;
			
      // if timer is not reset to 0, is_long_click will always be true;
      timer_for_button_hold = 0;
			timer_for_idle = 0;
    }
		
		timer_for_idle++;
		
		if ( timer_for_idle > IDLE_TIME * TIMER_2_FREQUENCY ) {
			curMode = neutral;
			timer_for_idle = 0;
			//is_selecting = false;
		}
	
		if(output_sound)
		{
			timer_for_sound++;
		}
		if(timer_for_sound > SOUND_OUTPUT * TIMER_2_FREQUENCY)
		{
			timer_for_sound = 0;
			output_sound = false;
		}
		
  }
}

void TIM3_IRQHandler() {
	// ticks = LED blink
	// checks whether the tim3 interrupt has occurred or not
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		if ( num_blink > 0 ) {
			GPIO_ToggleBits(GPIOD, display_LED_1); //flip led
			num_blink--;
			timer_for_idle = 0;
		}
	}
}

bool CanUpdateClickState() {
	return (!is_long_click && 
					!is_button_up && 
					timer_for_button_hold >= MIN_PRESS_TIME * TIMER_2_FREQUENCY);
}

void UpdateIngredientTiming() {
	if (ingredient_type == 0) time_milk = new_num_click;
	else if (ingredient_type == 1) time_espresso = new_num_click;
	else if (ingredient_type == 2) time_choc = new_num_click;
}

void UpdateTimingStatus() {
	within_double_click_period = (timer_for_button_released <= (DOUBLE_CLICK_TIME * TIMER_2_FREQUENCY));
	
	if (is_single_click && !within_double_click_period) {
		new_num_click++;
		output_sound = true;
		is_single_click = false;
	} else if (is_double_click) {
		// do nothing for now
		is_double_click = false;
	} else if (is_long_click) {
		curMode = programming;
		is_long_click = false;
		is_single_click = false;
		is_double_click = false;
		is_button_up = true;

		if(new_num_click)
			UpdateIngredientTiming();
		
		output_sound = true;
	}
}

void UpdateProgrammingStatus() {
	within_double_click_period = (timer_for_button_released <= (DOUBLE_CLICK_TIME * TIMER_2_FREQUENCY));
	
	if (is_single_click && !within_double_click_period) {
		ingredient_type = (ingredient_type + 1) % MAX_INGREDIENT_OPTION;
		is_single_click = false;
	} else if (is_double_click) {
		//display_ingredient_predef_timing = true;

		new_num_click = 0;
		is_double_click = false;
		
		DisplayCountdown();
		while(num_blink){}
		curMode = programming_update_timing;
	} else if (is_long_click) {
		curMode = brewing;
		is_long_click = false;
		is_single_click = false;
		is_double_click = false;
		is_button_up = true;
		LEDOff(GREEN);
	}
}

// this block of definitions should probably be higher up when refactoring
bool changeValve = false;
const uint32_t VALVE_OFF = 001;
const uint32_t VALVE_ESPRESSO = 1000;
const uint32_t VALVE_MILK = 1500;
const uint32_t VALVE_CHOCO = 2100;
uint32_t curValvePos = VALVE_OFF;

void UpdateBrewingStatus() {

	within_double_click_period = (timer_for_button_released <= (DOUBLE_CLICK_TIME * TIMER_2_FREQUENCY));
	
	if(is_single_click && !within_double_click_period) {
		coffee_type = (coffee_type + 1) % MAX_TYPE_OPTION;
		is_single_click = false;
	} else if (is_double_click) {	
		
		ingredients drink = {0, 0, 0};
		//memset(&drink, 0, sizeof(ingredients));
		if (coffee_type == 0) {
			//Mocha
			drink.espresso = time_espresso;
			drink.choco = time_choc;
			coffee_LED = MOCHA;
			
			xTaskCreate( vDispenseMocha, (const char*)"Dispense Coffee",
				STACK_SIZE_MIN, (void*) &drink, 0, NULL);
		} else if (coffee_type == 1) {
			// Espresso
			drink.espresso = time_espresso;
			coffee_LED = ESPRESSO;
			
			xTaskCreate( vDispenseEspresso, (const char*)"Dispense Coffee",
				STACK_SIZE_MIN, (void*) &drink, 0, NULL);
		} else if (coffee_type == 2) {
			// Latte
			drink.espresso = time_espresso;
			drink.milk = time_milk;
			coffee_LED = LATTE;
			
			xTaskCreate( vDispenseLatte, (const char*)"Dispense Coffee",
				STACK_SIZE_MIN, (void*) &drink, 0, NULL);
		}
		
		is_double_click = false;
		is_single_click = false;
		curMode = making;
	} else if (is_long_click) {
		curMode = programming;
		is_long_click = false;
		is_single_click = false;
		is_double_click = false;
		is_button_up = true;
	}
}

void UpdateNeutralStatus() {
	if (is_single_click) {
		curMode = brewing;
		//is_selecting = true;
		is_single_click = false;
	} else if (is_long_click) {
		curMode = programming;
		//is_selecting = true;
		is_long_click = false;
		is_double_click = false;
		is_single_click = false;
		is_button_up = true;
	}
}

void UpdateMakingStatus() {
	if(is_single_click) {
		curMode = brewing;
		is_single_click = false;
	} else if (is_long_click) {
		is_long_click = false;
	} else if( is_double_click) {
		is_double_click = false;
	}
}
void UpdateMachineStatus() {
	switch (curMode) {
		case neutral:
			UpdateNeutralStatus();
			break;
		case brewing:
			UpdateBrewingStatus();
			break;
		case programming:
			UpdateProgrammingStatus();
			break;
		case programming_update_timing:
			UpdateTimingStatus();
			break;
		case making:
			UpdateMakingStatus();
			break;
	};
}

void DisplayCountdown() {
	//choose which light to blink
	switch(ingredient_type) {
		case 0:
			display_LED_1 = MOCHA;
			break;
		case 1:
			display_LED_1 = ESPRESSO;
			break;
		case 2:
			display_LED_1 = LATTE;
			break;
	}

	
	//set countdown
	if ( ingredient_type == 0) {
		num_blink = time_milk * 2;
	}
	else if (ingredient_type == 1) {
		num_blink = time_espresso * 2;
	}
	else if (ingredient_type == 2) {
		num_blink = time_choc * 2;
	}
  //	countdown_timer_has_started = true;
}

void ShowUpdateTimingLED() {
	if (ingredient_type == 0) {
			display_LED_1 = MOCHA;
	} else if (ingredient_type == 1) {
			display_LED_1 = ESPRESSO;
	} else if (ingredient_type == 2) {
			display_LED_1 = LATTE;
	}
	
	LEDOn(GREEN);
	if ( is_button_up ) LEDOn(display_LED_1);
	else LEDOff(display_LED_1);
	
}

void ShowProgrammingLED() {
	if (ingredient_type == 0) {
		LEDOff(LATTE);
		LEDOff(ESPRESSO);
		LEDOn(MOCHA);
	} else if (ingredient_type == 1) {
		LEDOff(MOCHA);
		LEDOff(LATTE);
		LEDOn(ESPRESSO);
	} else if (ingredient_type == 2) {
		LEDOff(ESPRESSO);
		LEDOff(MOCHA);
		LEDOn(LATTE);
	}
	LEDOn(GREEN);

}

void ShowSelectingLED() {
	LEDOff(GREEN);
	if (coffee_type == 0) {
		LEDOff(LATTE);
		LEDOff(ESPRESSO);
		LEDOn(MOCHA);
	} else if (coffee_type == 1) {
		LEDOff(MOCHA);
		LEDOff(LATTE);
		LEDOn(ESPRESSO);
	} else if (coffee_type == 2) {
		LEDOff(ESPRESSO);
		LEDOff(MOCHA);
		LEDOn(LATTE);
	}
}

void ShowNeutralLED() {
	LEDOn(RED);
	LEDOn(ORANGE);
	LEDOn(BLUE);
	//commented out to test dispense task, remove in actual code
	LEDOff(GREEN);
}

void showMakingLED()
{
}

void ShowLED() {
	switch (curMode) {
		case neutral:
			ShowNeutralLED();
			break;
		case brewing:
			ShowSelectingLED();
			break;
		case programming:
			ShowProgrammingLED();
			break;
		case programming_update_timing:
			ShowUpdateTimingLED();
		case making:
			showMakingLED();
			break;
	};
}

//*******

void initSound(void);
float updateFilter(fir_8* filt, float val);
void initFilter(fir_8* theFilter);

void initServo(void);
void InitPWMTimer4(void);
void SetupPWM(void);

void vButtonTask(void *pvParameters);
void vSoundTask(void *pvParameters);
void vMainTask(void *pvParameters);

void vServoTask(void *pvParameters);

TaskHandle_t xHandleBlue = NULL;

//******************************************************************************
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured,
	   this is done through SystemInit() function which is called from startup
	   file (startup_stm32f4xx.s) before to branch to application main.
	   To reconfigure the default setting of SystemInit() function, refer to
	   system_stm32f4xx.c file
	 */
	
	/*!< Most systems default to the wanted configuration, with the noticeable 
		exception of the STM32 driver library. If you are using an STM32 with 
		the STM32 driver library then ensure all the priority bits are assigned 
		to be preempt priority bits by calling 
		NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); before the RTOS is started.
	*/
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	STM_EVAL_LEDInit(LED_BLUE);
	STM_EVAL_LEDInit(LED_GREEN);
	STM_EVAL_LEDInit(LED_ORANGE);
	STM_EVAL_LEDInit(LED_RED);
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);
	
	InitButton();
	InitTimer_2();
	InitTimer_3();
	EnableTimer2Interrupt();
	EnableTimer3Interrupt();
	
	initSound();
	initFilter(&filt);
	
	curMode = neutral;
	output_sound = false;
	timer_for_sound = 0;

	initServo();
	InitPWMTimer4();
	SetupPWM();
	
	xTaskCreate( vButtonTask, (const char*)"Button Task",
		STACK_SIZE_MIN, NULL, 0, NULL);
	xTaskCreate( vSoundTask, (const char*)"Sound Task",
		STACK_SIZE_MIN, NULL, 1, NULL);
	xTaskCreate( vServoTask, (const char*)"Servo Task",
		STACK_SIZE_MIN, (void *) NULL, 1, NULL);
	xTaskCreate( vMainTask, (const char*)"main project Task",
		STACK_SIZE_MIN, NULL, 0, NULL);
	
//	
//	ingredients latte = {3, 2, 0};
//	xTaskCreate( vDispenseCoffee, (const char*)"Dispense Coffee",
//		STACK_SIZE_MIN, (void *) &latte, 0, NULL);
	
	vTaskStartScheduler();
}


void vButtonTask(void *pvParameters)
{
	mode preMode = curMode;
	bool ChangeMode = false;
	while(1) {
		uint8_t button_pin = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
		uint32_t *curValvePos = (uint32_t *) pvParameters;
		if (button_pin) {
      if (is_button_up) {
        timer_for_button_hold = 0;
      }
      is_button_up = false;
			// reset the timer for idle when button event happened
			timer_for_idle = 0;
			if(curMode != making || curMode != neutral || preMode != making || preMode != neutral)
			{
				ChangeMode = ChangeMode || (preMode != curMode);
			} else
			{
				ChangeMode = false;
			}
			preMode = curMode;
			if(curMode == making || curMode == neutral || preMode == making || preMode == neutral)
			{
				ChangeMode = false;
			}
    } else {
      if (CanUpdateClickState()) {
				if (!is_single_click && !is_double_click && !ChangeMode) {
					is_single_click = true;
				} else if (is_single_click && !is_double_click) {
					is_single_click = false;
					is_double_click = true;
				}
				ChangeMode = false;
      }
			if(!is_button_up) 
			{
				timer_for_button_released = 0;
			}
      is_button_up = true;
    }
	}
}

void vSoundTask(void *pvParameters) {
	while(1) {
		if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE)) {
			SPI_I2S_SendData(CODEC_I2S, sample);
			if (sampleCounter & 0x00000001) {
				sawWave += NOTEFREQUENCY;
				if (sawWave > 1.0)
					sawWave -= 2.0;

				filteredSaw = updateFilter(&filt, sawWave);
				sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
			}
			sampleCounter++;
		}
		if (output_sound) {
			if(!sound_init) {
				sound_init = true;
				initSound();
			}
		} else {
			GPIO_ResetBits(GPIOD, GPIO_Pin_4);
			sound_init = false;
			vTaskDelay(10/portTICK_RATE_MS);
		}
	}
}

void vMainTask(void *pvParameters) {
	while(1) {
		UpdateMachineStatus();
		ShowLED();
	}
}

void vDispenseLatte(void *pvParameters) {
	uint16_t LED = coffee_LED;
	
	int espresso = time_espresso;
	int milk = time_milk;
	while (1) {
		while (espresso > 0) {
			
			if(curMode == making) {
				LEDOff(ESPRESSO);
				LEDOn(LATTE);
				LEDOff(MOCHA);
			}
			curValvePos = VALVE_ESPRESSO;
			changeValve = true;
			STM_EVAL_LEDOn(LED_GREEN);
			vTaskDelay(1000/portTICK_RATE_MS);
			STM_EVAL_LEDOff(LED_GREEN);
			vTaskDelay(1000/portTICK_RATE_MS);
			timer_for_idle = 0;
			espresso--;
		}
		
		while (milk > 0) {
			
			if(curMode == making) {
				LEDOff(ESPRESSO);
				LEDOn(LATTE);
				LEDOff(MOCHA);
			}
			curValvePos = VALVE_MILK;
			changeValve = true;
			STM_EVAL_LEDOn(LED_GREEN);
			vTaskDelay(1000/portTICK_RATE_MS);
			STM_EVAL_LEDOff(LED_GREEN);
			vTaskDelay(1000/portTICK_RATE_MS);
			timer_for_idle = 0;
			milk--;
		}
		
		output_sound = true;
		vTaskDelay(500/portTICK_RATE_MS);
		output_sound = true;
			
		vTaskDelete(NULL);
	}
}

void vDispenseMocha(void *pvParameters) {
	uint16_t LED = coffee_LED;
	
	int espresso = time_espresso;
	int choc = time_choc;
	while (1) {
		while (espresso > 0) {
			
			if(curMode == making) {
				LEDOff(ESPRESSO);
				LEDOff(LATTE);
				LEDOn(MOCHA);
			}
			
			curValvePos = VALVE_ESPRESSO;
			changeValve = true;
			STM_EVAL_LEDOn(LED_GREEN);
			vTaskDelay(1000/portTICK_RATE_MS);
			STM_EVAL_LEDOff(LED_GREEN);
			vTaskDelay(1000/portTICK_RATE_MS);
			timer_for_idle = 0;
			espresso--;
		}
		
		while (choc > 0) {
			
			if(curMode == making) {
				LEDOff(ESPRESSO);
				LEDOff(LATTE);
				LEDOn(MOCHA);
			}
			curValvePos = VALVE_CHOCO;
			changeValve = true;
			STM_EVAL_LEDOn(LED_GREEN);
			vTaskDelay(1000/portTICK_RATE_MS);
			STM_EVAL_LEDOff(LED_GREEN);
			vTaskDelay(1000/portTICK_RATE_MS);
			timer_for_idle = 0;
			choc--;
		}
		
		output_sound = true;
		vTaskDelay(500/portTICK_RATE_MS);
		output_sound = true;
		
		vTaskDelete(NULL);
	}
}

void vDispenseEspresso(void *pvParameters) {
	uint16_t LED = coffee_LED;
	
	int espresso = time_espresso;
	while (1) {
		while (espresso > 0) {
			
			if(curMode == making) {
				LEDOn(ESPRESSO);
				LEDOff(LATTE);
				LEDOff(MOCHA);
			}
			
			curValvePos = VALVE_ESPRESSO;
			changeValve = true;
			STM_EVAL_LEDOn(LED_GREEN);
			vTaskDelay(1000/portTICK_RATE_MS);
			STM_EVAL_LEDOff(LED_GREEN);
			vTaskDelay(1000/portTICK_RATE_MS);
			timer_for_idle = 0;
			espresso--;
		}
			
		
		output_sound = true;
		vTaskDelay(500/portTICK_RATE_MS);
		output_sound = true;
		vTaskDelete(NULL);
	}
}

void vServoTask(void *pvParameters) {
	while(1) {
		if (changeValve) {
			TIM4->CCR1 = curValvePos;
			changeValve = false;
		} else {
			vTaskDelay(10/portTICK_RATE_MS);
		}
	}
}

//******************************************************************************

void initSound() {
	SystemInit();

	//enables GPIO clock for PortD
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	codec_init();
	codec_ctrl_init();

	I2S_Cmd(CODEC_I2S, ENABLE);

	initFilter(&filt);
}

// the following code is for sound
// a very crude FIR lowpass filter
float updateFilter(fir_8* filt, float val) {
	uint16_t valIndex;
	uint16_t paramIndex;
	float outval = 0.0;

	valIndex = filt->currIndex;
	filt->tabs[valIndex] = val;

	for (paramIndex=0; paramIndex<8; paramIndex++)
	{
		outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
	}

	valIndex++;
	valIndex &= 0x07;

	filt->currIndex = valIndex;

	return outval;
}

void initFilter(fir_8* theFilter) {
	uint8_t i;

	theFilter->currIndex = 0;

	for (i=0; i<8; i++)
		theFilter->tabs[i] = 0.0;

	theFilter->params[0] = 3;
	theFilter->params[1] = 3;
	theFilter->params[2] = 3;
	theFilter->params[3] = 3;
	theFilter->params[4] = 3;
	theFilter->params[5] = 3;
	theFilter->params[6] = 3;
	theFilter->params[7] = 3;
}

void initServo(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	//Initialize PB6 (TIM4 Ch1) and PB7 (TIM4 Ch2)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // GPIO_High_Speed
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// Assign Alternate Functions to pins
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
}
	
void InitPWMTimer4(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//TIM4 Clock Enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	// Time Base Configuration for 50Hz
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 84 -1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM4, ENABLE);
}
	
void SetupPWM(void) {
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //Set output capture as PWM mode
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //Enable output compare
	TIM_OCInitStructure.TIM_Pulse = 0; // Initial duty cycle at 0%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // HIGH output compare active
	// Set the output capture channel 1 and 2 (upto 4)
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); // Channel 1
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure); // Channel 2
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
}
