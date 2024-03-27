/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
#include "m-sequence.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	GPIO_TypeDef *port_out;
	uint16_t pin_out;
	uint8_t active;
	uint16_t delay;
	uint16_t pulsLength;
	uint8_t signalform;
	uint16_t frequency;
	uint32_t timeRef;
	uint8_t stopTrigger;
	uint8_t logicLevel;
	TIM_HandleTypeDef *clockTimer;
	TIM_TypeDef *timBase;
	uint8_t degree;
	uint16_t pnDelay;
	uint8_t *mSeq;
	uint16_t mSeqCounter;
} device;

typedef struct{
	uint8_t source;
	uint16_t delay;
	uint8_t signalform;
}startSource;

typedef struct{
	uint8_t source;
	uint16_t delay;
	uint8_t signalform;
}stopSource;

typedef struct{
	uint8_t source;
	uint8_t active;
	uint8_t signalform;
}eventSource;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Measuring Devices
#define MD1 0
#define MD2 1
#define MD3 2
#define MD4 3
#define MD5 4
#define MD6 5
#define MD7 6

// Start / Stop / Clock sources
#define USB 0x00
#define Button1 0x01
#define Button2 0x02

//Signals
#define FallingTrigger 0
#define RisingTrigger 1
#define FallingEdge 2
#define RisingEdge 3
#define AnyEdge 4
#define FallingClock 5
#define RisingClock 6
#define M_Sequence 7


//Errors
#define SourceNotUsed 2
#define NoSuchStopSource 3
#define NoSuchStartSource 4
#define InvalidDataReceived 5
#define NoSuchMessage 6
#define NoSuchDevice 7
#define NoSuchInput 8
#define AllocationFailed 9
#define NoSuchColor 10
#define NoSuchSignalform 11

//Message
#define ErrorByte 0x00
#define EventByte 0x01
#define RunByte 0x02
#define SendWhoAmI 0xaa
#define MeasurementRun 0xbb
#define MeasurementStop 0xcc

//colors
#define ColorOff 0
#define ColorRed 1
#define ColorGreen 2
#define ColorBlue 3

//Setting commands
#define HelloByte 0xaa
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t saveReceivedData[50];
uint8_t dataReceived = 0;
device allDevices[9];
startSource start;
stopSource stop;
eventSource event;

static uint8_t measRun = 0;
uint8_t button1_event = 0;
uint8_t button2_event = 0;
uint8_t md6_event = 0;
uint8_t md5_event = 0;
uint32_t startTime = 0;
uint32_t measTime = 0;
uint32_t stopTime = 0;
uint8_t deviceCount = 0;
uint8_t maxSeqLength = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//
void Dev_init(){
//Function sets each Measuring Device to Init

	allDevices[MD1].port_out = MD1_O_GPIO_Port;
	allDevices[MD1].pin_out = MD1_O_Pin;
	allDevices[MD1].clockTimer = &htim2;
	allDevices[MD1].timBase = TIM2;
	allDevices[MD1].stopTrigger = 0;
	allDevices[MD1].mSeq = (uint8_t *) malloc(3);

	allDevices[MD2].port_out = MD2_O_GPIO_Port;
	allDevices[MD2].pin_out = MD2_O_Pin;
	allDevices[MD2].clockTimer = &htim3;
	allDevices[MD2].timBase = TIM3;
	allDevices[MD2].stopTrigger = 0;
	allDevices[MD2].mSeq = (uint8_t *) malloc(3);

	allDevices[MD3].port_out = MD3_O_GPIO_Port;
	allDevices[MD3].pin_out = MD3_O_Pin;
	allDevices[MD3].clockTimer = &htim4;
	allDevices[MD3].timBase = TIM4;
	allDevices[MD3].stopTrigger = 0;
	allDevices[MD3].mSeq = (uint8_t *) malloc(3);

	allDevices[MD4].port_out = MD4_O_GPIO_Port;
	allDevices[MD4].pin_out = MD4_O_Pin;
	allDevices[MD4].clockTimer = &htim7;
	allDevices[MD4].timBase = TIM7;
	allDevices[MD4].stopTrigger = 0;
	allDevices[MD4].mSeq = (uint8_t *) malloc(3);

	allDevices[MD5].port_out = MD5_O_GPIO_Port;
	allDevices[MD5].pin_out = MD5_O_Pin;
	allDevices[MD5].clockTimer = &htim9;
	allDevices[MD5].timBase = TIM9;
	allDevices[MD5].stopTrigger = 0;
	allDevices[MD5].mSeq = (uint8_t *) malloc(3);

	allDevices[MD6].port_out = MD6_O_GPIO_Port;
	allDevices[MD6].pin_out = MD6_O_Pin;
	allDevices[MD6].clockTimer = &htim10;
	allDevices[MD6].timBase = TIM10;
	allDevices[MD6].stopTrigger = 0;
	allDevices[MD6].mSeq = (uint8_t *) malloc(3);

	allDevices[MD7].port_out = MD7_O_GPIO_Port;
	allDevices[MD7].pin_out = MD7_O_Pin;
	allDevices[MD7].clockTimer = &htim11;
	allDevices[MD7].timBase = TIM11;
	allDevices[MD7].stopTrigger = 0;
	allDevices[MD7].mSeq = (uint8_t *) malloc(3);

	//saves number of devices in allDevices
	deviceCount = sizeof(allDevices) / sizeof(allDevices[0]);

	for (uint8_t i = MD1; i < deviceCount; i++){
		allDevices[i].active = 0;
		allDevices[i].delay = 0;
		allDevices[i].signalform = FallingTrigger;
		allDevices[i].frequency = 0;
		allDevices[i].timeRef = 1;
		allDevices[i].mSeqCounter = 0;
		allDevices[i].pulsLength = 4;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Dev_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //Button1 pressed
	  if(Debounce(Button1_GPIO_Port, Button1_Pin) && button1_event){
		  button1_event = 0;
		  if(start.source == Button1 || stop.source == Button1){
			  if(measRun == 0){
				  StartMeasurement(Button1);
			  }else{
				  StopMeasurement(Button1);
			  }
		  }else if (event.active && event.source == Button1) {
			  SendMessage(EventByte, Button1, 3);
		  }else{
			  uint8_t errorMessage = SourceNotUsed;
			  PrintError(errorMessage);
		  }
	  }
	  //Button2 pressed
	  if(Debounce(Button2_GPIO_Port, Button2_Pin) && button2_event){
		  button2_event = 0;
		  if(start.source == Button2 || stop.source == Button2){
			  if(measRun == 0){
				  StartMeasurement(Button2);
			  }else{
				  StopMeasurement(Button2);
			  }
		  }else if (event.active && event.source == Button2) {
			  SendMessage(EventByte, Button2, 3);
		  }else{
			  uint8_t errorMessage = SourceNotUsed;
			  PrintError(errorMessage);
		  }
	  }
	  //Interrupt signal MD6
	  if(Debounce(MD6_I_GPIO_Port, MD6_I_Pin) && md6_event){
		  md6_event = 0;
		  if(start.source == MD6 || stop.source == MD6){
			  if(measRun == 0){
				  StartMeasurement(MD6);
			  }else{
				  StopMeasurement(MD6);
			  }
		  }else if (event.active && event.source == MD6) {
			  SendMessage(EventByte, MD6, 3);
		  }else{
			  uint8_t errorMessage = SourceNotUsed;
			  PrintError(errorMessage);
		  }
	  }
	  //Interrupt Signal MD5
	  if(Debounce(MD5_I_GPIO_Port, MD5_I_Pin) && md5_event){
		  md5_event = 0;
		  if(start.source == MD5 || stop.source == MD5){
			  if(measRun == 0){
				  StartMeasurement(MD5);
			  }else{
				  StopMeasurement(MD5);
			  }
		  }else if (event.active && event.source == MD5) {
			  SendMessage(EventByte, MD5, 3);
		  }else{
			  uint8_t errorMessage = SourceNotUsed;
			  PrintError(errorMessage);
		  }
	  }

	  //Data received
	  if(dataReceived == 1){
		  dataReceived = 0;
		  ReadData(saveReceivedData);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  FreeAllMalloc();
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void LightLED(uint8_t color){

	HAL_GPIO_WritePin(LED_green_GPIO_Port, LED_green_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_blue_GPIO_Port, LED_blue_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_SET);
	uint8_t errorMessage = NoSuchColor;
	switch (color){
	case ColorRed:
		HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_RESET);
		break;
	case ColorGreen:
		HAL_GPIO_WritePin(LED_green_GPIO_Port, LED_green_Pin, GPIO_PIN_RESET);
		break;
	case ColorBlue:
		HAL_GPIO_WritePin(LED_blue_GPIO_Port, LED_blue_Pin, GPIO_PIN_RESET);
		break;
	case ColorOff:
		break;
	default:
		PrintError(errorMessage);
	}
}

void PrintError(uint8_t errorMessage){
	LightLED(ColorRed);
	uint8_t printError[] = {HelloByte, ErrorByte, errorMessage};
	CDC_Transmit_FS(printError, 3);
	while(!(dataReceived == 1 && saveReceivedData[0] == 0xaa && saveReceivedData[1] == 0x0a)){//Wait for user respond
	}
	LightLED(ColorOff);
	dataReceived = 0;
}

void SendMessage(uint8_t feature, uint8_t message, uint8_t length){
	uint8_t sendMessage[length];
	sendMessage[0] = HelloByte;
	if(length == 2){
		sendMessage[1] = message;
	}else if(length == 3){
		sendMessage[1] = feature;
		sendMessage[2] = message;
	}
	CDC_Transmit_FS(sendMessage, length);
}

int Debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET){
		HAL_Delay(100);
		if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET){
			HAL_Delay(100);
			return 1;
		}
	}
	return 0;
}

void FreeAllMalloc(){
	for(uint8_t i = MD1; i < deviceCount; i++){
		free(allDevices[i].mSeq);
	}
}


int CheckDelay(uint32_t measTime, uint16_t msDelay){
/* function returns 1 after adjusted delay time
* measTime: time since measurement started
* msDelay: delay time in ms*/
	if(measTime < msDelay){
		return 0;
	}else{
		return 1;
	}
}


void MD_InitSignalform(uint8_t i){
/* function puts dev to init level depending on its signalform
* dev: measuring device*/

	if(allDevices[i].signalform == FallingClock || allDevices[i].signalform == FallingTrigger || allDevices[i].signalform == FallingEdge){
		HAL_GPIO_WritePin(allDevices[i].port_out, allDevices[i].pin_out, GPIO_PIN_SET);
	}else if(allDevices[i].signalform == RisingClock || allDevices[i].signalform == RisingTrigger || allDevices[i].signalform == M_Sequence || allDevices[i].signalform == RisingEdge){
		HAL_GPIO_WritePin(allDevices[i].port_out, allDevices[i].pin_out, GPIO_PIN_RESET);
	}
	allDevices[i].mSeqCounter = 0;
}

void MD_Trigger(device dev, uint32_t runningTime){
/* function toggles level for pulsLength(ms) of dev to create a Triggersignal
* dev: measuring device
* runningTime: time since measurement runs*/

	if(runningTime > 0 && runningTime <= dev.pulsLength){
		HAL_GPIO_WritePin(dev.port_out, dev.pin_out, dev.signalform);
	}else{
		HAL_GPIO_WritePin(dev.port_out, dev.pin_out, !dev.signalform);
	}
}


void InitSignalformAll(){
//function puts all measuring devices to init level depending on their signalform

	for(uint8_t i = 0; i < deviceCount; i++){
		MD_InitSignalform(i);
	}
}

//reads the received message and executes the corresponding function
int ReadData(uint8_t buf[]){
/* function reads the received message and calls the corresponding function
* buf: save received byte messages */

	if(buf[0] != HelloByte){
		uint8_t errorMessage = InvalidDataReceived;
		PrintError(errorMessage);
		return 0;
	}
	uint8_t errorMessage = NoSuchMessage;
	switch (buf[1]){
	case 0x00:
		SetStartSource(buf[2], buf[3]<<8 | buf[4], buf[5]);
		break;
	case 0x01:
		SetStopSource(buf[2], buf[3]<<8 | buf[4], buf[5]);
		break;
	case 0x02:
		SetEvent(buf[2], buf[3], buf[4]);
		break;
	case 0x03:
		SetOutput(buf[2], buf[3], buf[4]<<8 | buf[5], buf[6],  buf[7]<<8 | buf[8], buf[9]<<8 | buf[10], buf[11], buf[12]);
		break;
	case 0x04:
		StartMeasurement(USB);
		break;
	case 0x05:
		StopMeasurement(USB);
		break;
	case 0xff:
		WhoAmI();
		break;
	default:
		PrintError(errorMessage);
		return 0;
	}
	return 1;
}

//
void SetStartSource(uint8_t newStartSource, uint16_t delay, uint8_t signalform){
/* function sets the requirements how to start a measurement
* newStartSource: Source of the signal which triggers to start a measurement
* delay: delay to wait before measurement starts
* signalform: type of signal which triggers to start a measurement */

	uint32_t tempFall = 0x00;
	uint32_t tempRise = 0x00;
	if(newStartSource > MD6){
		uint8_t errorMessage = NoSuchStartSource;
		PrintError(errorMessage);
		return;
	}else{
		start.source = newStartSource;
		start.delay = delay;
		start.signalform = signalform;
		uint8_t errorMessage = NoSuchSignalform;
		if(start.source > Button2){
			tempRise = EXTI->RTSR;
			tempFall = EXTI->FTSR;
			CLEAR_BIT(tempRise, (uint32_t) allDevices[newStartSource].pin_out);
			CLEAR_BIT(tempFall, (uint32_t) allDevices[newStartSource].pin_out);
			switch (start.signalform){
			case RisingEdge:
				SET_BIT(tempRise, (uint32_t) allDevices[newStartSource].pin_out);
				break;
			case FallingEdge:
				SET_BIT(tempFall, (uint32_t) allDevices[newStartSource].pin_out);
				break;
			case AnyEdge:
				SET_BIT(tempRise, (uint32_t) allDevices[newStartSource].pin_out);
				SET_BIT(tempFall, (uint32_t) allDevices[newStartSource].pin_out);
				break;
			default:
				PrintError(errorMessage);
			}
			EXTI->RTSR = tempRise;
			EXTI->FTSR = tempFall;
		}
	}
}

void SetStopSource(uint8_t newStopSource, uint16_t delay, uint8_t signalform){
/* function sets the requirements how to stop a measurement
* newStartSource: Source of the signal which triggers to stop a measurement
* delay: delay to wait before measurement stops
* signalform: type of signal which triggers to stop a measurement */
	uint32_t tempFall = 0x00;
	uint32_t tempRise = 0x00;
	if(newStopSource > MD6){
		uint8_t errorMessage = NoSuchStopSource;
		PrintError(errorMessage);
		return;
	}else{
		stop.source = newStopSource;
		stop.delay = delay;
		stop.signalform = signalform;
		uint8_t errorMessage = NoSuchSignalform;
		if(stop.source > Button2){
			tempRise = EXTI->RTSR;
			tempFall = EXTI->FTSR;
			CLEAR_BIT(tempRise, (uint32_t) allDevices[newStopSource].pin_out);
			CLEAR_BIT(tempFall, (uint32_t) allDevices[newStopSource].pin_out);
			switch (stop.signalform){
			case RisingEdge:
				SET_BIT(tempRise, (uint32_t) allDevices[newStopSource].pin_out);
				break;
			case FallingEdge:
				SET_BIT(tempFall, (uint32_t) allDevices[newStopSource].pin_out);
				break;
			case AnyEdge:
				SET_BIT(tempRise, (uint32_t) allDevices[newStopSource].pin_out);
				SET_BIT(tempFall, (uint32_t) allDevices[newStopSource].pin_out);
				break;
			default:
				PrintError(errorMessage);
			}
			EXTI->RTSR = tempRise;
			EXTI->FTSR = tempFall;
		}
	}
}

void SetEvent(uint8_t eventSource, uint8_t active, uint8_t signalform){
/* function sets the requirements how an event is triggered
* eventSource: Source of the signal which triggers an event
* active: turns the ability to trigger an event with this eventSource on (1) or off (0)
* signalform: type of signal which triggers an event*/
	event.source = eventSource;
	event.active = active;
	event.signalform = signalform;
	uint32_t tempFall = 0x00;
	uint32_t tempRise = 0x00;
	uint8_t errorMessage = NoSuchSignalform;
	if(event.source > Button2){
		tempRise = EXTI->RTSR;
		tempFall = EXTI->FTSR;
		CLEAR_BIT(tempRise, (uint32_t) allDevices[eventSource].pin_out);
		CLEAR_BIT(tempFall, (uint32_t) allDevices[eventSource].pin_out);
		switch (event.signalform){
		case RisingEdge:
			SET_BIT(tempRise, (uint32_t) allDevices[eventSource].pin_out);
			break;
		case FallingEdge:
			SET_BIT(tempFall, (uint32_t) allDevices[eventSource].pin_out);
			break;
		case AnyEdge:
			SET_BIT(tempRise, (uint32_t) allDevices[eventSource].pin_out);
			SET_BIT(tempFall, (uint32_t) allDevices[eventSource].pin_out);
			break;
		default:
			PrintError(errorMessage);
		}
		EXTI->RTSR = tempRise;
		EXTI->FTSR = tempFall;
	}
}

void SetOutput(uint8_t device, uint8_t active, uint16_t delay, uint8_t signalform, uint16_t pulsLength, uint16_t frequency, uint8_t stopTrigger, uint8_t degree /*uint16_t pnDelay*/){
/* function sets the chosen measuring device with the required features for an output sync signal
* device: measuring device
* active: device is active (1), is not active (0)
* delay: delay to wait after measurement started
* signalform: type of output sync signal
* stopTrigger == 1: generates a trigger signal when measurement stops
* degree: degree to generate an M-Sequence */

	if(device >= deviceCount){
		uint8_t errorMessage = NoSuchDevice;
		PrintError(errorMessage);
		return;
	}
	allDevices[device].active = active;
	allDevices[device].delay = delay;
	allDevices[device].signalform = signalform;
	allDevices[device].pulsLength = pulsLength;
	allDevices[device].frequency = frequency;
	allDevices[device].stopTrigger = stopTrigger;
	allDevices[device].degree = degree;

	if(active && allDevices[device].degree != 0){
		maxSeqLength = pow(2, allDevices[device].degree)-1;
		allDevices[device].mSeq = (uint8_t *)realloc(allDevices[device].mSeq, maxSeqLength);
	}

	if(active && frequency != 0){
		if(allDevices[device].signalform == M_Sequence){
			mls(allDevices[device].degree, allDevices[device].mSeq);
			allDevices[device].timeRef = 1e6 / frequency;//timeRef in µs
		}else{
			if(frequency >= 8){
				allDevices[device].timBase -> PSC = 32-1;
				allDevices[device].timeRef = 1e6 / (frequency * 2);//toggle every half period
			}else{
				allDevices[device].timBase -> PSC = 265-1;
				allDevices[device].timeRef = (1e6 / (frequency * 2 * 8)) - 300;
			}

		}
		allDevices[device].timBase -> ARR = allDevices[device].timeRef;
	}
	MD_InitSignalform(device);
}

void StartMeasurement(uint8_t usedSource){
/* function starts a measurement
* usedSource: signal which triggered the start of the measurement*/
	if(start.source == usedSource && measRun == 0){
		HAL_Delay(start.delay);
		LightLED(ColorGreen);
		measRun = 1;
		SendMessage(RunByte, MeasurementRun, 3);
		stopTime = 0;
		startTime = HAL_GetTick();
		HAL_TIM_Base_Start_IT(&htim6);
		for(uint8_t i = MD1; i < deviceCount; i++){
			if(allDevices[i].active){
				if(allDevices[i].signalform == RisingEdge || allDevices[i].signalform == FallingEdge){
					HAL_GPIO_WritePin(allDevices[i].port_out, allDevices[i].pin_out, !HAL_GPIO_ReadPin(allDevices[i].port_out, allDevices[i].pin_out));
				}else if(allDevices[i].frequency != 0){
					HAL_TIM_Base_Start_IT(allDevices[i].clockTimer);
				}
			}
		}
	}
}

void StopMeasurement(uint8_t usedSource){
/* function stops a measurement
* usedSource: signal which triggered the stop of the measurement*/
	if(stop.source == usedSource && measRun == 1){
		HAL_Delay(stop.delay);
		LightLED(ColorOff);
		measRun = 0;
		SendMessage(RunByte, MeasurementStop, 3);
		stopTime = HAL_GetTick();
		startTime = 0;
		for(uint8_t i = MD1; i < deviceCount; i++){
			if(allDevices[i].signalform == RisingEdge || allDevices[i].signalform == FallingEdge){
				HAL_GPIO_WritePin(allDevices[i].port_out, allDevices[i].pin_out, !HAL_GPIO_ReadPin(allDevices[i].port_out, allDevices[i].pin_out));
			}else if(allDevices[i].frequency != 0){
				HAL_TIM_Base_Stop_IT(allDevices[i].clockTimer);
			}
		}
		InitSignalformAll();
	}
}

void WhoAmI(){
	SendMessage(0, SendWhoAmI, 2);
}

void UpdateStartTrigger(){
/* function toggles level of all active devices for 4ms when measurement starts*/
	measTime = HAL_GetTick() - startTime;

	for(uint8_t i = MD1; i < deviceCount; i++){
		if(allDevices[i].active && CheckDelay(measTime, allDevices[i].delay)){
			uint32_t runningTime = measTime - allDevices[i].delay;
			switch (allDevices[i].signalform){
			case FallingTrigger:
			case RisingTrigger:
				MD_Trigger(allDevices[i], runningTime);
				break;
			}
		}
	}
}

void UpdateStopTrigger(){
/* function toggles level of all active devices for 4ms when measurement stops*/
	measTime = HAL_GetTick() - stopTime;
	for(uint8_t i = MD1; i < deviceCount; i++){
		if(allDevices[i].active && allDevices[i].stopTrigger == 1){
			MD_Trigger(allDevices[i], measTime);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//Interrupt Button 1
	if(GPIO_Pin == Button1_Pin){
		button1_event = 1;
	}
	//Interrupt Button 2
	if(GPIO_Pin == Button2_Pin){
		button2_event = 1;
	}
	//Interrupt MD5
	if(GPIO_Pin == MD5_I_Pin){
		md5_event = 1;
	}
	//Interrupt MD6
	if(GPIO_Pin == MD6_I_Pin){
		md6_event = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
/* function uses timer events as a reference to generate the output-sync-signals during the measurement
* *htim: timer that triggered an interrupt */
	if(htim == &htim6){
		if(measRun){
			UpdateStartTrigger();
		}else{
			UpdateStopTrigger();
		}
	}

	if(htim == &htim2 && CheckDelay(measTime, allDevices[MD1].delay)){
		if(allDevices[MD1].signalform != M_Sequence){
			HAL_GPIO_TogglePin(allDevices[MD1].port_out, allDevices[MD1].pin_out);
		}else{
			HAL_GPIO_WritePin(allDevices[MD1].port_out, allDevices[MD1].pin_out, allDevices[MD1].mSeq[allDevices[MD1].mSeqCounter]);
			allDevices[MD1].mSeqCounter++;
			if(allDevices[MD1].mSeqCounter > maxSeqLength-1){
				HAL_GPIO_WritePin(allDevices[MD1].port_out, allDevices[MD1].pin_out, GPIO_PIN_SET);
				allDevices[MD1].mSeqCounter = 0;
			}
		}
	}
	if(htim == &htim3 && CheckDelay(measTime, allDevices[MD2].delay)){
		if(allDevices[MD2].signalform != M_Sequence){
			HAL_GPIO_TogglePin(allDevices[MD2].port_out, allDevices[MD2].pin_out);
		}else{
			HAL_GPIO_WritePin(allDevices[MD2].port_out, allDevices[MD2].pin_out, allDevices[MD2].mSeq[allDevices[MD2].mSeqCounter]);
			allDevices[MD2].mSeqCounter++;
			if(allDevices[MD2].mSeqCounter > maxSeqLength-1){
				HAL_GPIO_WritePin(allDevices[MD2].port_out, allDevices[MD2].pin_out, GPIO_PIN_SET);
				allDevices[MD2].mSeqCounter = 0;
			}
		}
	}
	if(htim == &htim4 && CheckDelay(measTime, allDevices[MD3].delay)){
		if(allDevices[MD3].signalform != M_Sequence){
			HAL_GPIO_TogglePin(allDevices[MD3].port_out, allDevices[MD3].pin_out);
		}else{
			HAL_GPIO_WritePin(allDevices[MD3].port_out, allDevices[MD3].pin_out, allDevices[MD3].mSeq[allDevices[MD3].mSeqCounter]);
			allDevices[MD3].mSeqCounter++;
			if(allDevices[MD3].mSeqCounter > maxSeqLength-1){
				HAL_GPIO_WritePin(allDevices[MD3].port_out, allDevices[MD3].pin_out, GPIO_PIN_SET);
				allDevices[MD3].mSeqCounter = 0;
			}
		}
	}
	if(htim == &htim7 && CheckDelay(measTime, allDevices[MD4].delay)){
		if(allDevices[MD4].signalform != M_Sequence){
			HAL_GPIO_TogglePin(allDevices[MD4].port_out, allDevices[MD4].pin_out);
		}else{
			HAL_GPIO_WritePin(allDevices[MD4].port_out, allDevices[MD4].pin_out, allDevices[MD4].mSeq[allDevices[MD4].mSeqCounter]);
			allDevices[MD4].mSeqCounter++;
			if(allDevices[MD4].mSeqCounter > maxSeqLength-1){
				HAL_GPIO_WritePin(allDevices[MD4].port_out, allDevices[MD4].pin_out, GPIO_PIN_SET);
				allDevices[MD4].mSeqCounter = 0;
			}
		}
	}
	if(htim == &htim9 && CheckDelay(measTime, allDevices[MD5].delay)){
		if(allDevices[MD5].signalform != M_Sequence){
			HAL_GPIO_TogglePin(allDevices[MD5].port_out, allDevices[MD5].pin_out);
		}else{
			HAL_GPIO_WritePin(allDevices[MD5].port_out, allDevices[MD5].pin_out, allDevices[MD5].mSeq[allDevices[MD5].mSeqCounter]);
			allDevices[MD5].mSeqCounter++;
			if(allDevices[MD5].mSeqCounter > maxSeqLength-1){
				HAL_GPIO_WritePin(allDevices[MD5].port_out, allDevices[MD5].pin_out, GPIO_PIN_SET);
				allDevices[MD5].mSeqCounter = 0;
			}
		}
	}
	if(htim == &htim10 && CheckDelay(measTime, allDevices[MD6].delay)){
		if(allDevices[MD6].signalform != M_Sequence){
			HAL_GPIO_TogglePin(allDevices[MD6].port_out, allDevices[MD6].pin_out);
		}else{
			HAL_GPIO_WritePin(allDevices[MD6].port_out, allDevices[MD6].pin_out, allDevices[MD6].mSeq[allDevices[MD6].mSeqCounter]);
			allDevices[MD6].mSeqCounter++;
			if(allDevices[MD6].mSeqCounter > maxSeqLength-1){
				HAL_GPIO_WritePin(allDevices[MD6].port_out, allDevices[MD6].pin_out, GPIO_PIN_SET);
				allDevices[MD6].mSeqCounter = 0;
			}
		}
	}
	if(htim == &htim11 && CheckDelay(measTime, allDevices[MD7].delay)){
		if(allDevices[MD7].signalform != M_Sequence){
			HAL_GPIO_TogglePin(allDevices[MD7].port_out, allDevices[MD7].pin_out);
		}else{
			HAL_GPIO_WritePin(allDevices[MD7].port_out, allDevices[MD7].pin_out, allDevices[MD7].mSeq[allDevices[MD7].mSeqCounter]);
			allDevices[MD7].mSeqCounter++;
			if(allDevices[MD7].mSeqCounter > maxSeqLength-1){
				HAL_GPIO_WritePin(allDevices[MD7].port_out, allDevices[MD7].pin_out, GPIO_PIN_SET);
				allDevices[MD7].mSeqCounter = 0;
			}
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
