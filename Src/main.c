/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#if HAVE_USBCDC
#include "usbd_cdc.h"	//just for the XXX_USBCDC_PresenceHack()
#endif
#include "system_interfaces.h"
#include "serial_devices.h"
#include "util_circbuff2.h"

#include "lamps.h"
#include "task_notification_bits.h"

#include "task_monitor.h"


//get the debug cycle counter as a high-resolution clock
uint32_t inline getCyCnt() { return DWT->CYCCNT; }


#ifndef COUNTOF
#define COUNTOF(arr) (sizeof(arr)/sizeof(arr[0]))
#endif

#if ! HAVE_UART1 && ! HAVE_USBCDC
#error You must set at least one of HAVE_UART1 HAVE_USBCDC to 1 in project settings
#endif

//This controls whether we use the FreeRTOS heap implementation to also provide
//the libc malloc() and friends.
#define USE_FREERTOS_HEAP_IMPL 1


//resource usage statistics collected in default task for production tuning
#ifdef DEBUG
volatile size_t g_nHeapFree;
volatile size_t g_nMinEverHeapFree;
#if HAVE_UART1
volatile int g_nMaxUART1TxQueue;
volatile int g_nMaxUART1RxQueue;
#endif
#if HAVE_USBCDC
volatile int g_nMaxCDCTxQueue;
volatile int g_nMaxCDCRxQueue;
#endif
volatile int g_nMinStackFreeDefault;
volatile int g_nMinStackFreeMonitor;
//XXX others
#endif

#if USE_FREERTOS_HEAP_IMPL

#if configAPPLICATION_ALLOCATED_HEAP
//we define our heap (to be used by FreeRTOS heap_4.c implementation) to be
//exactly where we want it to be.
__attribute__((aligned(8))) 
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#endif
//we implemented a 'realloc' for a heap_4 derived implementation
extern void* pvPortRealloc( void* pvOrig, size_t xWantedSize );
//we implemented a 'heapwalk' function
typedef int (*CBK_HEAPWALK) ( void* pblk, uint32_t nBlkSize, int bIsFree, void* pinst );
extern int vPortHeapWalk ( CBK_HEAPWALK pfnWalk, void* pinst );

//'wrapped functions' for library interpositioning
//you must specify these gcc (linker-directed) options to cause the wrappers'
//delights to be generated:

// -Wl,--wrap,malloc -Wl,--wrap,free -Wl,--wrap,realloc -Wl,--wrap,calloc -Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_calloc_r

//hmm; can I declare these 'inline' and save a little code and stack?
void* __wrap_malloc ( size_t size ) { return pvPortMalloc ( size ); }
void __wrap_free ( void* pv ) { vPortFree ( pv ); }
void* __wrap_realloc ( void* pv, size_t size ) { return pvPortRealloc ( pv, size ); }

void* __wrap__malloc_r ( struct _reent* r, size_t size ) { return pvPortMalloc ( size ); }
void __wrap__free_r ( struct _reent* r, void* pv ) { vPortFree ( pv ); }
void* __wrap__realloc_r ( struct _reent* r, void* pv, size_t size ) { return pvPortRealloc ( pv, size ); }

#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//enable the core debug cycle counter to be used as a precision timer
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	//do a dummy alloc to cause the heap to be init'ed and so the memory stats as well
	vPortFree ( pvPortMalloc ( 0 ) );

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

#if HAVE_USBCDC
	//if you get a linker fail on the following, it is because some manual
	//changes to:
	//  .\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Inc\usbd_cdc.h
	//  .\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Src\usbd_cdc.c
	//  .\Src\usbd_cdc_if.c
	//must be applied.  There are backups of those files to help with that.
	//This has to be done manually, because the changes are in tool generated
	//code that gets overwritten when you re-run STM32CubeMX.  The nature of
	//those changes are such that when they are overwritten, you will still
	//be able to build but stuff won't work at runtime.  This hack will cause
	//the build to fail if you forget to merge those changes back on, thus
	//prompting you to do so.
	//There is a #fixups/fixup.bat to help with this.
	//Sorry for the inconvenience, but I don't think there is any better way
	//of making it obvious that this chore simply must be done.
	XXX_USBCDC_PresenceHack();	//this does nothing real; do not delete
#endif
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

   if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
    Error_Handler();  
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_PWR_EnableBkUpAccess();
  LL_RCC_ForceBackupDomainReset();
  LL_RCC_ReleaseBackupDomainReset();
  LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
  while(LL_RCC_LSE_IsReady() != 1)
  {
    
  }
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
  LL_RCC_EnableRTC();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_SetSystemCoreClock(72000000);
  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5);
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  LL_RTC_InitTypeDef RTC_InitStruct = {0};
  LL_RTC_TimeTypeDef RTC_TimeStruct = {0};

    LL_PWR_EnableBkUpAccess();
    /* Enable BKP CLK enable for backup registers */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_BKP);
  /* Peripheral clock enable */
  LL_RCC_EnableRTC();

  /* RTC interrupt Init */
  NVIC_SetPriority(RTC_Alarm_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(RTC_Alarm_IRQn);

  /* USER CODE BEGIN RTC_Init 1 */
	//XXX code to avoid resetting on every reboot goes here
  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC and set the Time and Date 
  */
  RTC_InitStruct.AsynchPrescaler = 0xFFFFFFFFU;
  LL_RTC_Init(RTC, &RTC_InitStruct);
  LL_RTC_SetAsynchPrescaler(RTC, 0xFFFFFFFFU);
  /** Initialize RTC and set the Time and Date 
  */
  RTC_TimeStruct.Hours = 0;
  RTC_TimeStruct.Minutes = 0;
  RTC_TimeStruct.Seconds = 0;
  LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_TimeStruct);
  /** Enable the Alarm A 
  */
  LL_RTC_EnableIT_ALR(RTC);
  /* USER CODE BEGIN RTC_Init 2 */
	//goofy CubeMX will not let you explicitly set the prescaler above 127 in
	//the UI, and the 'auto compute' does nothing other than set it to
	//0xFFFFFFFFU, which does nothing (and the max value is 0x000FFFFF, anyway)
	//and goofy code above inits it via LL_RTC_Init(), then attempts to set it
	//again via LL_RTC_SetAsynchPrescaler(), which won't work, anyway.
	if (LL_RTC_EnterInitMode(RTC) != ERROR)
	{
		//we are using a 32768 Hz LSE xtal.  You could reduce the divider to
		//have sub-second resolution, but you will need to keep that in mind
		//elsewhere when reading the clock, and anyway you'll not be able to
		//have decimal precision since it's not a multiple of 10.
		LL_RTC_SetAsynchPrescaler(RTC, 32767);	//we are using LSE xtal
		LL_RTC_ExitInitMode(RTC);
	}
	else
	{
		//horror
	}

	//XXX end of code to avoid resetting on every reboot goes here

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(TWIGGLE_GPIO_Port, TWIGGLE_Pin);

  /**/
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3 
                          |LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7 
                          |LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_10|LL_GPIO_PIN_11 
                          |LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15 
                          |LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6 
                          |LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = TWIGGLE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(TWIGGLE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


//====================================================


//this is made a function simply for tidiness, and locals lifetime
void __startWorkerTasks ( void )
{
	//kick off the monitor thread, which handles the user interactions
	{
	osThreadStaticDef(taskMonitor, thrdfxnMonitorTask, osPriorityNormal, 0, COUNTOF(g_tbMonitor), g_tbMonitor, &g_tcbMonitor);
	g_thMonitor = osThreadCreate(osThread(taskMonitor), NULL);
	}
}


//====================================================
//miscellaneous hooks of our creation


//in this logic, we prefer USBCDC over USART1
#if HAVE_USBCDC
#define PREFER_USBCDC 1
#elif HAVE_UART1
#define PREFER_USART1 1
#endif



#if PREFER_USBCDC

//well-discplined serial clients will assert DTR, and we
//can use that as an indication that a client application
//opened the port.
//NOTE:  These lines are often also set to an initial state
//by the host's driver, so do not consider these to be
//exclusively an indication of a client connecting.  Hosts
//usually will deassert these signals when this device
//enumerates.  Lastly, there is no guarantee that a client
//will assert DTR, so it's not 100% guarantee, just a pretty
//good indicator.
//NOTE:  we are in an ISR at this time
void USBCDC_DTR ( int bAssert )
{
	Monitor_ClientConnect ( bAssert );
}

//(unneeded)
//void USBCDC_RTS ( int bAssert ) { }
#endif




#if PREFER_USBCDC
void USBCDC_DataAvailable ( void )
#elif PREFER_USART1
void UART1_DataAvailable ( void )
#endif
#if PREFER_USBCDC || PREFER_USART1
{
	//this notification is required because our Monitor is implemented with the
	//non-blocking command interface, so we need to know when to wake and bake.
	Monitor_DAV();
}
#endif


#if PREFER_USBCDC
void USBCDC_TransmitEmpty ( void )
#elif PREFER_USART1
void UART1_TransmitEmpty ( void )
#endif
#if PREFER_USBCDC || PREFER_USART1
{
	//we don't really need this, but here's how you do it
	Monitor_TBMT();
}
#endif



//====================================================
//FreeRTOS hooks



void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	called if a stack overflow is detected. */
	volatile int i = 0;
	(void)i;
}



void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created. It is also called by various parts of the
	demo application. If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	volatile int i = 0;
	(void)i;
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
    
                 
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */

	//crank up serial ports
#if HAVE_UART1
	UART1_Init();	//UART1, alternative monitor
#endif
#if HAVE_USBCDC
	USBCDC_Init();	//CDC == monitor
#endif

	//bind the interfaces to the relevant devices
	//these 'HAVE_xxx' macros are in the preprocessor defs of the project
#if PREFER_USBCDC
	//we'll prefer the USB CDC if we've defined support for both
	g_pMonitorIOIf = &g_pifCDC;		//monitor is on USB CDC
#elif PREFER_USART1
	g_pMonitorIOIf = &g_pifUART1;	//monitor is on UART1
#endif


/*
	//Infinite loop
	for(;;)
	{
		_ledToggleGn();
		osDelay(500);
	}
*/


	//light some lamps on a countdown
	LightLamp ( 1000, &g_lltGn, _ledOnGn );

	//start up worker threads
	__startWorkerTasks();

	//================================================
	//temporary test crap
	{
	volatile size_t nPushed;

#if PREFER_USART1 && defined(DEBUG)
	//the uart1 monitor is for my debugging convenience, but it doesn't have a
	//'client connected' event, so squirt out a string to make it obvious we
	//are live
	g_pifUART1._transmitCompletely ( &g_pifUART1, "Hi, there!\r\n> ", -1, 1000 );
#endif

	(void) nPushed;
	nPushed = 0;	//(just for breakpoint)
	}

	//================================================
	//continue running this task
	//This task, the 'default' task, was generated by the tool, and it's easier
	//to just keep it than to fight the tool to destroy it (though some of that
	//fighting can be made a little easier if it was dynamically allocated,
	//then just exited).  We repurpose it after init to handle the lamps and
	//periodically sample performance data (useful for tuning pre-release).

	//Infinite loop
	uint32_t msWait = 1000;
	for(;;)
	{
		//wait on various task notifications
		uint32_t ulNotificationValue;
		BaseType_t xResult = xTaskNotifyWait( pdFALSE,	//Don't clear bits on entry.
				0xffffffff,	//Clear all bits on exit.
				&ulNotificationValue,	//Stores the notified value.
				pdMS_TO_TICKS(msWait) );
		if( xResult == pdPASS )
		{
			//the lights have changed
			if ( ulNotificationValue & TNB_LIGHTSCHANGED )
			{
				//YYY could do something, but we don't need to
			}
		}
		else	//timeout on wait
		{
			//YYY could do things to do on periodic idle timeout
		}

#ifdef DEBUG
		//these are to tune the freertos heap size; if we have a heap
#if USE_FREERTOS_HEAP_IMPL
		g_nHeapFree = xPortGetFreeHeapSize();
		g_nMinEverHeapFree = xPortGetMinimumEverFreeHeapSize();
#else
		g_nMinEverHeapFree = (char*)platform_get_last_free_ram( 0 ) - (char*)platform_get_first_free_ram( 0 );
#endif
#if HAVE_UART1
		g_nMaxUART1TxQueue = UART1_txbuff_max();
		g_nMaxUART1RxQueue = UART1_rxbuff_max();
#endif
#if HAVE_USBCDC
		g_nMaxCDCTxQueue = CDC_txbuff_max();
		g_nMaxCDCRxQueue = CDC_rxbuff_max();
#endif
		//free stack space measurements
		g_nMinStackFreeDefault = uxTaskGetStackHighWaterMark ( defaultTaskHandle );
		g_nMinStackFreeMonitor = uxTaskGetStackHighWaterMark ( g_thMonitor );
		//XXX others
#endif
		
		//turn out the lights, the party's over
		uint32_t now = HAL_GetTick();
		uint32_t remMin = 0xffffffff;	//nothing yet
		ProcessLightOffTime ( now, &remMin, &g_lltGn, _ledOffGn );

		//don't wait longer than 3 sec
		if ( remMin > 3000 )
			remMin = 3000;
		
		msWait = remMin;
	}

  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	volatile int i = 0;
	(void)i;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	volatile int i = 0;
	(void)i;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
