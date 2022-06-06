/**
 ******************************************************************************
 * @file           : app.c
 * @brief          : Implementation of application
 * @author		   : Patrick Locatelli (patrick.locatelli@unibg.it)
 * @version		   : 1.0
 * @date    	   : 13 Feb 2020
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "app.h"
#include "log.h"

/* Private define ------------------------------------------------------------*/
#define LOG_NAME_SIZE	15

/* Private variables ---------------------------------------------------------*/
Winter_t winter = { .state = SYSTEM_STATE_STARTUP, .oldState =
		SYSTEM_STATE_ERROR, .period = 1000, .logSensors = 0, .cardMounted = 0,
		.reachedLogBufferEnd = 0 };
PeripheralsInitStatus_t initStatus = { .LSM6DSL = 0, .HTS221 = 0, .LPS22HH = 0,
		.BLE = 0, .MAX17048 = 0, .batteryLevel = -1 };
LatestData_t latestData;

uint32_t myTick;

/* Extern variables ----------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_memtomem_dma2_channel1;
extern DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
extern DMA_HandleTypeDef hdma_dfsdm1_flt0;
extern int32_t buffer2sd[2][BUFFER_SIZE];

extern char SDPath[4];

/* Private function prototypes -----------------------------------------------*/
void init_Winter(void);
uint8_t init_LSM6DSL(void);
uint8_t init_HTS221(void);
uint8_t init_LPS22HH(void);
uint8_t init_BluetoothLowEnergy(void);
uint8_t do_InitJob(void);
void do_IdleJob(void);
void do_LogJob(void);
void do_ErrorJob(void);
uint8_t updateData(void);
void set_LogSensors(uint8_t sensors);
void set_LogFrequency(uint16_t frequency);

/*
 * @brief  Get the current main job period.
 * @param  None.
 * @return The current main job period.
 */
uint16_t App_GetPeriod(void) {
	return winter.period;
}

/*
 * @brief Set the main job period.
 * @param p The new main job period.
 */
void App_SetPeriod(uint16_t p) {
	winter.period = p;
}

/*
 * @brief  Get the current system state.
 * @param  None.
 * @return The current system state.
 */
SystemState_e App_GetSystemState(void) {
	return winter.state;
}

/*
 * @brief  Set the system state.
 * @param  s The new system state.
 * @return The success of the operation.
 */
uint8_t App_SetSystemState(SystemState_e s) {
	if (s != App_GetSystemState()) {
		// State exiting conditions
		switch (winter.state) {
		case SYSTEM_STATE_INIT:
			IO_ResetLED(LED_BLUE);
			break;
		case SYSTEM_STATE_IDLE:
			IO_ResetLED(LED_GREEN | LED_BLUE);
			break;
		case SYSTEM_STATE_LOG:
			IO_ResetLED(LED_GREEN);
			break;
		case SYSTEM_STATE_ERROR:
			IO_ResetLED(LED_RED);
			break;
		default:
			break;
		}

		// State entering conditions
		switch (s) {
		case SYSTEM_STATE_INIT:
			IO_SetLED(LED_BLUE);
			break;
		case SYSTEM_STATE_IDLE:
			// Set the new app period
			App_SetPeriod(PERIOD_IDLE);
			break;
		case SYSTEM_STATE_LOG:
			// Check the SD presence
			if (!BSP_SD_IsDetected())
				return 0;

			// Try to initialize the log
			char logName[LOG_NAME_SIZE];
			RTC_GetDateTime_FormattedChar(logName);
			if (!Log_Create_DataFile(logName)) {
				return 0;
			}

			IO_SetLED(LED_GREEN);
			HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, buffer2sd, BUFFER_SIZE * 2);

			break;

		case SYSTEM_STATE_ERROR:
			App_SetPeriod(PERIOD_ERROR);
			break;
		default:
			break;
		}

		// Change the state
		winter.oldState = winter.state;
		winter.state = s;

		// Reset the tick counter
		myTick = 0;
	}

	return s == App_GetSystemState();
}

/*
 * @brief  Start the log of the selected sensors, at the selected frequency.
 * @param  sensors The sensors to be logged. Each bit corresponds to a specific
 * 				   sensor, according to the masks defined in app.h.
 * @param  frequency The log frequency.
 * @return The success of the operation.
 */
uint8_t App_StartLog(void) {

	// Enter the log state
	if (!App_SetSystemState(SYSTEM_STATE_LOG)) {
		App_SetSystemState(SYSTEM_STATE_ERROR);
		return 0;
	} else
		return 1;
}

/*
 * @brief  Stop the current log session.
 * @param  None.
 * @return The success of the operation.
 */
uint8_t App_StopLog() {
	uint8_t result = 1;
	result &= Log_Close_DataFile(1);
	HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
	result &= App_SetSystemState(SYSTEM_STATE_IDLE);
	return result;
}

/*
 * @brief  Initialize the system.
 * @param  None.
 * @return The success of the operation.
 */
uint8_t App_InitSystem(void) {
	uint8_t result = 1;
	myTick = 0;

	// Set the state as INIT
	result &= App_SetSystemState(SYSTEM_STATE_INIT);

	// Register and enable priority of the job interrupt
	HAL_NVIC_SetPriority(JOB_IRQn, JOB_IRQn_Priority, 0);
	HAL_NVIC_EnableIRQ(JOB_IRQn);

	return result;
}

/*
 * @brief  Perform the job associated to the current system state.
 * @param  None.
 */
void App_DoJob(void) {
	switch (winter.state) {
	case SYSTEM_STATE_INIT:
		do_InitJob();
		break;
	case SYSTEM_STATE_IDLE:
		do_IdleJob();
		break;
	case SYSTEM_STATE_LOG:
		do_LogJob();
		break;
	case SYSTEM_STATE_ERROR:
		do_ErrorJob();
		break;
	default:
		break;
	}
}

/*
 * @brief  Handler of the SysTick event.
 * @param  None.
 * @return None.
 */
void App_Handle_SysTick(void) {
	++myTick;

	// Schedule the main job
	if (myTick % App_GetPeriod() == 0)
		HAL_NVIC_SetPendingIRQ(JOB_IRQn);

	// Schedule the Bluetooth job
	if (myTick % 100 == 0)
		HAL_NVIC_SetPendingIRQ(BLE_JOB_IRQn);
}

/*
 * @brief  Handler of the Bluetooth connection succeeded event.
 * @param  None.
 * @return None.
 */
void App_Handle_BTConnected(void) {
	//TODO
}

/*
 * @brief  Handler of the Bluetooth disconnection succeeded event.
 * @param  None.
 * @return None.
 */
void App_Handle_BTDisconnected(void) {
	//TODO
}

/*
 * @brief  Initialize the main structure of Winter.
 * @param  None.
 */
void init_Winter(void) {
// Default settings
	winter.logSensors = 0;
	winter.cardMounted = 0;
	winter.reachedLogBufferEnd = 0;
	winter.logFrequency = 0;
}

/*
 * @brief  Initialize LSM6DSL device with the default settings.
 * @param  None.
 * @return The success of the operation.
 */
uint8_t init_LSM6DSL(void) {
	if (LSM6DSL_Check_WhoAmI()) {
		uint8_t result = 1;
		LSM6DSL_Init_t config;

		// Default settings
		config.axl_odr = LSM6DSL_ODR_416Hz;
		config.axl_fs = LSM6DSL_FS_XL_2g;
		config.axl_mode = LSM6DSL_XL_HM_MODE_LOW_NORMAL;
		config.gyro_odr = LSM6DSL_ODR_416Hz;
		config.gyro_fs = LSM6DSL_FS_G_250dps;
		config.gyro_mode = LSM6DSL_G_HM_MODE_LOW_NORMAL;
		config.bdu = LSM6DSL_BDU_BLOCKING;

		// Double-tap settings
		config.int_enable = LSM6DSL_INTERRPUTS_ENABLED;
		config.int_enabledAxis =
				(0x07 << LSM6DSL_REG_TAP_CFG_AXIS_ENABLED_SHIFT);	// x, y, z
		config.int_lir = LSM6DSL_LIR_ENABLED;
		/*
		 * Threshold [5 bits]: 	x * axl_fs / 2^5
		 * Shock [2 bits]:		x != 0? x * 8 / axl_odr  : 4 / axl_odr
		 * Quiet [2 bits]:		x != 0? x * 4 / axl_odr  : 2 / axl_odr
		 * Duration [3 bits]:	x != 0? x * 32 / axl_odr : 16 / axl_odr
		 */
		config.int_tapThs = 0x0A;	// 625 mg
		config.int_tapShock = 0x03;	// 57.7 ms
		config.int_tapQuiet = 0x03;	// 28.8 ms
		config.int_tapDur = 0x05;	// 384.6 ms
		config.int_SDTapEnable = LSM6DSL_SDTAP_ENABLED;
		config.int1_dt = LSM6DSL_INT1_DTAP_ENABLED;

		result &= LSM6DSL_Config(&config);
		result &= LSM6DSL_Reset_DoubleTapInterrupt();

		return result;

	} else
		return 0;
}

/*
 * @brief  Initialize HTS221 device with the default settings.
 * @param  None.
 * @return The success of the operation.
 */
uint8_t init_HTS221(void) {
	if (HTS221_Check_WhoAmI())
		return HTS221_Config();
	else
		return 0;
}

/*
 * @brief  Initialize LPS22HH device with the default settings.
 * @param  None.
 * @return The success of the operation.
 */
uint8_t init_LPS22HH(void) {
	if (LPS22HH_Check_WhoAmI()) {
		LPS22HH_Init_t config;

		// Default settings
		config.odr = LPS22HH_ODR_10Hz;
		config.bdu = LPS22HH_BDU_BLOCKING;
		config.auto_inc_en = LPS22HH_IF_ADD_INC_ENABLED;
		config.low_noise_en = LPS22HH_LOW_NOISE_DISABLED;

		return LPS22HH_Config(&config);
	} else
		return 0;
}

/*
 * @brief  Initialize Bluetooth Low Energy module.
 * @param  None.
 * @return The success of the operation.
 */
uint8_t init_BluetoothLowEnergy(void) {
	/*Configure GPIO pin : BLE_CS_Pin */
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = BLE_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

	// Initialize the module
	return BLE_Init();
}

/*
 * @brief  Initialization job:
 * 			- initialize the main structure;
 *			- initialize the peripherals and mount SD card;
 * 			- write the initialization status file.
 * @param  None.
 * @return The success of the operation.
 */
uint8_t do_InitJob(void) {
	// Initialize the Winter main structure
	init_Winter();

	// Initialize all peripherals
	initStatus.LSM6DSL = init_LSM6DSL();
	initStatus.HTS221 = init_HTS221();
	initStatus.LPS22HH = init_LPS22HH();
	initStatus.BLE = init_BluetoothLowEnergy();
	initStatus.MAX17048 = MAX17048_Read_SOC(&initStatus.batteryLevel);

	/*
	 * This part of code is not needed, as the BSP_SD_Init() function is
	 * already called when mounting the file system with f_mount() (see below)
	 *
	 // Configure the SD card
	 if (BSP_SD_Init() != MSD_OK) {
	 App_SetSystemState(SYSTEM_STATE_ERROR);
	 return 0;
	 }
	 */

	// Try to mount SD card
	winter.cardMounted = (f_mount(&FatFsDisk, SDPath, 1) == FR_OK);
	if (!winter.cardMounted) {
		App_SetSystemState(SYSTEM_STATE_ERROR);
		return 0;
	}

	// Create an init status log file
	if (!Log_Create_InitStatusFile(&initStatus)) {
		App_SetSystemState(SYSTEM_STATE_ERROR);
		return 0;
	}

	// Register DMA Callback for Mem2Mem DMA Transfer
	void (*funPtr)(DMA_HandleTypeDef*);
	funPtr = &MicToMemTxCplt;
	HAL_DMA_RegisterCallback(&hdma_dfsdm1_flt0,
			HAL_DMA_XFER_HALFCPLT_CB_ID, funPtr);
	HAL_DMA_RegisterCallback(&hdma_dfsdm1_flt0,
			HAL_DMA_XFER_CPLT_CB_ID, funPtr);

	// Set the state as IDLE
	App_SetSystemState(SYSTEM_STATE_IDLE);

	return (initStatus.HTS221 & initStatus.LPS22HH
			& initStatus.BLE);
}

/*
 * @brief  Idle job:
 * 			- check the idle-to-sleep timeout
 * 			- blink the blue LED.
 * @param  None.
 * @return None.
 */
void do_IdleJob(void) {
	// Blink the LED after a certain amount of function calls
	if ((myTick % LED_BLINK_PERIOD_IDLE) == 0) {
		IO_SetLED(LED_BLUE);
		HAL_Delay(10);
		IO_ResetLED(LED_BLUE);
	}
}

/*
 * @brief  Log job:
 * 			- update the accelerometer and gyroscope readings;
 * 			- write the current readings to the log file;
 * 			- toggle the green LED.
 * @param  None.
 * @return None.
 */
void do_LogJob(void) {
__NOP();
}

/*
 * @brief  Error job:
 * 			- toggle the red LED.
 * @param  None.
 * @return None.
 */
void do_ErrorJob(void) {
	IO_ToggleLED(LED_RED);
}

/*
 * @brief  Update the readings from accelerometer and gyroscope.
 * @param  None.
 * @return The success of the operation.
 */
uint8_t updateData(void) {
	uint8_t result = 1;

	// Read from the inertial sensor only if required
	if ((winter.logSensors & SENS_AXL) != 0)
		result &= LSM6DSL_Read_Axl(latestData.axlDigits_sample,
				latestData.axl_sample);

	return result;
}

/*
 * @brief  Set the sensors whose data have to be logged.
 * @param  sensors The sensors to be logged. Each bit corresponds to a specific
 * 				   sensor, according to the masks defined in app.h.
 * @return None.
 */
void set_LogSensors(uint8_t sensors) {
	if (sensors != 0)
		winter.logSensors = sensors;
	else
		winter.logSensors = DEFAULT_LOG_SENSORS;
}

/*
 * @brief  Set the frequency at which data have to be logged.
 * @param  frequency The log frequency.
 * @return None.
 */
void set_LogFrequency(uint16_t frequency) {
	if (frequency > 0)
		winter.logFrequency = frequency;
	else
		winter.logFrequency = DEFAULT_LOG_PERIOD;
}
