/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "pdm2pcm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "ili9341.h"
#include "arm_math.h"
#include "test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	BUFFER_OFFSET_NONE, BUFFER_OFFSET_HALF,
} Buffer_offset;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST                    0                                               /* Test image */

#define AUDIO_FS                8000.0f                                         /* Sampling frequency */
#define RX_BUFFER_SIZE          128                                             /* Length of audio receive buffer */
#define DEC_BUFFER_SIZE         16                                              /* Length of audio buffer after decimation */
#define PCM_BUFFER_SIZE         1024                                            /* Length of audio PCM buffer */
#define NORMALIZED_BUFFER_SIZE  1024                                            /* Length of audio normalized buffer */
#define FFT_LENGTH              1024                                            /* Length of FFT samples */
#define FFT_BUFFER_SIZE         (FFT_LENGTH + 2)                                /* Length of FFT output buffer */
#define FFT_MAG_BUFFER_SIZE     (FFT_BUFFER_SIZE / 2)                           /* Length of FFT magnitude buffer*/
#define FFT_FR                  (2 * AUDIO_FS / FFT_LENGTH)                     /* FFT frequency resolution */

#define AUDIO_FLAG_HALF         0x01                                            /* Flag for audio half callback */
#define AUDIO_FLAG_FULL         0x02                                            /* Flag for audio full callback */
#define AUDIO_FLAG_COMPL        0x04                                            /* Flag for audio buffer completed  */
#define FFT_FLAG_COMPL          0x08                                            /* Flag for FFT buffer completed */

#define MAX_AUDIO_TASK_COUNTER  (PCM_BUFFER_SIZE / DEC_BUFFER_SIZE)             /* Max number of audio task execution */

#define OFFSET_Y                40                                              /* Offset of display */
#define SCALE_FACTOR            2                                               /* Scale factor of display */
#define FONT_WIDTH              7                                               /* Font width */
#define STRING_SIZE             12                                              /* Size of string buffer */
#define STRING_POS_X            (ILI9341_WIDTH - (STRING_SIZE * FONT_WIDTH))    /* Position X of string on display */
#define STRING_POS_Y            (ILI9341_HEIGHT / 6)                            /* Position Y of string on display */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SWAP_ENDIANESS(x) (uint16_t)( ((x) << 8) | ((x) >> 8) )
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

SPI_HandleTypeDef hspi1;

/* Definitions for AudioCapture */
osThreadId_t AudioCaptureHandle;
const osThreadAttr_t AudioCapture_attributes = { .name = "AudioCapture",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for FFTProcessing */
osThreadId_t FFTProcessingHandle;
const osThreadAttr_t FFTProcessing_attributes =
		{ .name = "FFTProcessing", .stack_size = 256 * 4, .priority =
				(osPriority_t) osPriorityAboveNormal, };
/* Definitions for DisplayOutput */
osThreadId_t DisplayOutputHandle;
const osThreadAttr_t DisplayOutput_attributes = { .name = "DisplayOutput",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for NormalizedMutex */
osMutexId_t NormalizedMutexHandle;
const osMutexAttr_t NormalizedMutex_attributes = { .name = "NormalizedMutex" };
/* Definitions for FFTMagMutex */
osMutexId_t FFTMagMutexHandle;
const osMutexAttr_t FFTMagMutex_attributes = { .name = "FFTMagMutex" };
/* Definitions for AudioCallbackReady */
osEventFlagsId_t AudioCallbackReadyHandle;
const osEventFlagsAttr_t AudioCallbackReady_attributes = { .name =
		"AudioCallbackReady" };
/* Definitions for FFTReady */
osEventFlagsId_t FFTReadyHandle;
const osEventFlagsAttr_t FFTReady_attributes = { .name = "FFTReady" };
/* Definitions for AudioBuffReady */
osEventFlagsId_t AudioBuffReadyHandle;
const osEventFlagsAttr_t AudioBuffReady_attributes =
		{ .name = "AudioBuffReady" };
/* USER CODE BEGIN PV */
arm_rfft_fast_instance_f32 fft_audio_instance = { 0 }; /* Instance FFT structure */

uint16_t rx_buff[RX_BUFFER_SIZE] = { 0 }; /* Audio receive buffer */
float32_t normalized_buff[NORMALIZED_BUFFER_SIZE] = { 0 }; /* Audio normalized buffer */

float32_t hann_table[NORMALIZED_BUFFER_SIZE] = { 0 }; /* Hann window multipliers */
float32_t fft_mag_buff[FFT_MAG_BUFFER_SIZE] = { 0 }; /* FFT Magnitude buffer */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_I2S2_Init(void);
static void MX_SPI1_Init(void);
void AudioCaptureTask(void *argument);
void FFTProcessingTask(void *argument);
void DisplayOutputTask(void *argument);

/* USER CODE BEGIN PFP */
void init_hann_window(float32_t *hann_table);
void pdm_to_pcm(const uint16_t *pdm_buff, uint16_t *pcm_buff,
		const Buffer_offset buffer_offset);
void normalize_buff(const int16_t *pcm_buff, float32_t *normalized_buff,
		const size_t buff_index);
void apply_hann_window(const float32_t *normalized_buff,
		const float32_t *hann_table, float32_t *windowed_buff);
void fft_compute(float32_t *windowed_buff, float32_t *fft_mag_buff);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_CRC_Init();
	MX_I2S2_Init();
	MX_PDM2PCM_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */

	/* Display driver initialize */
	ILI9341_Init();
	ILI9341_FillScreen(ILI9341_BLACK);
#if TEST
	ILI9341_DrawImage(0, 0, 240, 240, (uint16_t*) test_img_240x240);
#endif

	/* DMA initialize */
	HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*) rx_buff, RX_BUFFER_SIZE);

	/* FFT initialize */
	arm_rfft_fast_init_f32(&fft_audio_instance, FFT_LENGTH);

	/* Init Hann multipliers */
	init_hann_window(hann_table);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of NormalizedMutex */
	NormalizedMutexHandle = osMutexNew(&NormalizedMutex_attributes);

	/* creation of FFTMagMutex */
	FFTMagMutexHandle = osMutexNew(&FFTMagMutex_attributes);

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
	/* creation of AudioCapture */
	AudioCaptureHandle = osThreadNew(AudioCaptureTask, NULL,
			&AudioCapture_attributes);

	/* creation of FFTProcessing */
	FFTProcessingHandle = osThreadNew(FFTProcessingTask, NULL,
			&FFTProcessing_attributes);

	/* creation of DisplayOutput */
	DisplayOutputHandle = osThreadNew(DisplayOutputTask, NULL,
			&DisplayOutput_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* creation of AudioCallbackReady */
	AudioCallbackReadyHandle = osEventFlagsNew(&AudioCallbackReady_attributes);

	/* creation of FFTReady */
	FFTReadyHandle = osEventFlagsNew(&FFTReady_attributes);

	/* creation of AudioBuffReady */
	AudioBuffReadyHandle = osEventFlagsNew(&AudioBuffReady_attributes);

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	__HAL_CRC_DR_RESET(&hcrc);
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2S2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S2_Init(void) {

	/* USER CODE BEGIN I2S2_Init 0 */

	/* USER CODE END I2S2_Init 0 */

	/* USER CODE BEGIN I2S2_Init 1 */

	/* USER CODE END I2S2_Init 1 */
	hi2s2.Instance = SPI2;
	hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
	hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2S2_Init 2 */

	/* USER CODE END I2S2_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, SPI1_RES_Pin | SPI1_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_DC_GPIO_Port, SPI1_DC_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_RES_Pin SPI1_CS_Pin */
	GPIO_InitStruct.Pin = SPI1_RES_Pin | SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI1_DC_Pin */
	GPIO_InitStruct.Pin = SPI1_DC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_DC_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	/* Only when the interrupt is triggered by I2S connected to the microphone */
	if (hi2s->Instance == SPI2) {
		osEventFlagsSet(AudioCallbackReadyHandle, AUDIO_FLAG_HALF);
	}
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	/* Only when the interrupt is triggered by I2S connected to the microphone */
	if (hi2s->Instance == SPI2) {
		osEventFlagsSet(AudioCallbackReadyHandle, AUDIO_FLAG_FULL);
	}
}

void init_hann_window(float32_t *hann_table) {
	/* Initialize array with hann windows values */
	for (size_t i = 0; i < NORMALIZED_BUFFER_SIZE; i++) {
		hann_table[i] = 0.5f
				* (1.0f - cosf(2.0f * PI * i / (NORMALIZED_BUFFER_SIZE - 1)));
	}
}

void pdm_to_pcm(const uint16_t *pdm_buff, uint16_t *pcm_buff,
		const Buffer_offset buffer_offset) {
	// pdm_buff - source PDM
	// pdm_buff - destination PCM
	static uint16_t pdm_swap_buff[RX_BUFFER_SIZE / 2] = { 0 };

	/* PDM swap endianness */
	for (size_t i = 0; i < RX_BUFFER_SIZE / 2; i++) {
		size_t index = i + (RX_BUFFER_SIZE / 2) * buffer_offset;
		pdm_swap_buff[i] = SWAP_ENDIANESS(pdm_buff[index]);
	}

	/* PDM to PCM filter */
	PDM_Filter(pdm_swap_buff, pcm_buff, &PDM1_filter_handler);
}

void normalize_buff(const int16_t *pcm_buff, float32_t *normalized_buff,
		const size_t buff_index) {
	// pcm_buff - source PCM
	// normalized_buff - destination normalized PCM
	for (size_t i = 0; i < DEC_BUFFER_SIZE; i++) {
		int16_t int_val = (int16_t) pcm_buff[buff_index + i];
		float32_t float_val = (float32_t) int_val / (float32_t) INT16_MAX;
		if ((i > 0) && (float_val > 0.2 || float_val < -0.2)) {
			float_val = normalized_buff[buff_index + i - 1];
		}
		normalized_buff[buff_index + i] = float_val;
	}
}

void apply_hann_window(const float32_t *normalized_buff,
		const float32_t *hann_table, float32_t *windowed_buff) {
	/* Hann window is applied to reduce spectral leakage */
	for (size_t i = 0; i < NORMALIZED_BUFFER_SIZE; i++) {
		windowed_buff[i] = normalized_buff[i] * hann_table[i];
	}
}

void fft_compute(float32_t *windowed_buff, float32_t *fft_mag_buff) {
	static float32_t fft_out_buff[FFT_BUFFER_SIZE] = { 0 };
	/* Compute FFT */
	arm_rfft_fast_f32(&fft_audio_instance, windowed_buff, fft_out_buff, 0);

	/* Calculate magnitude from real and imaginary part of FFT */
	arm_cmplx_mag_f32(fft_out_buff, fft_mag_buff, FFT_MAG_BUFFER_SIZE);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_AudioCaptureTask */
/**
 * @brief  Function implementing the AudioCapture thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_AudioCaptureTask */
void AudioCaptureTask(void *argument) {
	/* USER CODE BEGIN 5 */
	uint16_t audio_task_counter = 0; /* Counts how many times task executed */
	static uint16_t dec_buff[DEC_BUFFER_SIZE] = { 0 }; /* Audio buffer after decimation */
	static int16_t pcm_buff[PCM_BUFFER_SIZE] = { 0 }; /* PCM audio buffer */
	Buffer_offset buffer_offset = BUFFER_OFFSET_NONE; /* Offset of writing to buffer */

	/* Infinite loop */
	for (;;) {
		uint32_t flags = osEventFlagsWait(AudioCallbackReadyHandle,
		AUDIO_FLAG_HALF | AUDIO_FLAG_FULL,
		osFlagsWaitAny, osWaitForever);

		/* Buffer differs depending on which interrupt was executed */
		if (flags & AUDIO_FLAG_HALF) {
			buffer_offset = BUFFER_OFFSET_HALF;
		} else if (flags & AUDIO_FLAG_FULL) {
			buffer_offset = BUFFER_OFFSET_NONE;
		}

		/* Convert PDM sound to PCM sound */
		pdm_to_pcm(rx_buff, dec_buff, buffer_offset);

		const size_t buff_index = audio_task_counter * DEC_BUFFER_SIZE;

		/* Copy converted data to transmit buffer */
		memcpy(&pcm_buff[buff_index], dec_buff,
		DEC_BUFFER_SIZE * 2);

		/* Normalize PCM buffer */
		osMutexAcquire(NormalizedMutexHandle, osWaitForever);
		normalize_buff(pcm_buff, normalized_buff, buff_index);
		osMutexRelease(NormalizedMutexHandle);

		audio_task_counter++;

		/* If buffer is full set flag */
		if (audio_task_counter == MAX_AUDIO_TASK_COUNTER) {
			osEventFlagsSet(AudioBuffReadyHandle, AUDIO_FLAG_COMPL);
			audio_task_counter = 0;
		}
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_FFTProcessingTask */
/**
 * @brief Function implementing the FFTProcessing thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_FFTProcessingTask */
void FFTProcessingTask(void *argument) {
	/* USER CODE BEGIN FFTProcessingTask */
	static float32_t windowed_buff[NORMALIZED_BUFFER_SIZE] = { 0 }; /* Normalized and windowed buffer */
	/* Infinite loop */
	for (;;) {
		osEventFlagsWait(AudioBuffReadyHandle,
		AUDIO_FLAG_COMPL,
		osFlagsWaitAny, osWaitForever);

		/* Apply Hann window */
		osMutexAcquire(NormalizedMutexHandle, osWaitForever);
		apply_hann_window(normalized_buff, hann_table, windowed_buff);
		osMutexRelease(NormalizedMutexHandle);

		/* Perform FFT */
		osMutexAcquire(FFTMagMutexHandle, osWaitForever);
		fft_compute(windowed_buff, fft_mag_buff);
		osMutexRelease(FFTMagMutexHandle);

		osEventFlagsSet(FFTReadyHandle, FFT_FLAG_COMPL);
	}
	/* USER CODE END FFTProcessingTask */
}

/* USER CODE BEGIN Header_DisplayOutputTask */
/**
 * @brief Function implementing the DisplayOutput thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_DisplayOutputTask */
void DisplayOutputTask(void *argument) {
	/* USER CODE BEGIN DisplayOutputTask */
	static int32_t fft_db_buff[ILI9341_HEIGHT] = { 0 }; /* Values from current execution of this task */
	static int32_t fft_prev_db_buff[ILI9341_HEIGHT] = { 0 }; /* Values from previous execution of this task */
	/* Infinite loop */
	for (;;) {
		osEventFlagsWait(FFTReadyHandle,
		FFT_FLAG_COMPL,
		osFlagsWaitAny, osWaitForever);

		int32_t peak_index = 0;

		osMutexAcquire(FFTMagMutexHandle, osWaitForever);

		for (size_t i = 0; i < ILI9341_HEIGHT; i++) {
			/* Get average and FFT magnitude to decibel scale */
			float32_t avg_val = (fft_mag_buff[i * 2] + fft_mag_buff[i * 2 + 1])
					/ 2;
			int32_t log_val = (int32_t) (20 * log10f(avg_val));
			int32_t scaled_val = (log_val + OFFSET_Y) * SCALE_FACTOR;
			fft_db_buff[i] = scaled_val;

			/* Compare with previous result to reduce drawing */
			if (fft_prev_db_buff[i] != fft_db_buff[i]) {
				size_t display_val = ILI9341_HEIGHT - fft_db_buff[i];
				size_t prev_display_val = ILI9341_HEIGHT - fft_prev_db_buff[i];
				ILI9341_DrawPixel(i, display_val, ILI9341_WHITE);
				ILI9341_DrawPixel(i, prev_display_val, ILI9341_BLACK);
			}
			fft_prev_db_buff[i] = fft_db_buff[i];

			/* Find peak index */
			if (fft_db_buff[peak_index] < fft_db_buff[i]) {
				peak_index = i;
			}
		}

		/* Write peak value on display */
		float32_t peak_value = peak_index * FFT_FR;
		char string_peak[STRING_SIZE] = { 0 };
		snprintf(string_peak, sizeof(string_peak), "%8.2f Hz", peak_value);
		ILI9341_WriteString(STRING_POS_X, STRING_POS_Y, string_peak, Font_7x10,
		ILI9341_WHITE, ILI9341_BLACK);

		osMutexRelease(FFTMagMutexHandle);
	}
	/* USER CODE END DisplayOutputTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM5 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM5) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
