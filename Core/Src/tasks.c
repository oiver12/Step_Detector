#include "lsm6dsl_reg.h"
#include "tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "stm32l4xx_hal.h"
#include "QuaternionFilter.h"
#include "lps22hb.h"
#include "Kalman/kalman.h"
#include "stm32l475e_iot01_qspi.h"
#include "WiFi/es_wifi.h"
#include "WiFi/wifi.h"

#define R_SPECIFIC 287.053
#define GRAVITY_G 9.80665
#define TEMPGRAD_L 0.0065

typedef enum {
	isWaiting,
	isSaving,
	isReading
} state_t;

typedef struct {
	float acc;
	float diffHeight;
	float kalmanHeight;
	float kalmanVertVel;
	float kalmanVertAcc;
	uint32_t time;
} measureData_t;

// Constants
#define BLOCK_SIZE   0x1000  // 4096 bytes
#define PAGE_SIZE    0x0100  // 256 bytes

// Starting address
#define START_ADDRESS 0x0000
// Page buffer
static uint8_t pageBuffer[PAGE_SIZE];

state_t currentState = isWaiting;
float lps22hb_stnd_height = 0.1298;    // Standardabweichung Höhenmessung
float lsm66dsl_stnd_acc = 0.2;          // Standardabweichung Beschleunigungssensor
bool initMeasr = false;

static  bool WebServerProcess(void);
static  WIFI_Status_t SendWebPage(uint8_t ledIsOn, uint8_t temperature);

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t adress;
} I2C_Device;

SemaphoreHandle_t i2c_mutex;
QueueHandle_t accQueue;
QueueHandle_t measureQueue;

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    I2C_Device *i2c_device = (I2C_Device *)handle;

    // Allocate a buffer to hold both the register address and the data
    uint8_t temp_buf[len + 1];
    temp_buf[0] = reg;           // First byte is the register address
    memcpy(&temp_buf[1], bufp, len);  // Copy the data into the buffer

    // Transmit the buffer (register address + data)
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(i2c_device->hi2c, i2c_device->adress, temp_buf, len + 1, 1000);
    if (status != HAL_OK) {
        printf("Error transmitting I2C \n");
        return -1;
    }

    return 0;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    I2C_Device *i2c_device = (I2C_Device *)handle;

    // Step 1: Send the register address
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(i2c_device->hi2c, i2c_device->adress, &reg, 1, 1000);
    if (status != HAL_OK) {
        printf("Error writing register address on I2C\n");
        return -1;
    }

    // Step 2: Read the data from the register
    status = HAL_I2C_Master_Receive(i2c_device->hi2c, i2c_device->adress, bufp, len, 1000);
    if (status != HAL_OK) {
        printf("Error reading data on I2C\n");
        return -1;
    }

    return 0;
}

void measureTask(void *argument) {
	printf("------Start measure Task----\r\n");
	stmdev_ctx_t dev_ctx;
	I2C_HandleTypeDef* i2c_handle = (I2C_HandleTypeDef*)argument;

	i2c_mutex = xSemaphoreCreateMutex();
	accQueue = xQueueCreate(10, sizeof(float));

	I2C_Device device = {
			.hi2c =i2c_handle,
			.adress = LSM6DSL_I2C_ADD_L
	};
	// Configure device I/O functions
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &device;

	// Initialize LSM6DSL
	//lsm6dsl_init(&dev_ctx);
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	uint8_t whoamI;
	lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do {
		lsm6dsl_reset_get(&dev_ctx, &whoamI);
	} while (whoamI);

	// Check device ID
	lsm6dsl_device_id_get(&dev_ctx, &whoamI);
	xSemaphoreGive(i2c_mutex);
	if (whoamI != LSM6DSL_ID) {
		printf("LSM6DSL not detected!\n");
		return;
	}
	printf("Configured LSM6DSL\r\n");
	// Configure accelerometer and gyroscope
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);
	lsm6dsl_gy_full_scale_set(&dev_ctx, LSM6DSL_2000dps);
	lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_104Hz);
	lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_104Hz);
	xSemaphoreGive(i2c_mutex);

	int16_t rawGyro[3];
	int16_t rawAcc[3];
	int16_t rawMag[3];
	unsigned int step = 0;
	QuaternionFilter_init();
	float rotation[4] = {0, 1, 0, 0};
	while(currentState != isSaving)
	{
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	uint32_t lastTime = HAL_GetTick();
	while(1)
	{
		xSemaphoreTake(i2c_mutex, portMAX_DELAY);
		lsm6dsl_angular_rate_raw_get(&dev_ctx, rawGyro);
		lsm6dsl_acceleration_raw_get(&dev_ctx, rawAcc);
		lsm6dsl_mag_calibrated_raw_get(&dev_ctx, rawMag);
		xSemaphoreGive(i2c_mutex);
		float an = -lsm6dsl_from_fs2g_to_mg(rawAcc[0]) / 1000.0f;
		float ae = lsm6dsl_from_fs2g_to_mg(rawAcc[1]) / 1000.0f;
		float ad = lsm6dsl_from_fs2g_to_mg(rawAcc[2]) / 1000.0f;
		float gyroX = (lsm6dsl_from_fs2000dps_to_mdps(rawGyro[1]) * M_PI) / 180000.0f;
		float gyroY = -(lsm6dsl_from_fs2000dps_to_mdps(rawGyro[0]) * M_PI) / 180000.0f;
		float gyroZ = (lsm6dsl_from_fs2000dps_to_mdps(rawGyro[2]) * M_PI) / 180000.0f;
		float deltaT = (float)(HAL_GetTick() - lastTime) / 1000.0f;
		lastTime = HAL_GetTick();
		QuaternionFilter_Mahony(an, ae, ad, gyroX, gyroY, gyroZ, deltaT, rotation);
		//printf("Here\r\n");
		float accX = (2 * (rotation[1] * rotation[3] - rotation[0] * rotation[2])) * an + (2 * (rotation[2] * rotation[3] + rotation[0] * rotation[1])) * ae + (2 * (rotation[0] * rotation[0] + rotation[3] * rotation[3]) - 1) * ad;
		accX *= 9.81f;
		xQueueSend(accQueue, &accX, (TickType_t)0);
		if(step % 10 == 0)
		{
			//printf("AccX: %.2f \r\n", accX);
			//float roll, pitch, yaw;
			//Quaternion_get_RPY(rotation[0], rotation[1], rotation[2], rotation[3], &roll, &pitch, &yaw);
			//printf("%.2f, %.2f, %.2f\r\n", roll, pitch, yaw);
			//printf("%.6f,%.6f,%.6f,%.6f\n", rotation[1], rotation[2], rotation[3], rotation[0]);
		}
		step++;
		vTaskDelay(1);
	}
}

static void ResetKalman(kalman_t *kf)
{
    kalman_filter_reset(kf);
    // kalman_filter_reset(kf);
    float deltaT = 0.025;
    /* Set State Transition Matrix */
    /* F = 1 t
           0 1 */
    matrix_t *F = &kf->F;
    matrix_set(F, 0, 0, 1);
    matrix_set(F, 0, 1, deltaT);
    matrix_set(F, 1, 0, 0);
    matrix_set(F, 1, 1, 1);
    /* Set Control Matrix */
    /* G = 0.5*t^2
           t        */
    matrix_t *G = &kf->G;
    matrix_set(G, 0, 0, 0.5 * deltaT * deltaT);
    matrix_set(G, 1, 0, deltaT);
    /* Set  Process Noise Matrix*/
    /* Q = (t^4)/4  (t^3)/2
           (t^3)/2   t^2       * kalman_process_noise^2*/
    matrix_t *Q = &kf->Q;
    matrix_set(Q, 0, 0, ((deltaT * deltaT * deltaT * deltaT) / 4) * lsm66dsl_stnd_acc * lsm66dsl_stnd_acc);
    matrix_set(Q, 0, 1, ((deltaT * deltaT * deltaT) / 2) * lsm66dsl_stnd_acc * lsm66dsl_stnd_acc);
    matrix_set(Q, 1, 0, ((deltaT * deltaT * deltaT) / 2) * lsm66dsl_stnd_acc * lsm66dsl_stnd_acc);
    matrix_set(Q, 1, 1, (deltaT * deltaT) * lsm66dsl_stnd_acc * lsm66dsl_stnd_acc);
    /* Set Observation Matrix */
    /* H = [1   0] */
    matrix_t *H = &kf->H;
    matrix_set(H, 0, 0, 1);
    matrix_set(H, 0, 1, 0);
    /* Set Measurement covariance matrix */
    matrix_t *R = &kf->R;
    matrix_set(R, 0, 0, lps22hb_stnd_height * lps22hb_stnd_height);
    /* Set Estimate Uncertainty Matrix*/
    /* P = 500   0
            0   500 */
    matrix_t *P = &kf->P;
    matrix_set(P, 0, 0, 500);
    matrix_set(P, 0, 1, 0);
    matrix_set(P, 1, 0, 0);
    matrix_set(P, 1, 1, 500);
    /* Set  Input Vector*/
    /* u = (a + g) */
    matrix_t *u = &kf->u;
    matrix_set(u, 0, 0, 0);
    /* Initialize State Vector */
    matrix_t *x = &kf->x;
    matrix_set(x, 0, 0, 0);
    matrix_set(x, 1, 0, 0);
}

void presKalmanTask(void *arguments)
{
	printf("------Start Kalman Task----\r\n");
	I2C_HandleTypeDef* i2c_handle = (I2C_HandleTypeDef*)arguments;
	measureQueue = xQueueCreate(5, sizeof(measureData_t));
	stmdev_lps22hb_t device = {
			.hi2c = i2c_handle,
			.deviceAddr_pres = 0xBBU,
			.deviceAddr_temp = 0xBBU
	};
	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	LPS22HB_P_Init(&device);
	LPS22HB_T_Init(&device);
	if(LPS22HB_P_ReadID(&device) != LPS22HB_WHO_AM_I_VAL)
	{
		xSemaphoreGive(i2c_mutex);
		printf("Error communicating with pressure sensor\r\n");
		return;
	}
	xSemaphoreGive(i2c_mutex);
	printf("Configured LPS22HB\r\n");
	while(currentState != isSaving)
	{
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	float startPressure = 0.0f;
	float startTemperature = 0.0f;
	for (size_t i = 0; i < 10; i++)
	{
		xSemaphoreTake(i2c_mutex, portMAX_DELAY);
		startPressure += LPS22HB_P_ReadPressure(&device);
		startTemperature += LPS22HB_T_ReadTemp(&device);
		xSemaphoreGive(i2c_mutex);
	    vTaskDelay(40 / portTICK_PERIOD_MS);
	}
	startPressure /= 10;
	startTemperature /= 10;

	kalman_t kf;
	kalman_filter_init(&kf);
	ResetKalman(&kf);
	kalman_predict(&kf);
	float deltaT = 0.025;
	matrix_t *F = &kf.F;
	matrix_t *G = &kf.G;
	matrix_t *Q = &kf.Q;
	matrix_t *u = &kf.u;
	matrix_t *x = &kf.x;
	matrix_t *z = &kf.z;
	uint32_t lastTime = HAL_GetTick();

	printf("Startpres: %.2f, StartTemp: %.2f\r\n", startPressure, startTemperature);
	initMeasr = true;
	while(1)
	{
		xSemaphoreTake(i2c_mutex, portMAX_DELAY);
		float pressure = LPS22HB_P_ReadPressure(&device);
		xSemaphoreGive(i2c_mutex);
		float heightDifferenz = ((startTemperature + 273.15f) / TEMPGRAD_L) * (powf(pressure/ startPressure, -TEMPGRAD_L * R_SPECIFIC / GRAVITY_G) - 1.0f);
		//printf("%.2f \r\n", heightDifferenz);

		float accSum = 0;
		int accSize = 0;
		float acc = 0;
		while (xQueueReceive(accQueue, &acc, (TickType_t)0) == pdTRUE)
		{
			accSum += acc;
		    accSize++;
		}
		if (accSize == 0)
		{
			printf("Cant recieve Acc\r\n");
		    acc = 9.81f;
		}
		else
			acc = accSum / accSize;

		float deltaT = (float)(HAL_GetTick() - lastTime) / 1000.0f;
		lastTime = HAL_GetTick();
		matrix_set(Q, 0, 0, ((deltaT * deltaT * deltaT * deltaT) / 4) * lsm66dsl_stnd_acc * lsm66dsl_stnd_acc);
		matrix_set(Q, 0, 1, ((deltaT * deltaT * deltaT) / 2) * lsm66dsl_stnd_acc * lsm66dsl_stnd_acc);
		matrix_set(Q, 1, 0, ((deltaT * deltaT * deltaT) / 2) * lsm66dsl_stnd_acc * lsm66dsl_stnd_acc);
		matrix_set(Q, 1, 1, (deltaT * deltaT) * lsm66dsl_stnd_acc * lsm66dsl_stnd_acc);
		/* Set G with new deltaT */
		matrix_set(G, 0, 0, 0.5 * deltaT * deltaT);
		matrix_set(G, 1, 0, deltaT);
		/* Set F with new deltaT */
		matrix_set(F, 0, 0, 1);
		matrix_set(F, 0, 1, deltaT);
		matrix_set(F, 1, 0, 0);
		matrix_set(F, 1, 1, 1);
		/* Set z measurement */
		matrix_set(z, 0, 0, heightDifferenz);
		/* Set Input */
		matrix_set(u, 0, 0, acc - 9.81);
		kalman_update(&kf);
		kalman_predict(&kf);

		measureData_t newData = {
				.acc = acc - 9.81,
				.diffHeight = heightDifferenz,
				.kalmanHeight = matrix_get(x, 0, 0),
				.kalmanVertVel = matrix_get(x, 1, 0),
				.kalmanVertAcc = matrix_get(x, 2, 0),
				.time = HAL_GetTick()
		};
		if (xQueueSendToBack(measureQueue, &newData, (TickType_t)1) != pdPASS)
		{
			printf("Failed to insert data into the queue.\n");
		}
		vTaskDelay(pdMS_TO_TICKS(40));
	}
}

// Helper to find block start address from any given address
static inline uint32_t GetBlockStartAddress(uint32_t addr) {
    return (addr & ~(BLOCK_SIZE - 1));
}

// Helper to erase a block
static uint8_t EraseBlock(uint32_t addr) {
    // Align address to block boundary
    uint32_t blockAddr = GetBlockStartAddress(addr);
    uint32_t sectorIndex = blockAddr / BLOCK_SIZE;
    printf("Erasing Sector at: %lu\n\r", sectorIndex);
    if(BSP_QSPI_Erase_Sector(sectorIndex) != QSPI_OK)
    {
    	return QSPI_ERROR;
    }
    while(BSP_QSPI_GetStatus() != QSPI_OK)
    {
    	vTaskDelay(pdMS_TO_TICKS(2));
    }
    return QSPI_OK;
}

void saveTask(void *arguments)
{
	printf("------Start Save Task----\r\n");
	while(!initMeasr)
		vTaskDelay(2);

	measureData_t receivedData;
	uint32_t currentWriteAddress = START_ADDRESS;
	uint32_t currentBlockStart   = GetBlockStartAddress(currentWriteAddress);
	uint8_t errorCode = BSP_QSPI_Init();
	if(errorCode != QSPI_OK)
	{
		printf("Can't init QSPI: 0x%02x \r\n", errorCode);
		return;
	}
	printf("Init QSPI Flash\r\n");
	while(currentState != isSaving)
	{
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	printf("--------------Start Saving---------------\r\n");
	// Erase the first block we start writing into
	if (EraseBlock(currentBlockStart) != QSPI_OK) {
		printf("Can't erase block!\r\n");
		return;
	}

	uint16_t pageIndex = 0;
	const size_t recordSize = sizeof(measureData_t) + 2;

	memset(pageBuffer, 0xFF, PAGE_SIZE);

	while (1)
	{
		// Wait indefinitely for new data
		if (xQueueReceive(measureQueue, &receivedData, portMAX_DELAY) == pdPASS) {
			// Check if we have enough room in the current page for this record
			if (pageIndex + recordSize > PAGE_SIZE) {
				// Not enough room in the current page, we need to write this page and move on
				// First write out the full page
				if (BSP_QSPI_Write(pageBuffer, currentWriteAddress, PAGE_SIZE) != QSPI_OK) {
					printf("Can't write to QSPI\r\n");
				}
				printf("Saving page\r\n");
				// Advance to next page
				currentWriteAddress += PAGE_SIZE;

				// Check if we crossed a block boundary
				if ((currentWriteAddress & (BLOCK_SIZE - 1)) == 0) {
					printf("Old Block full!\n\r");
					// We are exactly at a block boundary
					currentBlockStart = GetBlockStartAddress(currentWriteAddress);
					if (EraseBlock(currentBlockStart) != QSPI_OK) {
						printf("Cant erase next block!\r\n");
						// Handle block erase error
					}
				}

				// Reset page buffer and index
				memset(pageBuffer, 0xFF, PAGE_SIZE);
				pageIndex = 0;
			}

			// Now insert the new record into the pageBuffer
			pageBuffer[pageIndex] = 0x53;
			pageIndex += 1;

			memcpy(&pageBuffer[pageIndex], &receivedData, sizeof(measureData_t));
			pageIndex += sizeof(measureData_t);

			pageBuffer[pageIndex] = 0x45;
			pageIndex += 1;
		}
	}
}

static inline uint32_t AlignToNextPage(uint32_t addr) {
    return ((addr + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1));
}

void printData(measureData_t* data)
{
	printf("%f, %f, %f, %f, %f, %lu\r\n",
	       data->acc,
	       data->diffHeight,
	       data->kalmanHeight,
	       data->kalmanVertVel,
	       data->kalmanVertAcc,
	       data->time);
}

void readTask(void *arguments)
{
	printf("------Start read Task----\r\n");
	const size_t RECORD_SIZE =  2 + sizeof(measureData_t);
	uint8_t recordBuffer[RECORD_SIZE];
	uint32_t currentReadAddress = START_ADDRESS;
	uint8_t errorCode = BSP_QSPI_Init();
	if(errorCode != QSPI_OK)
	{
		printf("Can't init QSPI: 0x%02x \r\n", errorCode);
		return;
	}
	while(currentState != isReading)
	{
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	measureData_t data;

	// Keep reading until we find no valid record
	while (1) {
		// Read one record (26 bytes)
		if (BSP_QSPI_Read(recordBuffer, currentReadAddress, RECORD_SIZE) != QSPI_OK) {
			// Handle read error or break
			printf("QSPI Read Error at address 0x%08X\n", (unsigned int)currentReadAddress);
			break;
		}
		// Check start and end markers
		if (recordBuffer[0] == 0x53 && recordBuffer[RECORD_SIZE - 1] == 0x45) {

			memcpy(&data, recordBuffer+1, sizeof(measureData_t));
			printData(&data);

			// Move to next record
			currentReadAddress += RECORD_SIZE;
		}
		else
		{
			//We only go to the next page, if we are not at the start from the page
			uint32_t nextPageStart = AlignToNextPage(currentReadAddress);
			if (nextPageStart == currentReadAddress) {
				// Already at a page boundary, means no more valid data.
				break;
			}

			// Move to next page start
			currentReadAddress = nextPageStart;

			// Try reading again at the start of next page
			if (BSP_QSPI_Read(recordBuffer, currentReadAddress, RECORD_SIZE) != QSPI_OK) {
				printf("QSPI Read Error at address 0x%08X\n", (unsigned int)currentReadAddress);
				break;
			}

			if (recordBuffer[0] == 0x53 && recordBuffer[RECORD_SIZE - 1] == 0x45) {
				// Valid record at next page boundary

				memcpy(&data, recordBuffer+1, sizeof(measureData_t));
				printData(&data);

				currentReadAddress += RECORD_SIZE;
			} else {
				printf("Tried new block at 0x%08X which is %u but nothing found\r\n", (unsigned int)currentReadAddress, recordBuffer[0]);
				// Still no valid record found. Assume no more records.
				break;
			}
		}
	}

	printf("No more valid records found.\n");
	vTaskDelete(NULL);
}
void startSaving()
{
	if(currentState != isWaiting)
	{
		printf("Can't set state!\r\n");
		return;
	}
	printf("Start saving...\r\n");
	currentState = isSaving;
}
void startReading()
{
	if(currentState != isWaiting)
	{
		printf("Can't set state!\r\n");
		return;
	}
	printf("Start reading...\r\n");
	currentState = isReading;
}
