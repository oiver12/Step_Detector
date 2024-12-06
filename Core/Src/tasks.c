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

#define SOCKET                 0

static  uint8_t http[1024];
static  uint8_t  IP_Addr[4];
static  int     LedState = 0;

float lps22hb_stnd_height = 0.1298;    // Standardabweichung Höhenmessung
float lsm66dsl_stnd_acc = 0.2;          // Standardabweichung Beschleunigungssensor

static  bool WebServerProcess(void);
static  WIFI_Status_t SendWebPage(uint8_t ledIsOn, uint8_t temperature);

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t adress;
} I2C_Device;

SemaphoreHandle_t i2c_mutex;
QueueHandle_t accQueue;

static  WIFI_Status_t SendWebPage(uint8_t ledIsOn, uint8_t temperature);
static  bool WebServerProcess(void);

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
	printf("Starting Kalmantask \r\n");
	I2C_HandleTypeDef* i2c_handle = (I2C_HandleTypeDef*)arguments;
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
		//double vertVel = matrix_get(x, 1, 0);
		//printf("VertVelocity: %.2f \r\n", vertVel);
		double height = matrix_get(x, 0, 0);
		//printf("Height: %.2f\r\n", height);

		vTaskDelay(pdMS_TO_TICKS(40));
	}
}

void saveTask(void *arguments)
{
	uint8_t errorCode = BSP_QSPI_Init();
	if(errorCode != QSPI_OK)
	{
		printf("Can't init QSPI: 0x%02x \r\n", errorCode);
		return;
	}
	printf("Initialized QSPI\r\n");
}

void wifiServerTask(void *arguments)
{
	uint8_t (*IP_Addr)[4] = (uint8_t (*)[4])arguments;
	printf("Server is running and waiting for an HTTP  Client connection to %d.%d.%d.%d\n",IP_Addr[0],IP_Addr[1],IP_Addr[2],IP_Addr[3]);
	vTaskDelete(NULL);
}


