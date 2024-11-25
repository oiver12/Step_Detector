#include "lsm6dsl_reg.h"
#include "../Inc/tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    // Replace with your platform-specific I2C write function.
    // Example: HAL_I2C_Mem_Write(...);
    return 0;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    // Replace with your platform-specific I2C read function.
    // Example: HAL_I2C_Mem_Read(...);
    return 0;
}

void measureTask(void *argument) {
	stmdev_ctx_t dev_ctx;

	// Configure device I/O functions
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = NULL;

	// Initialize LSM6DSL
	lsm6dsl_init(&dev_ctx);

	uint8_t whoamI;
	lsm6dsl_reset_set(dev_ctx, PROPERTY_ENABLE);
	do {
		lsm6dsl_reset_get(dev_ctx, &whoamI);
	} while (whoamI);

	// Check device ID
	lsm6dsl_device_id_get(dev_ctx, &whoamI);
	if (whoamI != LSM6DSL_ID) {
		printf("LSM6DSL not detected!\n");
		return;
	}

	// Configure accelerometer and gyroscope
	lsm6dsl_xl_full_scale_set(dev_ctx, LSM6DSL_2g);
	lsm6dsl_gy_full_scale_set(dev_ctx, LSM6DSL_2000dps);
	lsm6dsl_xl_data_rate_set(dev_ctx, LSM6DSL_XL_ODR_104Hz);
	lsm6dsl_gy_data_rate_set(dev_ctx, LSM6DSL_GY_ODR_104Hz);
}
