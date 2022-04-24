/********************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include "rsi_main.h"
#include "rsi_gpio.h"

/**
 * redpine_gpio_deinit(): to de-initialize gpio
 * @return:
 * @params:
 */
void redpine_gpio_deinit(struct rsi_common *common)
{
	gpio_free(common->ulp_gpio_read);
	gpio_free(common->ulp_gpio_write);
}
EXPORT_SYMBOL_GPL(redpine_gpio_deinit);

/**
 * redpine_gpio_init(): to initialize gpio
 * @return:
 * @params:
 */
void redpine_gpio_init(struct rsi_common *common)
{
	int rc = 0;
	char *read_gpio = "device_status";
	char *write_gpio = "host_intention";

	/* gpio_free() is dangerous to use.FIXME*/
	gpio_free(common->ulp_gpio_read);
	gpio_free(common->ulp_gpio_write);
	rc = gpio_request(common->ulp_gpio_write, write_gpio);
	if (rc) {
		redpine_dbg(ERR_ZONE, "%s: %s setup failed with err: %d\n",
			__func__, write_gpio, rc);
		return;
	}
	rc = gpio_request(common->ulp_gpio_read, read_gpio);
	if (rc) {
		redpine_dbg(ERR_ZONE, "%s: %s setup failed with err: %d\n",
			__func__, read_gpio, rc);
		return;
	}
	rc = gpio_direction_output(common->ulp_gpio_write, 0);
	if (rc) {
		redpine_dbg(ERR_ZONE, "%s: failed to set %s direction, err: %d\n",
			__func__, write_gpio, rc);
		return;
	}
	rc = gpio_direction_input(common->ulp_gpio_read);
	if (rc) {
		redpine_dbg(ERR_ZONE, "%s: failed to set %s direction, err: %d\n",
			__func__, read_gpio, rc);
		return;
	}
}
EXPORT_SYMBOL_GPL(redpine_gpio_init);

/**
 * redpine_set_host_status() - This function is used to toggle host gpio.
 *
 * @value: The value of the host gpio either TRUE or FALSE.
 *
 * Return: None.
 */
void redpine_set_host_status(int value, struct rsi_common *common)
{
	__gpio_set_value(common->ulp_gpio_write, value);
}
EXPORT_SYMBOL_GPL(redpine_set_host_status);

/**
 * redpine_get_device_status() - This function is used to read the LMAC gpio to find
 * the LMAC sleep status.
 *
 * Return: True if gpio status high, false if gpio status low.
 */
int redpine_get_device_status(struct rsi_common *common)
{
	return __gpio_get_value(common->ulp_gpio_read);
}
EXPORT_SYMBOL_GPL(redpine_get_device_status);

