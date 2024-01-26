#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

#include <stdio.h>
#include <float.h>

#include <nrfx_twi.h>
#include <nrfx_twis.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#define I2C_NODE DT_NODELABEL(mysensor)
#define LED0_NODE DT_ALIAS(led0)
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define SLEEP_TIME_MS 1000

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static uint8_t mfg_data[29] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //Thingy52 Value 1
							0x00, 0x00, 0x00, 0x00,	// sps30 Value 1
							0xFF, 0xFF, 0xFF, 0xFF,	// sps30 Value 2
							0xFF, 0xFF, 0xFF, 0xFF, // sps30 Value 3
							0x00, 0x00, 0x00, 0x00, // sps30 Value 4
							0x00, 0x00, 0x00, 0x00, // sps30 Value 5
							0x01}; //SET NUMBER

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_SVC_DATA16, mfg_data, 29),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

uint8_t CalcCrc(uint8_t data[2])
{
	uint8_t crc = 0xFF;
	for (int i = 0; i < 2; i++)
	{
		crc ^= data[i];
		for (uint8_t bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80)
			{
				crc = (crc << 1) ^ 0x31u;
			}
			else
			{
				crc = (crc << 1);
			}
		}
	}
	return crc;
}

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&led))
	{
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return 0;
	}

	int err;

	printk("Starting Beacon Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
	}

	const struct device *const dev = DEVICE_DT_GET_ONE(st_hts221);

	if (!device_is_ready(dev))
	{
		printk("sensor: device not ready.\n");
		return 0;
	}

	if (IS_ENABLED(CONFIG_HTS221_TRIGGER))
	{
		struct sensor_trigger trig = {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = SENSOR_CHAN_ALL,
		};
		if (sensor_trigger_set(dev, &trig, NULL) < 0)
		{
			printf("Cannot configure trigger\n");
			return 0;
		}
	}

	printk("checking if device is ready \n");
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
	if (!device_is_ready(dev_i2c.bus))
	{
		printk("I2C bus %s is not ready!\n\r", dev_i2c.bus->name);
		return 0;
	}
	int started = 0;
	while (started == 0)
	{
		printk("Trying to Start Measuring \n");
		uint8_t data[2] = {0x03, 0x00};
		uint8_t config[5] = {0x00, 0x10, data[0], data[1], CalcCrc(data)};
		ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
		if (ret != 0)
		{
			printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr, config[0]);
			return 0;
		}
		else
		{
			started = 1;
		}
	}
	while (1)
	{
		uint8_t readyDataFlag_buffer[3];
		uint8_t readyDataFlag_register[2] = {0x02, 0x02};
		ret = i2c_write_dt(&dev_i2c, &readyDataFlag_register, 2);
		if (ret != 0)
		{
			gpio_pin_toggle_dt(&led);
			printk("Failed to write/read I2C device address %x at Reg. \r\n", dev_i2c.addr);
		}
		i2c_read_dt(&dev_i2c, &readyDataFlag_buffer, 3);

		if (readyDataFlag_buffer[1] == 0x01)
		{
			gpio_pin_toggle_dt(&led);
			printk("New Data Available: ");
			uint8_t read_buffer[60];
			uint8_t read_register[2] = {0x03, 0x00};
			ret = i2c_write_dt(&dev_i2c, &read_register, 2);
			if (ret != 0)
			{
				printk("Failed to write/read I2C device address %x at Reg. \r\n", dev_i2c.addr);
			}
			ret = i2c_read_dt(&dev_i2c, &read_buffer, 60);

			uint32_t resultsFLOAT[10];
			uint8_t resultsINT[40];
			for (int i = 0; i < 10; i++)
			{
				// save float
				uint32_t bitresult = read_buffer[(i * 6)] << 24 | read_buffer[(i * 6) + 1] << 16 | read_buffer[(i * 6) + 3] << 8 | read_buffer[(i * 6) + 4];
				resultsFLOAT[i] = bitresult;

				// save int
				resultsINT[(i * 4)] = read_buffer[(i * 6) + 4];
				resultsINT[(i * 4) + 1] = read_buffer[(i * 6) + 3];
				resultsINT[(i * 4) + 2] = read_buffer[(i * 6) + 1];
				resultsINT[(i * 4) + 3] = read_buffer[(i * 6) + 0];
			}
			if(mfg_data[28] == 1)
			{
				for(int i = 0; i < 5; i++){
					mfg_data[(i*4) + 8] = resultsINT[i*4];
					mfg_data[(i*4) + 9] = resultsINT[(i*4)+1];
					mfg_data[(i*4) + 10] = resultsINT[(i*4)+2];
					mfg_data[(i*4) + 11] = resultsINT[(i*4)+3];
				}
			}
			else if(mfg_data[28] == 2)
			{
				for(int i = 0; i < 5; i++){
					mfg_data[(i*4) + 8] = resultsINT[((i+5)*4)];
					mfg_data[(i*4) + 9] = resultsINT[((i+5)*4)+1];
					mfg_data[(i*4) + 10] = resultsINT[((i+5)*4)+2];
					mfg_data[(i*4) + 11] = resultsINT[((i+5)*4)+3];
				}
			}
		}

		if(mfg_data[28] == 1)
		{
			struct sensor_value temp;

			if (sensor_sample_fetch(dev) < 0)
			{
				printf("Sensor sample update error\n");
				return 0;
			}

			if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0)
			{
				printf("Cannot read HTS221 temperature channel\n");
				return 0;
			}

			uint16_t temp_val1_low = temp.val1 & 0xFFFF;
			uint16_t temp_val1_high = (temp.val1 >> 16) & 0xFFFF;
			uint16_t temp_val2_low = temp.val2 & 0xFFFF;
			uint16_t temp_val2_high = (temp.val2 >> 16) & 0xFFFF;

			/* Update mfg_data with the split values */
			mfg_data[0] = temp_val1_low & 0xFF;
			mfg_data[1] = (temp_val1_low >> 8) & 0xFF;
			mfg_data[2] = temp_val1_high & 0xFF;
			mfg_data[3] = (temp_val1_high >> 8) & 0xFF;

			mfg_data[4] = temp_val2_low & 0xFF;
			mfg_data[5] = (temp_val2_low >> 8) & 0xFF;
			mfg_data[6] = temp_val2_high & 0xFF;
			mfg_data[7] = (temp_val2_high >> 8) & 0xFF;

			/* Start advertising */
			err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad),
								sd, ARRAY_SIZE(sd));
			if (err)
			{
				printk("Advertising failed to start (err %d)\n", err);
				return 0;
			}

			k_sleep(K_MSEC(500));

			err = bt_le_adv_stop();
			if (err)
			{
				printk("Advertising failed to stop (err %d)\n", err);
				return 0;
			}

			mfg_data[28] = 2;

			k_sleep(K_MSEC(500));
		}
		else if(mfg_data[28] == 2)
		{
			struct sensor_value hum;

			if (sensor_sample_fetch(dev) < 0)
			{
				printf("Sensor sample update error\n");
				return 0;
			}

			if (sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum) < 0)
			{
				printf("Cannot read HTS221 humidity channel\n");
				return 0;
			}

			uint16_t hum_val1_low = hum.val1 & 0xFFFF;
			uint16_t hum_val1_high = (hum.val1 >> 16) & 0xFFFF;
			uint16_t hum_val2_low = hum.val2 & 0xFFFF;
			uint16_t hum_val2_high = (hum.val2 >> 16) & 0xFFFF;

			/* Update mfg_data with the split values */
			mfg_data[0] = hum_val1_low & 0xFF;
			mfg_data[1] = (hum_val1_low >> 8) & 0xFF;
			mfg_data[2] = hum_val1_high & 0xFF;
			mfg_data[3] = (hum_val1_high >> 8) & 0xFF;

			mfg_data[4] = hum_val2_low & 0xFF;
			mfg_data[5] = (hum_val2_low >> 8) & 0xFF;
			mfg_data[6] = hum_val2_high & 0xFF;
			mfg_data[7] = (hum_val2_high >> 8) & 0xFF;

			/* Start advertising */
			err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad),
								sd, ARRAY_SIZE(sd));
			if (err)
			{
				printk("Advertising failed to start (err %d)\n", err);
				return 0;
			}

			k_sleep(K_MSEC(500));

			err = bt_le_adv_stop();
			if (err)
			{
				printk("Advertising failed to stop (err %d)\n", err);
				return 0;
			}
			
			mfg_data[28] = 1;

			k_sleep(K_MSEC(500));
		}
	}
	return 0;
}