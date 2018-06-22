// SPDX-License-Identifier: GPL-2.0+
/*
 * AD5770R Digital to analog converters driver
 *
 * Copyright 2018 Analog Devices Inc.
 */

#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/property.h>
#include <linux/gpio/consumer.h>

/* SPI configuration registers */
#define AD5770R_INTERFACE_CONFIG_A	0x00
#define AD5770R_INTERFACE_CONFIG_B	0x01
#define AD5770R_CHIP_TYPE		0x03
#define AD5770R_PRODUCT_ID_L		0x04
#define AD5770R_PRODUCT_ID_H		0x05
#define AD5770R_CHIP_GRADE		0x06
#define AD5770R_SCRATCH_PAD		0x0A
#define AD5770R_SPI_REVISION		0x0B
#define AD5770R_VENDOR_L		0x0C
#define AD5770R_VENDOR_H		0x0D
#define AD5770R_STREAM_MODE		0x0E
#define AD5770R_INTERFACE_CONFIG_C	0x10
#define AD5770R_INTERFACE_STATUS_A	0x11

/* AD5770R configuration registers */
#define AD5770R_CHANNEL_CONFIG		0x14
#define AD5770R_OUTPUT_RANGE_CH0	0x15
#define AD5770R_OUTPUT_RANGE_CH1	0x16
#define AD5770R_OUTPUT_RANGE_CH2	0x17
#define AD5770R_OUTPUT_RANGE_CH3	0x18
#define AD5770R_OUTPUT_RANGE_CH4	0x19
#define AD5770R_OUTPUT_RANGE_CH5	0x1A
#define AD5770R_REFERENCE		0x1B
#define AD5770R_ALARM_CONFIG		0x1C
#define AD5770R_OUTPUT_FILTER_CH0	0x1D
#define AD5770R_OUTPUT_FILTER_CH1	0x1E
#define AD5770R_OUTPUT_FILTER_CH2	0x1F
#define AD5770R_OUTPUT_FILTER_CH3	0x20
#define AD5770R_OUTPUT_FILTER_CH4	0x21
#define AD5770R_OUTPUT_FILTER_CH5	0x22
#define AD5770R_MONITOR_SETUP		0x23
#define AD5770R_STATUS			0x24
#define AD5770R_HW_LDAC			0x25
#define AD5770R_CH0_DAC_LSB		0x26
#define AD5770R_CH0_DAC_MSB		0x27
#define AD5770R_CH1_DAC_LSB		0x28
#define AD5770R_CH1_DAC_MSB		0x29
#define AD5770R_CH2_DAC_LSB		0x2A
#define AD5770R_CH2_DAC_MSB		0x2B
#define AD5770R_CH3_DAC_LSB		0x2C
#define AD5770R_CH3_DAC_MSB		0x2D
#define AD5770R_CH4_DAC_LSB		0x2E
#define AD5770R_CH4_DAC_MSB		0x2F
#define AD5770R_CH5_DAC_LSB		0x30
#define AD5770R_CH5_DAC_MSB		0x31
#define AD5770R_DAC_PAGE_MASK_LSB	0x32
#define AD5770R_DAC_PAGE_MASK_MSB	0x33
#define AD5770R_CH_SELECT		0x34
#define AD5770R_INPUT_PAGE_MASK_LSB	0x35
#define AD5770R_INPUT_PAGE_MASK_MSB	0x36
#define AD5770R_SW_LDAC			0x37
#define AD5770R_CH0_INPUT_LSB		0x38
#define AD5770R_CH0_INPUT_MSB		0x39
#define AD5770R_CH1_INPUT_LSB		0x3A
#define AD5770R_CH1_INPUT_MSB		0x3B
#define AD5770R_CH2_INPUT_LSB		0x3C
#define AD5770R_CH2_INPUT_MSB		0x3D
#define AD5770R_CH3_INPUT_LSB		0x3E
#define AD5770R_CH3_INPUT_MSB		0x3F
#define AD5770R_CH4_INPUT_LSB		0x40
#define AD5770R_CH4_INPUT_MSB		0x41
#define AD5770R_CH5_INPUT_LSB		0x42
#define AD5770R_CH5_INPUT_MSB		0x43
#define AD5770R_CH_ENABLE		0x44

/* AD5770R_INTERFACE_CONFIG_A */
#define AD5770R_ITF_CFG_A_SW_RESET(x)		(((x) & 0x1) | 0x80)
#define AD5770R_ITF_CFG_A_ADDR_ASC_MSB(x)	(((x) & 0x1) << 5)

/* AD5770R_INTERFACE_CONFIG_B */
#define AD5770R_ITF_CFG_B_SINGLE_INST(x)	(((x) & 0x1) << 7)
#define AD5770R_ITF_CFG_B_SHORT_INST(x)		(((x) & 0x1) << 3)

/* AD5770R_INTERFACE_CONFIG_C */
#define AD5770R_ITF_CFG_C_STRUCT_REGR_ACC(x)	(((x) & 0x1) << 5)

/* AD5770R_CHANNEL_CONFIG */
#define AD5770R_CFG_CH0_SINK_EN(x)		(((x) & 0x1) << 7)
#define AD5770R_CFG_CH5_SHUTDOWN_B(x)		(((x) & 0x1) << 5)
#define AD5770R_CFG_CH4_SHUTDOWN_B(x)		(((x) & 0x1) << 4)
#define AD5770R_CFG_CH3_SHUTDOWN_B(x)		(((x) & 0x1) << 3)
#define AD5770R_CFG_CH2_SHUTDOWN_B(x)		(((x) & 0x1) << 2)
#define AD5770R_CFG_CH1_SHUTDOWN_B(x)		(((x) & 0x1) << 1)
#define AD5770R_CFG_CH0_SHUTDOWN_B(x)		(((x) & 0x1) << 0)

/* AD5770R_OUTPUT_RANGE */
#define AD5770R_RANGE_OUTPUT_SCALING(x)		(((x) & 0x3F) << 2)
#define AD5770R_RANGE_MODE(x)			((x) & 0x03)

/* AD5770R_REFERENCE */
#define AD5770R_REF_RESISTOR_SEL(x)		(((x) & 0x1) << 2)
#define AD5770R_REF_VOLTATE_SEL(x)		(((x) & 0x3) << 0)

/* AD5770R_ALARM_CONFIG */
#define AD5770R_BGND_CRC_MSK(x)			(((x) & 0x1) << 7)
#define AD5770R_IREF_FAULT_MSK(x)		(((x) & 0x1) << 6)
#define AD5770R_NEGATIVE_CH0_MSK(x)		(((x) & 0x1) << 5)
#define AD5770R_OVER_TEMP_MSK(x)		(((x) & 0x1) << 4)
#define AD5770R_TEMP_WARNING_MSK(x)		(((x) & 0x1) << 3)
#define AD5770R_BGND_CRC_EN(x)			(((x) & 0x1) << 2)
#define AD5770R_THERMAL_SHUTDOWN_EN(x)		(((x) & 0x1) << 1)
#define AD5770R_OPEN_DRAIN_EN(x)		(((x) & 0x1) << 0)

/* AD5770R_MONITOR_SETUP */
#define AD5770R_SETUP_MON_FUNCTION(x)		(((x) & 0x3) << 6)
#define AD5770R_SETUP_MUX_BUFFER(x)		(((x) & 0x1) << 5)
#define AD5770R_SETUP_IB_EXT_EN(x)		(((x) & 0x1) << 4)
#define AD5770R_SETUP_MON_CH(x)			(((x) & 0x7) << 0)

/* AD5770R_CH_DAC */
#define AD5770R_CH_DAC_DATA_LSB(x)		(((x) & 0x3F) << 2)
#define AD5770R_CH_DAC_DATA_MSB(x)		(((x) & 0xFF) << 0)

/* AD5770R_CH_SELECT */
#define AD5770R_CH_SELECT_SEL_CH(x, channel)	(((x) & 0x1) << (channel))

/* AD5770R_CH_ENABLE */
#define AD5770R_CH_SET(x, channel)		(((x) & 0x1) << (channel))

#define AD5770R_REG_READ(x)			(((x) & 0x7F) | 0x80)

#define AD5770R_MAX_CHANNELS	6
#define AD5770R_MAX_CH_MODES	14

enum ad5770r_mon_func {
	AD5770R_DISABLE = 0,
	AD5770R_VOLTAGE_MONITORING,
	AD5770R_CURRENT_MONITORING,
	AD5770R_TEMPERATURE_MONITORING
};

enum ad5770r_ch {
	AD5770R_CH0 = 0,
	AD5770R_CH1,
	AD5770R_CH2,
	AD5770R_CH3,
	AD5770R_CH4,
	AD5770R_CH5
};

enum ad5770r_ch0_modes {
	AD5770R_CH0_0_300 = 0,
	AD5770R_CH0_NEG_60_0,
	AD5770R_CH0_NEG_60_300
};

enum ad5770r_ch1_modes {
	AD5770R_CH1_0_140_LOW_HEAD = 1,
	AD5770R_CH1_0_140_LOW_NOISE,
	AD5770R_CH1_0_250
};

enum ad5770r_ch2_5_modes {
	AD5770R_CH_LOW_RANGE = 0,
	AD5770R_CH_HIGH_RANGE
};

enum ad5770r_ref_v {
	AD5770R_EXT_REF_2_5_V = 0,
	AD5770R_INT_REF_1_25_V_OUT_ON,
	AD5770R_EXT_REF_1_25_V,
	AD5770R_INT_REF_1_25_V_OUT_OFF
};

struct ad5770r_mon_setup {
	enum ad5770r_mon_func	mon_func;
	bool			mux_buffer;
	bool			ib_ext_en;
	enum ad5770r_ch		mon_ch;
};

struct ad5770r_out_range {
	unsigned char	out_scale;
	unsigned char	out_range_mode;
};

struct ad5770r_dev_spi_setting {
	bool	addr_ascension;
	bool	single_instruction; /* for multibyte read/write */
	u8	stream_mode_length;
};

struct ad5770r_alarm_cfg {
	bool open_drain_en;
	bool th_sdwn_en;
	bool bgnd_crc_en;
	bool temp_wrn_msk;
	bool over_temp_msk;
	bool neg_ch0_msk;
	bool iref_fault_msk;
	bool bgnd_crc_msk;
};

/**
 * struct ad5770R_state - driver instance specific data
 * @spi:		spi_device
 * @regmap:		regmap
 * @gpio_reset		gpio descriptor
 * @output_mode		array contains channels output ranges
 * @alarm_config	alarm configuration options
 * @mon_setup		setup for monitoring function
 * @ext_ref		external reference flag
 * @ref_sel		referecen selection variable
 * @ch_config		variable which contains channel configuration
 * @ch_enable		variable for enabling channels
 */
struct ad5770r_state {
	struct spi_device *spi;
	struct regmap	*regmap;
	struct gpio_desc *gpio_reset;
	struct ad5770r_out_range	output_mode[AD5770R_MAX_CHANNELS];
	struct ad5770r_alarm_cfg	alarm_config;
	struct ad5770r_mon_setup	mon_setup;
	bool				ext_ref;
	enum ad5770r_ref_v		ref_sel;
	bool				ch_config[7];
	bool				ch_enable[6];
};

static const struct regmap_config ad5770r_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.read_flag_mask = BIT(0),
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

struct ad5770r_output_modes {

	enum ad5770r_ch ch;
	unsigned char mode;
	int min;
	int max;
};

static struct ad5770r_output_modes ad5770r_rng_tbl[] = {
	{ AD5770R_CH0, AD5770R_CH0_0_300, 0, 300 },
	{ AD5770R_CH0, AD5770R_CH0_NEG_60_0, -60, 0 },
	{ AD5770R_CH0, AD5770R_CH0_NEG_60_300, -60, 300 },
	{ AD5770R_CH1, AD5770R_CH1_0_140_LOW_HEAD, 0, 140 },
	{ AD5770R_CH1, AD5770R_CH1_0_140_LOW_NOISE, 0, 140 },
	{ AD5770R_CH1, AD5770R_CH1_0_250, 0, 250 },
	{ AD5770R_CH2, AD5770R_CH_LOW_RANGE, 0, 55 },
	{ AD5770R_CH2, AD5770R_CH_HIGH_RANGE, 0, 150 },
	{ AD5770R_CH3, AD5770R_CH_LOW_RANGE, 0, 45 },
	{ AD5770R_CH3, AD5770R_CH_HIGH_RANGE, 0, 100 },
	{ AD5770R_CH4, AD5770R_CH_LOW_RANGE, 0, 45 },
	{ AD5770R_CH4, AD5770R_CH_HIGH_RANGE, 0, 100 },
	{ AD5770R_CH5, AD5770R_CH_LOW_RANGE, 0, 45 },
	{ AD5770R_CH5, AD5770R_CH_HIGH_RANGE, 0, 100 },
};

#define AD5770R_IDAC_CHANNEL(index, reg) {			\
	.type = IIO_CURRENT,					\
	.address = reg,						\
	.indexed = 1,						\
	.channel = index,					\
	.output = 1,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			BIT(IIO_CHAN_INFO_SCALE),		\
}

static const struct iio_chan_spec ad5770r_channels[] = {
	AD5770R_IDAC_CHANNEL(0, AD5770R_CH0_DAC_MSB),
	AD5770R_IDAC_CHANNEL(1, AD5770R_CH1_DAC_MSB),
	AD5770R_IDAC_CHANNEL(2, AD5770R_CH2_DAC_MSB),
	AD5770R_IDAC_CHANNEL(3, AD5770R_CH3_DAC_MSB),
	AD5770R_IDAC_CHANNEL(4, AD5770R_CH4_DAC_MSB),
	AD5770R_IDAC_CHANNEL(5, AD5770R_CH5_DAC_MSB),
};

static int ad5770r_set_channel_en(struct ad5770r_state *st,
				  bool *channel_enable)
{
	unsigned char regval;

	regval = AD5770R_CH_SET(channel_enable[0], AD5770R_CH0) |
		 AD5770R_CH_SET(channel_enable[1], AD5770R_CH1) |
		 AD5770R_CH_SET(channel_enable[2], AD5770R_CH2) |
		 AD5770R_CH_SET(channel_enable[3], AD5770R_CH3) |
		 AD5770R_CH_SET(channel_enable[4], AD5770R_CH4) |
		 AD5770R_CH_SET(channel_enable[5], AD5770R_CH5);

	return regmap_write(st->regmap, AD5770R_CH_ENABLE, regval);
}

static int ad5770r_channel_config(struct ad5770r_state *st,
				  bool *channel_config)
{
	unsigned char regval;

	regval = AD5770R_CFG_CH0_SHUTDOWN_B(channel_config[0]) |
		 AD5770R_CFG_CH1_SHUTDOWN_B(channel_config[1]) |
		 AD5770R_CFG_CH2_SHUTDOWN_B(channel_config[2]) |
		 AD5770R_CFG_CH3_SHUTDOWN_B(channel_config[3]) |
		 AD5770R_CFG_CH4_SHUTDOWN_B(channel_config[4]) |
		 AD5770R_CFG_CH5_SHUTDOWN_B(channel_config[5]) |
		 AD5770R_CFG_CH0_SINK_EN(channel_config[6]);

	return regmap_write(st->regmap, AD5770R_CHANNEL_CONFIG, regval);
}

static int ad5770r_set_output_mode(struct ad5770r_state *st,
				   const struct ad5770r_out_range *out_mode,
				   enum ad5770r_ch channel)
{
	unsigned char regval;

	regval = AD5770R_RANGE_OUTPUT_SCALING(out_mode->out_scale) |
		 AD5770R_RANGE_MODE(out_mode->out_range_mode);

	return regmap_write(st->regmap,
			    AD5770R_OUTPUT_RANGE_CH0 + channel, regval);
}

static int ad5770r_set_alarm(struct ad5770r_state *st,
			     const struct ad5770r_alarm_cfg *const cfg)
{
	unsigned char regval;

	regval = AD5770R_OPEN_DRAIN_EN(cfg->open_drain_en) |
		 AD5770R_THERMAL_SHUTDOWN_EN(cfg->th_sdwn_en) |
		 AD5770R_BGND_CRC_EN(cfg->bgnd_crc_en) |
		 AD5770R_TEMP_WARNING_MSK(cfg->temp_wrn_msk) |
		 AD5770R_OVER_TEMP_MSK(cfg->over_temp_msk) |
		 AD5770R_NEGATIVE_CH0_MSK(cfg->neg_ch0_msk) |
		 AD5770R_IREF_FAULT_MSK(cfg->iref_fault_msk) |
		 AD5770R_BGND_CRC_MSK(cfg->bgnd_crc_msk);

	return regmap_write(st->regmap, AD5770R_ALARM_CONFIG,
			    regval);
}

static int ad5770r_set_monitor_setup(struct ad5770r_state *st,
				     const struct ad5770r_mon_setup *mon_setup)
{
	unsigned char regval;

	regval = AD5770R_SETUP_MON_CH(mon_setup->mon_ch) |
		 AD5770R_SETUP_IB_EXT_EN(mon_setup->ib_ext_en) |
		 AD5770R_SETUP_MUX_BUFFER(mon_setup->mux_buffer) |
		 AD5770R_SETUP_MON_FUNCTION(mon_setup->mon_func);

	return regmap_write(st->regmap, AD5770R_MONITOR_SETUP, regval);
}

static int ad5770r_set_reference(struct ad5770r_state *st,
				 bool external_reference,
				 enum ad5770r_ref_v reference_selector)
{
	unsigned char regval;

	regval = AD5770R_REF_RESISTOR_SEL(external_reference) |
		 AD5770R_REF_VOLTATE_SEL(reference_selector);

	return regmap_write(st->regmap, AD5770R_REFERENCE, regval);

}

static int ad5770r_soft_reset(struct ad5770r_state *st)
{
	unsigned char regval;

	regval = AD5770R_ITF_CFG_A_SW_RESET(1);

	return regmap_write(st->regmap, AD5770R_INTERFACE_CONFIG_A, regval);
}

static int ad5770r_reset(struct ad5770r_state *st)
{
	if (st->gpio_reset) {
		gpiod_set_value(st->gpio_reset, 0);
		udelay(100);
		gpiod_set_value(st->gpio_reset, 1);
	} else {
		/* Perform a software reset */
		return ad5770r_soft_reset(st);
	}

	/* data must not be written during reset timeframe */
	mdelay(1); /* TODO update with value from datasheet once available */

	return 0;
}

static int ad5770r_search_mode(struct ad5770r_state *st,
			       enum ad5770r_ch ch, int *min, int *max)
{
	int i;

	for (i = 0; i < AD5770R_MAX_CH_MODES; i++)
		if (ad5770r_rng_tbl[i].ch == ch &&
		    ad5770r_rng_tbl[i].mode ==
		    st->output_mode[ch].out_range_mode) {
			*min = ad5770r_rng_tbl[i].min;
			*max = ad5770r_rng_tbl[i].max;
			return 0;
		}

	return -EINVAL;
}

static int ad5770r_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct ad5770r_state *st = iio_priv(indio_dev);
	int max, min, ret;
	unsigned char buf[2];

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_bulk_read(st->regmap,
				       AD5770R_REG_READ(chan->address),
				       buf, 2);
		*val = ((u16)buf[0] << 6) + (buf[1] >> 2);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = ad5770r_search_mode(st, chan->channel, &min, &max);

		if (ret < 0)
			return ret;

		*val = max - min;
		*val2 = 14;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad5770r_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct ad5770r_state *st = iio_priv(indio_dev);
	int ret;
	unsigned char buf[2];

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		buf[0] = ((u16)val >> 6) & 0xFF;
		buf[1] = (val & 0x3F) << 2;
		ret = regmap_bulk_write(st->regmap, chan->address,
					buf, 2);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int ad5770r_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct ad5770r_state *st = iio_priv(indio_dev);
	int ret;

	if (readval == NULL)
		ret = regmap_write(st->regmap, reg, writeval);
	else
		ret = regmap_read(st->regmap, AD5770R_REG_READ(reg), readval);

	return ret;
}

static const struct iio_info ad5770r_info = {
	.read_raw = ad5770r_read_raw,
	.write_raw = ad5770r_write_raw,
	.debugfs_reg_access = &ad5770r_reg_access,
};

static int ad5770r_check_output_mode(struct ad5770r_state *st)
{
	int ret, i;

	if (st->output_mode[AD5770R_CH0].out_range_mode >
	    AD5770R_CH0_NEG_60_300) {
		ret = -EINVAL;
		st->output_mode[AD5770R_CH0].out_range_mode = AD5770R_CH0_0_300;
	}

	if (st->output_mode[AD5770R_CH1].out_range_mode <
	    AD5770R_CH1_0_140_LOW_HEAD ||
	    st->output_mode[AD5770R_CH1].out_range_mode >
	    AD5770R_CH1_0_250) {
		ret = -EINVAL;
		st->output_mode[AD5770R_CH1].out_range_mode =
			AD5770R_CH1_0_140_LOW_NOISE;
	}

	for (i = AD5770R_CH2; i < AD5770R_MAX_CHANNELS; i++)
		if (st->output_mode[i].out_range_mode > AD5770R_CH_HIGH_RANGE) {
			ret = -EINVAL;
			st->output_mode[i].out_range_mode =
				AD5770R_CH_LOW_RANGE;
		}

	return ret;
}

static void ad5770r_parse_dt(struct ad5770r_state *st)
{
	unsigned int i;
	unsigned char tmparray[8], tmp;
	int ret;

	/* Default range for CH1 */
	st->output_mode[AD5770R_CH1].out_range_mode = 2;
	if (!device_property_read_u8_array(&st->spi->dev, "adi,output",
					   tmparray, 6)) {

		ret = ad5770r_check_output_mode(st);

		if (ret)
			for (i = 0; i < AD5770R_MAX_CHANNELS; i++)
				st->output_mode[i].out_range_mode = tmparray[i];
		else
			dev_dbg(&st->spi->dev,
				"Using default range for DAC channels");
	}

	if (!device_property_read_u8_array(&st->spi->dev, "adi,alarm",
					   tmparray, 8)) {
		st->alarm_config.open_drain_en = tmparray[0];
		st->alarm_config.th_sdwn_en = tmparray[1];
		st->alarm_config.bgnd_crc_en = tmparray[2];
		st->alarm_config.temp_wrn_msk = tmparray[3];
		st->alarm_config.over_temp_msk = tmparray[4];
		st->alarm_config.neg_ch0_msk = tmparray[5];
		st->alarm_config.iref_fault_msk = tmparray[6];
		st->alarm_config.bgnd_crc_msk = tmparray[7];
	}

	if (!device_property_read_u8_array(&st->spi->dev, "adi,monitor",
					   tmparray, 4)) {
		st->mon_setup.mon_func = tmparray[0];
		st->mon_setup.mux_buffer = tmparray[1];
		st->mon_setup.ib_ext_en = tmparray[2];
		st->mon_setup.mon_ch = tmparray[3];
	}

	if (!device_property_read_u8(&st->spi->dev, "adi,ref-voltage-microvolt",
				     &tmp)) {
		if (tmp > AD5770R_INT_REF_1_25_V_OUT_OFF) {
			st->ref_sel = AD5770R_INT_REF_1_25_V_OUT_OFF;
			dev_dbg(&st->spi->dev,
				"range invalid, using default");
		} else
			st->ref_sel = tmp;
	}

	st->ext_ref = 0; /* use internal reference */
	if (device_property_read_u8(&st->spi->dev, "adi,external-ref",
				    &tmp))
		dev_dbg(&st->spi->dev,
			"Missing \"external-ref\", using internal reference");

	if (!device_property_read_u8(&st->spi->dev, "adi,external-ref",
				     &tmp))
		st->ext_ref = tmp;

	memset(st->ch_config, 0, sizeof(st->ch_config));
	if (device_property_read_u8_array(&st->spi->dev,
					  "adi,channel-cfg", tmparray, 7))
		dev_dbg(&st->spi->dev,
			"Missing \"channel-config\", powerdown all channels");

	memcpy(st->ch_enable, tmparray, sizeof(st->ch_enable));
	if (device_property_read_u8_array(&st->spi->dev,
					  "adi,channel-enable", tmparray, 6))
		dev_dbg(&st->spi->dev,
			"Missing \"channel-enable\", powerdown all channels");

	if (!device_property_read_u8_array(&st->spi->dev,
					   "adi,channel-cfg", tmparray, 7))
		memcpy(st->ch_config, tmparray, sizeof(st->ch_config));

	if (!device_property_read_u8_array(&st->spi->dev,
					   "adi,channel-enable", tmparray, 6))
		memcpy(st->ch_enable, tmparray, sizeof(st->ch_enable));
}

static int ad5770r_init(struct ad5770r_state *st)
{
	int ret, i;

	ad5770r_parse_dt(st);

	st->gpio_reset = devm_gpiod_get_optional(&st->spi->dev, "reset",
			 GPIOD_OUT_HIGH);

	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	/* Perform a reset */
	ret = ad5770r_reset(st);
	if (ret < 0)
		return ret;

	/* Set output range */
	for (i = 0; i < AD5770R_MAX_CHANNELS; i++)
		ret = ad5770r_set_output_mode(st,
					      &st->output_mode[i], i);

	ret = ad5770r_set_alarm(st, &st->alarm_config);

	if (ret < 0)
		return ret;

	ret = ad5770r_set_monitor_setup(st, &st->mon_setup);

	if (ret < 0)
		return ret;

	ret = ad5770r_set_reference(st, st->ext_ref, st->ref_sel);

	if (ret < 0)
		return ret;

	ret = ad5770r_channel_config(st, st->ch_config);

	if (ret < 0)
		return ret;

	ret = ad5770r_set_channel_en(st, st->ch_enable);

	return ret;
}

static int ad5770r_probe(struct spi_device *spi)
{
	struct ad5770r_state *st;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	regmap = devm_regmap_init_spi(spi, &ad5770r_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}
	st->regmap = regmap;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad5770r_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad5770r_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad5770r_channels);

	ret = ad5770r_init(st);

	if (ret < 0) {
		dev_err(&spi->dev, "AD5770R init failed\n");
		return ret;
	}

	return devm_iio_device_register(&st->spi->dev, indio_dev);
}

static const struct spi_device_id ad5770r_id[] = {
	{ "ad5770r", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad5770r_id);

static struct spi_driver ad5770r_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe = ad5770r_probe,
	.id_table = ad5770r_id,
};

module_spi_driver(ad5770r_driver);

MODULE_AUTHOR("Mircea Caprioru <mircea.caprioru@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5770 IDAC");
MODULE_LICENSE("GPL v2");
