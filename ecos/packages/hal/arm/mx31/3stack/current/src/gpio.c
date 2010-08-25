#include <cyg/hal/gpio.h>

static cyg_uint32 gpio_base[3] = {GPIO1_BASE_ADDR, GPIO2_BASE_ADDR, GPIO3_BASE_ADDR};

void mxc_request_iomux(iomux_pin_name_t pin, iomux_pin_ocfg_t out, iomux_pin_icfg_t in)
{
    cyg_uint32 l, reg;
    
    cyg_uint32 mux_index = PIN_TO_IOMUX_INDEX(pin);
	cyg_uint32 mux_field = PIN_TO_IOMUX_FIELD(pin);
	cyg_uint32 mux_mask = GET_FIELD_MASK(MUX_CTL_BIT_LEN, mux_field);

    reg = IOMUXSW_MUX_CTL + (mux_index * 4);
    l = readl(reg);
	l = (l & (~mux_mask)) | (((out << 4) | in) << (mux_field * MUX_CTL_BIT_LEN));
	writel(l, reg);
}

void mxc_iomux_set_pad(iomux_pin_name_t pin, cyg_uint32 config)
{
	cyg_uint32 reg, l;
	cyg_uint32 pad_index = (pin >> PAD_I) & ((1 << (PAD_F - PAD_I)) - 1);
	cyg_uint32 pad_field = (pin >> PAD_F) & ((1 << (MUX_IO_I - PAD_F)) - 1);
	cyg_uint32 pad_mask = GET_FIELD_MASK(MUX_PAD_BIT_LEN, pad_field);

	reg = IOMUXSW_PAD_CTL + (pad_index * 4);
	l = readl(reg);
	l = (l & (~pad_mask)) | (config << (pad_field * MUX_PAD_BIT_LEN));
	writel(l, reg);
}

void mxc_set_gpio_direction(iomux_pin_name_t pin, int is_input)
{
    cyg_uint32 gpio = IOMUX_TO_GPIO(pin);
    cyg_uint32 port = GPIO_TO_PORT(gpio);
    cyg_uint32 index = GPIO_TO_INDEX(gpio);
    
    cyg_uint32 reg = gpio_base[port] + 0x04;
	cyg_uint32 l;

	l = readl(reg);
	if (is_input)
		l &= ~(1 << index);
	else
		l |= 1 << index;
	writel(l, reg);
}

void mxc_set_gpio_dataout(iomux_pin_name_t pin, cyg_uint32 data)
{
    cyg_uint32 gpio = IOMUX_TO_GPIO(pin);
    cyg_uint32 port = GPIO_TO_PORT(gpio);
    cyg_uint32 index = GPIO_TO_INDEX(gpio);
    
    cyg_uint32 reg = gpio_base[port];
	cyg_uint32 l = 0;

	l = (readl(reg) & (~(1 << index))) | (data << index);
	writel(l, reg);
}

int mxc_get_gpio_datain(iomux_pin_name_t pin)
{
	cyg_uint32 gpio = IOMUX_TO_GPIO(pin);
    cyg_uint32 port = GPIO_TO_PORT(gpio);
    cyg_uint32 index = GPIO_TO_INDEX(gpio);

	cyg_uint32 reg = gpio_base[port];
	return (readl(reg) >> index) & 0x1;
}
