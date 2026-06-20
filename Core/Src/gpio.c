#include "gpio.h"

void GpioInit(struct gpio *gpio, GPIO_TypeDef *port, uint16_t pin)
{
    gpio->port  = port;
    gpio->pin   = pin;
    gpio->state = 0;

    gpio->on     = GpioOn;
    gpio->off    = GpioOff;
    gpio->toggle = GpioToggle;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void GpioOn(struct gpio *gpio)
{
    HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_SET);
    gpio->state = 1;
}

void GpioOff(struct gpio *gpio)
{
    HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_RESET);
    gpio->state = 0;
}

void GpioToggle(struct gpio *gpio)
{
    gpio->state = !gpio->state;
    HAL_GPIO_WritePin(gpio->port, gpio->pin, gpio->state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
