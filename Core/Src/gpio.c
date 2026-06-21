#include "gpio.h"

void GPIOInit(GPIO *gpio, GPIO_TypeDef *port, uint16_t pin)
{
    gpio->port  = port;
    gpio->pin   = pin;
    gpio->state = 0;

    gpio->on     = GPIOOn;
    gpio->off    = GPIOOff;
    gpio->toggle = GPIOToggle;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void GPIOOn(GPIO *gpio)
{
    HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_SET);
    gpio->state = 1;
}

void GPIOOff(GPIO *gpio)
{
    HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_RESET);
    gpio->state = 0;
}

void GPIOToggle(GPIO *gpio)
{
    gpio->state = !gpio->state;
    HAL_GPIO_WritePin(gpio->port, gpio->pin, gpio->state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
