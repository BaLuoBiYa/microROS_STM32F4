#include "gpio.h"

void GPIO_Init(GPIO *gpio, GPIO_TypeDef *port, uint16_t pin)
{
    gpio->port  = port;
    gpio->pin   = pin;
    gpio->state = 0;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void GPIO_On(GPIO *gpio)
{
    HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_SET);
    gpio->state = 1;
}

void GPIO_Off(GPIO *gpio)
{
    HAL_GPIO_WritePin(gpio->port, gpio->pin, GPIO_PIN_RESET);
    gpio->state = 0;
}

void GPIO_Toggle(GPIO *gpio)
{
    gpio->state = !gpio->state;
    HAL_GPIO_WritePin(gpio->port, gpio->pin, gpio->state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
