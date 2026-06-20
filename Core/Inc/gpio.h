#ifndef GPIO_H
#define GPIO_H
#include "main.h"

struct gpio {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t state;

    void (*on)(struct gpio *gpio);
    void (*off)(struct gpio *gpio);
    void (*toggle)(struct gpio *gpio);
};

void GpioInit(struct gpio *gpio, GPIO_TypeDef *port, uint16_t pin);
void GpioOn(struct gpio *gpio);
void GpioOff(struct gpio *gpio);
void GpioToggle(struct gpio *gpio);

#endif  // !GPIO_H