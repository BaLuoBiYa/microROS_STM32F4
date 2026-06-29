#ifndef GPIO_H
#define GPIO_H
#include "main.h"

typedef struct GPIO GPIO;

struct GPIO {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t state;
};

void GPIOInit(GPIO *gpio, GPIO_TypeDef *port, uint16_t pin);
void GPIOOn(GPIO *gpio);
void GPIOOff(GPIO *gpio);
void GPIOToggle(GPIO *gpio);

#endif  // !GPIO_H