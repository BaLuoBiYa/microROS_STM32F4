#ifndef GPIO_H
#define GPIO_H
#include "main.h"

typedef struct GPIO GPIO;

struct GPIO {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t state;
};

void GPIO_Init(GPIO *gpio, GPIO_TypeDef *port, uint16_t pin);
void GPIO_On(GPIO *gpio);
void GPIO_Off(GPIO *gpio);
void GPIO_Toggle(GPIO *gpio);

#endif  // !GPIO_H