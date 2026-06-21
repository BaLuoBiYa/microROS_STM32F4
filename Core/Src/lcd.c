#include "lcd.h"

void StartDisplay(void *argument)
{
    LCD_Init();                       // LCD ILI9341 初始化
    LCD_SetDir(0);                    // 设置显示方向：0-竖屏、1-横屏
    LCD_Fill(0, 0, 240, 320, WHITE);  // 整个背景填充白色

    for (;;) {
        osDelay(10);
    }
}