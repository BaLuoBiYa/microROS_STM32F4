#include "lcd.h"
#include "lcd_digits_data.h"

/* ── RLE 解码 + 调用 BSP 函数逐行刷新（RGB565） ─────────────── */
/**
 * @brief 解码 RLE 压缩数据，通过 LCD_DispFlush 逐行写入 ILI9341
 * @param rle      RLE 字节数组
 * @param len      数组长度（字节数）
 * @param palette  4 色调色板，palette[0~3] 各为 RGB565 颜色
 */
static void LCD_DrawRLE(const uint8_t *rle, uint32_t len,
                        const uint16_t palette[4])
{
    /* 静态缓冲区 — 避免栈溢出（FreeRTOS 任务栈通常很小） */
    static uint16_t rowBuf[240];
    uint32_t i     = 0; /* RLE 字节索引               */
    uint16_t row   = 0; /* 当前行 Y 坐标              */
    uint16_t col   = 0; /* 当前行内的列 X 坐标        */
    uint16_t color = 0;
    uint16_t run   = 0; /* 当前 run 剩余像素数        */

    while (i < len && row < 320) {
        /* 取下一个 RLE 条目 */
        if (run == 0) {
            uint8_t byte = rle[i++];
            color        = palette[(byte >> 6) & 0x3];
            run          = (byte & 0x3F) + 1; /* 1~64 */
        }

        /* 填充当前行，直到 run 耗尽或行满 */
        while (run > 0 && col < 240) {
            rowBuf[col++] = color;
            run--;
        }

        /* 本行已满 → 调用 BSP 函数刷新这一行 */
        /* 注意：LCD_DispFlush 的后两个参数实际是 x_end, y_end（含），不是宽高 */
        if (col >= 240) {
            LCD_DispFlush(0, row, 239, row, rowBuf);
            col = 0;
            row++;
        }
    }

    /* 最后不足一行的残余像素 */
    if (col > 0 && row < 320) {
        for (uint16_t c = col; c < 240; c++) {
            rowBuf[c] = palette[0]; /* 剩余填充背景色 */
        }
        LCD_DispFlush(0, row, 239, row, rowBuf);
    }
}

/* ── 状态机：循环显示数字 0→1→2→3 ──────────────────────────── */
void StartDisplay(void *argument)
{
    LCD_Init();    /* LCD ILI9341 初始化       */
    LCD_SetDir(0); /* 竖屏                     */

    /* RGB565 调色板 — 每个数字使用不同前景色（BSP 颜色宏） */
    static const uint16_t palettes[4][4] = {
        /* [0]=背景,   [3]=前景 */
        {BLACK, BLACK, BLACK, RED},    /* 0: 黑底红字 */
        {BLACK, BLACK, BLACK, GREEN},  /* 1: 黑底绿字 */
        {BLACK, BLACK, BLACK, BLUE},   /* 2: 黑底蓝字 */
        {BLACK, BLACK, BLACK, YELLOW}, /* 3: 黑底黄字 */
    };

    /* RLE 数据指针表 */
    static const struct {
        const uint8_t *data;
        uint32_t len;
    } digits[4] = {
        {digit_0_rle, sizeof(digit_0_rle)},
        {digit_1_rle, sizeof(digit_1_rle)},
        {digit_2_rle, sizeof(digit_2_rle)},
        {digit_3_rle, sizeof(digit_3_rle)},
    };

    uint8_t state = 0; /* 当前显示的数字 */

    for (;;) {
        /* 全屏填背景色 */
        LCD_Fill(0, 0, 239, 319, palettes[state][0]);

        /* RLE 解码 → 逐行 BSP 刷新 */
        LCD_DrawRLE(digits[state].data, digits[state].len, palettes[state]);

        /* 切换到下一个数字 */
        state = (state + 1) & 0x3;

        osDelay(2000); /* 停留 2 秒 */
    }
}