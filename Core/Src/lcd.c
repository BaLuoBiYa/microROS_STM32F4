#include "lcd.h"
#include "lcd_digits_data.h"

extern osMessageQueueId_t dispNumHandle;

/* ── 紧凑字模逐行解码绘制 ─────────────────────────────── */
/**
 * @brief 解码紧凑 RLE 字模并逐行刷新到 LCD
 * @param g       字模描述符（RLE 数据 + 宽高）
 * @param x0,y0   屏幕目标左上角
 * @param palette 4 色调色板
 */
static void LCD_DrawGlyph(const DigitGlyph_t *g,
                          uint16_t x0, uint16_t y0,
                          const uint16_t palette[4])
{
    static uint16_t rowBuf[48] __attribute__((section(".ccmram")));
    uint32_t i = 0;
    uint16_t x = 0, y = 0;

    while (i < g->len && y < g->height) {
        uint8_t byte   = g->data[i++];
        uint16_t color = palette[(byte >> 6) & 0x3];
        uint16_t run   = (byte & 0x3F) + 1;

        while (run > 0 && y < g->height) {
            uint16_t n = run;
            if (x + n > g->width)
                n = g->width - x;
            for (uint16_t k = 0; k < n; k++)
                rowBuf[x + k] = color;
            x += n;
            run -= n;

            if (x >= g->width) {
                LCD_DispFlush(x0, y0 + y, x0 + g->width - 1, y0 + y, rowBuf);
                x = 0;
                y++;
            }
        }
    }
}

/* ── 屏幕参数 ────────────────────────────────────────── */
#define SCR_W 240
#define SCR_H 320
#define GAP   4 /* 数字间距（像素） */

void StartDisplay(void *argument)
{
    LCD_Init();
    LCD_SetDir(0);

    /* 每位数字独立调色板 */
    static const uint16_t palettes[10][4] = {
        {WHITE, BLACK, BLACK, RED},       /* 0 */
        {WHITE, BLACK, BLACK, GREEN},     /* 1 */
        {WHITE, BLACK, BLACK, BLUE},      /* 2 */
        {WHITE, BLACK, BLACK, YELLOW},    /* 3 */
        {WHITE, BLACK, BLACK, MAGENTA},   /* 4 */
        {WHITE, BLACK, BLACK, CYAN},      /* 5 */
        {WHITE, BLACK, BLACK, GRED},      /* 6 */
        {WHITE, BLACK, BLACK, BROWN},     /* 7 */
        {WHITE, BLACK, BLACK, LIGHTBLUE}, /* 8 */
        {WHITE, BLACK, BLACK, BRRED},     /* 9 */
    };

    uint16_t num = 0;
    LCD_Fill(0, 0, SCR_W - 1, SCR_H - 1, WHITE);

    for (;;) {
        osMessageQueueGet(dispNumHandle, &num, NULL, osWaitForever);

        /* 拆解十进制数字 */
        uint8_t d[3];
        int n;
        if (num >= 100) {
            d[0] = num / 100;
            d[1] = (num / 10) % 10;
            d[2] = num % 10;
            n    = 3;
        } else if (num >= 10) {
            d[0] = num / 10;
            d[1] = num % 10;
            n    = 2;
        } else {
            d[0] = num;
            n    = 1;
        }

        /* 计算整体宽高 */
        uint16_t tw = 0, mh = 0;
        for (int i = 0; i < n; i++) {
            tw += digit_glyphs[d[i]].width;
            if (digit_glyphs[d[i]].height > mh)
                mh = digit_glyphs[d[i]].height;
        }
        tw += GAP * (n - 1);

        /* 居中偏移 */
        uint16_t sx = (SCR_W - tw) / 2;
        uint16_t sy = (SCR_H - mh) / 2;

        /* 清屏 + 逐个绘制数字 */
        LCD_Fill(0, 0, SCR_W - 1, SCR_H - 1, WHITE);
        uint16_t cx = sx;
        for (int i = 0; i < n; i++) {
            const DigitGlyph_t *g = &digit_glyphs[d[i]];
            uint16_t yo           = (mh - g->height) / 2; /* 垂直居中对齐 */
            LCD_DrawGlyph(g, cx, sy + yo, palettes[d[i]]);
            cx += g->width + GAP;
        }
    }
}