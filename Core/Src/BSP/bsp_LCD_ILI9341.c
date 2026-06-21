#include "BSP/bsp_LCD_ILI9341.h"


/*****************************************************************************
 ** 全局有效    声明、定义
 ****************************************************************************/
xLCD_TypeDef xLCD = {0};  // 管理LCD重要参数


/*****************************************************************************
 ** 本地有效    声明、定义
 ****************************************************************************/
#define LCD_BL_ON  LCD_BL_GPIO->BSRR = LCD_BL_PIN;        // 背光引脚，置高电平
#define LCD_BL_OFF LCD_BL_GPIO->BSRR = LCD_BL_PIN << 16;  // 背光引脚，置低电平


static void setCursor(uint16_t Xpos, uint16_t Ypos);  // 设置光标

volatile typedef struct  // LCD地址结构体
{
    uint16_t LCD_REG;
    uint16_t LCD_RAM;
} LCD_TypeDef;
// 使用NOR/SRAM的 Bank1.sector1,地址位HADDR[27,26]=11 A6作为数据命令区分线
// 注意设置时STM32内部会右移一位对其! 111 1110=0X7E
#define LCD ((LCD_TypeDef *) 0x6001FFFE)  // (0x60000000 | 0x0001FFFE)


// us延时
static void delay_us(volatile uint32_t times)  // 定义一个us延时函数，减少移植时对外部文件依赖; 本函数仅作粗略延时使用，而并非精准延时;
{
    times = times * 20;
    while (--times) {
        __NOP();
    }
}


/******************************************************************************
 * 函  数： delay_ms
 * 功  能： ms 延时函数
 * 备  注： 1、系统时钟168MHz
 *          2、打勾：Options/ c++ / One ELF Section per Function
            3、编译优化级别：Level 3(-O3)
 * 参  数： uint32_t  ms  毫秒值
 * 返回值： 无
 ******************************************************************************/
static volatile uint32_t ulTimesMS;  // 使用volatile声明，防止变量被编译器优化
static void delay_ms(uint16_t ms)
{
    ulTimesMS = ms * 16500;
    while (ulTimesMS) {
        ulTimesMS--;  // 操作外部变量，防止空循环被编译器优化掉
    }
}


// 读寄存器;
uint16_t readReg(uint16_t LCD_Reg)
{
    LCD->LCD_REG = LCD_Reg;  // 写入要读的寄存器序号
    delay_us(5);
    return LCD->LCD_RAM;  // 返回读到的值
}


// BGR转换RGB值; 返回值：RGB格式的颜色值
uint16_t LCD_BGR2RGB(uint16_t c)
{
    uint16_t r, g, b, rgb;
    b   = (c >> 0) & 0x1f;
    g   = (c >> 5) & 0x3f;
    r   = (c >> 11) & 0x1f;
    rgb = (b << 11) + (g << 5) + (r << 0);
    return (rgb);
}


// 读取个某点的颜色值; x,y:坐标、返回值:此点的颜色
uint16_t LCD_ReadPoint(uint16_t x, uint16_t y)
{
    uint16_t r = 0, g = 0, b = 0;
    if (x >= xLCD.width || y >= xLCD.height)
        return 0;  // 超过了范围,直接返回
    setCursor(x, y);
    LCD->LCD_REG = 0X2E;  // 发送读GRAM指令

    r = LCD->LCD_RAM;  // dummy Read

    delay_us(20);
    r = LCD->LCD_RAM;  // 实际坐标颜色

    delay_us(20);
    b = LCD->LCD_RAM;
    g = r & 0XFF;  // 第一次读取的是RG的值,R在前,G在后,各占8位
    g <<= 8;

    return (((r >> 11) << 11) | ((g >> 10) << 5) | (b >> 11));
}


// LCD开启显示
void LCD_DisplayOn(void)
{
    LCD->LCD_REG = 0X29;  // 开启显示
}


// LCD关闭显示
void LCD_DisplayOff(void)
{
    LCD->LCD_REG = 0X28;  // 关闭显示
}


// 设置光标位置; Xpos:横坐标、Ypos:纵坐标
static void setCursor(uint16_t Xpos, uint16_t Ypos)
{
    LCD->LCD_REG = 0X2A;
    LCD->LCD_RAM = Xpos >> 8;
    LCD->LCD_RAM = Xpos & 0XFF;
    LCD->LCD_REG = 0X2B;
    LCD->LCD_RAM = Ypos >> 8;
    LCD->LCD_RAM = Ypos & 0XFF;
}


/******************************************************************
 * 函数名： LCD_DrawPoint
 * 功  能： 画点函数
 * 参  数： x,y:    坐标
 *          _color: 此点的颜色
 * 备  注：
 *****************************************************************/
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t _color)
{
    // 按需考虑是否启用范围控制
    // if( x < xLCD.width && y < xLCD.height)

    LCD->LCD_REG = 0X2A;  // 设置x坐标
    LCD->LCD_RAM = x >> 8;
    LCD->LCD_RAM = x & 0XFF;
    LCD->LCD_REG = 0X2B;  // 设置y坐标
    LCD->LCD_RAM = y >> 8;
    LCD->LCD_RAM = y & 0XFF;
    LCD->LCD_REG = 0X2C;  // 开始写GRAM
    LCD->LCD_RAM = _color;
}


/******************************************************************
 * 函数名： LCD_Init
 * 功  能： 初始化LCD，适用驱动芯片ILI9341
 * 参  数：
 * 备  注：
 *****************************************************************/

void LCD_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};  // GPIO引脚功能配置结构体

    xLCD.FlagInit = 0;  // LCD初始化成功标志

    // 使能GPIO端口
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;  // 使能GPIOA、B、C、D、时钟

#ifdef USE_HAL_DRIVER  // HAL库 配置
    /// 初始化引脚-背光
    GPIO_InitStruct.Pin   = LCD_BL_PIN;                 // 引脚编号
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;        // 引脚工作模式：推挽输出
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                // 内部上下拉：上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // 引脚翻转速率：最快
    HAL_GPIO_Init(LCD_BL_GPIO, &GPIO_InitStruct);       // 初始化
    // 通信引脚 GPIOD部分
    GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;            // 引脚工作模式：复用推挽输出
    GPIO_InitStruct.Pull      = GPIO_PULLUP;                // 内部上下拉：上拉
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;  // 引脚翻转速率：最快
    GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;             // 引脚功能：FSMC
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);                 // 初始化
    // 通信引脚 GPIOE部分
    GPIO_InitStruct.Pin       = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;            // 引脚工作模式：复用推挽输出
    GPIO_InitStruct.Pull      = GPIO_PULLUP;                // 内部上下拉：上拉
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;  // 引脚翻转速率：最快
    GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;             // 引脚功能：FSMC
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);                 // 初始化
#endif

#ifdef USE_STDPERIPH_DRIVER  // 标准库
    // 初始化引脚-背光
    GPIO_InitStruct.GPIO_Pin   = LCD_BL_PIN;        // 引脚
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;     // 普通输出模式
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     // 推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  // 50MHz
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;      // 上拉
    GPIO_Init(LCD_BL_GPIO, &GPIO_InitStruct);       // 初始化 ,推挽输出,控制背光
    // 初始化引脚-GPIOD部分
    GPIO_InitStruct.GPIO_Pin   = (3 << 0) | (3 << 4) | (7 << 8) | (3 << 14);  // PD0,1,4,5,8,9,10,14,15 AF OUT
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;                                // 复用输出
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;                               // 推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;                           // 100MHz
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;                                // 上拉
    GPIO_Init(GPIOD, &GPIO_InitStruct);                                       // 初始化
    // 初始化引脚-GPIOE部分
    GPIO_InitStruct.GPIO_Pin   = (0X1FF << 7);       // PE7~15,AF OUT
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       // 复用输出
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  // 100MHz
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       // 上拉
    GPIO_Init(GPIOE, &GPIO_InitStruct);              // 初始化
    // 初始化引脚-RS_PD11
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_11;        // RS
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       // 复用输出
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  // 100MHz
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       // 上拉
    GPIO_Init(GPIOD, &GPIO_InitStruct);              // 初始化
    // 初始化引脚-NE1_PD7
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_7;         // PD7, FSMC_NE1
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;       // 复用输出
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;  // 100MHz
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;       // 上拉
    GPIO_Init(GPIOD, &GPIO_InitStruct);              // 初始化
    // 配置引脚的功能
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);   // D2
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);   // D3
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);   // NOE_RD
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);   // NWE_WE
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);   // D13
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);   // D14
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);  // D15
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);  // D0
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);  // D1
    // 配置引脚的功能
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);   //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);   //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);   //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);  //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);  //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);  //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);  //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);  //
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);  //
    // 配置引脚的功能
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);  // RS
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);   // CS
#endif

    // 使能FSMC时钟
    RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;  // 使能外设FSMC的时钟

    // 每个BANK的区域1~4, 要配置3个寄存器
    // 区域1:BTCR0、1，BWTR0;
    // 区域1:BTCR2、3，BWTR1;
    // 区域2：BTCR4、5，BWTR2;
    // 区域4：BTCR6、7，BWTR3;
    FSMC_Bank1->BTCR[0]     = 0X00000000;
    FSMC_Bank1->BTCR[0 + 1] = 0X00000000;
    FSMC_Bank1E->BWTR[0]    = 0X00000000;

    // 操作BCR寄存器    使用异步模式
    FSMC_Bank1->BTCR[0] |= 0x01 << 12;  // 存储器写使能
    FSMC_Bank1->BTCR[0] |= 0x01 << 14;  // 读写使用不同的时序
    FSMC_Bank1->BTCR[0] |= 0x01 << 4;   // 存储器数据宽度为16bit

    // 读时序控制寄存器
    FSMC_Bank1->BTCR[0 + 1] |= 0x00 << 28;  // 模式A
    FSMC_Bank1->BTCR[0 + 1] |= 0X0F << 0;   // 地址建立时间(ADDSET)为15个HCLK 1/168M=6ns*15=90ns
    FSMC_Bank1->BTCR[0 + 1] |= 0x3C << 8;   // 数据保存时间(DATAST)为60个HCLK    =6*60=360ns
    // 写时序控制寄存器
    FSMC_Bank1E->BWTR[0] |= 0x00 << 28;  // 模式A
    FSMC_Bank1E->BWTR[0] |= 0x09 << 0;   // 地址建立时间(ADDSET)为9个HCLK=54ns
    FSMC_Bank1E->BWTR[0] |= 0x08 << 8;   // 数据保存时间(DATAST)为6ns*9个HCLK=54ns

    // 使能BANK1，区域1
    FSMC_Bank1->BTCR[0] |= 0x01;  // 使能BANK1，区域1

    delay_ms(50);           // delay 50 ms
    LCD->LCD_REG = 0x0000;  // 写入要写的寄存器序号
    LCD->LCD_RAM = 0x0000;  // 写入数据
    delay_ms(50);           // delay 50 ms
    xLCD.id = readReg(0x0000);

    LCD->LCD_REG = 0XD3;          // 尝试9341 ID的读取
    xLCD.id      = LCD->LCD_RAM;  // dummy read
    xLCD.id      = LCD->LCD_RAM;  // 读到0X00
    xLCD.id      = LCD->LCD_RAM;  // 读取93
    xLCD.id <<= 8;
    xLCD.id |= LCD->LCD_RAM;  // 读取41

    // 重新配置写时序控制寄存器的时序
    FSMC_Bank1E->BWTR[0] &= ~(0XF << 0);  // 地址建立时间(ADDSET)清零
    FSMC_Bank1E->BWTR[0] |= 2 << 8;       // 数据保存时间(DATAST)为6ns*3个HCLK=18ns
    FSMC_Bank1E->BWTR[0] &= ~(0XF << 8);  // 数据保存时间清零
    FSMC_Bank1E->BWTR[0] |= 3 << 0;       // 地址建立时间(ADDSET)为3个HCLK =18ns

    // 屏的参数配置，不用修改，按厂家参数即可
    LCD->LCD_REG = 0xCF;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0xC1;
    LCD->LCD_RAM = 0X30;
    LCD->LCD_REG = 0xED;
    LCD->LCD_RAM = 0x64;
    LCD->LCD_RAM = 0x03;
    LCD->LCD_RAM = 0X12;
    LCD->LCD_RAM = 0X81;
    LCD->LCD_REG = 0xE8;
    LCD->LCD_RAM = 0x85;
    LCD->LCD_RAM = 0x10;
    LCD->LCD_RAM = 0x7A;
    LCD->LCD_REG = 0xCB;
    LCD->LCD_RAM = 0x39;
    LCD->LCD_RAM = 0x2C;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x34;
    LCD->LCD_RAM = 0x02;
    LCD->LCD_REG = 0xF7;
    LCD->LCD_RAM = 0x20;
    LCD->LCD_REG = 0xEA;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_REG = 0xC0;  // Power control
    LCD->LCD_RAM = 0x1B;  // VRH[5:0]
    LCD->LCD_REG = 0xC1;  // Power control
    LCD->LCD_RAM = 0x01;  // SAP[2:0];BT[3:0]
    LCD->LCD_REG = 0xC5;  // VCM control
    LCD->LCD_RAM = 0x30;  // 3F
    LCD->LCD_RAM = 0x30;  // 3C
    LCD->LCD_REG = 0xC7;  // VCM control2
    LCD->LCD_RAM = 0XB7;
    LCD->LCD_REG = 0x36;  // Memory Access Control
    LCD->LCD_RAM = 0x48;
    LCD->LCD_REG = 0x3A;
    LCD->LCD_RAM = 0x55;
    LCD->LCD_REG = 0xB1;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x1A;
    LCD->LCD_REG = 0xB6;  // Display Function Control
    LCD->LCD_RAM = 0x0A;
    LCD->LCD_RAM = 0xA2;
    LCD->LCD_REG = 0xF2;  // 3Gamma Function Disable
    LCD->LCD_RAM = 0x00;
    LCD->LCD_REG = 0x26;  // Gamma curve selected
    LCD->LCD_RAM = 0x01;
    LCD->LCD_REG = 0xE0;  // Set Gamma
    LCD->LCD_RAM = 0x0F;
    LCD->LCD_RAM = 0x2A;
    LCD->LCD_RAM = 0x28;
    LCD->LCD_RAM = 0x08;
    LCD->LCD_RAM = 0x0E;
    LCD->LCD_RAM = 0x08;
    LCD->LCD_RAM = 0x54;
    LCD->LCD_RAM = 0XA9;
    LCD->LCD_RAM = 0x43;
    LCD->LCD_RAM = 0x0A;
    LCD->LCD_RAM = 0x0F;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_REG = 0XE1;  // Set Gamma
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x15;
    LCD->LCD_RAM = 0x17;
    LCD->LCD_RAM = 0x07;
    LCD->LCD_RAM = 0x11;
    LCD->LCD_RAM = 0x06;
    LCD->LCD_RAM = 0x2B;
    LCD->LCD_RAM = 0x56;
    LCD->LCD_RAM = 0x3C;
    LCD->LCD_RAM = 0x05;
    LCD->LCD_RAM = 0x10;
    LCD->LCD_RAM = 0x0F;
    LCD->LCD_RAM = 0x3F;
    LCD->LCD_RAM = 0x3F;
    LCD->LCD_RAM = 0x0F;
    LCD->LCD_REG = 0x2B;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x01;
    LCD->LCD_RAM = 0x3f;
    LCD->LCD_REG = 0x2A;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0x00;
    LCD->LCD_RAM = 0xef;
    LCD->LCD_REG = 0x11;  // 退出睡眠模式
    delay_ms(120);
    LCD->LCD_REG = 0x29;  // 打开显示

    LCD_SetDir(0);  // 设置显示的方向
    LCD_Fill(0, 0, xLCD.width, xLCD.height, BLACK);
    LCD_BL_ON;  // 打开LCD背光
    xLCD.FlagInit = 1;
}


/******************************************************************
 * 函数名： LCD_SetDir
 * 功  能： 设置显示方向
 * 参  数： uint8_t dir     0-竖屏、1-横屏
 * 备  注： 如果使用触摸屏，每次更换方向后，都需要重新校准
 *          完整的寄存器参数值： 0-正竖屏，3-倒竖屏，5-正横屏, 6-倒横屏; 注意：如果使用触摸屏，每次更换方向后，都需要重新校准
 * 返  回： 无
 *****************************************************************/
void LCD_SetDir(uint8_t dir)
{
    uint16_t regval = 0;
    uint16_t temp;

    if (dir == 1)
        dir = 6;

    if (dir == 0 || dir == 3)  // 竖屏
    {
        xLCD.dir    = 0;
        xLCD.width  = LCD_WIDTH;
        xLCD.height = LCD_HEIGHT;
    } else  // 横屏
    {
        xLCD.dir    = 1;
        xLCD.width  = LCD_HEIGHT;
        xLCD.height = LCD_WIDTH;
    }

    if (dir == 0)
        regval |= (0 << 7) | (0 << 6) | (0 << 5);  // 从左到右,从上到下
    if (dir == 3)
        regval |= (1 << 7) | (1 << 6) | (0 << 5);  // 从右到左,从下到上
    if (dir == 5)
        regval |= (0 << 7) | (1 << 6) | (1 << 5);  // 从上到下,从右到左
    if (dir == 6)
        regval |= (1 << 7) | (0 << 6) | (1 << 5);  // 从下到上,从左到右

    regval |= 0X08;
    LCD->LCD_REG = 0X36;    // 写入要写的寄存器序号
    LCD->LCD_RAM = regval;  // 写入数据

    if (regval & 0X20) {
        if (xLCD.width < xLCD.height)  // 交换X,Y
        {
            temp        = xLCD.width;
            xLCD.width  = xLCD.height;
            xLCD.height = temp;
        }
    } else {
        if (xLCD.width > xLCD.height)  // 交换X,Y
        {
            temp        = xLCD.width;
            xLCD.width  = xLCD.height;
            xLCD.height = temp;
        }
    }

    LCD->LCD_REG = 0X2A;
    LCD->LCD_RAM = 0;
    LCD->LCD_RAM = 0;
    LCD->LCD_RAM = (xLCD.width - 1) >> 8;
    LCD->LCD_RAM = (xLCD.width - 1) & 0XFF;
    LCD->LCD_REG = 0X2B;
    LCD->LCD_RAM = 0;
    LCD->LCD_RAM = 0;
    LCD->LCD_RAM = (xLCD.height - 1) >> 8;
    LCD->LCD_RAM = (xLCD.height - 1) & 0XFF;
}


/******************************************************************
 * 函数名： LCD_Fill
 * 功  能： 在指定区域内填充单个颜色
 * 参  数： uint16_t sx     左上角X坐标
 *          uint16_t sy     左上角Y坐标
 *          uint16_t ex     右下角X坐标
 *          uint16_t ey     右下角Y坐标
 *          uint16_t color  颜色值
 * 返  回： 无
 *****************************************************************/
void LCD_Fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color)
{
    uint16_t xlen = 0;
    xlen          = ex - sx + 1;
    for (uint16_t i = sy; i <= ey; i++) {
        setCursor(sx, i);     // 设置光标位置
        LCD->LCD_REG = 0X2C;  // 开始写GRAM
        for (uint16_t j = 0; j < xlen; j++)
            LCD->LCD_RAM = color;  // 显示颜色
    }
}

/******************************************************************
 * 函数名： LCD_DispFlush
 * 功  能： 在指定区域内填充指定数据
 * 备  注： 本函数，适用于图片数据填充、16位、高位在前(与上面的图片显示函数相反);
 *          本函数，适用于LVGL移植的函数：disp_flush()，能有效地快速刷屏
 * 参  数： uint16_t   x        左上角起始X坐标
 *          uint16_t   y        左上角起始Y坐标
 *          uint16_t   width    宽度：每行有多少个16位数据; 可以理解为图片的宽
 *          uint16_t   height   高度：每行有多少个16位数据; 可以理解为图片的高
 *          uint16_t  *pData    数据地址
 * 返  回： 无
 *****************************************************************/
void LCD_DispFlush(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pData)
{
    for (uint16_t nowY = y; nowY <= height; nowY++)  // 逐行显示
    {
        LCD->LCD_REG = 0X2A;                            // 发送设置X坐标的指令
        LCD->LCD_RAM = x >> 8;                          // X坐标的高8位
        LCD->LCD_RAM = x;                               // X坐标的低8位，因为指令读值只是低8位有效，所以等效于 x & 0XFF
        LCD->LCD_REG = 0X2B;                            // 发送设置Y坐标的指令
        LCD->LCD_RAM = nowY >> 8;                       // Y坐标的高8位
        LCD->LCD_RAM = nowY;                            // Y坐标的低8位，因为指令读值只是低8位有效，所以等效于 Y & 0XFF
        LCD->LCD_REG = 0X2C;                            // 指令：开始写入GRAM
        for (uint16_t nowX = x; nowX <= width; nowX++)  // 一行中，从左到右逐个像素
        {
            LCD->LCD_RAM = *pData++;  // 写入每个点的16位颜色数据, RGB565值
        }
    }
}