#if !defined(_NOKIA5110_H_)
#define _NOKIA5110_H_


#define	LCD_X_RES	84	// разрешение по горизонтали
#define	LCD_Y_RES	48	// разрешение по вертикали

// Режимы вывода текста
#define LCD_TWICE_WIDTH		0x1
#define LCD_TWICE_HEIGHT	0x2
#define LCD_INVERSION		0x4

// Режимы вывода графики
#define LCD_OR   0
#define LCD_AND  1
#define LCD_XOR  2


// Сигналы управления индикатором NOKIA5110
///////////////////////////////////////////////////////////////////////////////
#define NOKIA5110_CE	B,2,L	// PB2,Выв.14. Выход. (Вход CE (Chip Enable)
								// индикатора NOKIA 5110)
								
#define NOKIA5110_DIN	B,3,H	// PB3/MOSI,Выв.15. Выход. (Вход DIN (Data In)
								// индикатора NOKIA 5110)
								
#define NOKIA5110_CLK	B,5,H	// PB5/SCK,Выв.17. Выход. (Вход CLK индикатора
								// NOKIA 5110)
								
#define NOKIA5110_DC	B,1,H	// PB1,Выв.13. Выход. (Вход DC (Data/Command)
								// индикатора NOKIA 5110)
								
#define NOKIA5110_RST	B,0,L	// PB0,Выв.12. Выход. (Вход RST индикатора
								// NOKIA 5110)



// Инициализация индикатора
extern PT_THREAD(LCD_Init(struct pt *Context));

extern uint8_t LCD_Busy(void);

// Выводит содержимое измененной области буфера в индикатор
extern PT_THREAD(LCD_Update(struct pt *Context));

// Очищает область буфера с указанными координатами
extern uint8_t LCD_ClearBuf(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2);

// Очищает область индикатора с указанными координатами
extern PT_THREAD(LCD_Clear(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2));

// Формирует изображение строки в буфере, расположенном в ОЗУ
extern uint8_t LCD_StrBuf(uint8_t X, uint8_t Y, uint8_t idFont, char *Str,
	uint8_t Settings);

// Выводит изображение строки на индикатор
extern PT_THREAD(LCD_Str(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, char *Str, uint8_t Settings));

// Формирует изображение строки в буфере, расположенном в ОЗУ.
// Исходная строка при этом располагается во FLASH.
extern uint8_t LCD_StrBuf_P(uint8_t X, uint8_t Y, uint8_t idFont,
	const char *Str, uint8_t Settings);

// Выводит изображение строки в индикатор.
// Исходная строка при этом располагается во FLASH.
extern PT_THREAD(LCD_Str_P(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, const char *Str, uint8_t Settings));

// Формирует изображение символа в буфере, расположенном в ОЗУ
extern uint8_t LCD_ChrBuf(uint8_t X, uint8_t Y, uint8_t idFont, char Chr,
	uint8_t Settings);

// Выводит изображение символа на индикатор
extern PT_THREAD(LCD_Chr(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, char Chr, uint8_t Settings));

extern PT_THREAD(LCD_Contrast(struct pt *Context, uint8_t Contrast));


// ГРАФИКА
///////////////////////////////////////////////////////////////////////////////

// Установка режима отрисовки графики
extern void LCD_DrawMode(uint8_t Mode);

// Выводит один пиксел на индикатор
extern PT_THREAD(LCD_Pixel(struct pt *Context, uint8_t X, uint8_t Y));

// Рисует линию между двумя точками на индикаторе
extern PT_THREAD(LCD_Line(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2));

// Рисует окружность
extern PT_THREAD(LCD_Circle(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t R));

// Рисование прямоугольника
extern PT_THREAD(LCD_Rect(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2, uint8_t Fill));

#endif
