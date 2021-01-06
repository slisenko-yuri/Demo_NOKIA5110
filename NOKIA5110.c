#include "Config.h"
#include "SPI.h"
#include "Font.h"
#include "NOKIA5110.h"
#include "Mt.h"

///////////////////////////////////////////////////////////////////////////////
// Библиотека для поддержки индикатора NOKIA-5110 на базе контроллера PCD8544
///////////////////////////////////////////////////////////////////////////////

/*
=========================================================
Подключение индикатора NOKIA-5110 к ATmega328P
=========================================================
NOKIA 5110		ATmega328P		Название сигнала
---------------------------------------------------------
01 (RST)		14  (PB0)		Сброс индикатора (0)
02 (CE)			16  (PB2)		Выбор индикатора (0)
03 (DC)			15  (PB1)		Команда(0)/Данные(1)
04 (DIN)		17  (PB3)		Вход данных индикатора
05 (CLK)		19  (PB5)		Тактирование
06 (+3.3В)		7,20(VCC)		Питание
07 (LIGHT)		GND				Подсветка
08 (GND)		8,22(GND)		Общий
---------------------------------------------------------

=====================================
Подключение программатора AVRISP mkII
к микроконтроллеру ATmega328P (DIP28)
=====================================
Программатор	ATmega328P
AVRISP mkII
--------------------------
1 (MISO)		18 (PB4)
2 (VCC)			7,20(VCC)
3 (SCK)			19 (PB5)
4 (MOSI)		17 (PB3)
5 (RESET)		1 (RESET)
6 (GND)			8,22(GND)
--------------------------
*/




// Команды индикатора NOKIA 5110
///////////////////////////////////////////////////////////////////////////////

#define	CMD_FUNCTION_SET	0x20	// power down control;
									// entry mode;
									// extended instruction set control (H)
									// 0 0 1 0 0 PD V H

// Command Set in Basic Mode (H = 0)

#define CMD_DISPLAY_CONTROL	0x08	// sets display configuration
									// 0 0 0 0 1 D 0 E

#define	CMD_ADR_Y			0x40	// sets Y-address of RAM; 0 <= Y <= 5
									// 0 1 0 0 0 Y2 Y1 Y0

#define	CMD_ADR_X			0x80	// sets X-address part of RAM; 0 <= X <= 83
									// 1 X6 X5 X4 X3 X2 X1 X0

// Command Set in addition mode (H = 1)

#define CMD_TEMP_CONTROL	0x04	// set Temperature Coefficient (TCx)
									// 0 0 0 0 0 1 TC1 TC0
									
#define CMD_BIAS_SYSTEM		0x10	// set Bias System (BSx)
									// 0 0 0 1 0 BS2 BS1 BS0

#define CMD_SET_VOP			0x80	// it is command to set voltage for
									// VLCD (Voltage Operation).
									// 1 Vop6 Vop5 Vop4 Vop3 Vop2 Vop1 Vop0

// Номера бит в командах индикатора NOKIA 5110
//////////////////////////////////////////////////////////////////////////
#define	NUM_PD	2	// It is Bit to select operation mode.
					// 0 - chip is active
					// 1 - chip is in Power-down mode

#define NUM_V	1	// It is Bit to select the format of increasing
					// address value.
					// 0 - horizontal addressing
					// 1 - vertical addressing

#define NUM_H	0	// It is Bit to select format of using commands of
					// SCD.
					// 0 - use basic instruction set
					// 1 - use extended instruction set

#define NUM_D	2	// D and E:
#define NUM_E	0	// 00 - display blank
					// 01 - all display segments on
					// 10 - normal mode
					// 11 - inverse video mode

#define TC0		0	// VLCD temperature coefficient 0
#define TC1		1	// VLCD temperature coefficient 1
#define TC2		2	// VLCD temperature coefficient 2
#define TC3		3	// VLCD temperature coefficient 3

#define BS0		0	// MUX RATE 1:100
#define BS1		1	// MUX RATE 1:80
#define BS2		2	// MUX RATE 1:65/1:65
#define BS3		3	// MUX RATE 1:48
#define BS4		4	// MUX RATE 1:40/1:34
#define BS5		5	// MUX RATE 1:24
#define BS6		6	// MUX RATE 1:18/1:16
#define BS7		7	// MUX RATE 1:10/1:9/1:8

#define STR_FLASH	TRUE
#define STR_RAM		FALSE

// Макрос обменивает между собой значения параметров x и y
#define SWAP(x, y)	{uint8_t temp; temp = x; x = y; y = temp;}




union Union64
{
	uint8_t byte[8];
	uint64_t value;
};




static uint8_t	Buf[LCD_Y_RES / 8][LCD_X_RES]; // Буфер для индикатора

// Координаты измененной области буфера, которая еще не передана в индикатор
static uint8_t xlChanged;
static uint8_t xhChanged;
static uint8_t ylChanged;
static uint8_t yhChanged;

// Координаты области буфера, которая в данный момент передается в индикатор
static uint8_t xlTransfer;
static uint8_t xhTransfer;
static uint8_t ylTransfer;
static uint8_t yhTransfer;

static uint8_t DrawMode; // Режим вывода графики




// Массив, необходимый для вывода символов удвоенной высоты
static const uint8_t Bit4to8[] PROGMEM =
{
	0x00, 0x03, 0x0C, 0x0F, 0x30, 0x33, 0x3C, 0x3F,
	0xC0, 0xC3, 0xCC, 0xCF, 0xF0, 0xF3, 0xFC, 0xFF
};




///////////////////////////////////////////////////////////////////////////////
// Из двух значений возвращает максимальное.
///////////////////////////////////////////////////////////////////////////////
static uint16_t Max(uint16_t a, uint16_t b)
{
	if (a > b) return a;
	return b;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Из двух значений возвращает минимальное.
///////////////////////////////////////////////////////////////////////////////
static uint16_t Min(uint16_t a, uint16_t b)
{
	if (a < b) return a;
	return b;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Запись команды в индикатор.
///////////////////////////////////////////////////////////////////////////////
static uint8_t SendCmd(uint8_t Value)
{
	if (!SPI_WaitReady()) return FALSE;
	
	OFF(NOKIA5110_DC);
	
	SPI_SendByteToLCD(Value);

	return TRUE;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Запись байта данных в индикатор.
///////////////////////////////////////////////////////////////////////////////
static uint8_t SendData(uint8_t Value)
{
	if (!SPI_WaitReady()) return FALSE;
	
	ON(NOKIA5110_DC);
	
	SPI_SendByteToLCD(Value);

	return TRUE;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Запись массива данных в индикатор.
///////////////////////////////////////////////////////////////////////////////
static uint8_t SendBufPtr(uint8_t* Buf, uint8_t Size)
{
	if (!SPI_WaitReady()) return FALSE;
	
	ON(NOKIA5110_DC); // Здесь всегда устанавливаем режим передачи данных
						// (не команд)
	SPI_SendBufPtrToLCD(Buf, Size);

	return TRUE;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Формирует изображение строки в буфере, расположенном в ОЗУ.
// Параметры:
// X - Начальная координата по горизонтали (0...83)
// Y - Начальная координата по вертикали (0...47)
// idFont - Идентификатор шрифта (например: FONT_6x8). Идентификаторы
//          реализованных шрифтов объявлены в файле Font.h
// Str - Адрес строки во FLASH, либо в RAM
// Settings - Режим отображения строки. Может содержать комбинацию
//        следующих флагов:
//        LCD_TWICE_WIDTH (Удвоение ширины символов строки)
//        LCD_TWICE_HEIGHT (Удвоение высоты символов строки)
//        LCD_INVERSION (Инверсное изображение символов строки)
// fFLASH - Флаг, указывающий местонахождение строки, указанной параметром
//          Str. Если флаг равен TRUE, то это означает, что строка
//          находится во FLASH, иначе в RAM.
// ПРИМЕЧАНИЯ:
// -Начальной координатой является верхняя левая точка первого символа строки.
// -Функция используется только внутри библиотеки NOKIA5110.
///////////////////////////////////////////////////////////////////////////////
static uint8_t StrBuf(uint8_t X, uint8_t Y, uint8_t idFont, const char *Str,
	uint8_t Settings, uint8_t fFLASH)
{
	uint8_t	col, row, j, num, m, data;
	uint8_t xl, xh, yl, yh;
	uint8_t len;
	uint8_t chr;
	uint16_t tmp;
	uint8_t widthFont; // Ширина символа в пикселах
	uint8_t heightFont; // Высота символа в пикселах
	uint8_t cntByteChangeCol; // Количество изменяемых байтов буфера для
								// одного столбца символа
	uint8_t cntByteSymCol; // Количество байт в одном столбце символа
	union Union64 mask; // Маска для столбцов символа
	union Union64 column; // Столбец символа
	uint8_t const *addrFont; // Адрес шрифта
	uint8_t twiceW = 0; // 1 = удвоение ширины символа
	uint8_t twiceH = 0; // 1 = удвоение высоты символа
	uint8_t inv = 0; // 1 = инверсное изображение символа
	
	widthFont = FONT_Width(idFont); // Ширина символов шрифта
	heightFont = FONT_Height(idFont); // Высота символов шрифта

	if (Settings & LCD_TWICE_WIDTH)
		twiceW = 1; // Удвоенная ширина

	if (Settings & LCD_TWICE_HEIGHT)
		twiceH = 1; // Удвоенная высота

	if (Settings & LCD_INVERSION)
		inv = 1; // Инверсное изображение символов

	if (fFLASH)
		tmp = strlen_P(Str); // Вычисляем длину строки во FLASH
	else
		tmp = strlen(Str); // Вычисляем длину строки в RAM

	if (tmp < 255) len = tmp; else len = 255;

	// Если в данный момент идет передача буфера в индикатор,
	// то нужно проверить, не пересекает ли передаваемая область
	// новую изменяемую область
	
	// Вычисляем новые координаты точек изменяемой области буфера с учетом
	// находящихся там данных
	xl = Min(X, xlChanged);
	tmp = Min(LCD_X_RES, X + len * (widthFont << twiceW));
	xh = Max(tmp, xhChanged);

	yl = Min(Y, ylChanged);
	tmp = Min(LCD_Y_RES, Y + (heightFont << twiceH));
	yh = Max(tmp, yhChanged);

	// Если в данный момент идет передача в индикатор, то проверяем
	// пересечение новой изменяемой области буфера с передаваемой областью
	if (LCD_Busy())
	{
		// Если изменяемая область пересекается с передаваемой, то выходим
		if (!(xh <= xlTransfer) || (xhTransfer <= xl) ||
			(yh <= ylTransfer) || (yhTransfer <= yl)) return FALSE;
	}

	// Сохраняем новые координаты точек изменяемой области буфера
	xlChanged = xl;
	xhChanged = xh;
	ylChanged = yl;
	yhChanged = yh;

	// Получаем адрес шрифта (массива байтов, представляющего собой графическое
	// представление символов).
	addrFont = FONT_Addr(idFont) + 2;

	// Вычисляем маску, которую будем использовать для столбцов символа.
	// В этой маске количество единиц равно высоте выбранного шрифта.
	mask.value = 1;
	mask.value <<= (heightFont << twiceH);
	mask.value--; // Получаем маску в виде всех единиц для значимых точек
					// столбца символа.

	#if defined(LCD_ROTATE)
	// Получаем маску, сдвинутую к старшим разрядам.
	mask.value <<= sizeof(mask) * 8 - (heightFont << twiceH) - Y % 8;
	#else
	mask.value <<= Y % 8; // Сдвигаем маску с учетом координаты Y.
	#endif

	// Вычисляем количество байт, требуемых для одного столбца символа
	// (без учета удвоения по высоте и ширине).
	cntByteSymCol = (heightFont - 1) / 8 + 1;

	// Т.к. символ может располагаться начиная с любой координаты, а не только
	// с кратной размеру байта (например, один столбец символа с высотой 8
	// точек может располагаться в двух байтах), то необходимо вычислить
	// количество изменяемых байтов в буфере для одного столбца символа.

	// Подсчитываем количество байт для одного столбца символа, которые будут
	// подвергнуты изменению.

	cntByteChangeCol = 0;

	#if defined(LCD_ROTATE)
	for (num = sizeof(mask); num > 0; num--)
	{
		if (mask.byte[num - 1] != 0) cntByteChangeCol++;
		else break;
	}
	#else
	for (num = 0; num < sizeof(mask); num++)
	{
		if (mask.byte[num] != 0) cntByteChangeCol++;
		else break;
	}
	#endif

	mask.value = ~mask.value; // Инвертируем маску.

	// Цикл по каждому символу строки
	///////////////////////////////////////////////////////////////////////////
	for (j = 0; j < len; j++)
	{
		if (X >= LCD_X_RES) break;
		
		if (fFLASH)
			// Если символ строки находится во FLASH
			chr = pgm_read_byte(&Str[j]);
		else
			// Если символ строки находится в ОЗУ
			chr = Str[j];

		// Корректируем код символа в соответствии с кодировкой
		///////////////////////////////////////////////////////////////////////

		if ((chr >= 0x20) && (chr <= 0x7F))
		{
			// Смещение в таблице для символов ASCII[0x20-0x7F]
			chr -= 0x20; //chr -= 32;
		}
		else if (chr >= 0xC0)
		{
			// Смещение в таблице для символов CP1251[0xC0-0xFF]
			chr -= 0x60; //chr -= 96;
		}
		else
		{
			// Остальные игнорируем (их нет в таблице для экономии памяти)
			chr = 0; // Пробел
		}

		// Цикл по всем столбцам символа
		///////////////////////////////////////////////////////////////////////
		for (col = 0; col < widthFont; col++)
		{
			if (X >= LCD_X_RES) break;

			column.value = 0;

			// Читаем байты для одного очередного столбца символа
			///////////////////////////////////////////////////////////////////
			for (num = 0; num < cntByteSymCol; num++)
			{
				if (twiceH) // Если удвоенная высота символа
				{
					m = pgm_read_byte(&(addrFont[chr * widthFont *
						cntByteSymCol + col * cntByteSymCol + num]));

					#if defined(LCD_ROTATE)
					column.byte[sizeof(column) - 1 - (num * 2)] =
						pgm_read_byte(&(Bit4to8[m >> 4]));
					column.byte[sizeof(column) - 1 - (num * 2 + 1)] =
						pgm_read_byte(&(Bit4to8[m & 0xF]));
					#else
					column.byte[num * 2] = pgm_read_byte(&(Bit4to8[m & 0xF]));
					column.byte[num * 2 + 1] = pgm_read_byte(&(Bit4to8[m >> 4]));
					#endif
				}
				else
				{
					#if defined(LCD_ROTATE)
					column.byte[sizeof(column) - 1 - num] =
						pgm_read_byte(&(addrFont[chr * widthFont * cntByteSymCol +
							col * cntByteSymCol + num]));
					#else
					column.byte[num] =
						pgm_read_byte(&(addrFont[chr * widthFont * cntByteSymCol +
							col * cntByteSymCol + num]));
					#endif
				}
			}

			// Сдвигаем столбец с учетом координаты Y.
			#if defined(LCD_ROTATE)
			column.value >>= Y % 8;
			#else
			column.value <<= Y % 8;
			#endif
			
			// Если инверсное изображение, то инвертируем пиксели столбца.
			if (inv)
			{
				column.value ^= ~mask.value;
			}
			
			// Пишем столбец в буфер
			for (m = 0; m <= twiceW; m++)
			{
				if (X < LCD_X_RES)
				{
					// Цикл по изменяемым байтам буфера для одного столбца
					// символа					
					for (num = 0; num < cntByteChangeCol; num++)
					{
						row = Y + num * 8;
						if (row < LCD_Y_RES)
						{
							#if defined(LCD_ROTATE)
							row = (LCD_Y_RES / 8 - 1) - row / 8;
							data = Buf[row][(LCD_X_RES - 1) - X];
							data &= mask.byte[(sizeof(mask) - 1) - num];
							data |= column.byte[(sizeof(column) - 1) - num];
							Buf[row][(LCD_X_RES - 1) - X] = data;
							#else
							row /= 8;
							data = Buf[row][X];
							data &= mask.byte[num];
							data |= column.byte[num];
							Buf[row][X] = data;
							#endif
						}
					}
					X++;
				}
				else
				{
					X = LCD_X_RES;
					break;
				}
			}
		}
	}
	if (X > LCD_X_RES) X = LCD_X_RES;
	
	return TRUE;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Инициализация индикатора.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Init(struct pt *Context))
{
	PT_BEGIN(Context); // Начало протопотока

	MT_MutexWait(Context, MUTEX_LCD); // Захватываем мьютекс

	DRIVER(NOKIA5110_DC, OUT);
	DRIVER(NOKIA5110_RST, OUT);

	ON(NOKIA5110_RST); // RST=0
	_delay_us(50); // Задержка >= 100нс
	OFF(NOKIA5110_RST); // RST=1

	// Отправляем команды дисплею
	// 0010 0PVH (P-Power Down, V-Vertical, H-Extended)
	// Включаем расширенный набор команд (LCD Extended Commands)
	PT_WAIT_UNTIL(Context, SendCmd(CMD_FUNCTION_SET | (0<<NUM_PD) |
		(0<<NUM_V) | (1<<NUM_H)));
	
	// Настройка питания (LCD bias mode 1:48) (Один из 8-ми режимов расчета
	// смещения уровней для управления LCD)
	// 0001 0bbb
	PT_WAIT_UNTIL(Context, SendCmd(CMD_BIAS_SYSTEM | BS3)); 

	// Установка температурного коэффициента (Temp coefficent) (Один из 4-х
	// режимов температурной коррекции)
	// 0000 01tt
	PT_WAIT_UNTIL(Context, SendCmd(CMD_TEMP_CONTROL | TC0));

	// Установка контрастности (LCD Vop) (Напряжение питания ЖК-элементов)
	// 1vvv vvvv
	PT_WAIT_UNTIL(Context, SendCmd(CMD_SET_VOP | 0x38));

	// Включаем стандартный набор команд и горизонтальную адресацию (LCD
	// Standard Commands,Horizontal addressing mode)
	// 0010 0PVH (P-Power Down, V-Vertical, H-Extended)
	PT_WAIT_UNTIL(Context, SendCmd(CMD_FUNCTION_SET | (0<<NUM_PD) | (0<<NUM_V) | (0<<NUM_H)));
	
	// 0000 1D0E (D=1-Нормальное отображение,D=0-Экран очищен,E=1-Инверсия)
	PT_WAIT_UNTIL(Context, SendCmd(CMD_DISPLAY_CONTROL | (1<<NUM_D) | (0<<NUM_E)));

	DrawMode = LCD_OR;

	// Инициализация координат измененой области буфера
	xlChanged = LCD_X_RES;
	xhChanged = 0;
	ylChanged = LCD_Y_RES;
	yhChanged = 0;

	// Инициализация координат области буфера, передаваемой в индикатор
	xlTransfer = 0;
	xhTransfer = 0;
	ylTransfer = 0;
	yhTransfer = 0;

	MT_MutexFree(MUTEX_LCD); // Освобождаем мьютекс
	
	PT_END(Context); // Завершение протопотока
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Возвращает TRUE, если индикатор занят, т.е. в данный момент происходит
// передача из буфера в индикатор.
///////////////////////////////////////////////////////////////////////////////
uint8_t LCD_Busy(void)
{
	if (yhTransfer > ylTransfer)
		return TRUE;

	return FALSE;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Выводит содержимое измененной области буфера в индикатор.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Update(struct pt *Context))
{
	PT_BEGIN(Context); // Начало протопотока
	
	MT_MutexWait(Context, MUTEX_LCD); // Захват мьютекса

	// Если в буфере изменилась информация, то ее нужно передать в индикатор
	if ((xhChanged > xlChanged) && (yhChanged > ylChanged))
	{
		// Запоминаем координаты передаваемой области
		xlTransfer = xlChanged;
		xhTransfer = xhChanged;
		ylTransfer = ylChanged;
		yhTransfer = yhChanged;
		
		// Освобождаем координаты измененной области буфера
		xlChanged = LCD_X_RES;
		xhChanged = 0;
		ylChanged = LCD_Y_RES;
		yhChanged = 0;

		while (yhTransfer > ylTransfer)
		{
			#if defined(LCD_ROTATE)
			// Устанавливаем начальный адрес по X
			PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X |
				(LCD_X_RES - xhTransfer)));

			// Устанавливаем начальный адрес по Y
			PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y |
				((LCD_Y_RES - 1 - ylTransfer) / 8)));

			// Передаем данные в индикатор
			PT_WAIT_UNTIL(Context, SendBufPtr(&Buf[(LCD_Y_RES - 1 - ylTransfer)
				/ 8][LCD_X_RES - xhTransfer], xhTransfer - xlTransfer));
			#else

			// Устанавливаем начальный адрес по X
			PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X | xlTransfer));

			// Устанавливаем начальный адрес по Y
			PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y | (ylTransfer / 8)));

			// Передаем данные в индикатор
			PT_WAIT_UNTIL(Context, SendBufPtr(&Buf[ylTransfer / 8][xlTransfer],
				xhTransfer - xlTransfer));
			#endif

			// Корректируем координаты передаваемой области буфера с
			// учетом переданной порции данных
			ylTransfer += (8 - (ylTransfer % 8));
		}
		
		// Освобождем координаты передаваемой области
		xlTransfer = 0;
		xhTransfer = 0;
		ylTransfer = 0;
		yhTransfer = 0;
	}
	
	MT_MutexFree(MUTEX_LCD); // Освобождаем мьютекс
	
	PT_END(Context); // Завершение протопотока
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Очищает область буфера с указанными координатами.
// Параметры:
// X1 - Координата верхнего левого угла очищаемой области по горизонтали
//      (0...83),
// Y1 - Координата верхнего левого угла очищаемой области по вертикали
//      (0...47),
// X2 - Координата нижнего правого угла очищаемой области по горизонтали
//      (0...83),
// Y1 - Координата нижнего правого угла очищаемой области по вертикали
//      (0...47).
//
// ПРИМЕР:
// // Очистить весь буфер
// PT_WAIT_UNTIL(Context, LCD_ClearBuf(0, 0, 83, 47));
// // Вывод измененной части буфера в индикатор
// PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
///////////////////////////////////////////////////////////////////////////////
uint8_t LCD_ClearBuf(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2)
{
	uint8_t sizeY;
	union Union64 mask;
	uint8_t cntByteChangeCol;
	uint8_t xl, xh, yl, yh;
	uint8_t x, num;

	// Разбираемся со входными параметрами

	// Если левая координата больше правой, то меняем их значения между собой
	if (X1 > X2) SWAP(X1, X2)

	// Если верхняя координата больше нижней, то меняем их значения между собой
	if (Y1 > Y2) SWAP(Y1, Y2)

	if ((X1 >= LCD_X_RES) || (Y1 >= LCD_Y_RES)) return TRUE;
	if (X2 >= LCD_X_RES) X2 = LCD_X_RES - 1;
	if (Y2 >= LCD_Y_RES) Y2 = LCD_Y_RES - 1;
	X2++;
	Y2++;

	// Вычисляем новые координаты для изменяемой области буфера 
	xl = Min(X1, xlChanged);
	xh = Max(X2, xhChanged);
	yl = Min(Y1, ylChanged);
	yh = Max(Y2, yhChanged);

	// Если в данный момент идет передача в индикатор, то проверяем
	// пересечение передаваемой области с изменяемой
	if (LCD_Busy())
	{
		// Если изменяемая область пересекается с передаваемой, то выходим
		if (!(xh <= xlTransfer) || (xhTransfer <= xl) || (yh <= ylTransfer) ||
			(yhTransfer <= yl))
			return FALSE;
	}

	// Устанавливаем новые координаты изменяемой области буфера
	xlChanged = xl;
	xhChanged = xh;
	ylChanged = yl;
	yhChanged = yh;
	
	// Вычисляем размер очищаемой области по вертикали
	sizeY = Y2 - Y1;

	// Вычисляем маску, которую будем использовать для очистки столбцов.
	// В этой маске количество единиц равно высоте очищаемой области
	mask.value = 1;
	mask.value <<= sizeY;
	mask.value--; // Получаем маску в виде всех единиц для значимых точек
					// столбцов очищаемой области

	#if defined(LCD_ROTATE)
	// Получаем маску, сдвинутую к старшим разрядам
	mask.value <<= sizeof(mask) * 8 - sizeY - Y1 % 8;
	#else
	mask.value <<= Y1 % 8;
	#endif

	// Подсчитываем количество байт для одного столбца очищаемой области,
	// которые будут подвергнуты изменению.

	cntByteChangeCol = 0;

	#if defined(LCD_ROTATE)
	for (num = sizeof(mask); num > 0; num--)
	{
		if (mask.byte[num - 1] != 0) cntByteChangeCol++;
		else break;
	}
	#else
	for (num = 0; num < sizeof(mask); num++)
	{
		if (mask.byte[num] != 0) cntByteChangeCol++;
		else break;
	}
	#endif
	
	mask.value = ~mask.value; // Инвертируем маску
	
	// Цикл по каждому столбцу очищаемой области
	for (x = X1; x < X2; x++)
	{
		for (num = 0; num < cntByteChangeCol; num++)
		{
			#if defined(LCD_ROTATE)
			Buf[(LCD_Y_RES - 1 - (Y1 + num * 8)) / 8][LCD_X_RES - 1 - x] &=
				mask.byte[sizeof(mask) - 1 - num];
			#else
			Buf[(Y1 + num * 8) / 8][x] &= mask.byte[num];
			#endif
		}
	}

	return TRUE;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Очищает область индикатора с указанными координатами.
// Параметры:
// X1 - Координата верхнего левого угла очищаемой области по горизонтали
//      (0...83),
// Y1 - Координата верхнего левого угла очищаемой области по вертикали
//      (0...47),
// X2 - Координата нижнего правого угла очищаемой области по горизонтали
//      (0...83),
// Y1 - Координата нижнего правого угла очищаемой области по вертикали
//      (0...47).
//
// ПРИМЕР:
// PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 0, 83, 47));
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Clear(struct pt *Context, uint8_t X1, uint8_t Y1, uint8_t X2,
	uint8_t Y2))
{
	static struct pt ContextChild; // Контекст для дочернего протопотока

	PT_BEGIN(Context); // Начало протопотока

	PT_WAIT_UNTIL(Context, LCD_ClearBuf(X1, Y1, X2, Y2));
	PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));	

	PT_END(Context); // Завершение протопотока
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Формирует изображение строки в буфере, расположенном в ОЗУ по координатам,
// указанным параметрами X и Y.
// Параметры:
// X - Начальная координата по горизонтали (0...83).
// Y - Начальная координата по вертикали (0...47).
// idFont - Идентификатор шрифта (например: FONT_6x8). Идентификаторы
//          реализованных шрифтов объявлены в файле Font.h.
// Str - Адрес строки в ОЗУ.
// Settings - Режим отображения строки. Может содержать комбинацию
//        следующих флагов:
//        LCD_TWICE_WIDTH (Удвоение ширины символов строки)
//        LCD_TWICE_HEIGHT (Удвоение высоты символов строки)
//        LCD_INVERSION (Инверсное изображение символов строки)
// ПРИМЕЧАНИЯ:
// -Начальной координатой является верхняя левая точка первого символа строки.
//
// ПРИМЕР:
// PT_WAIT_UNTIL(Context, LCD_StrBuf(42, 12, FONT_6x8, Msg,
//   LCD_TWICE_WIDTH | LCD_TWICE_HEIGHT));
///////////////////////////////////////////////////////////////////////////////
uint8_t	LCD_StrBuf(uint8_t X, uint8_t Y, uint8_t idFont, char *Str,
	uint8_t Settings)
{
	return StrBuf(X, Y, idFont, Str, Settings, STR_RAM);
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Выводит изображение строки в индикатор по координатам, указанным параметрами
// X и Y.
// Параметры:
// X - Начальная координата по горизонтали (0...83).
// Y - Начальная координата по вертикали (0...47).
// idFont - Идентификатор шрифта (например: FONT_6x8). Идентификаторы
//          реализованных шрифтов объявлены в файле Font.h
// Str - Адрес строки в ОЗУ.
// Settings - Режим отображения строки. Может содержать комбинацию
//        следующих флагов:
//        LCD_TWICE_WIDTH (Удвоение ширины символов строки)
//        LCD_TWICE_HEIGHT (Удвоение высоты символов строки)
//        LCD_INVERSION (Инверсное изображение символов строки)
// ПРИМЕЧАНИЯ:
// -Начальной координатой является верхняя левая точка первого символа строки.
//
// ПРИМЕР:
// PT_SPAWN(Context, &ContextChild,
//   LCD_Str(&ContextChild, 0, 0, FONT_6x8, Msg, 0));
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Str(struct pt *Context, uint8_t X, uint8_t Y, uint8_t idFont,
	char *Str, uint8_t Settings))
{
	static struct pt ContextChild; // Контекст для дочернего протопотока

	PT_BEGIN(Context); // Начало протопотока
	PT_WAIT_UNTIL(Context, LCD_StrBuf(X, Y, idFont, Str, Settings));
	PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
	PT_END(Context); // Завершение протопотока
}
//////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Формирует изображение строки в буфере, расположенном в ОЗУ по координатам,
// указанным параметрами X и Y.
// Параметры:
// X - Начальная координата по горизонтали (0...83).
// Y - Начальная координата по вертикали (0...47).
// idFont - Идентификатор шрифта (например: FONT_6x8). Идентификаторы
//          реализованных шрифтов объявлены в файле Font.h.
// Str_P - Адрес строки во FLASH.
// Settings - Режим отображения строки. Может содержать комбинацию
//        следующих флагов:
//        LCD_TWICE_WIDTH (Удвоение ширины символов строки)
//        LCD_TWICE_HEIGHT (Удвоение высоты символов строки)
//        LCD_INVERSION (Инверсное изображение символов строки)
// ПРИМЕЧАНИЯ:
// -Начальной координатой является верхняя левая точка первого символа строки.
//
// ПРИМЕР:
// PT_WAIT_UNTIL(Context, LCD_StrBuf_P(0, 12, FONT_6x8, PSTR("MSG"), 0));
///////////////////////////////////////////////////////////////////////////////
uint8_t	LCD_StrBuf_P(uint8_t X, uint8_t Y, uint8_t idFont, const char *Str_P,
	uint8_t Settings)
{
	return StrBuf(X, Y, idFont, Str_P, Settings, STR_FLASH);
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Выводит изображение строки в индикатор по координатам, указанным
// параметрами X и Y.
// Параметры:
// X - Начальная координата по горизонтали (0...83).
// Y - Начальная координата по вертикали (0...47).
// idFont - Идентификатор шрифта (например: FONT_6x8). Идентификаторы
//          реализованных шрифтов объявлены в файле Font.h
// Str_P - Адрес строки во FLASH.
// Settings - Режим отображения строки. Может содержать комбинацию
//        следующих флагов:
//        LCD_TWICE_WIDTH (Удвоение ширины символов строки)
//        LCD_TWICE_HEIGHT (Удвоение высоты символов строки)
//        LCD_INVERSION (Инверсное изображение символов строки)
// ПРИМЕЧАНИЯ:
// -Начальной координатой является верхняя левая точка первого символа строки.
//
// ПРИМЕР1:
// PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 3, 12, FONT_6x8,
//                                                         PSTR("СТРОКА"), 0));
// ПРИМЕР2:
// // Вывод строки c удвоенной шириной и высотой.
// const char Str_P[] PROGMEM = "СТРОКА"; // Объявление строки
// PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 0, FONT_8x16,
//   Str_P, LCD_TWICE_WIDTH | LCD_TWICE_HEIGHT));
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Str_P(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, const char *Str_P, uint8_t Settings))
{
	static struct pt ContextChild; // Контекст для дочернего протопотока

	PT_BEGIN(Context); // Начало протопотока
	PT_WAIT_UNTIL(Context, LCD_StrBuf_P(X, Y, idFont, Str_P, Settings));
	PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
	PT_END(Context); // Завершение протопотока
}
//////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Формирует изображение символа в буфере, расположенном в ОЗУ по координатам,
// указанным параметрами X и Y.
// Параметры:
// X - Начальная координата по горизонтали (0...83).
// Y - Начальная координата по вертикали (0...47).
// idFont - Идентификатор шрифта (например: FONT_6x8). Идентификаторы
//          реализованных шрифтов объявлены в файле Font.h.
// Chr - Код символа.
// Settings - Режим отображения символа. Может содержать комбинацию
//        следующих флагов:
//        LCD_TWICE_WIDTH (Удвоение ширины символа)
//        LCD_TWICE_HEIGHT (Удвоение высоты символа)
//        LCD_INVERSION (Инверсное изображение символа)
// ПРИМЕЧАНИЯ:
// -Начальной координатой является верхняя левая точка первого символа строки.
//
// ПРИМЕР:
// PT_WAIT_UNTIL(Context, LCD_ChrBuf(42, 12, FONT_6x8, 'A', 0));
///////////////////////////////////////////////////////////////////////////////
uint8_t LCD_ChrBuf(uint8_t X, uint8_t Y, uint8_t idFont, char Chr,
	uint8_t Settings)
{
	char str[2];

	str[0] = Chr;
	str[1] = '\0';

	return StrBuf(X, Y, idFont, str, Settings, STR_RAM);
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Выводит изображение символа в индикатор по координатам, указанным
// параметрами X и Y.
// Параметры:
// X - Начальная координата по горизонтали (0...83).
// Y - Начальная координата по вертикали (0...47).
// idFont - Идентификатор шрифта (например: FONT_6x8). Идентификаторы
//          реализованных шрифтов объявлены в файле Font.h
// Chr - Код символа.
// Settings - Режим отображения символа. Может содержать комбинацию
//        следующих флагов:
//        LCD_TWICE_WIDTH (Удвоение ширины символа)
//        LCD_TWICE_HEIGHT (Удвоение высоты символа)
//        LCD_INVERSION (Инверсное изображение символа)
// ПРИМЕЧАНИЯ:
// -Начальной координатой является верхняя левая точка первого символа строки.
//
// ПРИМЕР:
// PT_SPAWN(Context, &ContextChild,
//   LCD_Chr(&ContextChild, 0, 0, FONT_6x8, 'A', 0));
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Chr(struct pt *Context, uint8_t X, uint8_t Y, uint8_t idFont,
	char Chr, uint8_t Settings))
{
	static struct pt ContextChild;

	PT_BEGIN(Context); // Начало протопотока
	PT_WAIT_UNTIL(Context, LCD_ChrBuf(X, Y, idFont, Chr, Settings));
	PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
	PT_END(Context); // Завершение протопотока
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Установка контраста.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Contrast(struct pt *Context, uint8_t Contrast))
{
	PT_BEGIN(Context); // Начало протопотока
	
	// Расширенный набор команд
	PT_WAIT_UNTIL(Context, SendCmd(CMD_FUNCTION_SET | (0<<NUM_PD) |
		(0<<NUM_V) | (1<<NUM_H)));

	// Установка уровня контрастности
	PT_WAIT_UNTIL(Context, SendCmd(CMD_SET_VOP | Contrast));

	// Стандартный набор команд, горизонтальная адресация
	PT_WAIT_UNTIL(Context, SendCmd(CMD_FUNCTION_SET | (0<<NUM_PD) |
		(0<<NUM_V) | (0<<NUM_H)));
	
	PT_END(Context); // Завершение протопотока
}
///////////////////////////////////////////////////////////////////////////////




// ГРАФИКА
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Установка режима отрисовки графики.
// Параметр:
// Mode - Может принимать одно из следующих значений:
//        LCD_OR,
//        LCD_AND,
//        LCD_XOR.
///////////////////////////////////////////////////////////////////////////////
void LCD_DrawMode(uint8_t Mode)
{
	DrawMode = Mode;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Выводит один пиксел на индикатор.
// Параметры:
// X - Координата по горизонтали (0...83).
// Y - Координата по вертикали (0...48).
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Pixel(struct pt *Context, uint8_t X, uint8_t Y))
{
	uint8_t mask;
	uint8_t data;
	static uint8_t row;

	PT_BEGIN(Context); // Начало протопотока

	if ((X >= LCD_X_RES) || (Y >= LCD_Y_RES))
		PT_EXIT(Context);

	// Ждем момента, когда можно будет записать пиксел в память
	while (LCD_Busy())
	{
		// Если изменяемая область не пересекается с передаваемой, то выходим
		// из цикла.
		// (Т.е. если отображаемая точка не лежит в области буфера, которая в
		// данный момент передается в индикатор)
		if (!(X < xlTransfer) || (xhTransfer <= X) ||
			(Y < ylTransfer) || (yhTransfer <= Y))
			PT_YIELD(Context);
		else
			break;
	}

	#if defined(LCD_ROTATE)
	mask = 0x80 >> (Y % 8);
	row = (LCD_Y_RES - 1 - Y) / 8;
	data = Buf[row][LCD_X_RES - 1 - X];
	#else
	mask = 1 << (Y % 8);
	row = Y / 8;
	data = Buf[row][X];
	#endif
	
	switch (DrawMode)
	{
	case LCD_OR: data |= mask; break;
	case LCD_AND: data &= ~mask; break;
	case LCD_XOR: data ^= mask; break;
	}
	
	#if defined(LCD_ROTATE)
	Buf[row][LCD_X_RES - 1 - X] = data;
	#else
	Buf[row][X] = data;
	#endif

	// Теперь нужно передать измененный байт в индикатор
	
	MT_MutexWait(Context, MUTEX_LCD); // Захватываем мьютекс
	
	#if defined(LCD_ROTATE)
	// Устанавливаем начальный адрес по X
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X | (LCD_X_RES - 1 - X)));
	
	// Устанавливаем начальный адрес по Y
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y | row));

	// Передаем данные в индикатор
	PT_WAIT_UNTIL(Context, SendData(Buf[row][LCD_X_RES - 1 - X]));
	#else
	// Устанавливаем начальный адрес по X
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X | X));

	// Устанавливаем начальный адрес по Y
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y | row));

	// Передаем данные в индикатор
	PT_WAIT_UNTIL(Context, SendData(Buf[row][X]));
	#endif

	MT_MutexFree(MUTEX_LCD);

	PT_END(Context); // Завершение протопотока
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Рисует линию между двумя точками на индикаторе (алгоритм Брезенхэма).
// Параметры:
// X1 - Координата первой точки по горизонтали (0...83),
// Y1 - Координата первой точки по вертикали (0...47),
// X2 - Координата последней точки по горизонтали (0...83),
// Y2 - Координата последней точки по вертикали (0...47).
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Line(struct pt *Context, uint8_t X1, uint8_t Y1, uint8_t X2,
	uint8_t Y2))
{
	static struct pt ContextChild; // Контекст для дочернего протопотока
    static int16_t dX, dY;
	static int16_t stepX, stepY;
	static int16_t fraction;
	static uint8_t x, y;

	PT_BEGIN(Context); // Начало протопотока

	x = X1;
	y = Y1;

	// dY   Y2 - Y1
	// -- = -------
	// dX   X2 - X1

	dY = Y2 - Y1;
	dX = X2 - X1;

	if (dY < 0)
	{
		// dY отрицательное
		dY = -dY;
		stepY = -1;
	}
	else
	{
		// dY положительное
		stepY = 1;
	}

	if (dX < 0)
	{
		// dX отрицательное
		dX = -dX;
		stepX = -1;
	}
	else
	{
		// dX положительное
		stepX = 1;
	}

	dX <<= 1;
	dY <<= 1;

	PT_SPAWN(Context, &ContextChild, LCD_Pixel(&ContextChild, X1, Y1));

	// Рисуем следующие точки до конца
	if (dX > dY)
	{
		fraction = dY - (dX >> 1);
		while (x != X2)
		{
			if (fraction >= 0)
			{
				y += stepY; 
				fraction -= dX;
			}
			x += stepX; 
			fraction += dY;

			PT_SPAWN(Context, &ContextChild, LCD_Pixel(&ContextChild, x, y)); 
		}
	}
	else
	{
		fraction = dX - (dY >> 1);
		while (y != Y2) 
		{
			if (fraction >= 0)
			{
				x += stepX; 
				fraction -= dY;
			}
			y += stepY; 
			fraction += dX;

			PT_SPAWN(Context, &ContextChild, LCD_Pixel(&ContextChild, x, y)); 
		}
	}

	PT_END(Context); // Завершение протопотока
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Рисует окружность с центром, координаты которого указаны параметрами X и Y с
// радиусом R.
// Параметры:
// X - Координата центра окружности по горизонтали (0...83),
// Y - Координата центра окружности по вертикали (0...47),
// R - Радиус окружности (0...83).
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Circle(struct pt *Context, uint8_t X, uint8_t Y, uint8_t R))
{
	static struct pt ContextChild; // Контекст для дочернего протопотока
	static int16_t d;
	static int8_t xc, yc;

	PT_BEGIN(Context); // Начало протопотока

	yc = R;
	d = 3 - ((int16_t)R << 1);
	xc = 0;
	
	while (xc <= yc)
	{
		PT_SPAWN(Context, &ContextChild,
			LCD_Pixel(&ContextChild, xc + X, yc + Y));

		if (yc != 0)
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, xc + X, -yc + Y));

		if ((xc != 0) && (yc != 0))
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, -xc + X, -yc + Y));

		if (xc != 0)
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, -xc + X, yc + Y));
		
		if (xc != yc)
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, yc + X, xc + Y));

		if ((xc != 0) && (xc != yc))
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, yc + X, -xc + Y));

		if ((xc != 0) && (yc != 0) && (xc != yc))
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, -yc + X, -xc + Y));

		if ((yc != 0) && (xc != yc))
			PT_SPAWN(Context, &ContextChild,
				LCD_Pixel(&ContextChild, -yc + X, xc + Y));
		
		if (d < 0)
		{
			d = d + 4 * xc + 6;
		}
		else
		{
			d = d + 4 * (xc - yc) + 10;
			yc--;
		}
		xc++;
	};
	
	PT_END(Context);
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Рисование прямоугольника.
// Параметры:
// X1 - Координата верхнего левого угла прямоугольника по горизонтали (0...83),
// Y1 - Координата верхнего левого угла прямоугольника по вертикали (0...47),
// X2 - Координата нижнего правого угла прямоугольника по горизонтали (0...83),
// Y1 - Координата нижнего правого угла прямоугольника по вертикали (0...47),
// Fill - Флаг устанавливается, если нужно, чтобы внутренняя область
//        прямоугольника была закрашена.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Rect(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2, uint8_t Fill))
{
	static struct pt ContextChild; // Контекст для дочернего протопотока
	static uint8_t sX1, sY1, sX2, sY2;
	static uint8_t xl, xh, yl, yh;
	uint8_t sizeY;
	uint8_t cntByteChangeCol;
	union Union64 mask;
	uint8_t x, num;
	uint8_t data;
	#if defined(LCD_ROTATE)
	uint8_t y;
	#endif

	PT_BEGIN(Context); // Начало протопотока 

	// Разбираемся со входными параметрами

	// Если левая координата больше правой, то меняем их значения между собой
	if (X1 > X2) SWAP(X1, X2)

	// Если верхняя координата больше нижней, то меняем их значения между собой
	if (Y1 > Y2) SWAP(Y1, Y2)

	if (Fill)
	{
		if ((X1 >= LCD_X_RES) || (Y1 >= LCD_Y_RES))
			PT_EXIT(Context);

		if (X2 >= LCD_X_RES) X2 = LCD_X_RES - 1;
		if (Y2 >= LCD_Y_RES) Y2 = LCD_Y_RES - 1;
		X2++;
		Y2++;

		// Вычисляем новые координаты для изменяемой области буфера
		xl = Min(X1, xlChanged);
		xh = Max(X2, xhChanged);
		yl = Min(Y1, ylChanged);
		yh = Max(Y2, yhChanged);

		sX1 = X1; sY1 = Y1; sX2 = X2; sY2 = Y2;

		// Если в данный момент идет передача в индикатор, то проверяем
		// пересечение с передаваемой областью, и, если это так, то ждем, пока
		// не завершится передача
		while (LCD_Busy())
		{
			// Если изменяемая область не пересекается с передаваемой, то
			// выходим из цикла
			if (!(xh <= xlTransfer) || (xhTransfer <= xl) ||
				(yh <= ylTransfer) || (yhTransfer <= yl))
				PT_YIELD(Context);
		}
		
		// Сохраняем новые координаты изменяемой области
		xlChanged = xl;
		xhChanged = xh;
		ylChanged = yl;
		yhChanged = yh;
		
		sizeY = sY2 - sY1;

		// Вычисляем маску, которую будем использовать для отображения
		// прямоугольника.
		// В этой маске количество единиц равно высоте прямоугольника.
		mask.value = 1;
		mask.value <<= sizeY;
		mask.value--; // Получаем маску в виде всех единиц для значимых
						// точек столбцов прямоугольника

		#if defined(LCD_ROTATE)
		// Получаем маску, сдвинутую к старшим разрядам
		mask.value <<= sizeof(mask) * 8 - sizeY - sY1 % 8;
		#else
		mask.value <<= sY1 % 8;
		#endif

		// Подсчитываем количество байт для одного столбца прямоугольника,
		// которые будут подвергнуты изменению.
		cntByteChangeCol = 0;

		#if defined(LCD_ROTATE)
		for (num = sizeof(mask); num > 0; num--)
		{
			if (mask.byte[num - 1] != 0) cntByteChangeCol++;
			else break;
		}
		#else
		for (num = 0; num < sizeof(mask); num++)
		{
			if (mask.byte[num] != 0) cntByteChangeCol++;
			else break;
		}
		#endif

		if (DrawMode == LCD_AND) mask.value = ~mask.value;

		// Цикл по каждому столбцу прямоугольника
		for (x = sX1; x < sX2; x++)
		{
			for (num = 0; num < cntByteChangeCol; num++)
			{
				#if defined(LCD_ROTATE)
				y = (LCD_Y_RES - 1 - (sY1 + num * 8)) / 8;
				data = Buf[y][LCD_X_RES - 1 - x];
				if (DrawMode == LCD_OR) data |=
					mask.byte[sizeof(mask) - 1 - num];
				else if (DrawMode == LCD_XOR) data ^=
					mask.byte[sizeof(mask) - 1 - num];
				else data &= mask.byte[sizeof(mask) - 1 - num];
				Buf[y][LCD_X_RES - 1 - x] = data;
				#else
				data = Buf[(sY1 + num * 8) / 8][x];
				if (DrawMode == LCD_OR) data |= mask.byte[num];
				else if (DrawMode == LCD_XOR) data ^= mask.byte[num];
				else data &= mask.byte[num];
				Buf[(sY1 + num * 8) / 8][x] = data;
				#endif
			}
		}
		
		// Вывод измененной области буфера в индикатор
		PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));

	}
	else
	{
		sX1 = X1; sY1 = Y1; sX2 = X2; sY2 = Y2;

		PT_SPAWN(Context, &ContextChild,
			LCD_Line(&ContextChild, sX1, sY1, sX2, sY1));

		if (sY2 > sY1)
			PT_SPAWN(Context, &ContextChild,
				LCD_Line(&ContextChild, sX1, sY2, sX2, sY2));

		if ((sY2 - sY1) > 1)
			PT_SPAWN(Context, &ContextChild,
				LCD_Line(&ContextChild, sX1, sY1 + 1, sX1, sY2 - 1));

		if ((sX2 > sX1) && ((sY2 - sY1) > 1))
			PT_SPAWN(Context, &ContextChild,
				LCD_Line(&ContextChild, sX2, sY1 + 1, sX2, sY2 - 1));
	}
	
	PT_END(Context); // Завершение протопотока
}
//////////////////////////////////////////////////////////////////////////
