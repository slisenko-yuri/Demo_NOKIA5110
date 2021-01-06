/*
 * Demo_NOKIA5110.c
 */ 

///////////////////////////////////////////////////////////////////////////////
// Демонстрационный пример для графического индикатора NOKIA-5110 с разрешением
// 84x48 на базе контроллера PCD8544. Управление индикатором осуществляется с
// помощью интерфейса SPI.
// Подключение индикатора к ATmega328P представлено в файле
// NOKIA-5110-ATMEGA328P.pdf
// Прошивка производится с помощью программатора AVRISP mkII.
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// FUSES: EXT=0xFD, HIGH=0xD7, LOW=0xE2 // Для работы от внутр. генератора
//----------------------------------------------------------------------------
// BODLEVEL: 2V7
// RSTDISBL: -
// DWEN: -
// SPIEN: +
// WDTON: -
// EESAVE: +
// BOOTSZ: 256W_3F00
// BOOTRST: -
// CKDIV8: -
// CKOUT: -
// SUT_CKSEL: INTRCOSC_8MHZ_6CK_14CK_65MS
///////////////////////////////////////////////////////////////////////////////




#include "Config.h"
#include "Mt.h"
#include "SPI.h"
#include "Font.h"
#include "NOKIA5110.h"



// Размер бегущей информационной строки
#define STR_INFO_SIZE	100

// Бегущая информационная строка, которая будет выводиться в верхней части
// индикатора
char StrInfo[STR_INFO_SIZE];

static uint8_t fLcdReady = FALSE;




///////////////////////////////////////////////////////////////////////////////
// Возвращает следующий символ после Chr
///////////////////////////////////////////////////////////////////////////////
uint8_t IncChar(uint8_t Chr)
{
	Chr++;
	if (Chr == 0x80) Chr = 0xC0;
	if (Chr == 0) Chr = 0x20;
	return Chr;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Задача Task_Info.
// Выводит бегущую строку в верхней части индикатора шрифтом 6x8.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(Task_Info(struct pt *Context))
{
	static struct pt ContextChild; // Контекст для дочернего протопотока
	static char Buf[20];
	static uint8_t size;
	static uint8_t pos1, pos2;
	static uint8_t i;

	PT_BEGIN(Context); // Начало протопотока

	SPI_Init(); // Инициализация SPI

	// Инициализация индикатора
	PT_SPAWN(Context, &ContextChild, LCD_Init(&ContextChild));

	// Очищаем индикатор
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild,
		0, 0, LCD_X_RES - 1, LCD_Y_RES - 1));

	// Инициализируем информационную строку
	strcpy_P(StrInfo, PSTR("   Шрифт 6x8: 0123456789АБВГДЕЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯабвгдежзийклмнопртуфхцчшщъыьэюя"));

	// Извещаем другие потоки о том, что индикатор готов к работе
	fLcdReady = TRUE;

	// Вычисляем количество символов информационной строки, выводимых на
	// индикатор
	size = LCD_X_RES / FONT_Width(FONT_6x8) + 1;

	Buf[size] = '\0';
	pos1 = 0; // Начинаем с самого первого отображаемого символа

	while (TRUE)
	{
		pos2 = pos1++;
		if (pos1 >= strlen(StrInfo)) pos1 = 0;

		// Копируем часть информационной строки в локальный буфер
		for (i = 0; i < size; i++)
		{
			Buf[i] = StrInfo[pos2++];
			if (pos2 >= strlen(StrInfo)) pos2 = 0;
		}

		// Выводим информационную строку

		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild, 0, 0, FONT_6x8,
			Buf, 0));

		MT_SleepMs(Context, 900); // Спим 900мс
	}

	PT_END(Context); // Завершение протопотока
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Задача Task_Fonts.
// Демонстрация шрифтов и графических функций.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(Task_Fonts(struct pt *Context))
{
	static struct pt ContextChild; // Контекст для дочернего протопотока
	static char Buf[20];
	static uint8_t chr1, chr2;
	static uint8_t x1, y1, x2, y2;
	static uint8_t size;
	static uint8_t i;
	static uint8_t cnt;

	PT_BEGIN(Context); // Начало протопотока

	// Ждем, пока не завершится инициализация индикатора
	PT_WAIT_UNTIL(Context, fLcdReady);

	BeginDemo:

	// Демонстрация шрифтов
	///////////////////////////////////////////////////////////////////////////

	// Копируем в информационную строку набор символов для демонстрации
	// шрифта 6x8
	strcpy_P(StrInfo, PSTR("   Шрифт 6x8: 0123456789АБВГДЕЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯабвгдежзийклмнопртуфхцчшщъыьэюя"));

	// Демонстрация шрифтов 8x8, 8x8t и 8x8n
	///////////////////////////////////////////////////////////////////////////

	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	size = LCD_X_RES / FONT_Width(FONT_8x8) - 5 + 1;
	Buf[size] = '\0';

	// Вывод названий демонстрируемых шрифтов
	PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 12, FONT_6x8,
		PSTR("8x8: "), 0));
	PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 24, FONT_6x8,
		PSTR("8x8t:"), 0));
	PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 36, FONT_6x8,
		PSTR("8x8n:"), 0));

	chr1 = 0x20; // Начинаем с самого первого отображаемого символа

	// Установка таймаута 80сек
	MT_TimeoutMs(TIMEOUT_DEMO, 80000U);

	while (TRUE)
	{
		// Формируем строку для вывода
		chr2 = chr1;
		for (i = 0; i < size; i++)
		{
			Buf[i] = chr2;
			chr2 = IncChar(chr2);
		}
		chr1 = IncChar(chr1);
		
		// Выводим символы шрифта 8x8
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild, 30, 12,
			FONT_8x8, Buf, 0));

		// Выводим символы шрифта 8x8t
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild, 30, 24,
			FONT_8x8t, Buf, 0));

		// Выводим символы шрифта 8x8n
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild, 30, 36,
			FONT_8x8n, Buf, 0));

		MT_SleepMs(Context, 500); // Спим 500мс

		// Если таймаут 80сек закончился, то выход из цикла
		if (MT_TimeoutGet(TIMEOUT_DEMO) == 0) break;
	}


	// Демонстрация шрифтов 8x16, 8x16n
	///////////////////////////////////////////////////////////////////////////

	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	size = LCD_X_RES / FONT_Width(FONT_8x16) - 5 + 1;
	Buf[size] = '\0';

	// Вывод названий демонстрируемых шрифтов
	PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 12 + 4,
		FONT_6x8, PSTR("8x16: "), 0));

	PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 28 + 4,
		FONT_6x8, PSTR("8x16n:"), 0));

	// Установка таймаута 80сек
	MT_TimeoutMs(TIMEOUT_DEMO, 80000U);

	chr1 = 0x20; // Начинаем с самого первого символа шрифта

	while (TRUE)
	{
		// Формируем строку для вывода
		chr2 = chr1;
		for (i = 0; i < size; i++)
		{
			Buf[i] = chr2;
			chr2 = IncChar(chr2);
		}
		chr1 = IncChar(chr1);
		
		// Выводим символы шрифта 8x16
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild,
			36, 12, FONT_8x16, Buf, 0));

		// Выводим символы шрифта 8x16n
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild,
			36, 28, FONT_8x16n, Buf, 0));

		MT_SleepMs(Context, 500); // Спим 500мс

		if (MT_TimeoutGet(TIMEOUT_DEMO) == 0) break;
	}


	// Демонстрация шрифтов 10x14, 12x16
	///////////////////////////////////////////////////////////////////////////

	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	size = LCD_X_RES / FONT_Width(FONT_10x14) + 1;
	Buf[size] = '\0';

	// Выводим названия демонстрируемых шрифтов
	PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 12 + 4,
		FONT_6x8, PSTR("10x14:"), 0));
	PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 28 + 4,
		FONT_6x8, PSTR("12x16:"), 0));

	// Установка таймаута 80сек
	MT_TimeoutMs(TIMEOUT_DEMO, 80000U);

	chr1 = 0x20; // Начинаем с самого первого отображаемого символа

	while (TRUE)
	{
		// Формируем строку для вывода
		chr2 = chr1;
		for (i = 0; i < size; i++)
		{
			Buf[i] = chr2;
			chr2 = IncChar(chr2);
		}
		chr1 = IncChar(chr1);
		
		// Выводим символы шрифта 10x14
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild, 36, 12,
			FONT_10x14, Buf, 0));

		// Выводим символы шрифта 12x16
		PT_SPAWN(Context, &ContextChild, LCD_Str(&ContextChild, 36, 28,
			FONT_12x16, Buf, 0));

		MT_SleepMs(Context, 500); // Спим 500мс

		if (MT_TimeoutGet(TIMEOUT_DEMO) == 0) break;
	}


	// Демонстрация вывода точек
	///////////////////////////////////////////////////////////////////////////

	// Формируем содержимое информационной строки
	strcpy_P(StrInfo, PSTR("    Вывод точек"));
	
	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild,
		0, 8, LCD_X_RES - 1, LCD_Y_RES - 1));
	
	MT_SleepMs(Context, 500); // Спим 500мс

	for (cnt = 0; cnt < 3; cnt++)
	{
		// Устанавливаем режим вывода графики
		switch (cnt)
		{
		case 0: LCD_DrawMode(LCD_OR); break;
		case 1: LCD_DrawMode(LCD_AND); break;
		case 2: LCD_DrawMode(LCD_XOR); break;
		}
		
		// Установка таймаута 5000мс
		MT_TimeoutMs(TIMEOUT_DEMO, 5000);

		i = 0;
		
		// Крутимся в цикле, пока не завершится таймаут
		while (MT_TimeoutGet(TIMEOUT_DEMO) != 0)
		{
			// Вычисляем случайную координату x1 в диапазоне 0...83
			x1 = random() % LCD_X_RES;
			
			// Вычисляем случайную координату y1 в диапазоне 9...47
			y1 = (random() % (LCD_Y_RES - 9)) + 9;

			// Рисуем точку по полученным координатам
			PT_SPAWN(Context, &ContextChild, LCD_Pixel(&ContextChild,
				x1, y1));
			
			i++;
			
			// После каждых 8-ми проходов цикла отдаем управление в планировщик
			if ((i % 8) == 0) PT_YIELD(Context);
		}		
	}


	// Демонстрация вывода линий
	///////////////////////////////////////////////////////////////////////////

	// Формируем содержимое информационной строки
	strcpy_P(StrInfo, PSTR("    Вывод линий"));
	
	for (cnt = 0; cnt < 4; cnt++)
	{
		switch (cnt)
		{
		case 0:
			// Чистим нижнюю область индикатора
			// (все точки с координатами Y >= 8)
			PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
				LCD_X_RES - 1, LCD_Y_RES - 1));
			LCD_DrawMode(LCD_OR); // Установка режима вывода графики
			break;
		case 1:
			// Закрашиваем нижнюю область индикатора
			// (все точки с координатами Y >= 9)
			PT_SPAWN(Context, &ContextChild, LCD_Rect(&ContextChild, 0, 9,
				LCD_X_RES - 1, LCD_Y_RES - 1, TRUE));
			LCD_DrawMode(LCD_AND); // Установка режима вывода графики
			break;
		default:
			// Чистим нижнюю область индикатора
			// (все точки с координатами Y >= 8)
			PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
				LCD_X_RES - 1, LCD_Y_RES - 1));
			LCD_DrawMode(LCD_XOR); // Установка режима вывода графики
			break;
		}

		if (cnt < 3)
		{
			// Вывод линий по случайным координатам

			// Вычисляем случайную координату x2 в диапазоне 0...83
			x2 = random() % LCD_X_RES;
			
			// Вычисляем случайную координату y2 в диапазоне 9...47
			y2 = (random() % (LCD_Y_RES - 9)) + 9;

			// Установка таймаута 4000мс
			MT_TimeoutMs(TIMEOUT_DEMO, 4000);

			// Крутимся в цикле, пока не завершится таймаут
			while (MT_TimeoutGet(TIMEOUT_DEMO) != 0)
			{
				x1 = x2;
				y1 = y2;
				
				// Вычисляем случайную координату x2 в диапазоне 0...83
				x2 = random() % LCD_X_RES;
				
				// Вычисляем случайную координату y2 в диапазоне 9...47
				y2 = (random() % (LCD_Y_RES - 9)) + 9;

				// Рисуем линию по полученным координатам
				PT_SPAWN(Context, &ContextChild, LCD_Line(&ContextChild,
					x1, y1, x2, y2));
				
				MT_SleepMs(Context, 100); // Спим 100мс

				if (cnt >= 2)
					// Стираем линию (в режиме LCD_XOR)
					PT_SPAWN(Context, &ContextChild, LCD_Line(&ContextChild,
						x1, y1, x2, y2));
			}
		}
	}
	

	// Демонстрация закрашенных прямоугольников
	///////////////////////////////////////////////////////////////////////////

	// Формируем содержимое информационной строки
	strcpy_P(StrInfo, PSTR("    Закрашенные прямоугольники"));

	LCD_DrawMode(LCD_XOR); // Установка режима вывода графики

	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	// Установка таймаута 5000мс
	MT_TimeoutMs(TIMEOUT_DEMO, 5000);

	// Крутимся в цикле, пока не завершится таймаут
	while (MT_TimeoutGet(TIMEOUT_DEMO) != 0)
	{
		// Вычисляем случайную координату x1 в диапазоне 0...83
		x1 = random() % LCD_X_RES;
			
		// Вычисляем случайную координату y1 в диапазоне 9...47
		y1 = (random() % (LCD_Y_RES - 9)) + 9;
			
		// Вычисляем случайную координату x2 в диапазоне 0...83
		x2 = random() % LCD_X_RES;
			
		// Вычисляем случайную координату y2 в диапазоне 9...47
		y2 = (random() % (LCD_Y_RES - 9)) + 9;

		// Рисуем прямоугольник
		PT_SPAWN(Context, &ContextChild, LCD_Rect(&ContextChild,
			x1, y1, x2, y2, TRUE));

		MT_SleepMs(Context, 200); // Спим 200мс
	}

	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));
	
	// Установка режима вывода графики
	LCD_DrawMode(LCD_OR);

	for (i = 0; i < 18; i++)
	{
		// Рисуем прямоугольник
		PT_SPAWN(Context, &ContextChild, LCD_Rect(&ContextChild,
			40 - i * 2, 29 - i, 40 + i * 2, 29 + i, TRUE));

		MT_SleepMs(Context, 200); // Спим 200мс
	}
	MT_SleepMs(Context, 1000); // Спим 1000мс


	// Демонстрация окружностей
	///////////////////////////////////////////////////////////////////////////

	// Формируем содержимое информационной строки
	strcpy_P(StrInfo, PSTR("    Окружности"));

	// Чистим нижнюю область индикатора (все точки с координатами Y >= 8)
	PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
		LCD_X_RES - 1, LCD_Y_RES - 1));

	LCD_DrawMode(LCD_OR); // Установка режима вывода графики

	for (i = 0; i < 5; i++)
	{
		// Рисуем окружность
		PT_SPAWN(Context, &ContextChild, LCD_Circle(&ContextChild,
			2 + i * 15, 29, 1 + i * 4));

		MT_SleepMs(Context, 500); // Спим 200мс
	}

	MT_SleepMs(Context, 1000); // Спим 1000мс


	for (cnt = 0; cnt < 2; cnt++)
	{
		switch (cnt)
		{
		case 0: 
			// Чистим нижнюю область индикатора
			// (все точки с координатами Y >= 8)
			PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
				LCD_X_RES - 1, LCD_Y_RES - 1));
			LCD_DrawMode(LCD_OR); // Установка режима вывода графики
			break;
		default:
			// Чистим нижнюю область индикатора
			// (все точки с координатами Y >= 8)
			PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 8,
				LCD_X_RES - 1, LCD_Y_RES - 1));
			LCD_DrawMode(LCD_XOR); // Установка режима вывода графики
		}
		
		// Установка таймаута 5000мс
		MT_TimeoutMs(TIMEOUT_DEMO, 5000);
		
		// Крутимся в цикле, пока не завершится таймаут
		while (MT_TimeoutGet(TIMEOUT_DEMO) != 0)
		{
			// Вычисляем случайную координату x1 в диапазоне 0...83
			x1 = random() % LCD_X_RES;

			// Вычисляем случайную координату y1 в диапазоне 10...46
			y1 = (random() % (LCD_Y_RES - 11)) + 10;

			// Вычисляем случайный радиус в диапазоне 0...14
			size = random() % 15;

			if (size > (y1 - 9)) size = size % (y1 - 9);
			if (size <= 1) size++;

			// Рисуем окружность
			PT_SPAWN(Context, &ContextChild, LCD_Circle(&ContextChild, x1, y1,
				size));

			MT_SleepMs(Context, 100); // Спим 100мс

			if (cnt >= 0)
				// Стираем окружность (в режиме LCD_XOR)
				PT_SPAWN(Context, &ContextChild, LCD_Circle(&ContextChild,
					x1, y1, size));
		}
	}
	
	goto BeginDemo;

	PT_END(Context); // Завершение протопотока
}
//////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// main()
///////////////////////////////////////////////////////////////////////////////
int main(void)
{
	// Настройка тактовой частоты

	#if (DIVIDER_OSC == 1)
	clock_prescale_set(clock_div_1); // Устанавливаем делитель 1/1
	#elif (DIVIDER_OSC == 2)
	clock_prescale_set(clock_div_2); // Устанавливаем делитель 1/2
	#elif (DIVIDER_OSC == 4)
	clock_prescale_set(clock_div_4); // Устанавливаем делитель 1/4
	#elif (DIVIDER_OSC == 8)
	clock_prescale_set(clock_div_8); // Устанавливаем делитель 1/8
	#elif (DIVIDER_OSC == 16)
	clock_prescale_set(clock_div_16); // Устанавливаем делитель 1/16
	#elif (DIVIDER_OSC == 32)
	clock_prescale_set(clock_div_32); // Устанавливаем делитель 1/32
	#elif (DIVIDER_OSC == 64)
	clock_prescale_set(clock_div_64); // Устанавливаем делитель 1/64
	#elif (DIVIDER_OSC == 128)
	clock_prescale_set(clock_div_128); // Устанавливаем делитель 1/128
	#elif (DIVIDER_OSC == 256)
	clock_prescale_set(clock_div_256); // Устанавливаем делитель 1/256
	#else
	#error Unknown divider for MAIN_OSC
	#endif


	sei();
	
	// Настройка менеджера задач
	MT_Init();

	// Запуск задач
	MT_TaskInit(Task_Info, TASK_ACTIVE);
	MT_TaskInit(Task_Fonts, TASK_ACTIVE);
	
    while (TRUE)
    {
		MT_DISPATCH(); // Один проход менеджера задач
    }
}
