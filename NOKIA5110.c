#include "Config.h"
#include "SPI.h"
#include "Font.h"
#include "NOKIA5110.h"
#include "Mt.h"

///////////////////////////////////////////////////////////////////////////////
// ���������� ��� ��������� ���������� NOKIA-5110 �� ���� ����������� PCD8544
///////////////////////////////////////////////////////////////////////////////

/*
=========================================================
����������� ���������� NOKIA-5110 � ATmega328P
=========================================================
NOKIA 5110		ATmega328P		�������� �������
---------------------------------------------------------
01 (RST)		14  (PB0)		����� ���������� (0)
02 (CE)			16  (PB2)		����� ���������� (0)
03 (DC)			15  (PB1)		�������(0)/������(1)
04 (DIN)		17  (PB3)		���� ������ ����������
05 (CLK)		19  (PB5)		������������
06 (+3.3�)		7,20(VCC)		�������
07 (LIGHT)		GND				���������
08 (GND)		8,22(GND)		�����
---------------------------------------------------------

=====================================
����������� ������������� AVRISP mkII
� ���������������� ATmega328P (DIP28)
=====================================
������������	ATmega328P
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




// ������� ���������� NOKIA 5110
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

// ������ ��� � �������� ���������� NOKIA 5110
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

// ������ ���������� ����� ����� �������� ���������� x � y
#define SWAP(x, y)	{uint8_t temp; temp = x; x = y; y = temp;}




union Union64
{
	uint8_t byte[8];
	uint64_t value;
};




static uint8_t	Buf[LCD_Y_RES / 8][LCD_X_RES]; // ����� ��� ����������

// ���������� ���������� ������� ������, ������� ��� �� �������� � ���������
static uint8_t xlChanged;
static uint8_t xhChanged;
static uint8_t ylChanged;
static uint8_t yhChanged;

// ���������� ������� ������, ������� � ������ ������ ���������� � ���������
static uint8_t xlTransfer;
static uint8_t xhTransfer;
static uint8_t ylTransfer;
static uint8_t yhTransfer;

static uint8_t DrawMode; // ����� ������ �������




// ������, ����������� ��� ������ �������� ��������� ������
static const uint8_t Bit4to8[] PROGMEM =
{
	0x00, 0x03, 0x0C, 0x0F, 0x30, 0x33, 0x3C, 0x3F,
	0xC0, 0xC3, 0xCC, 0xCF, 0xF0, 0xF3, 0xFC, 0xFF
};




///////////////////////////////////////////////////////////////////////////////
// �� ���� �������� ���������� ������������.
///////////////////////////////////////////////////////////////////////////////
static uint16_t Max(uint16_t a, uint16_t b)
{
	if (a > b) return a;
	return b;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// �� ���� �������� ���������� �����������.
///////////////////////////////////////////////////////////////////////////////
static uint16_t Min(uint16_t a, uint16_t b)
{
	if (a < b) return a;
	return b;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������ ������� � ���������.
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
// ������ ����� ������ � ���������.
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
// ������ ������� ������ � ���������.
///////////////////////////////////////////////////////////////////////////////
static uint8_t SendBufPtr(uint8_t* Buf, uint8_t Size)
{
	if (!SPI_WaitReady()) return FALSE;
	
	ON(NOKIA5110_DC); // ����� ������ ������������� ����� �������� ������
						// (�� ������)
	SPI_SendBufPtrToLCD(Buf, Size);

	return TRUE;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ��������� ����������� ������ � ������, ������������� � ���.
// ���������:
// X - ��������� ���������� �� ����������� (0...83)
// Y - ��������� ���������� �� ��������� (0...47)
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h
// Str - ����� ������ �� FLASH, ���� � RAM
// Settings - ����� ����������� ������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������� ������)
//        LCD_TWICE_HEIGHT (�������� ������ �������� ������)
//        LCD_INVERSION (��������� ����������� �������� ������)
// fFLASH - ����, ����������� ��������������� ������, ��������� ����������
//          Str. ���� ���� ����� TRUE, �� ��� ��������, ��� ������
//          ��������� �� FLASH, ����� � RAM.
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
// -������� ������������ ������ ������ ���������� NOKIA5110.
///////////////////////////////////////////////////////////////////////////////
static uint8_t StrBuf(uint8_t X, uint8_t Y, uint8_t idFont, const char *Str,
	uint8_t Settings, uint8_t fFLASH)
{
	uint8_t	col, row, j, num, m, data;
	uint8_t xl, xh, yl, yh;
	uint8_t len;
	uint8_t chr;
	uint16_t tmp;
	uint8_t widthFont; // ������ ������� � ��������
	uint8_t heightFont; // ������ ������� � ��������
	uint8_t cntByteChangeCol; // ���������� ���������� ������ ������ ���
								// ������ ������� �������
	uint8_t cntByteSymCol; // ���������� ���� � ����� ������� �������
	union Union64 mask; // ����� ��� �������� �������
	union Union64 column; // ������� �������
	uint8_t const *addrFont; // ����� ������
	uint8_t twiceW = 0; // 1 = �������� ������ �������
	uint8_t twiceH = 0; // 1 = �������� ������ �������
	uint8_t inv = 0; // 1 = ��������� ����������� �������
	
	widthFont = FONT_Width(idFont); // ������ �������� ������
	heightFont = FONT_Height(idFont); // ������ �������� ������

	if (Settings & LCD_TWICE_WIDTH)
		twiceW = 1; // ��������� ������

	if (Settings & LCD_TWICE_HEIGHT)
		twiceH = 1; // ��������� ������

	if (Settings & LCD_INVERSION)
		inv = 1; // ��������� ����������� ��������

	if (fFLASH)
		tmp = strlen_P(Str); // ��������� ����� ������ �� FLASH
	else
		tmp = strlen(Str); // ��������� ����� ������ � RAM

	if (tmp < 255) len = tmp; else len = 255;

	// ���� � ������ ������ ���� �������� ������ � ���������,
	// �� ����� ���������, �� ���������� �� ������������ �������
	// ����� ���������� �������
	
	// ��������� ����� ���������� ����� ���������� ������� ������ � ������
	// ����������� ��� ������
	xl = Min(X, xlChanged);
	tmp = Min(LCD_X_RES, X + len * (widthFont << twiceW));
	xh = Max(tmp, xhChanged);

	yl = Min(Y, ylChanged);
	tmp = Min(LCD_Y_RES, Y + (heightFont << twiceH));
	yh = Max(tmp, yhChanged);

	// ���� � ������ ������ ���� �������� � ���������, �� ���������
	// ����������� ����� ���������� ������� ������ � ������������ ��������
	if (LCD_Busy())
	{
		// ���� ���������� ������� ������������ � ������������, �� �������
		if (!(xh <= xlTransfer) || (xhTransfer <= xl) ||
			(yh <= ylTransfer) || (yhTransfer <= yl)) return FALSE;
	}

	// ��������� ����� ���������� ����� ���������� ������� ������
	xlChanged = xl;
	xhChanged = xh;
	ylChanged = yl;
	yhChanged = yh;

	// �������� ����� ������ (������� ������, ��������������� ����� �����������
	// ������������� ��������).
	addrFont = FONT_Addr(idFont) + 2;

	// ��������� �����, ������� ����� ������������ ��� �������� �������.
	// � ���� ����� ���������� ������ ����� ������ ���������� ������.
	mask.value = 1;
	mask.value <<= (heightFont << twiceH);
	mask.value--; // �������� ����� � ���� ���� ������ ��� �������� �����
					// ������� �������.

	#if defined(LCD_ROTATE)
	// �������� �����, ��������� � ������� ��������.
	mask.value <<= sizeof(mask) * 8 - (heightFont << twiceH) - Y % 8;
	#else
	mask.value <<= Y % 8; // �������� ����� � ������ ���������� Y.
	#endif

	// ��������� ���������� ����, ��������� ��� ������ ������� �������
	// (��� ����� �������� �� ������ � ������).
	cntByteSymCol = (heightFont - 1) / 8 + 1;

	// �.�. ������ ����� ������������� ������� � ����� ����������, � �� ������
	// � ������� ������� ����� (��������, ���� ������� ������� � ������� 8
	// ����� ����� ������������� � ���� ������), �� ���������� ���������
	// ���������� ���������� ������ � ������ ��� ������ ������� �������.

	// ������������ ���������� ���� ��� ������ ������� �������, ������� �����
	// ����������� ���������.

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

	mask.value = ~mask.value; // ����������� �����.

	// ���� �� ������� ������� ������
	///////////////////////////////////////////////////////////////////////////
	for (j = 0; j < len; j++)
	{
		if (X >= LCD_X_RES) break;
		
		if (fFLASH)
			// ���� ������ ������ ��������� �� FLASH
			chr = pgm_read_byte(&Str[j]);
		else
			// ���� ������ ������ ��������� � ���
			chr = Str[j];

		// ������������ ��� ������� � ������������ � ����������
		///////////////////////////////////////////////////////////////////////

		if ((chr >= 0x20) && (chr <= 0x7F))
		{
			// �������� � ������� ��� �������� ASCII[0x20-0x7F]
			chr -= 0x20; //chr -= 32;
		}
		else if (chr >= 0xC0)
		{
			// �������� � ������� ��� �������� CP1251[0xC0-0xFF]
			chr -= 0x60; //chr -= 96;
		}
		else
		{
			// ��������� ���������� (�� ��� � ������� ��� �������� ������)
			chr = 0; // ������
		}

		// ���� �� ���� �������� �������
		///////////////////////////////////////////////////////////////////////
		for (col = 0; col < widthFont; col++)
		{
			if (X >= LCD_X_RES) break;

			column.value = 0;

			// ������ ����� ��� ������ ���������� ������� �������
			///////////////////////////////////////////////////////////////////
			for (num = 0; num < cntByteSymCol; num++)
			{
				if (twiceH) // ���� ��������� ������ �������
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

			// �������� ������� � ������ ���������� Y.
			#if defined(LCD_ROTATE)
			column.value >>= Y % 8;
			#else
			column.value <<= Y % 8;
			#endif
			
			// ���� ��������� �����������, �� ����������� ������� �������.
			if (inv)
			{
				column.value ^= ~mask.value;
			}
			
			// ����� ������� � �����
			for (m = 0; m <= twiceW; m++)
			{
				if (X < LCD_X_RES)
				{
					// ���� �� ���������� ������ ������ ��� ������ �������
					// �������					
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
// ������������� ����������.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Init(struct pt *Context))
{
	PT_BEGIN(Context); // ������ �����������

	MT_MutexWait(Context, MUTEX_LCD); // ����������� �������

	DRIVER(NOKIA5110_DC, OUT);
	DRIVER(NOKIA5110_RST, OUT);

	ON(NOKIA5110_RST); // RST=0
	_delay_us(50); // �������� >= 100��
	OFF(NOKIA5110_RST); // RST=1

	// ���������� ������� �������
	// 0010 0PVH (P-Power Down, V-Vertical, H-Extended)
	// �������� ����������� ����� ������ (LCD Extended Commands)
	PT_WAIT_UNTIL(Context, SendCmd(CMD_FUNCTION_SET | (0<<NUM_PD) |
		(0<<NUM_V) | (1<<NUM_H)));
	
	// ��������� ������� (LCD bias mode 1:48) (���� �� 8-�� ������� �������
	// �������� ������� ��� ���������� LCD)
	// 0001 0bbb
	PT_WAIT_UNTIL(Context, SendCmd(CMD_BIAS_SYSTEM | BS3)); 

	// ��������� �������������� ������������ (Temp coefficent) (���� �� 4-�
	// ������� ������������� ���������)
	// 0000 01tt
	PT_WAIT_UNTIL(Context, SendCmd(CMD_TEMP_CONTROL | TC0));

	// ��������� ������������� (LCD Vop) (���������� ������� ��-���������)
	// 1vvv vvvv
	PT_WAIT_UNTIL(Context, SendCmd(CMD_SET_VOP | 0x38));

	// �������� ����������� ����� ������ � �������������� ��������� (LCD
	// Standard Commands,Horizontal addressing mode)
	// 0010 0PVH (P-Power Down, V-Vertical, H-Extended)
	PT_WAIT_UNTIL(Context, SendCmd(CMD_FUNCTION_SET | (0<<NUM_PD) | (0<<NUM_V) | (0<<NUM_H)));
	
	// 0000 1D0E (D=1-���������� �����������,D=0-����� ������,E=1-��������)
	PT_WAIT_UNTIL(Context, SendCmd(CMD_DISPLAY_CONTROL | (1<<NUM_D) | (0<<NUM_E)));

	DrawMode = LCD_OR;

	// ������������� ��������� ��������� ������� ������
	xlChanged = LCD_X_RES;
	xhChanged = 0;
	ylChanged = LCD_Y_RES;
	yhChanged = 0;

	// ������������� ��������� ������� ������, ������������ � ���������
	xlTransfer = 0;
	xhTransfer = 0;
	ylTransfer = 0;
	yhTransfer = 0;

	MT_MutexFree(MUTEX_LCD); // ����������� �������
	
	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ���������� TRUE, ���� ��������� �����, �.�. � ������ ������ ����������
// �������� �� ������ � ���������.
///////////////////////////////////////////////////////////////////////////////
uint8_t LCD_Busy(void)
{
	if (yhTransfer > ylTransfer)
		return TRUE;

	return FALSE;
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������� ���������� ���������� ������� ������ � ���������.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Update(struct pt *Context))
{
	PT_BEGIN(Context); // ������ �����������
	
	MT_MutexWait(Context, MUTEX_LCD); // ������ ��������

	// ���� � ������ ���������� ����������, �� �� ����� �������� � ���������
	if ((xhChanged > xlChanged) && (yhChanged > ylChanged))
	{
		// ���������� ���������� ������������ �������
		xlTransfer = xlChanged;
		xhTransfer = xhChanged;
		ylTransfer = ylChanged;
		yhTransfer = yhChanged;
		
		// ����������� ���������� ���������� ������� ������
		xlChanged = LCD_X_RES;
		xhChanged = 0;
		ylChanged = LCD_Y_RES;
		yhChanged = 0;

		while (yhTransfer > ylTransfer)
		{
			#if defined(LCD_ROTATE)
			// ������������� ��������� ����� �� X
			PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X |
				(LCD_X_RES - xhTransfer)));

			// ������������� ��������� ����� �� Y
			PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y |
				((LCD_Y_RES - 1 - ylTransfer) / 8)));

			// �������� ������ � ���������
			PT_WAIT_UNTIL(Context, SendBufPtr(&Buf[(LCD_Y_RES - 1 - ylTransfer)
				/ 8][LCD_X_RES - xhTransfer], xhTransfer - xlTransfer));
			#else

			// ������������� ��������� ����� �� X
			PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X | xlTransfer));

			// ������������� ��������� ����� �� Y
			PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y | (ylTransfer / 8)));

			// �������� ������ � ���������
			PT_WAIT_UNTIL(Context, SendBufPtr(&Buf[ylTransfer / 8][xlTransfer],
				xhTransfer - xlTransfer));
			#endif

			// ������������ ���������� ������������ ������� ������ �
			// ������ ���������� ������ ������
			ylTransfer += (8 - (ylTransfer % 8));
		}
		
		// ���������� ���������� ������������ �������
		xlTransfer = 0;
		xhTransfer = 0;
		ylTransfer = 0;
		yhTransfer = 0;
	}
	
	MT_MutexFree(MUTEX_LCD); // ����������� �������
	
	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������� ������� ������ � ���������� ������������.
// ���������:
// X1 - ���������� �������� ������ ���� ��������� ������� �� �����������
//      (0...83),
// Y1 - ���������� �������� ������ ���� ��������� ������� �� ���������
//      (0...47),
// X2 - ���������� ������� ������� ���� ��������� ������� �� �����������
//      (0...83),
// Y1 - ���������� ������� ������� ���� ��������� ������� �� ���������
//      (0...47).
//
// ������:
// // �������� ���� �����
// PT_WAIT_UNTIL(Context, LCD_ClearBuf(0, 0, 83, 47));
// // ����� ���������� ����� ������ � ���������
// PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
///////////////////////////////////////////////////////////////////////////////
uint8_t LCD_ClearBuf(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2)
{
	uint8_t sizeY;
	union Union64 mask;
	uint8_t cntByteChangeCol;
	uint8_t xl, xh, yl, yh;
	uint8_t x, num;

	// ����������� �� �������� �����������

	// ���� ����� ���������� ������ ������, �� ������ �� �������� ����� �����
	if (X1 > X2) SWAP(X1, X2)

	// ���� ������� ���������� ������ ������, �� ������ �� �������� ����� �����
	if (Y1 > Y2) SWAP(Y1, Y2)

	if ((X1 >= LCD_X_RES) || (Y1 >= LCD_Y_RES)) return TRUE;
	if (X2 >= LCD_X_RES) X2 = LCD_X_RES - 1;
	if (Y2 >= LCD_Y_RES) Y2 = LCD_Y_RES - 1;
	X2++;
	Y2++;

	// ��������� ����� ���������� ��� ���������� ������� ������ 
	xl = Min(X1, xlChanged);
	xh = Max(X2, xhChanged);
	yl = Min(Y1, ylChanged);
	yh = Max(Y2, yhChanged);

	// ���� � ������ ������ ���� �������� � ���������, �� ���������
	// ����������� ������������ ������� � ����������
	if (LCD_Busy())
	{
		// ���� ���������� ������� ������������ � ������������, �� �������
		if (!(xh <= xlTransfer) || (xhTransfer <= xl) || (yh <= ylTransfer) ||
			(yhTransfer <= yl))
			return FALSE;
	}

	// ������������� ����� ���������� ���������� ������� ������
	xlChanged = xl;
	xhChanged = xh;
	ylChanged = yl;
	yhChanged = yh;
	
	// ��������� ������ ��������� ������� �� ���������
	sizeY = Y2 - Y1;

	// ��������� �����, ������� ����� ������������ ��� ������� ��������.
	// � ���� ����� ���������� ������ ����� ������ ��������� �������
	mask.value = 1;
	mask.value <<= sizeY;
	mask.value--; // �������� ����� � ���� ���� ������ ��� �������� �����
					// �������� ��������� �������

	#if defined(LCD_ROTATE)
	// �������� �����, ��������� � ������� ��������
	mask.value <<= sizeof(mask) * 8 - sizeY - Y1 % 8;
	#else
	mask.value <<= Y1 % 8;
	#endif

	// ������������ ���������� ���� ��� ������ ������� ��������� �������,
	// ������� ����� ����������� ���������.

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
	
	mask.value = ~mask.value; // ����������� �����
	
	// ���� �� ������� ������� ��������� �������
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
// ������� ������� ���������� � ���������� ������������.
// ���������:
// X1 - ���������� �������� ������ ���� ��������� ������� �� �����������
//      (0...83),
// Y1 - ���������� �������� ������ ���� ��������� ������� �� ���������
//      (0...47),
// X2 - ���������� ������� ������� ���� ��������� ������� �� �����������
//      (0...83),
// Y1 - ���������� ������� ������� ���� ��������� ������� �� ���������
//      (0...47).
//
// ������:
// PT_SPAWN(Context, &ContextChild, LCD_Clear(&ContextChild, 0, 0, 83, 47));
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Clear(struct pt *Context, uint8_t X1, uint8_t Y1, uint8_t X2,
	uint8_t Y2))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������

	PT_BEGIN(Context); // ������ �����������

	PT_WAIT_UNTIL(Context, LCD_ClearBuf(X1, Y1, X2, Y2));
	PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));	

	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ��������� ����������� ������ � ������, ������������� � ��� �� �����������,
// ��������� ����������� X � Y.
// ���������:
// X - ��������� ���������� �� ����������� (0...83).
// Y - ��������� ���������� �� ��������� (0...47).
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h.
// Str - ����� ������ � ���.
// Settings - ����� ����������� ������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������� ������)
//        LCD_TWICE_HEIGHT (�������� ������ �������� ������)
//        LCD_INVERSION (��������� ����������� �������� ������)
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
//
// ������:
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
// ������� ����������� ������ � ��������� �� �����������, ��������� �����������
// X � Y.
// ���������:
// X - ��������� ���������� �� ����������� (0...83).
// Y - ��������� ���������� �� ��������� (0...47).
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h
// Str - ����� ������ � ���.
// Settings - ����� ����������� ������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������� ������)
//        LCD_TWICE_HEIGHT (�������� ������ �������� ������)
//        LCD_INVERSION (��������� ����������� �������� ������)
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
//
// ������:
// PT_SPAWN(Context, &ContextChild,
//   LCD_Str(&ContextChild, 0, 0, FONT_6x8, Msg, 0));
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Str(struct pt *Context, uint8_t X, uint8_t Y, uint8_t idFont,
	char *Str, uint8_t Settings))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������

	PT_BEGIN(Context); // ������ �����������
	PT_WAIT_UNTIL(Context, LCD_StrBuf(X, Y, idFont, Str, Settings));
	PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
	PT_END(Context); // ���������� �����������
}
//////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ��������� ����������� ������ � ������, ������������� � ��� �� �����������,
// ��������� ����������� X � Y.
// ���������:
// X - ��������� ���������� �� ����������� (0...83).
// Y - ��������� ���������� �� ��������� (0...47).
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h.
// Str_P - ����� ������ �� FLASH.
// Settings - ����� ����������� ������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������� ������)
//        LCD_TWICE_HEIGHT (�������� ������ �������� ������)
//        LCD_INVERSION (��������� ����������� �������� ������)
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
//
// ������:
// PT_WAIT_UNTIL(Context, LCD_StrBuf_P(0, 12, FONT_6x8, PSTR("MSG"), 0));
///////////////////////////////////////////////////////////////////////////////
uint8_t	LCD_StrBuf_P(uint8_t X, uint8_t Y, uint8_t idFont, const char *Str_P,
	uint8_t Settings)
{
	return StrBuf(X, Y, idFont, Str_P, Settings, STR_FLASH);
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������� ����������� ������ � ��������� �� �����������, ���������
// ����������� X � Y.
// ���������:
// X - ��������� ���������� �� ����������� (0...83).
// Y - ��������� ���������� �� ��������� (0...47).
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h
// Str_P - ����� ������ �� FLASH.
// Settings - ����� ����������� ������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������� ������)
//        LCD_TWICE_HEIGHT (�������� ������ �������� ������)
//        LCD_INVERSION (��������� ����������� �������� ������)
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
//
// ������1:
// PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 3, 12, FONT_6x8,
//                                                         PSTR("������"), 0));
// ������2:
// // ����� ������ c ��������� ������� � �������.
// const char Str_P[] PROGMEM = "������"; // ���������� ������
// PT_SPAWN(Context, &ContextChild, LCD_Str_P(&ContextChild, 0, 0, FONT_8x16,
//   Str_P, LCD_TWICE_WIDTH | LCD_TWICE_HEIGHT));
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Str_P(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, const char *Str_P, uint8_t Settings))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������

	PT_BEGIN(Context); // ������ �����������
	PT_WAIT_UNTIL(Context, LCD_StrBuf_P(X, Y, idFont, Str_P, Settings));
	PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
	PT_END(Context); // ���������� �����������
}
//////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ��������� ����������� ������� � ������, ������������� � ��� �� �����������,
// ��������� ����������� X � Y.
// ���������:
// X - ��������� ���������� �� ����������� (0...83).
// Y - ��������� ���������� �� ��������� (0...47).
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h.
// Chr - ��� �������.
// Settings - ����� ����������� �������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������)
//        LCD_TWICE_HEIGHT (�������� ������ �������)
//        LCD_INVERSION (��������� ����������� �������)
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
//
// ������:
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
// ������� ����������� ������� � ��������� �� �����������, ���������
// ����������� X � Y.
// ���������:
// X - ��������� ���������� �� ����������� (0...83).
// Y - ��������� ���������� �� ��������� (0...47).
// idFont - ������������� ������ (��������: FONT_6x8). ��������������
//          ������������� ������� ��������� � ����� Font.h
// Chr - ��� �������.
// Settings - ����� ����������� �������. ����� ��������� ����������
//        ��������� ������:
//        LCD_TWICE_WIDTH (�������� ������ �������)
//        LCD_TWICE_HEIGHT (�������� ������ �������)
//        LCD_INVERSION (��������� ����������� �������)
// ����������:
// -��������� ����������� �������� ������� ����� ����� ������� ������� ������.
//
// ������:
// PT_SPAWN(Context, &ContextChild,
//   LCD_Chr(&ContextChild, 0, 0, FONT_6x8, 'A', 0));
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Chr(struct pt *Context, uint8_t X, uint8_t Y, uint8_t idFont,
	char Chr, uint8_t Settings))
{
	static struct pt ContextChild;

	PT_BEGIN(Context); // ������ �����������
	PT_WAIT_UNTIL(Context, LCD_ChrBuf(X, Y, idFont, Chr, Settings));
	PT_SPAWN(Context, &ContextChild, LCD_Update(&ContextChild));
	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ��������� ���������.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Contrast(struct pt *Context, uint8_t Contrast))
{
	PT_BEGIN(Context); // ������ �����������
	
	// ����������� ����� ������
	PT_WAIT_UNTIL(Context, SendCmd(CMD_FUNCTION_SET | (0<<NUM_PD) |
		(0<<NUM_V) | (1<<NUM_H)));

	// ��������� ������ �������������
	PT_WAIT_UNTIL(Context, SendCmd(CMD_SET_VOP | Contrast));

	// ����������� ����� ������, �������������� ���������
	PT_WAIT_UNTIL(Context, SendCmd(CMD_FUNCTION_SET | (0<<NUM_PD) |
		(0<<NUM_V) | (0<<NUM_H)));
	
	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




// �������
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ��������� ������ ��������� �������.
// ��������:
// Mode - ����� ��������� ���� �� ��������� ��������:
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
// ������� ���� ������ �� ���������.
// ���������:
// X - ���������� �� ����������� (0...83).
// Y - ���������� �� ��������� (0...48).
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Pixel(struct pt *Context, uint8_t X, uint8_t Y))
{
	uint8_t mask;
	uint8_t data;
	static uint8_t row;

	PT_BEGIN(Context); // ������ �����������

	if ((X >= LCD_X_RES) || (Y >= LCD_Y_RES))
		PT_EXIT(Context);

	// ���� �������, ����� ����� ����� �������� ������ � ������
	while (LCD_Busy())
	{
		// ���� ���������� ������� �� ������������ � ������������, �� �������
		// �� �����.
		// (�.�. ���� ������������ ����� �� ����� � ������� ������, ������� �
		// ������ ������ ���������� � ���������)
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

	// ������ ����� �������� ���������� ���� � ���������
	
	MT_MutexWait(Context, MUTEX_LCD); // ����������� �������
	
	#if defined(LCD_ROTATE)
	// ������������� ��������� ����� �� X
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X | (LCD_X_RES - 1 - X)));
	
	// ������������� ��������� ����� �� Y
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y | row));

	// �������� ������ � ���������
	PT_WAIT_UNTIL(Context, SendData(Buf[row][LCD_X_RES - 1 - X]));
	#else
	// ������������� ��������� ����� �� X
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_X | X));

	// ������������� ��������� ����� �� Y
	PT_WAIT_UNTIL(Context, SendCmd(CMD_ADR_Y | row));

	// �������� ������ � ���������
	PT_WAIT_UNTIL(Context, SendData(Buf[row][X]));
	#endif

	MT_MutexFree(MUTEX_LCD);

	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������ ����� ����� ����� ������� �� ���������� (�������� ����������).
// ���������:
// X1 - ���������� ������ ����� �� ����������� (0...83),
// Y1 - ���������� ������ ����� �� ��������� (0...47),
// X2 - ���������� ��������� ����� �� ����������� (0...83),
// Y2 - ���������� ��������� ����� �� ��������� (0...47).
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Line(struct pt *Context, uint8_t X1, uint8_t Y1, uint8_t X2,
	uint8_t Y2))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������
    static int16_t dX, dY;
	static int16_t stepX, stepY;
	static int16_t fraction;
	static uint8_t x, y;

	PT_BEGIN(Context); // ������ �����������

	x = X1;
	y = Y1;

	// dY   Y2 - Y1
	// -- = -------
	// dX   X2 - X1

	dY = Y2 - Y1;
	dX = X2 - X1;

	if (dY < 0)
	{
		// dY �������������
		dY = -dY;
		stepY = -1;
	}
	else
	{
		// dY �������������
		stepY = 1;
	}

	if (dX < 0)
	{
		// dX �������������
		dX = -dX;
		stepX = -1;
	}
	else
	{
		// dX �������������
		stepX = 1;
	}

	dX <<= 1;
	dY <<= 1;

	PT_SPAWN(Context, &ContextChild, LCD_Pixel(&ContextChild, X1, Y1));

	// ������ ��������� ����� �� �����
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

	PT_END(Context); // ���������� �����������
}
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// ������ ���������� � �������, ���������� �������� ������� ����������� X � Y �
// �������� R.
// ���������:
// X - ���������� ������ ���������� �� ����������� (0...83),
// Y - ���������� ������ ���������� �� ��������� (0...47),
// R - ������ ���������� (0...83).
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Circle(struct pt *Context, uint8_t X, uint8_t Y, uint8_t R))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������
	static int16_t d;
	static int8_t xc, yc;

	PT_BEGIN(Context); // ������ �����������

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
// ��������� ��������������.
// ���������:
// X1 - ���������� �������� ������ ���� �������������� �� ����������� (0...83),
// Y1 - ���������� �������� ������ ���� �������������� �� ��������� (0...47),
// X2 - ���������� ������� ������� ���� �������������� �� ����������� (0...83),
// Y1 - ���������� ������� ������� ���� �������������� �� ��������� (0...47),
// Fill - ���� ���������������, ���� �����, ����� ���������� �������
//        �������������� ���� ���������.
///////////////////////////////////////////////////////////////////////////////
PT_THREAD(LCD_Rect(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2, uint8_t Fill))
{
	static struct pt ContextChild; // �������� ��� ��������� �����������
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

	PT_BEGIN(Context); // ������ ����������� 

	// ����������� �� �������� �����������

	// ���� ����� ���������� ������ ������, �� ������ �� �������� ����� �����
	if (X1 > X2) SWAP(X1, X2)

	// ���� ������� ���������� ������ ������, �� ������ �� �������� ����� �����
	if (Y1 > Y2) SWAP(Y1, Y2)

	if (Fill)
	{
		if ((X1 >= LCD_X_RES) || (Y1 >= LCD_Y_RES))
			PT_EXIT(Context);

		if (X2 >= LCD_X_RES) X2 = LCD_X_RES - 1;
		if (Y2 >= LCD_Y_RES) Y2 = LCD_Y_RES - 1;
		X2++;
		Y2++;

		// ��������� ����� ���������� ��� ���������� ������� ������
		xl = Min(X1, xlChanged);
		xh = Max(X2, xhChanged);
		yl = Min(Y1, ylChanged);
		yh = Max(Y2, yhChanged);

		sX1 = X1; sY1 = Y1; sX2 = X2; sY2 = Y2;

		// ���� � ������ ������ ���� �������� � ���������, �� ���������
		// ����������� � ������������ ��������, �, ���� ��� ���, �� ����, ����
		// �� ���������� ��������
		while (LCD_Busy())
		{
			// ���� ���������� ������� �� ������������ � ������������, ��
			// ������� �� �����
			if (!(xh <= xlTransfer) || (xhTransfer <= xl) ||
				(yh <= ylTransfer) || (yhTransfer <= yl))
				PT_YIELD(Context);
		}
		
		// ��������� ����� ���������� ���������� �������
		xlChanged = xl;
		xhChanged = xh;
		ylChanged = yl;
		yhChanged = yh;
		
		sizeY = sY2 - sY1;

		// ��������� �����, ������� ����� ������������ ��� �����������
		// ��������������.
		// � ���� ����� ���������� ������ ����� ������ ��������������.
		mask.value = 1;
		mask.value <<= sizeY;
		mask.value--; // �������� ����� � ���� ���� ������ ��� ��������
						// ����� �������� ��������������

		#if defined(LCD_ROTATE)
		// �������� �����, ��������� � ������� ��������
		mask.value <<= sizeof(mask) * 8 - sizeY - sY1 % 8;
		#else
		mask.value <<= sY1 % 8;
		#endif

		// ������������ ���������� ���� ��� ������ ������� ��������������,
		// ������� ����� ����������� ���������.
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

		// ���� �� ������� ������� ��������������
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
		
		// ����� ���������� ������� ������ � ���������
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
	
	PT_END(Context); // ���������� �����������
}
//////////////////////////////////////////////////////////////////////////
