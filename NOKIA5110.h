#if !defined(_NOKIA5110_H_)
#define _NOKIA5110_H_


#define	LCD_X_RES	84	// ���������� �� �����������
#define	LCD_Y_RES	48	// ���������� �� ���������

// ������ ������ ������
#define LCD_TWICE_WIDTH		0x1
#define LCD_TWICE_HEIGHT	0x2
#define LCD_INVERSION		0x4

// ������ ������ �������
#define LCD_OR   0
#define LCD_AND  1
#define LCD_XOR  2


// ������� ���������� ����������� NOKIA5110
///////////////////////////////////////////////////////////////////////////////
#define NOKIA5110_CE	B,2,L	// PB2,���.14. �����. (���� CE (Chip Enable)
								// ���������� NOKIA 5110)
								
#define NOKIA5110_DIN	B,3,H	// PB3/MOSI,���.15. �����. (���� DIN (Data In)
								// ���������� NOKIA 5110)
								
#define NOKIA5110_CLK	B,5,H	// PB5/SCK,���.17. �����. (���� CLK ����������
								// NOKIA 5110)
								
#define NOKIA5110_DC	B,1,H	// PB1,���.13. �����. (���� DC (Data/Command)
								// ���������� NOKIA 5110)
								
#define NOKIA5110_RST	B,0,L	// PB0,���.12. �����. (���� RST ����������
								// NOKIA 5110)



// ������������� ����������
extern PT_THREAD(LCD_Init(struct pt *Context));

extern uint8_t LCD_Busy(void);

// ������� ���������� ���������� ������� ������ � ���������
extern PT_THREAD(LCD_Update(struct pt *Context));

// ������� ������� ������ � ���������� ������������
extern uint8_t LCD_ClearBuf(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2);

// ������� ������� ���������� � ���������� ������������
extern PT_THREAD(LCD_Clear(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2));

// ��������� ����������� ������ � ������, ������������� � ���
extern uint8_t LCD_StrBuf(uint8_t X, uint8_t Y, uint8_t idFont, char *Str,
	uint8_t Settings);

// ������� ����������� ������ �� ���������
extern PT_THREAD(LCD_Str(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, char *Str, uint8_t Settings));

// ��������� ����������� ������ � ������, ������������� � ���.
// �������� ������ ��� ���� ������������� �� FLASH.
extern uint8_t LCD_StrBuf_P(uint8_t X, uint8_t Y, uint8_t idFont,
	const char *Str, uint8_t Settings);

// ������� ����������� ������ � ���������.
// �������� ������ ��� ���� ������������� �� FLASH.
extern PT_THREAD(LCD_Str_P(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, const char *Str, uint8_t Settings));

// ��������� ����������� ������� � ������, ������������� � ���
extern uint8_t LCD_ChrBuf(uint8_t X, uint8_t Y, uint8_t idFont, char Chr,
	uint8_t Settings);

// ������� ����������� ������� �� ���������
extern PT_THREAD(LCD_Chr(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t idFont, char Chr, uint8_t Settings));

extern PT_THREAD(LCD_Contrast(struct pt *Context, uint8_t Contrast));


// �������
///////////////////////////////////////////////////////////////////////////////

// ��������� ������ ��������� �������
extern void LCD_DrawMode(uint8_t Mode);

// ������� ���� ������ �� ���������
extern PT_THREAD(LCD_Pixel(struct pt *Context, uint8_t X, uint8_t Y));

// ������ ����� ����� ����� ������� �� ����������
extern PT_THREAD(LCD_Line(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2));

// ������ ����������
extern PT_THREAD(LCD_Circle(struct pt *Context, uint8_t X, uint8_t Y,
	uint8_t R));

// ��������� ��������������
extern PT_THREAD(LCD_Rect(struct pt *Context, uint8_t X1, uint8_t Y1,
	uint8_t X2, uint8_t Y2, uint8_t Fill));

#endif
