#if !defined(_FONT_H_)
#define _FONT_H_




// Идентификаторы используемых шрифтов

//#define FONT_5x12	1
#define FONT_6x8		10
//#define FONT_6x12	11
#define FONT_8x8		20
#define FONT_8x8t	21
#define FONT_8x8n	22
//#define FONT_8x12		30
//#define FONT_8x13n	31
//#define FONT_8x14		32
//#define FONT_8x14t	33
//#define FONT_8x14n	34
#define FONT_8x16	35
#define FONT_8x16n	36
//#define FONT_8x16g	37
#define FONT_8x16t	38
#define FONT_10x14	40
#define FONT_12x16	50


// Возвращает адрес шрифта в программной памяти микроконтроллера
extern uint8_t const *FONT_Addr(uint8_t idFont);

// Возвращает ширину символа в пикселах для шрифта, указанного параметром idFont
extern uint8_t FONT_Width(uint8_t idFont);

// Возвращает высоту символа в пикселах для шрифта, указанного параметром idFont
extern uint8_t FONT_Height(uint8_t idFont);

#endif
