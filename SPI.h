#if !defined(_SPI_H_)
#define _SPI_H_

///////////////////////////////////////////////////////////////////////////////
// ���������� ��� ������ � SPI.
///////////////////////////////////////////////////////////////////////////////

// ������������� ������ SPI
extern void SPI_Init(void);

// �������� ���������� �������� SPI
extern uint8_t SPI_WaitReady(void);

// �������� ���� ���� � ���������.
extern void SPI_SendByteToLCD(uint8_t Value);

// ��������� �������� ������ �������� ����������.
extern void SPI_SendBufPtrToLCD(uint8_t *Buf, uint8_t Size);

// ���������� ��������� ���������� �������� SPI
extern uint8_t SPI_Ready(void);


#endif
