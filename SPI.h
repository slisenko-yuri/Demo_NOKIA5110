#if !defined(_SPI_H_)
#define _SPI_H_

///////////////////////////////////////////////////////////////////////////////
// Библиотека для работы с SPI.
///////////////////////////////////////////////////////////////////////////////

// Инициализация модуля SPI
extern void SPI_Init(void);

// Ожидание готовности драйвера SPI
extern uint8_t SPI_WaitReady(void);

// Передает один байт в индикатор.
extern void SPI_SendByteToLCD(uint8_t Value);

// Запускает передачу данных ведомому устройству.
extern void SPI_SendBufPtrToLCD(uint8_t *Buf, uint8_t Size);

// Возвращает состояние готовности драйвера SPI
extern uint8_t SPI_Ready(void);


#endif
