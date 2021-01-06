//////////////////////////////////////////////////////////////////////////
// Модуль поддержки многозадачности.
// Предназначен для работы совместно с библиотекой макросов Protothreads.
//////////////////////////////////////////////////////////////////////////

#include "Mt.h"


//////////////////////////////////////////////////////////////////////////
// Проверка корректности установленных директив
//////////////////////////////////////////////////////////////////////////

#if defined(MT_USE_SYSTIMER)

#if !defined(F_CPU)
#error "MT: F_CPU must be defined"
#endif

#if !defined(MT_SYSTIMER_PERIOD_MS)
#error "MT: MT_SYSTIMER_PERIOD_MS must be defined"
#endif

#if ((MT_SYSTIMER_PERIOD_MS < 1) || (MT_SYSTIMER_PERIOD_MS > 20))
#error "MT: MT_SYSTIMER_PERIOD_MS must be in the range 1...20"
#endif

#if (MT_SYSTIMER_PERIOD_MS >= 20) && (F_CPU > (50 * (1024 * (1 + 255))))
#error "MT: CPU freq must be less then 13107200 Hz for MT_SYSTIMER_PERIOD_MS 20"
#endif

#endif




//////////////////////////////////////////////////////////////////////////
// Переменные модуля
//////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////
// ЗАДАЧИ
//////////////////////////////////////////////////////////////////////////

// Указатели на задачи
MT_TASK_TYPE __MT_Task[__MT_TASK_COUNT];

// Контексты задач
struct pt __MT_Context[__MT_TASK_COUNT];




#if !defined(MT_USE_GPIOR)

// Битовые маски активных задач (готовых к выполнению)
MT_TYPE __MT_TaskActiveFlags;

// ID текущей задачи
uint8_t __MT_TaskCur;

// Битовая маска текущей задачи (Установленный бит с номером, равным ID
// текущей задачи).
MT_TYPE __MT_TaskCurMask;

#endif




#if defined(MT_USE_TASK_RUN_STOP)

// Битовые маски пассивных задач
MT_TYPE	__MT_TaskPassiveFlags;

// Битовые маски запросов для перевода задач в активное состояние
MT_TYPE	__MT_TaskRequestActiveFlags;

#endif




#if defined(MT_USE_HIGH_PRIORITY_TASK)

// ID приоритетной задачи
uint8_t __MT_TaskPriority;

// Битовая маска приоритетной задачи
MT_TYPE __MT_TaskPriorityMask;

#endif




// Количество зарегистрированных задач
// (Используется только при инициализации задач)
uint8_t __MT_TaskCount;





#if defined(MT_USE_SYSTIMER)




//////////////////////////////////////////////////////////////////////////
// Таймеры
//////////////////////////////////////////////////////////////////////////

#if defined(MT_USE_TASK_SLEEP)

// Счетчики тактов системного таймера для "спящих" задач
uint16_t __MT_TaskSysTimer[__MT_TASK_COUNT];

#endif




//////////////////////////////////////////////////////////////////////////
// Таймауты
//////////////////////////////////////////////////////////////////////////

#if (__MT_TIMEOUT_COUNT > 0)

// Счетчики для таймаутов
MT_TIMEOUT_TYPE __MT_Timeout[__MT_TIMEOUT_COUNT];

#endif





#if defined(MT_USE_GETSYSTIMER) || (__MT_TIMEOUT_COUNT > 0)

// Счетчик тактов системного таймера
uint32_t __MT_SysTimer = 0;

#endif




#endif




//////////////////////////////////////////////////////////////////////////
// Мьютексы
//////////////////////////////////////////////////////////////////////////

#if (__MT_MUTEX_COUNT > 0)

// Состояния мьютексов (>=1 - захвачен, 0 - свободен)
uint8_t __MT_Mutex[__MT_MUTEX_COUNT];

// ID задач, захвативших соответствующий мьютекс
uint8_t __MT_MutexOwner[__MT_MUTEX_COUNT];
										
// Битовые маски задач, которые ждут освобождения соответствующего
// мьютекса
MT_TYPE __MT_MutexWaitFlags[__MT_MUTEX_COUNT];
								
#endif




//////////////////////////////////////////////////////////////////////////
// Драйверы
//////////////////////////////////////////////////////////////////////////

#if (__MT_DRV_COUNT > 0)

// Битовые маски задач, ждущих завершения работы соответствующих драйверов
MT_TYPE __MT_DrvWaitFlags[__MT_DRV_COUNT];

#endif
					



#if defined(MT_USE_SYSTIMER)

//////////////////////////////////////////////////////////////////////////
// Обработчик прерывания системного таймера
//////////////////////////////////////////////////////////////////////////

#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__) ||\
	defined(__AVR_ATmega8515__) ||\
	defined(__AVR_ATmega16__) || defined(__AVR_ATmega16A__) ||\
	defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__) ||\
	defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) ||\
	defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__)

#if (MT_SYSTIMER == 0)
ISR(TIMER0_COMP_vect)
#elif (MT_SYSTIMER == 1)
ISR(TIMER1_COMPA_vect)
#elif (MT_SYSTIMER == 2)
ISR(TIMER2_COMP_vect)
#elif (MT_SYSTIMER == 3)
ISR(TIMER3_COMPA_vect)
#endif

#else

#if (MT_SYSTIMER == 0)
ISR(TIMER0_COMPA_vect)
#elif (MT_SYSTIMER == 1)
ISR(TIMER1_COMPA_vect)
#elif (MT_SYSTIMER == 2)
ISR(TIMER2_COMPA_vect)
#elif (MT_SYSTIMER == 3)
ISR(TIMER3_COMPA_vect)
#elif (MT_SYSTIMER == 4)
ISR(TIMER4_COMPA_vect)
#elif (MT_SYSTIMER == 5)
ISR(TIMER5_COMPA_vect)
#endif 
#endif
{
	#if defined(MT_USE_TASK_SLEEP)
	
	uint8_t task;
	MT_TYPE mask;

	for (task = 0, mask = 1; task < __MT_TASK_COUNT; task++, mask <<= 1)
	{
		if (__MT_TaskSysTimer[task] == 0) continue;

		__MT_TaskSysTimer[task]--;

		if (__MT_TaskSysTimer[task] == 0)
		{
			__MT_TaskActiveFlags |= mask; // Разрешить работу задаче, ожидающей
											// таймер
		}
	}
	
	#endif // #if defined(MT_USE_TASK_SLEEP)

	#if defined(MT_USE_GETSYSTIMER) || (__MT_TIMEOUT_COUNT > 0)
	__MT_SysTimer++;
	#endif

	#if defined(MT_SYSTIMER_CALLBACK)
	MT_SYSTIMER_CALLBACK();
	#endif
}
//------------------------------------------------------------------------
#endif





#if defined(MT_USE_SYSTIMER)

//////////////////////////////////////////////////////////////////////////
// Расчет параметров таймера
//////////////////////////////////////////////////////////////////////////

#if ((MT_SYSTIMER == 2) &&\
	!defined(__AVR_ATmega128__) && !defined(__AVR_ATmega128A__) &&\
	!defined(__AVR_ATmega64__) && !defined(__AVR_ATmega64A__)) ||\
	((MT_SYSTIMER == 0) &&\
	(defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__) ||\
	defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__)))

#if (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (1UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 1"
#define	MT_SYSTIMER_DIVIDER	1UL
#define DIV_VALUE	TIM2_DIV_1

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (8UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 8"
#define	MT_SYSTIMER_DIVIDER	8UL
#define DIV_VALUE	TIM2_DIV_8

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (32UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 32"
#define	MT_SYSTIMER_DIVIDER	32UL
#define DIV_VALUE	TIM2_DIV_32

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (64UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 64"
#define	MT_SYSTIMER_DIVIDER	64UL
#define DIV_VALUE	TIM2_DIV_64

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (128UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 128"
#define	MT_SYSTIMER_DIVIDER	128UL
#define DIV_VALUE	TIM2_DIV_128

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (256UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 256"
#define	MT_SYSTIMER_DIVIDER	256UL
#define DIV_VALUE	TIM2_DIV_256

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (1024UL * (1 + 255UL)))
#warning "MT: Prescaler for system timer = 1024"
#define	MT_SYSTIMER_DIVIDER	1024UL
#define DIV_VALUE	TIM2_DIV_1024

#else
#error "MT: The system timer cannot be configured! (Prescaler for system timer cannot be more than 1024)"
#endif


#elif ((MT_SYSTIMER == 0) &&\
	!defined(__AVR_ATmega128__) && !defined(__AVR_ATmega128A__) &&\
	!defined(__AVR_ATmega64__) && !defined(__AVR_ATmega64A__)) ||\
	((MT_SYSTIMER == 2) &&\
	(defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__) ||\
	defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__)))

#if (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (1UL * (1 + 255UL)))
#warning "MT: The prescaler for the system timer = 1"
#define	MT_SYSTIMER_DIVIDER	1UL
#define DIV_VALUE	TIM_DIV_1

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (8UL * (1 + 255UL)))
#warning "MT: The prescaler for the system timer = 8"
#define	MT_SYSTIMER_DIVIDER	8UL
#define DIV_VALUE	TIM_DIV_8

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (64UL * (1 + 255UL)))
#warning "MT: The prescaler for the system timer = 64"
#define	MT_SYSTIMER_DIVIDER	64UL
#define DIV_VALUE	TIM_DIV_64

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (256UL * (1 + 255UL)))
#warning "MT: The prescaler for the system timer = 256"
#define	MT_SYSTIMER_DIVIDER	256UL
#define DIV_VALUE	TIM_DIV_256

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (1024UL * (1 + 255UL)))
#warning "MT: The prescaler for the system timer = 1024"
#define	MT_SYSTIMER_DIVIDER	1024UL
#define DIV_VALUE	TIM_DIV_1024

#else
#error "MT: The system timer cannot be configured! (Prescaler for system timer cannot be more than 1024)"
#endif


#elif ((MT_SYSTIMER == 1) ||\
	(MT_SYSTIMER == 3) || (MT_SYSTIMER == 4) || (MT_SYSTIMER == 5))

#if (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (1UL * (1 + 65535UL)))
#warning "MT: The prescaler for the system timer = 1"
#define	MT_SYSTIMER_DIVIDER	1UL
#define DIV_VALUE	TIM_DIV_1

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (8UL * (1 + 65535UL)))
#warning "MT: The prescaler for the system timer = 8"
#define	MT_SYSTIMER_DIVIDER	8UL
#define DIV_VALUE	TIM_DIV_8

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (64UL * (1 + 65535UL)))
#warning "MT: The prescaler for the system timer = 64"
#define	MT_SYSTIMER_DIVIDER	64UL
#define DIV_VALUE	TIM_DIV_64

#elif (1000UL / MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (256UL * (1 + 65535UL)))
#warning "MT: The prescaler for the system timer = 256"
#define	MT_SYSTIMER_DIVIDER	256UL
#define DIV_VALUE	TIM_DIV_256

#elif (1000UL /MT_SYSTIMER_PERIOD_MS) >= (F_CPU / (1024UL * (1 + 65535UL)))
#warning "MT: The prescaler for the system timer = 1024"
#define	MT_SYSTIMER_DIVIDER	1024UL
#define DIV_VALUE	TIM_DIV_1024

#else
#error "MT: The system timer cannot be configured! (Prescaler for system timer cannot be more than 1024)"
#endif

#endif // #if (MT_SYSTIMER == 0)




// Вычисляем два значения для регистра OCR.
// То значение, с помощью которого будет получен более точный период
// срабатывания системного таймера и будет выбрано

// Значение OCR без дробной части
#define	OCR_VALUE1 (((F_CPU) * 100UL) /\
	(MT_SYSTIMER_DIVIDER * ((1000UL * 100) / MT_SYSTIMER_PERIOD_MS)) - 1)

// Значение OCR без дробной части, увеличенное на 1
#define	OCR_VALUE2	(OCR_VALUE1 + 1UL)

// Частота, получаемая с помощью OCR_VALUE1, умноженная на 100
#define	FREQ_VALUE1\
	(((F_CPU) * 100UL) / (MT_SYSTIMER_DIVIDER * (1UL + (OCR_VALUE1))))

// Частота, получаемая с помощью OCR_VALUE2, умноженная на 100
#define	FREQ_VALUE2\
	(((F_CPU) * 100UL) / (MT_SYSTIMER_DIVIDER * (1UL + (OCR_VALUE2))))

// Требуемая частота, умноженная на 100
#define	FREQ_VALUE	((1000UL * 100) / MT_SYSTIMER_PERIOD_MS)

// Определяем, какое из значений (OCR_VALUE1 или OCR_VALUE2) будет
// точнее для требуемого периода срабатывания таймера
#if (FREQ_VALUE1 - FREQ_VALUE) < (FREQ_VALUE - FREQ_VALUE2)

#define	OCR_VALUE	OCR_VALUE1

#if (FREQ_VALUE1 - FREQ_VALUE) < (FREQ_VALUE / 1000UL)
#warning "MT: Inaccuracy of the system timer period is less than 0.1%"
#elif (FREQ_VALUE1 - FREQ_VALUE) <= (FREQ_VALUE / 100UL)
#warning "MT: Inaccuracy of the system timer period is 0.1%...1%"
#else
#warning "MT: Inaccuracy of the system timer period is more 1%!"
#endif

#else

#define	OCR_VALUE	OCR_VALUE2

#if (FREQ_VALUE - FREQ_VALUE2) < (FREQ_VALUE / 1000UL)
#warning "MT: Inaccuracy of the system timer period is less than 0.1%"
#elif (FREQ_VALUE - FREQ_VALUE2) <= (FREQ_VALUE / 100UL)
#warning "MT: Inaccuracy of the system timer period is 0.1%...1%"
#else
#warning "MT: Inaccuracy of the system timer period is more 1%!"
#endif

#endif

#endif




#if defined(MT_USE_SYSTIMER)

//------------------------------------------------------------------------
// Константы для настройки системного таймера AVR
//------------------------------------------------------------------------

#if (MT_SYSTIMER == 0)

#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__)
#error "MT: Using timer 0 for ATmega8(A) is not allowed!"
#else

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Инициализация системного таймера для использования менеджером задач
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_SysTimerInit(void)
{
	// Настройка таймера на режим CTC с генерацией прерывания.
	// Формула: Ftimer = Fclk / (N * (1 + OCRnx)),
	// где N - предделитель (1, 8, 32(*), 64, 128(*), 256, 1024)

	// Для вычисления значения OCR2A используется формула:
	// OCRnx = Fclk / (N * Ftimer) - 1

	ENTER_CRITICAL();
	
	#if defined(__AVR_ATmega8515__) ||\
		defined(__AVR_ATmega16__) || defined(__AVR_ATmega16A__) ||\
		defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__) ||\
		defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) ||\
		defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__)

	OCR0 = OCR_VALUE;

	// Режим CTC (Генерация прерывания по достижению OCRn)
	TCCR0 = (0<<FOC0) | (0<<WGM00) | (0<<COM01) | (0<<COM00) |
		(1<<WGM01) | DIV_VALUE;

	// Разрешаем прерывания таймера по совпадению с регистром OCRn
	TIMSK |= (1<<OCIE0) | (0<<TOIE0);

	#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__)

	TCCR0B = (0<<FOC0A) | (0<<FOC0B) | (0<<WGM02) | DIV_VALUE;
	OCR0A = OCR_VALUE;

	// Режим CTC (Генерация прерывания по достижению OCRnA)
	TCCR0A = (0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) |
		(1<<WGM01) | (0<<WGM00);

	// Разрешаем прерывания таймера по совпадению с регистром OCRnA
	TIMSK |= (0<<OCIE0B) | (1<<OCIE0A) | (0<<TOIE0);

	#else

	TCCR0B = (0<<FOC0A) | (0<<FOC0B) | (0<<WGM02) | DIV_VALUE;
	OCR0A = OCR_VALUE;

	// Режим CTC (Генерация прерывания по достижению OCRnA)
	TCCR0A = (0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) |
		(1<<WGM01) | (0<<WGM00);

	// Разрешаем прерывания таймера по совпадению с регистром OCRnA
	TIMSK0 |= (0<<OCIE0B) | (1<<OCIE0A) | (0<<TOIE0);

	#endif

	EXIT_CRITICAL();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif

#elif (MT_SYSTIMER == 1)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Инициализация системного таймера для использования менеджером задач
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_SysTimerInit(void)
{
	// Настройка таймера на режим CTC с генерацией прерывания.
	// Формула: Ftimer = Fclk / (N * (1 + OCRnx)),
	// где N - предделитель (1, 8, 32(*), 64, 128(*), 256, 1024)

	// Для вычисления значения OCRnA используется формула:
	// OCRnx = Fclk / (N * Ftimer) - 1

	ENTER_CRITICAL();
	
	#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__) ||\
		defined(__AVR_ATmega8515__) ||\
		defined(__AVR_ATmega16__) || defined(__AVR_ATmega16A__) ||\
		defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__) ||\
		defined(__AVR_ATmega64__)  || defined(__AVR_ATmega64A__) ||\
		defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__)

	TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) |
		DIV_VALUE;

	OCR1A = OCR_VALUE;

	// Режим CTC (Генерация прерывания по достижению OCRnA)
	TCCR1A = (0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) |
		(0<<WGM11) | (0<<WGM10);

	// Разрешаем прерывания таймера по совпадению с регистром OCRnA
	TIMSK = (TIMSK & ~((1<<TICIE1) | (1<<OCIE1A) | (1<<OCIE1B) |
		(1<<TOIE1))) | (1<<OCIE1A);

	#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__)
	
	TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) |
		DIV_VALUE;

	OCR1A = OCR_VALUE;

	// Режим CTC (Генерация прерывания по достижению OCRnA)
	TCCR1A = (0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) |
		(0<<WGM11) | (0<<WGM10);

	// Разрешаем прерывания таймера по совпадению с регистром OCRnA
	TIMSK = (TIMSK & ~((1<<ICIE1) | (1<<OCIE1B) | (1<<OCIE1A) |
		(1<<TOIE1))) | (0<<ICIE1) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);
	
	#else
	
	TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) |
		DIV_VALUE;

	OCR1A = OCR_VALUE;

	// Режим CTC (Генерация прерывания по достижению OCRnA)
	TCCR1A = (0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) |
		(0<<WGM11) | (0<<WGM10);
	
	// Разрешаем прерывания таймера по совпадению с регистром OCRnA
	TIMSK1 = (0<<ICIE1) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);
	
	#endif

	EXIT_CRITICAL();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#elif (MT_SYSTIMER == 2)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Инициализация системного таймера для использования менеджером задач
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_SysTimerInit(void)
{
	// Настройка таймера 2 на режим CTC с генерацией прерывания.
	// Формула: Ftimer = Fclk / (N * (1 + OCR2A)),
	// где N - предделитель (1, 8, 32, 64, 128, 256, 1024)

	// Для вычисления значения OCR2A используеется формула:
	// OCR2A = Fclk / (N * Ftimer) - 1
	
	ENTER_CRITICAL();
	
	#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__) ||\
		defined(__AVR_ATmega16__) || defined(__AVR_ATmega16A__) ||\
		defined(__AVR_ATmega32__) || defined(__AVR_ATmega32A__) ||\
		defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) ||\
		defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__)

	OCR2 = OCR_VALUE;
	
	// WGM20:WGM21=0:1 (режим CTC), COM21:COM20=0:0 (OC2 disconnected)
	TCCR2 = (0<<FOC2) | (0<<WGM20) | (0<<COM21) | (0<<COM20) |
		(1<<WGM21) | DIV_VALUE;

	TIMSK |= 1<<OCIE2;

	#else
	
	TCCR2B = (0<<FOC2A) | (0<<FOC2B) | (0<<WGM22) | DIV_VALUE;
	OCR2A = OCR_VALUE;

	// Режим CTC (Генерация прерывания по достижению OCR2A)
	TCCR2A = (0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) |
		(1<<WGM21) | (0<<WGM20);

	// Разрешаем прерывания таймера 2 по совпадению с регистром OCR2A
	TIMSK2 |= (0<<OCIE2B) | (1<<OCIE2A) | (0<<TOIE2);
	
	#endif

	EXIT_CRITICAL();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#elif (MT_SYSTIMER == 3)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Инициализация системного таймера для использования менеджером задач
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_SysTimerInit(void)
{
	// Настройка таймера на режим CTC с генерацией прерывания.
	// Формула: Ftimer = Fclk / (N * (1 + OCRnx)),
	// где N - предделитель (1, 8, 32(*), 64, 128(*), 256, 1024)

	// Для вычисления значения OCRnA используеется формула:
	// OCRnx = Fclk / (N * Ftimer) - 1

	ENTER_CRITICAL();
	
	TCCR3B = (0<<ICNC3) | (0<<ICES3) | (0<<WGM33) | (1<<WGM32) |
		DIV_VALUE;

	OCR3A = OCR_VALUE;

	// Режим CTC (Генерация прерывания по достижению OCRnA)
	TCCR3A = (0<<COM3A1) | (0<<COM3A0) | (0<<COM3B1) | (0<<COM3B0) |
		(0<<WGM31) | (0<<WGM30);

	#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega64A__) ||\
		defined(__AVR_ATmega128__) || defined(__AVR_ATmega128A__)
	
	// Разрешаем прерывания таймера по совпадению с регистром OCRnA
	ETIMSK = (ETIMSK & ~((1<<TICIE3) | (1<<OCIE3A) | (1<<OCIE3B) |
		(1<<TOIE3) | (1<<OCIE3C) | (1<<OCIE3C))) | (1<<OCIE3A);
	#else
	// Разрешаем прерывания таймера по совпадению с регистром OCRnA
	TIMSK3 = (0<<ICIE3) | (0<<OCIE3B) | (1<<OCIE3A) | (0<<TOIE3);
	#endif

	EXIT_CRITICAL();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#elif (MT_SYSTIMER == 4)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Инициализация системного таймера для использования менеджером задач
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_SysTimerInit(void)
{
	// Настройка таймера на режим CTC с генерацией прерывания.
	// Формула: Ftimer = Fclk / (N * (1 + OCRnx)),
	// где N - предделитель (1, 8, 32(*), 64, 128(*), 256, 1024)

	// Для вычисления значения OCRnA используется формула:
	// OCRnx = Fclk / (N * Ftimer) - 1
	
	ENTER_CRITICAL();
	
	TCCR4B = (0<<ICNC4) | (0<<ICES4) | (0<<WGM43) | (1<<WGM42) |
		DIV_VALUE;

	OCR4A = OCR_VALUE;

	// Режим CTC (Генерация прерывания по достижению OCRnA)
	TCCR4A = (0<<COM4A1) | (0<<COM4A0) | (0<<COM4B1) | (0<<COM4B0) |
		(0<<WGM41) | (0<<WGM40);

	// Разрешаем прерывания таймера по совпадению с регистром OCRnA
	TIMSK4 = (0<<ICIE4) | (0<<OCIE4B) | (1<<OCIE4A) | (0<<TOIE4);

	EXIT_CRITICAL();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#elif (MT_SYSTIMER == 5)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Инициализация системного таймера для использования менеджером задач
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_SysTimerInit(void)
{
	// Настройка таймера на режим CTC с генерацией прерывания.
	// Формула: Ftimer = Fclk / (N * (1 + OCRnx)),
	// где N - предделитель (1, 8, 32(*), 64, 128(*), 256, 1024)

	// Для вычисления значения OCRnA используеется формула:
	// OCRnx = Fclk / (N * Ftimer) - 1

	ENTER_CRITICAL();
	
	TCCR5B = (0<<ICNC5) | (0<<ICES5) | (0<<WGM53) | (1<<WGM52) |
		DIV_VALUE;

	OCR5A = OCR_VALUE;

	// Режим CTC (Генерация прерывания по достижению OCRnA)
	TCCR5A = (0<<COM5A1) | (0<<COM5A0) | (0<<COM5B1) | (0<<COM5B0) |
		(0<<WGM51) | (0<<WGM50);

	// Разрешаем прерывания таймера по совпадению с регистром OCRnA
	TIMSK5 = (0<<ICIE5) | (0<<OCIE5B) | (1<<OCIE5A) | (0<<TOIE5);

	EXIT_CRITICAL();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif

#endif





#if defined(MT_USE_GETSYSTIMER) || (__MT_TIMEOUT_COUNT > 0)
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Возвращает счетчик таймерных тиков (32 разряда)
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint32_t MT_GetSysTimer(void)
{
	uint32_t ticks;

	ENTER_CRITICAL();
	ticks = __MT_SysTimer;
	EXIT_CRITICAL();

	return ticks;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Инициализация менеджера задач
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void MT_Init(void)
{
	uint8_t i;

	__MT_TaskCount = 0; // Количество зарегистрированных задач

	#if defined(MT_USE_GETSYSTIMER) || (__MT_TIMEOUT_COUNT > 0)
	__MT_SysTimer = 0;
	#endif

	__MT_TaskActiveFlags = 0; // Нет активных задач
	__MT_TaskCur = 0;
	__MT_TaskCurMask = 1 << 0;
	

	#if defined(MT_USE_HIGH_PRIORITY_TASK)
	__MT_TaskPriority = ID_UNKNOWN; // Нет приоритетной задачи
	__MT_TaskPriorityMask = 0; // Нет приоритетной задачи
	#endif
	
	for (i = 0; i < __MT_TASK_COUNT; i++)
	{
		__MT_Task[i] = NULL; // Адрес задачи
		#if defined(MT_USE_TASK_SLEEP)
		__MT_TaskSysTimer[i] = 0; // Таймер ожидания ("спячки") для задачи
		#endif
	}
	
	#if (__MT_MUTEX_COUNT > 0)
	for (i = 0; i < __MT_MUTEX_COUNT; i++)
	{
		// Битовые маски задач, ждущих освобождения мьютекса.
		// (0 - нет задач, ждущих освобождения мьютекса)
		__MT_MutexWaitFlags[i] = 0; 
		
		// Состояния мьютекса (0 - свободен)		
		__MT_Mutex[i] = 0;

		// ID задачи, владеющей мьютексом.
		// (0xFF - нет задачи, владеющей мьютексом)
		__MT_MutexOwner[i] = ID_UNKNOWN;
	}
	#endif
	
	#if (__MT_TIMEOUT_COUNT > 0)
	for (i = 0; i < __MT_TIMEOUT_COUNT; i++)
	{
		__MT_Timeout[i] = 0; // Таймауты
	}
	#endif

	#if (__MT_DRV_COUNT > 0)
	for (i = 0; i < __MT_DRV_COUNT; i++)
	{
		__MT_DrvWaitFlags[i] = 0; // Битовые маски задач, ждущих
									// обслуживания драйвером
	}
	#endif
	
	#if defined(MT_USE_SYSTIMER)
	MT_SysTimerInit();
	#endif
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Инициализация задачи
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
uint8_t MT_TaskInit(MT_TASK_TYPE Task, uint8_t fActive)
{
	if (__MT_TaskCount >= __MT_TASK_COUNT) return ID_UNKNOWN;

	// Инициализация контекста задачи
	PT_INIT(&__MT_Context[__MT_TaskCount]);

	// Сохраняем указатель на задачу
	__MT_Task[__MT_TaskCount] = Task;

	if (fActive)
	{
		// Если задача активна, то установить соответствующий бит в
		// переменной __MT_TaskActiveFlags
		__MT_TaskActiveFlags |= (MT_TYPE)1 << __MT_TaskCount;
	}

	#if defined(MT_USE_TASK_RUN_STOP)
	else
	{
		// Если задача пассивна, то установить соответствующий бит в
		// переменной __MT_TaskPassiveFlags
		__MT_TaskPassiveFlags |= (MT_TYPE)1 << __MT_TaskCount;
	}
	#endif

	__MT_TaskCount++;

	return (__MT_TaskCount - 1); // Вернуть ID задачи
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




#if defined(MT_USE_TASK_SLEEP)

//////////////////////////////////////////////////////////////////////////
// Функции MT_SetSleep и MT_TaskSetSleep устанавливают время, в течение
// которого задача будет находиться в неактивном состоянии. Время
// указывается в тактах системного таймера. Для указания времени в
// миллисекундах необходимо использовать макрос MS_TO_SYSTICK. С помощью
// Функции MT_SetSleep задача может перевести в состояние ожидания только
// саму себя, а с помощью функции MT_TaskSetSleep любую другую задачу.
// После передачи управления планировщику (например, с помощью макроса
// PT_END или PT_YIELD) задача не будет вызываться планировщиком, пока не
// обнулится соответствующий счетчик времени "спячки" (переменная
// __MT_TaskSysTimer[idTask]), установленный с помощью этой функции, после
// чего задача будет переведена в активное состояние установкой
// соответствующего бита в переменной __MT_TaskActiveFlags.
// Декремент счетчика, а также установка бита активности после обнуления
// счетчика производится в обработчике прерываний системного таймера.
// Для использования этих функций в проекте необходимо в файле Mt_Cfg.h
// раскомментировать следующую строку:
// #define MT_USE_TASK_SLEEP
//
//  ПРИМЕР:
//
//  PT_THREAD(TaskN(struct pt *Context))
//  {
//	  PT_BEGIN(Context);
//
//	  //...
//
//	  // Установка счетчика времени "спячки", равного 1000 мс.
//	  MT_SetSleep(MS_TO_SYSTICK(1000));
//
//	  // Передача управления планировщику. Планировщик вызовет данную задачу
//	  // только по прошествии времени, равного 1000 мс, после чего задача
//	  // продолжит свое исполнение с места, расположенного после
//	  // макроса PT_YIELD.
//	  PT_YIELD(Context);
//
//	  //...
//
//	  PT_END(Context);
//  }
//////////////////////////////////////////////////////////////////////////

void MT_SetSleep(uint16_t Takts)
{
	if (Takts)
	{
		ENTER_CRITICAL();

		// Обнуляем бит активности задачи
		__MT_TaskActiveFlags &= ~__MT_TaskCurMask;

		// Устанавливаем время "спячки"
		__MT_TaskSysTimer[__MT_TaskCur] = Takts;
	
		EXIT_CRITICAL();
	}
}
//////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////
// Перевод задачи в "спячку". Время указывается в тактах системного
// таймера.
// Может перевести в "спячку" другие задачи
// В этой функции дополнительно требуется указать ID текущей задачи.
//////////////////////////////////////////////////////////////////////////
void MT_TaskSetSleep(uint8_t idTask, uint16_t Takts)
{
	if (Takts)
	{
		ENTER_CRITICAL();
		
		// Обнуляем бит активности задачи
		__MT_TaskActiveFlags &= ~((MT_TYPE)1 << idTask);
		
		// Устанавливаем время "спячки"
		__MT_TaskSysTimer[idTask] = Takts;

		EXIT_CRITICAL();
	}
}
//////////////////////////////////////////////////////////////////////////

#endif




#if (__MT_MUTEX_COUNT > 0)
//////////////////////////////////////////////////////////////////////////
// Захват мьютекса.
// Возвращает 1, если мьютекс успешно захвачен.
// Возвращает 0, если текущая задача переведена в ждущее состояние до
// освобождения мьютекса задачей, захватившей его.
// ПРИМЕЧАНИЕ: Данный макрос нельзя вызывать вподряд без соответствующего
// вызова MT_MutexFree() более 255 раз (не хватит разрядности мьютекса)!
//////////////////////////////////////////////////////////////////////////
uint8_t __MT_MutexWait(uint8_t Mutex)
{
	MT_TYPE mask;

	if (__MT_TaskCur == __MT_MutexOwner[Mutex])
	{
		// Если задача уже владеет этим мьютексом, то инкрементируем его
		// значение.
		__MT_Mutex[Mutex]++;
	}
	else // Если текущая задача не владеет мьютексом
	{	
		if (__MT_Mutex[Mutex] != 0) // Если мьютекс занят
		{
			// Читаем битовую маску текущей задачи
			mask = __MT_TaskCurMask;

			ENTER_CRITICAL();

			// Запоминаем битовую маску задачи, ожидающей освобождения
			// данного мьютекса.
			__MT_MutexWaitFlags[Mutex] |= mask;

			// Снимаем бит активности задачи.
			__MT_TaskActiveFlags &= ~mask;

			EXIT_CRITICAL();

			return 0;
		}
		__MT_Mutex[Mutex] = 1; // Мьютекс занят.
		
		// Сохраняем ID задачи, захватившей мьютекс.
		__MT_MutexOwner[Mutex] = __MT_TaskCur;
	}
	return 1;
}
//////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////
// Освобождение мьютекса
//////////////////////////////////////////////////////////////////////////
void MT_MutexFree(uint8_t Mutex)
{
	ENTER_CRITICAL();

	__MT_Mutex[Mutex]--;

	if (__MT_Mutex[Mutex] == 0) // Если мьютекс свободен
	{
		// Активируем задачи, которые ждут освобождения данного мьютекса
		__MT_TaskActiveFlags |= __MT_MutexWaitFlags[Mutex];

		// Очищаем битовые маски задач, ждущих освобождения данного
		// мьютекса
		__MT_MutexWaitFlags[Mutex] = 0;
		
		// Больше мьютексом никто не владеет
		__MT_MutexOwner[Mutex] = ID_UNKNOWN;
	}

	EXIT_CRITICAL();
}
//////////////////////////////////////////////////////////////////////////

#endif // __MT_MUTEX_COUNT > 0




#if defined(MT_USE_TASK_RUN_STOP)

//////////////////////////////////////////////////////////////////////////
// Запуск задачи (перевод в активное состояние) при запрещенных
// прерываниях
//////////////////////////////////////////////////////////////////////////
void MT_TaskSetActiveIrq(uint8_t idTask)
{
	MT_TYPE mask;
	
	mask = (MT_TYPE)1 << idTask; // Битовая маска задачи
	
	if (__MT_TaskPassiveFlags & mask) // Если задача пассивна
	{
		// Перевод задачи в активное состояние
		__MT_TaskActiveFlags |= mask;
		__MT_TaskPassiveFlags &= ~mask;
	}
	else
	{
		// Устанавливаем запрос на активацию задачи
		__MT_TaskRequestActiveFlags |= mask;
	}
}
//////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////
// Перевод задачи в активное состояние
//////////////////////////////////////////////////////////////////////////
void MT_TaskSetActive(uint8_t idTask)
{
	ENTER_CRITICAL();
	MT_TaskSetActiveIrq(idTask);
	EXIT_CRITICAL();
}
//////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////
// Перевод текущей задачи в пассивное состояние
//////////////////////////////////////////////////////////////////////////
uint8_t MT_TaskSetPassive(void)
{
	uint8_t result;

	ENTER_CRITICAL();
	if (__MT_TaskRequestActiveFlags & __MT_TaskCurMask)
	{
		////__MT_TaskActiveFlags |= 1 << __MT_TaskCur; // Лишнее, т.к. уже установлен
		__MT_TaskRequestActiveFlags &= ~__MT_TaskCurMask;
		
		// Задача осталась активной, т.к. был запрос на перевод в активное
		// состояние
		result = 0;
	}
	else
	{
		// Перевод задачи в пассивное состояние
		__MT_TaskActiveFlags &= ~__MT_TaskCurMask;
		__MT_TaskPassiveFlags |= __MT_TaskCurMask;
		
		
		result = 1; // Задача перешла в пассивное состояние
	}
	EXIT_CRITICAL();
	return result;
}
//////////////////////////////////////////////////////////////////////////
#endif



