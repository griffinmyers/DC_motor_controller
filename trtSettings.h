// trtSettings.h
// include before trtkernel

#define MAXNBRTASKS 3
#define MAXNBRSEMAPHORES 7 // 3 sem (1-3) are needed for uart
#define MAXNBRMUTEX 1

#define PRESCALER 1024 // the actual value for timer1 prescalar
#define PRESCALEBITS 5 // the bits to be set in TCCR1b 

#define F_CPU 16000000UL // clock frequency in Hz
#define TICKSPERSECOND F_CPU / PRESCALER

/* UART baud rate */
#define UART_BAUD  9600
