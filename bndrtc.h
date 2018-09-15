#include <avr/io.h>

#define F_CPU 4000000UL
#include <util/delay.h>

#define PA_OUT_SREG PA7
#define PA_OUT_SCL PA4
#define PA_OUT_DO PA5

#define PB_OUT_OEN PB2
#define PB_OUT_RCK PB1

#define PIN_DCF (PINB & 1) // PB0

#define DDRA_INIT (1 << PA_OUT_SREG) | (1 << PA_OUT_SCL) | (1 << PA_OUT_DO)
#define DDRB_INIT (1 << PB_OUT_OEN) | (1 << PB_OUT_RCK)

#define SPI_SELECT_SREG PORTA |= (1 << PA_OUT_SREG)
#define SPI_SELECT_RTC PORTA &= ~(1 << PA_OUT_SREG)

#define RTC_REG_SEC 0x03
#define RTC_REG_CKO 0x0F
#define RTC_REG_YEA 0x09

#define PWM_ENABLE TCCR0A = _BV(COM0A1) | _BV(WGM01) | _BV(WGM00)
#define PWM_DISABLE TCCR0A = _BV(WGM01) | _BV(WGM00)

#define DCF_STAT_FIND_SIGNAL 0
#define DCF_STAT_CHECK_PHASE 1
#define DCF_STAT_WAIT_FOR_SYNC 3
#define DCF_STAT_RECEIVING 7
#define DCF_STAT_SUCCESS 15
