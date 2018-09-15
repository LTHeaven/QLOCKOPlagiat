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

#define PIN_ES_IST 	(1UL<<0)
#define PIN_FUENF_1 (1UL<<1)
#define PIN_ZEHN_1 	(1UL<<2)
#define PIN_ZWANZIG (1UL<<3)
#define PIN_VIERTEL (1UL<<4)
#define PIN_VOR 	(1UL<<5)
#define PIN_FUNK 	(1UL<<6)
#define PIN_NACH 	(1UL<<7)
#define PIN_HALB 	(1UL<<8)
#define PIN_EL 		(1UL<<9)
#define PIN_F 		(1UL<<10)
#define PIN_UENF 	(1UL<<11)
#define PIN_EIN 	(1UL<<12)
#define PIN_S 		(1UL<<13)
#define PIN_ZWEI 	(1UL<<14)
#define PIN_DREI 	(1UL<<15)
#define PIN_VIER 	(1UL<<16)
#define PIN_SECHS 	(1UL<<17)
#define PIN_ACHT 	(1UL<<18)
#define PIN_SIEBEN 	(1UL<<19)
#define PIN_ZWOELF 	(1UL<<20)
#define PIN_ZEH 	(1UL<<21)
#define PIN_N 		(1UL<<22)
#define PIN_EUN 	(1UL<<23)
#define PIN_UHR 	(1UL<<24)

#define PIN_ELF 	(PIN_EL | PIN_F)
#define PIN_FUENF_2 (PIN_F | PIN_UENF)
#define PIN_EINS 	(PIN_EIN | PIN_S)
#define PIN_ZEHN_2 	(PIN_ZEH | PIN_N)
#define PIN_NEUN 	(PIN_N | PIN_EUN)