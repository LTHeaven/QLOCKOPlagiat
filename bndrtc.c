#include "bndrtc.h"

#include <stdint.h>

#include <avr/interrupt.h>

#include <avr/sleep.h>

#include <util/parity.h>

uint8_t spiBuf;
uint8_t displayBuf[4];

uint8_t dcfStat;
uint8_t dcfCounter;
uint8_t dcfPhase;
uint16_t dcfBuffer;
uint8_t dcfMin;
uint8_t dcfHr;
uint8_t rtcTimeBuffer[3];

uint32_t completeDisplay;

void updateDisplayBufForTime();

// Perform SPI transfer of displayBuf and pulse RCLK
void updateDisplay(void) {
  SPI_SELECT_SREG;
  USIDR = displayBuf[3];
  USISR = _BV(USIOIF);
  do {
    USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
  } while (!(USISR & _BV(USIOIF)));

  USIDR = displayBuf[2];
  USISR = _BV(USIOIF);
  do {
    USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
  } while (!(USISR & _BV(USIOIF)));

  USIDR = displayBuf[1];
  USISR = _BV(USIOIF);
  do {
    USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
  } while (!(USISR & _BV(USIOIF)));

  USIDR = displayBuf[0];
  USISR = _BV(USIOIF);
  do {
    USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
  } while (!(USISR & _BV(USIOIF)));

  PORTB |= (1 << PB_OUT_RCK);
  PORTB &= ~(1 << PB_OUT_RCK);
}

uint8_t SPITransferMst(uint8_t dat) {
  USIDR = dat;
  USISR = _BV(USIOIF);
  do {
    USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
  } while (!(USISR & _BV(USIOIF)));
  return USIDR;
}

void rtcReadRegs(uint8_t regAdr, uint8_t * buf, uint8_t len) {
  SPI_SELECT_RTC;
  regAdr &= 0x1F;
  regAdr |= 0xA0;
  SPITransferMst(regAdr);
  while (len != 0) {
    * buf = SPITransferMst(0);
    buf++;
    len--;
  }
  SPI_SELECT_SREG;
}

void rtcWriteRegs(uint8_t regAdr, uint8_t * buf, uint8_t len) {
  SPI_SELECT_RTC;
  regAdr &= 0x1F;
  regAdr |= 0x20;
  SPITransferMst(regAdr);
  while (len != 0) {
    SPITransferMst( * buf);
    buf++;
    len--;
  }
  SPI_SELECT_SREG;
}

void init(void) {
  SPI_SELECT_SREG;
  PORTB |= (1 << PB_OUT_OEN);
  DDRA = DDRA_INIT;
  DDRB = DDRB_INIT;

  MCUCR = 0;

  // Clear shift registers
  SPI_SELECT_RTC;
  SPI_SELECT_SREG;
  PORTB = (1 << PB_OUT_OEN) | (1 << PB_OUT_RCK);
  PORTB = (1 << PB_OUT_OEN);

  // ADC Init
  ADMUX = 0; // Select PA0
  ADCSRB = _BV(ADLAR); // Left Adjust
  DIDR0 = _BV(ADC0D); // Disable PA0 digital buffer

  // Set prescaler to 2 => CPU_CLK = 4 MHz
  CLKPR = _BV(CLKPCE);
  CLKPR = _BV(CLKPS0);
}

void initRTC(void) {
  _delay_ms(2000); // Wait for oscillator start-up

  // Perform refresh and set CLK_EXT to 16384 Hz
  spiBuf = 0b00000001;
  rtcWriteRegs(RTC_REG_CKO, & spiBuf, 1);
  spiBuf = 0b00100001;
  rtcWriteRegs(RTC_REG_CKO, & spiBuf, 1);
  _delay_ms(120);
  rtcReadRegs(RTC_REG_SEC, rtcTimeBuffer, 3);
}

// Set PWM rate depnding on brighntess measurement
void updatePWMRate(uint8_t adcVal) {
  if (adcVal < 0x10) {
    // High brightness, LEDs full on
    PWM_DISABLE;
    return;
  }

  if (adcVal > 0xF0)
    OCR0A = 0xF0; // Low brightness, clamp at 16%
  else
    OCR0A = adcVal;
  PWM_ENABLE;

}

void resetDCF() {
  dcfStat = DCF_STAT_FIND_SIGNAL;
  dcfCounter = 0;
  dcfBuffer = 0;
  dcfMin = 0xFF;
  dcfHr = 0xFF;
}

void updateDCF() {
  uint8_t dcfBit;
  switch (dcfStat) {
  case DCF_STAT_WAIT_FOR_SYNC:
    if (dcfBuffer == 0) {
      dcfCounter = 0;
      dcfStat = DCF_STAT_RECEIVING;
    } else
    if (dcfCounter == 0)
      dcfStat = DCF_STAT_FIND_SIGNAL;
    else
      dcfCounter--;
    return;
  case DCF_STAT_RECEIVING:
    if (dcfCounter == 59) {
      dcfStat = DCF_STAT_SUCCESS;
      //write time to RTC
      rtcTimeBuffer[0] = 0x00;
      rtcTimeBuffer[1] = dcfMin;
      rtcTimeBuffer[2] = dcfHr;
      rtcWriteRegs(RTC_REG_SEC, rtcTimeBuffer, 3);
      return;
    }

    if ((dcfBuffer & 0xFFF0) != 0 || (dcfBuffer & 3) != 3) {
      resetDCF();
      return;
    }
    dcfBit = (uint8_t) dcfBuffer & 4;
    if (dcfCounter > 19) {
      if (dcfCounter == 20 && !dcfBit) {
        resetDCF();
        return;
      }

      if (dcfCounter > 20 && dcfCounter < 29) {
        dcfMin >>= 1;
        if (dcfBit)
          dcfMin |= 0x80;
        if (dcfCounter == 28) {
          if (parity_even_bit(dcfMin)) {
            resetDCF();
            return;
          }
          dcfMin &= 0x7f;
        }
      }
      if (dcfCounter > 28 && dcfCounter < 36) {
        dcfHr >>= 1;
        if (dcfBit)
          dcfHr |= 0x80;
        if (dcfCounter == 35) {
          dcfHr >>= 1;
          if (parity_even_bit(dcfHr)) {
            resetDCF();
            return;
          }
          dcfHr &= 0x3f;
        }
      }
    } else if (dcfCounter == 0 && dcfBit) {
      resetDCF();
      return;
    }
    dcfCounter++;
    return;
  default:
    return;
  }

}

// Timer0 Overflow: Triggered at 64 Hz ( = CLK_EXT / 256)
ISR(TIM0_OVF_vect) {
  updateDisplayBufForTime();
  updateDisplay();
  //rtcReadRegs(RTC_REG_SEC, displayBuf, 2); // Fetch time from RTC
  if (ADCSRA & _BV(ADIF))
    updatePWMRate(ADCH);

  if ((dcfPhase & 0x03) == 0) {
    // 16 Hz

    dcfBuffer >>= 1;
    if (PIN_DCF)
      dcfBuffer += 0x8000;

    if ((dcfPhase & 0x3F) == 0) {

      //displayBuf[2] = dcfStat;
      //displayBuf[3] = dcfMin;
      // 1 Hz
      if (dcfStat == DCF_STAT_FIND_SIGNAL || dcfStat == DCF_STAT_CHECK_PHASE) {
        if ((dcfBuffer & 3) != 3 || (dcfBuffer >> 8) != 0) {
          dcfPhase++;
          dcfCounter = 0;
          dcfStat = DCF_STAT_FIND_SIGNAL;
        } else {
          dcfStat = DCF_STAT_CHECK_PHASE;
          dcfCounter++;
          if (dcfCounter == 12) {
            dcfStat = DCF_STAT_WAIT_FOR_SYNC;
            dcfCounter = 60;
          }
        }
      } else {
        updateDCF();
      }

    }
    rtcReadRegs(RTC_REG_SEC, rtcTimeBuffer, 3);
    //daily time sync
    if (dcfStat == DCF_STAT_SUCCESS && rtcTimeBuffer[1] == 0 && rtcTimeBuffer[2] == 3) {
      resetDCF();
    }
  }
  dcfPhase++;
}

void incrementHour(uint8_t * hour) {
  * hour += 1;
  if ( * hour > 23) {
    * hour = 0;
  }
}

void updateDisplayBuffers() {
  displayBuf[0] = (uint8_t) completeDisplay;
  displayBuf[1] = (uint8_t)(completeDisplay >> 8);
  displayBuf[2] = (uint8_t)(completeDisplay >> 16);
  displayBuf[3] = (uint8_t)(completeDisplay >> 24);
}

void updateDisplayBufForTime() {
  completeDisplay = 0;

  if (dcfStat != DCF_STAT_SUCCESS) {
    if (dcfStat >= DCF_STAT_WAIT_FOR_SYNC) {
      completeDisplay |= PIN_FUNK;
    } else {
      if (dcfPhase & 0x20) {
        completeDisplay |= PIN_FUNK;
      }
    }
  }

  if (!(rtcTimeBuffer[0] & 0x80)) {
    uint8_t hour = (rtcTimeBuffer[2] % 16) + ((rtcTimeBuffer[2] / 16) * 10);
    uint8_t minutes = (rtcTimeBuffer[1] % 16) + ((rtcTimeBuffer[1] / 16) * 10);

    uint8_t displayHour = hour;
    completeDisplay |= PIN_ES_IST;
    if (minutes < 5) {
      completeDisplay |= PIN_UHR;
    } else if (minutes < 10) {
      completeDisplay |= PIN_FUENF_1 | PIN_NACH;
    } else if (minutes < 15) {
      completeDisplay |= PIN_ZEHN_1 | PIN_NACH;
    } else if (minutes < 20) {
      completeDisplay |= PIN_VIERTEL | PIN_NACH;
    } else if (minutes < 25) {
      completeDisplay |= PIN_ZWANZIG | PIN_NACH;
    } else if (minutes < 30) {
      completeDisplay |= PIN_FUENF_1 | PIN_VOR | PIN_HALB;
      incrementHour( & displayHour);
    } else if (minutes < 35) {
      completeDisplay |= PIN_HALB;
      incrementHour( & displayHour);
    } else if (minutes < 40) {
      completeDisplay |= PIN_FUENF_1 | PIN_NACH | PIN_HALB;
      incrementHour( & displayHour);
    } else if (minutes < 45) {
      completeDisplay |= PIN_ZWANZIG | PIN_VOR;
      incrementHour( & displayHour);
    } else if (minutes < 50) {
      completeDisplay |= PIN_VIERTEL | PIN_VOR;
      incrementHour( & displayHour);
    } else if (minutes < 55) {
      completeDisplay |= PIN_ZEHN_1 | PIN_VOR;
      incrementHour( & displayHour);
    } else if (minutes < 60) {
      completeDisplay |= PIN_FUENF_1 | PIN_VOR;
      incrementHour( & displayHour);
    }

    if (displayHour == 0 || displayHour == 12) {
      completeDisplay |= PIN_ZWOELF;
    } else if (displayHour == 1 || displayHour == 13) {
      if (minutes <= 5) {
        completeDisplay |= PIN_EIN;
      } else {
        completeDisplay |= PIN_EINS;
      }
    } else if (displayHour == 2 || displayHour == 14) {
      completeDisplay |= PIN_ZWEI;
    } else if (displayHour == 3 || displayHour == 15) {
      completeDisplay |= PIN_DREI;
    } else if (displayHour == 4 || displayHour == 16) {
      completeDisplay |= PIN_VIER;
    } else if (displayHour == 5 || displayHour == 17) {
      completeDisplay |= PIN_FUENF_2;
    } else if (displayHour == 6 || displayHour == 18) {
      completeDisplay |= PIN_SECHS;
    } else if (displayHour == 7 || displayHour == 19) {
      completeDisplay |= PIN_SIEBEN;
    } else if (displayHour == 8 || displayHour == 20) {
      completeDisplay |= PIN_ACHT;
    } else if (displayHour == 9 || displayHour == 21) {
      completeDisplay |= PIN_NEUN;
    } else if (displayHour == 10 || displayHour == 22) {
      completeDisplay |= PIN_ZEHN_2;
    } else if (displayHour == 11 || displayHour == 23) {
      completeDisplay |= PIN_ELF;
    }
  }
  updateDisplayBuffers();
}

int main(void) {
  cli();
  init();
  initRTC();

  displayBuf[0] = 0x00;
  displayBuf[1] = 0x00;
  displayBuf[2] = 0x00;
  displayBuf[3] = 0x00;
  PORTB = 0; // Enable shift register outputs

  dcfPhase = 0;
  resetDCF();

  // Setup Timer0 PWM + Interrupt generation
  PWM_DISABLE;
  TCCR0B = _BV(CS02) | _BV(CS01);
  sei();
  TIMSK0 = _BV(TOIE0);

  while (1) {
    sleep_disable();
    ADCSRA = _BV(ADEN) | _BV(ADIF) | _BV(ADPS1); // Perform AD conversion during sleep
    // Sleep until Timer0 Overflow
    sleep_enable();
    sleep_cpu();
  }

  return -1;
}