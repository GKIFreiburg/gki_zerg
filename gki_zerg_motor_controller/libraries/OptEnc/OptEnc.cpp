#include "OptEnc.h"

void OptEnc::mapPins(int sel1pin, int sel2pin, int selxypin, int oepin, int resetpin) {
  this->sel1pin  = sel1pin;
  this->sel2pin  = sel2pin;
  this->selxypin  = selxypin;
  this->oepin  = oepin;
  this->resetpin  = resetpin;
}

void OptEnc::mapData(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7) {
  this->databus[0]  = d0;
  this->databus[1]  = d1;
  this->databus[2]  = d2;
  this->databus[3]  = d3;
  this->databus[4]  = d4;
  this->databus[5]  = d5;
  this->databus[6]  = d6;
  this->databus[7]  = d7;
}

unsigned char OptEnc::readByte()
{
  unsigned char data = 0;
  for (int i = 0; i < 8; ++i) {
    data |= (digitalRead(databus[i])) << i;
  }
  return data;
}



void OptEnc::reset() {
  digitalWrite(resetpin, LOW);
  digitalWrite(resetpin, HIGH);
}

void OptEnc::setupfrq() {
  // Set Timer 2 CTC mode with no prescaling.  OC2A toggles on compare match
    //
    // WGM22:0 = 010: CTC Mode, toggle OC 
    // WGM2 bits 1 and 0 are in TCCR2A,
    // WGM2 bit 2 is in TCCR2B
    // COM2A0 sets OC2A (arduino pin 11 on Uno or Duemilanove) to toggle on compare match
    //
    TCCR2A = ((1 << WGM21) | (1 << COM2A0));

    // Set Timer 2  No prescaling  (i.e. prescale division = 1)
    //
    // CS22:0 = 001: Use CPU clock with no prescaling
    // CS2 bits 2:0 are all in TCCR2B
    TCCR2B = (1 << CS20);

    // Make sure Compare-match register A interrupt for timer2 is disabled
    TIMSK2 = 0;
    // This value determines the output frequency
    OCR2A = 1;
}


long OptEnc::readCount(boolean oe)
{
  digitalWrite(oepin, oe);
  long out = 0;
  for (int j = 0; j < 4; j++) {
    if (j / 2 > 0) digitalWrite(sel2pin, HIGH);
    else digitalWrite(sel2pin, LOW);
    if (j % 2 > 0) digitalWrite(sel1pin, LOW);
    else digitalWrite(sel1pin, HIGH);
    for (int i = 0; i < 8; ++i) {
      if (digitalRead(databus[i])) out += 1L<<(i+8*j);
    }
  }
  digitalWrite(oepin, !oe);
  return out;
}

void OptEnc::getOdo() {
  digitalWrite(selxypin, HIGH);
  odo.hr = -1 * readCount(false);
  odo.hl = readCount(true);
  digitalWrite(selxypin, LOW);
  odo.vr = -1 * readCount(false);
  odo.vl = readCount(true);
}

