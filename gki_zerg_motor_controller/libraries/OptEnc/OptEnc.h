#include <Arduino.h>
#ifndef __OPTENC__
#define __OPTENC__
class OptEnc {
  public:
    //Variables
    
    //Pins that are controlled
    // selects which byte to read (0,1 = msb; 1,1 = 2nd; 0,0 = 3rd; 1,0 = lsb)
    int sel1pin;
    int sel2pin;
    // selects front or back motor
    int selxypin;
    // select if reading or not  LOW ACTIVE
    int oepin;
    // resets counter if reading or not  LOW ACTIVE
    int resetpin;
    
    int databus[8];
    
    struct odometrie{
      long vr;
      long hr;
      long vl;
      long hl;
    };

    odometrie odo;
    //Functions
    
    //maps the pins
    void mapPins(int sel1pin, int sel2pin, int selxypin, int oepin, int resetpin);
    //maps datapins
    void mapData(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7);
    //read the data byte
    unsigned char readByte();
    //read all 4 bytes of the counter into an ulong
    long readCount(boolean oe);
    //resets
    void reset();
    void setupfrq();
    long readCount2(boolean oe);
    unsigned long readCount3(boolean oe);
    void getOdo();
};
#endif
