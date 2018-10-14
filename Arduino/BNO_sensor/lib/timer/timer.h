#ifndef __TIMER_H__
#define __TIMER_H__

#include "Arduino.h"

class Timer {
    word next;
    byte armed;
public:
    Timer () : next(0), armed (0) {}
    
    byte poll(word ms =0);
    word remaining() const;
    byte idle() const { return !armed; }
    void set(word ms);
};

#endif