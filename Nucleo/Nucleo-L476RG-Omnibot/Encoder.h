#ifndef ENCODER_H
#define ENCODER_H

class Encoder {
public:
  virtual int get_position()=0;
  virtual long int get_time_micros()=0;
};

class Encoder1 : public Encoder {
public:
  Encoder1();
  int get_position();
  long int get_time_micros();
};

class Encoder2 : public Encoder {
public:
  Encoder2();
  int get_position();
  long int get_time_micros();
};

class Encoder3 : public Encoder {
public:
  Encoder3();
  int get_position();
  long int get_time_micros();
};

#endif
