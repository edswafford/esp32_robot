#ifndef Encoder_h_
#define Encoder_h_

#include <Arduino.h>
#include <ESP32Encoder.h>

class Encoder
{
public:
	Encoder(uint8_t pin1, uint8_t pin2, int counts_per_rev);

	int getRPM();
	int getTicks();

private:
	int counts_per_rev_;
	unsigned long prev_update_time_;
    long prev_encoder_ticks_;
	ESP32Encoder encoder;

};


#endif
