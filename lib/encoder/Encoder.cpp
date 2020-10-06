
#include "Encoder.h"

Encoder::Encoder(uint8_t pin1, uint8_t pin2, int counts_per_rev)
{

	counts_per_rev_ = counts_per_rev;
	// Attache pins for use as encoder pins
	encoder.attachHalfQuad(pin1, pin2);

	// clear the encoder's raw count and set the tracked count to zero
	encoder.clearCount();
}

int Encoder::getTicks() {
	return prev_encoder_ticks_;
}

int Encoder::getRPM()
{
	long encoder_ticks = encoder.getCount();
	//this function calculates the motor's RPM based on encoder ticks and delta time
	unsigned long current_time = millis();
	unsigned long dt = current_time - prev_update_time_;

	//convert the time from milliseconds to minutes
	double dtm = (double)dt / 60000;
	double delta_ticks = encoder_ticks - prev_encoder_ticks_;

	//Serial.printf("ticks %ld  prev ticks %ld delta_ticks %g\n", encoder_ticks, prev_encoder_ticks_, delta_ticks);

	//calculate wheel's speed (RPM)

	prev_update_time_ = current_time;
	prev_encoder_ticks_ = encoder_ticks;

	return (delta_ticks / counts_per_rev_) / dtm;
}