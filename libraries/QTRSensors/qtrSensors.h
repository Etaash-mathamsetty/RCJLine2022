#include <stdlib.h>
#include <stdio.h>
#include <alloca.h>
#include "Arduino.h"

class QTRSensors {
public:
	QTRSensors(const uint8_t* pins, int _length, uint8_t _emitter_pin, int _max_time = 2500) {
		this->QtrPins = (uint8_t*)calloc(_length, sizeof(uint8_t));
		for (int i = 0; i < _length; i++) {
			*(QtrPins + i) = *(pins + i);
			pinMode(*(pins + i), OUTPUT);
		}
		pinMode(_emitter_pin, OUTPUT);
		digitalWrite(_emitter_pin, HIGH);
		this->emitter_pin = _emitter_pin;
		this->QtrValues = (int*)calloc(_length, sizeof(int));
		this->OffValues = (int*)calloc(_length, sizeof(int));
		this->max_time = _max_time;
		this->length = _length;
	}

	void calibrate(void (*sig_func )(bool)) {
		this->Update();
		//run function before calibration (true = black, false = white)
		(*sig_func)(true);
		delay(10000);
		int black_avg = 0;
		int* black_values = alloca(sizeof(int) * length);
		for(int i = 0; i < length; i++){
			black_avg += get_val(i);
			black_values[i] = get_val(i);
		}
		black_avg /= length;
		(*sig_func)(false);
		delay(10000);
		int white_avg = 0;
		int* white_values = alloca(sizeof(int) * length);
		for(int i = 0; i < length; i++){
			white_avg += get_val(i);
			white_values[i] = get_val(i);
		}
		white_avg /= length;
		for(int i = 0; i < length; i++){
			OffValues[i] = (black_avg + white_avg)/2;
			OffValues[i] -= (black_values[i] + white_values[i])/2;
		}
	}

	~QTRSensors() {
		free(this->QtrValues);
		free(this->QtrPins);
		free(this->OffValues);
	}

	void Update() {
		int startTime = 0;
		int time = 0;

		for (int i = 0; i < length; i++)
			*(this->QtrValues + i) = this->max_time;
		for (int i = 0; i < this->length; i++) {
			digitalWrite(*(QtrPins + i), HIGH);
			pinMode(*(QtrPins + i), OUTPUT);
		}

		delayMicroseconds(10);

		for (int i = 0; i < this->length; i++) {
			pinMode(*(QtrPins + i), INPUT);
			digitalWrite(*(QtrPins + i), LOW);
		}

		time = 0;
		startTime = micros();
		while (time < this->max_time) {
			time = micros() - startTime;
			for (int i = 0; i < this->length; i++) {
				if ((digitalRead(*(QtrPins + i)) == LOW) && time < *(this->QtrValues + i)) {
					*(QtrValues + i) = time;
				}
			}
		}
	}

	int get_val(int index) {
		return *(QtrValues + index) + OffValues[index];
	}
	
	

	int operator[](int index){
		return get_val(index);
	}
	
	uint32_t get_line() {
		uint32_t avg = 0;
		uint32_t sum = 0;
		for (int i = 0; i < this->length; i++) {
			uint32_t value = this->get_val(i);
			if (value > 50) {
				avg += value * (i * 1000);
				sum += value;
			}
		}
		return (uint32_t)(avg / sum);
	}
private:
	int* QtrValues;
	int* OffValues; // add calibration function later
	uint8_t emitter_pin;
	uint8_t* QtrPins;
	int max_time;
	int length;
};
