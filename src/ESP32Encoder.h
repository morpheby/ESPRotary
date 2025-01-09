#pragma once
#include <driver/gpio.h>
#include <driver/pulse_cnt.h>
#ifndef ARDUINO
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portable.h>
#include <freertos/semphr.h>
#endif

#define 	_INT16_MAX 32766
#define  	_INT16_MIN -32766

enum class encType {
	single,
	half,
	full
};

enum class puType {
	up,
	down,
	none
};

class ESP32Encoder;

typedef void (*enc_isr_cb_t)(void*);

class ESP32Encoder {
public:
	/**
	 * @brief Construct a new ESP32Encoder object
	 */
	ESP32Encoder();
	~ESP32Encoder();
	void attachHalfQuad(int aPinNumber, int bPinNumber);
	void attachFullQuad(int aPinNumber, int bPinNumber);
	void attachSingleEdge(int aPinNumber, int bPinNumber);
	int64_t getCount();
	void clearCount();
	void pauseCount();
	void resumeCount();
	void detach();
	bool isAttached(){return attached;}
	void setCount(int64_t value);
	void setFilter(uint16_t value);
	static puType useInternalWeakPullResistors;
private:
	void *queue;
	void attach(int aPinNumber, int bPinNumber, encType et);
	bool attached;
	gpio_num_t aPinNumber;
	gpio_num_t bPinNumber;
	pcnt_unit_handle_t unit;
	int64_t overflow = 0;
};

//Added by Sloeber
#pragma once
