#pragma once

#include <Arduino.h>
#include <ch32yyxx_tim.h>
#include <queue.h>

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

class CH32VEncoder;

typedef void (*enc_isr_cb_t)(void*);

class CH32VEncoder {
public:
	/**
	 * @brief Construct a new CH32VEncoder object
	 */
	CH32VEncoder();
	~CH32VEncoder();
	void attachHalfQuad(uint32_t pinChannelA, uint32_t pinChannelB);
	void attachFullQuad(uint32_t pinChannelA, uint32_t pinChannelB);
	void attachSingleEdge(uint32_t pinChannelA, uint32_t pinChannelB);
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
	void attach(uint32_t pinChannelA, uint32_t pinChannelB, encType et);
	bool attached;
	bool reversed = false;
	TIM_TypeDef *tim;
	int64_t overflow = 0;
	QueueHandle_t queue = nullptr;
};

//Added by Sloeber
#pragma once
