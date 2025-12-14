/*
 * CH32VEncoder.cpp
 *
 *  Created on: Dec 14, 2025
 *      Author: Ilya Mikhaltsou
 */

#include <CH32VEncoder.h>
#include <Arduino.h>
#include <queue.h>

puType CH32VEncoder::useInternalWeakPullResistors = puType::none;

static QueueHandle_t timQueues[4] = {0, 0, 0, 0};

CH32VEncoder::CH32VEncoder():
	attached{false},
	tim{TIM1},
	channelA{0},
	channelB{0}
{
}

CH32VEncoder::~CH32VEncoder() {
	detach();
}

void CH32VEncoder::detach(){
	attached = false;
	while (1) {
		// NOT IMPLEMENTED
	}
}

void CH32VEncoder::attach(TIM_TypeDef *tim, int pinChannelA, int pinChannelB, encType et) {
	if (attached) {
		return;
	}

	// store timer instance
	this->tim = tim;
	
	// Configure GPIOs as inputs (use Arduino-compatible helpers if available)
	if(useInternalWeakPullResistors == puType::down){
		pinMode(pinChannelA, INPUT_PULLUP);
		pinMode(pinChannelB, INPUT_PULLUP);
	} else if(useInternalWeakPullResistors == puType::up){
		pinMode(pinChannelA, INPUT_PULLDOWN);
		pinMode(pinChannelB, INPUT_PULLDOWN);
	} else {
		pinMode(pinChannelA, INPUT);
		pinMode(pinChannelB, INPUT);
	}

	// Enable timer peripheral clock and pick IRQ based on timer instance
	uint32_t rcc_periph = 0;
	IRQn_Type irq = TIM1_UP_IRQn;
	if (tim == TIM1) {
		rcc_periph = RCC_APB2Periph_TIM1;
		irq = TIM1_UP_IRQn;
		if (timQueues[0] == nullptr) {
    		timQueues[0] = xQueueCreate(10, sizeof(int));
			queue = timQueues[0];
		}
	} else if (tim == TIM2) {
		rcc_periph = RCC_APB1Periph_TIM2;
		irq = TIM2_IRQn;
		if (timQueues[1] == nullptr) {
    		timQueues[1] = xQueueCreate(10, sizeof(int));
			queue = timQueues[1];
		}
	} else if (tim == TIM3) {
		rcc_periph = RCC_APB1Periph_TIM3;
		irq = TIM3_IRQn;
		if (timQueues[2] == nullptr) {
    		timQueues[2] = xQueueCreate(10, sizeof(int));
			queue = timQueues[2];
		}
	} else if (tim == TIM4) {
		rcc_periph = RCC_APB1Periph_TIM4;
		irq = TIM4_IRQn;
		if (timQueues[3] == nullptr) {
    		timQueues[3] = xQueueCreate(10, sizeof(int));
			queue = timQueues[3];
		}
	} else {
		// unknown timer; fallback to TIM1
		rcc_periph = RCC_APB2Periph_TIM1;
		irq = TIM1_UP_IRQn;
	}
	if (rcc_periph) {
		if (rcc_periph & RCC_APB2Periph_TIM1) {
			RCC_APB2PeriphClockCmd(rcc_periph, ENABLE);
		} else {
			RCC_APB1PeriphClockCmd(rcc_periph, ENABLE);
		}
	}

	// De-init and basic setup
	TIM_DeInit(tim);

	// Configure encoder input capture channels (TI1 and TI2)

	switch (et) {
	case encType::single:
	case encType::half:
		TIM_EncoderInterfaceConfig(tim, TIM_EncoderMode_TI1,
								   TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
		break;
	case encType::full:
		TIM_EncoderInterfaceConfig(tim, TIM_EncoderMode_TI12,
								   TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
		break;
	}

	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0b0011 ; // Fsampling=f=Fck_int, N=8
	TIM_ICInit(tim, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInit(tim, &TIM_ICInitStructure);

	// 16-bit counter full range and clear counter
	TIM_SetAutoreload(tim, 65535);
	TIM_SetCounter(tim, 65536 / 2);

	// Enable update interrupt so overruns/updates can be handled in ISR
	TIM_ITConfig(tim, TIM_IT_Update, ENABLE);

	NVIC_EnableIRQ(irq);
	NVIC_SetPriority(irq, 0x20);

	// start timer counting
	TIM_Cmd(tim, ENABLE);

	attached = true;
}

void CH32VEncoder::attachHalfQuad(TIM_TypeDef *tim, int aPinNumber, int bPinNumber) {
	attach(tim, aPinNumber, bPinNumber, encType::half);
}

void CH32VEncoder::attachSingleEdge(TIM_TypeDef *tim, int aPinNumber, int bPinNumber) {
	attach(tim, aPinNumber, bPinNumber, encType::single);
}

void CH32VEncoder::attachFullQuad(TIM_TypeDef *tim, int aPinNumber, int bPinNumber) {
	attach(tim, aPinNumber, bPinNumber, encType::full);
}

void CH32VEncoder::setCount(int64_t value) {
	overflow = value;
	xQueueReset(queue);
	TIM_SetCounter(tim, 0);
}

int64_t CH32VEncoder::getCount() {
    int pcnt_count = 0;
	while (xQueueReceive(queue, &pcnt_count, 0) == pdTRUE) {
		// Append overflowed steps to the counter
		overflow += pcnt_count;
	}
	pcnt_count = TIM_GetCounter(tim);
	return pcnt_count + overflow;
}

void CH32VEncoder::clearCount() {
	overflow = 0;
	xQueueReset(queue);
	TIM_SetCounter(tim, 0);
}

void CH32VEncoder::pauseCount() {
	TIM_Cmd(tim, DISABLE);
}

void CH32VEncoder::resumeCount() {
	TIM_Cmd(tim, ENABLE);
}

void CH32VEncoder::setFilter(uint16_t value) {
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = value;
	TIM_ICInit(tim, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(tim, &TIM_ICInitStructure);
}

extern "C" {

#if defined(TIM1_BASE)
ISR void TIM1_UP_IRQHandler()
{
	BaseType_t shouldYield = pdFALSE;

	if (timQueues[0] != nullptr) {
		if (TIM_GetCounter(TIM1) >= 65536 / 2) {
			// Underflow
			xQueueSendFromISR(timQueues[0], (void *) (-65536), &shouldYield);
		} else {
			// Overflow
			xQueueSendFromISR(timQueues[0], (void *) (65536), &shouldYield);
		}
	}
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);

	portYIELD_FROM_ISR(shouldYield);
}
#endif //TIM1_BASE

#if defined(TIM2_BASE)
ISR void TIM2_IRQHandler()
{
	BaseType_t shouldYield = pdFALSE;

	if (timQueues[1] != nullptr) {
		if (TIM_GetCounter(TIM2) > 0x8FFF) {
			// Underflow
			xQueueSendFromISR(timQueues[1], (void *) (-65536), &shouldYield);
		} else {
			// Overflow
			xQueueSendFromISR(timQueues[1], (void *) (65536), &shouldYield);
		}
	}
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	portYIELD_FROM_ISR(shouldYield);
}

#endif //TIM2_BASE

#if defined(TIM3_BASE)
ISR void TIM3_IRQHandler()
{
	BaseType_t shouldYield = pdFALSE;

	if (timQueues[2] != nullptr) {
		if (TIM_GetCounter(TIM3) > 0x8FFF) {
			// Underflow
			xQueueSendFromISR(timQueues[2], (void *) (-65536), &shouldYield);
		} else {
			// Overflow
			xQueueSendFromISR(timQueues[2], (void *) (65536), &shouldYield);
		}
	}
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);

	portYIELD_FROM_ISR(shouldYield);
}
#endif //TIM3_BASE

#if defined(TIM4_BASE)
ISR void TIM4_IRQHandler()
{
	BaseType_t shouldYield = pdFALSE;

	if (timQueues[3] != nullptr) {
		if (TIM_GetCounter(TIM4) > 0x8FFF) {
			// Underflow
			xQueueSendFromISR(timQueues[3], (void *) (-65536), &shouldYield);
		} else {
			// Overflow
			xQueueSendFromISR(timQueues[3], (void *) (65536), &shouldYield);
		}
	}
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);

	portYIELD_FROM_ISR(shouldYield);
}
#endif //TIM4_BASE

}
