/*
 * CH32VEncoder.cpp
 *
 *  Created on: Dec 14, 2025
 *      Author: Ilya Mikhaltsou
 */

#include "ch32v30x_isr.h"
#include "core_config.h"
#include <CH32VEncoder.h>
#include <ch32vxxx/ch32vxxx_isr.h>
#include <Arduino.h>
#include <queue.h>

#define COUNTER_MAX 65536

#ifdef TIM_MODULE_OPTIONAL
#define _ENCODER_ISR(x) _REMAP_ISR(x, CH32VEnc_ISR)
#else
#define _ENCODER_ISR(x) _ISR_DEF(x)
#endif

_ENCODER_ISR(TIM1_UP_IRQHandler);
_ENCODER_ISR(TIM2_IRQHandler);
_ENCODER_ISR(TIM3_IRQHandler);
_ENCODER_ISR(TIM4_IRQHandler);

puType CH32VEncoder::useInternalWeakPullResistors = puType::none;

static QueueHandle_t timQueues[4] = {0, 0, 0, 0};

CH32VEncoder::CH32VEncoder():
	attached{false},
	tim{TIM1}
{
}

CH32VEncoder::~CH32VEncoder() {
	detach();
}

void CH32VEncoder::detach(){
	if (!attached) {
		return;
	}

	TIM_Cmd(tim, DISABLE);
	TIM_ITConfig(tim, TIM_IT_Update, DISABLE);

	if (tim == TIM1) {
		NVIC_DisableIRQ(TIM1_UP_IRQn);
	} else if (tim == TIM2) {
		NVIC_DisableIRQ(TIM2_IRQn);
	} else if (tim == TIM3) {
		NVIC_DisableIRQ(TIM3_IRQn);
	} else if (tim == TIM4) {
		NVIC_DisableIRQ(TIM4_IRQn);
	}

	TIM_DeInit(tim);

	attached = false;
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
	
	if (tim == TIM1) {
		queue = timQueues[0];
	} else if (tim == TIM2) {
		queue = timQueues[1];
	} else if (tim == TIM3) {
		queue = timQueues[2];
	} else if (tim == TIM4) {
		queue = timQueues[3];
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
	TIM_SetAutoreload(tim, COUNTER_MAX - 1);
	TIM_SetCounter(tim, COUNTER_MAX / 2);

	overflow = -COUNTER_MAX / 2;

#ifdef TIM_MODULE_OPTIONAL
	// Set ISR handlers
	if (tim == TIM1) {
		ISR_Set_TIM1_UP_IRQHandler(_REMAP_ISR_NAME(TIM1_UP_IRQHandler, CH32VEnc_ISR));
	} else if (tim == TIM2) {
		ISR_Set_TIM2_IRQHandler(_REMAP_ISR_NAME(TIM2_IRQHandler, CH32VEnc_ISR));
	} else if (tim == TIM3) {
		ISR_Set_TIM3_IRQHandler(_REMAP_ISR_NAME(TIM3_IRQHandler, CH32VEnc_ISR));
	} else if (tim == TIM4) {
		ISR_Set_TIM4_IRQHandler(_REMAP_ISR_NAME(TIM4_IRQHandler, CH32VEnc_ISR));
	}
#endif

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
	overflow = value - COUNTER_MAX / 2;
	xQueueReset(queue);
	TIM_SetCounter(tim, COUNTER_MAX / 2);
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
	overflow = -COUNTER_MAX / 2;
	xQueueReset(queue);
	TIM_SetCounter(tim, COUNTER_MAX / 2);
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
_ENCODER_ISR(TIM1_UP_IRQHandler)
{
	static int i;
	BaseType_t shouldYield = pdFALSE;

	if (timQueues[0] != nullptr) {
		if (TIM_GetCounter(TIM1) >= COUNTER_MAX / 2) {
			// Underflow
			i = -COUNTER_MAX;
		} else {
			// Overflow
			i = COUNTER_MAX;
		}
		if (xQueueIsQueueFullFromISR(timQueues[0]) == pdFALSE) {
			xQueueSendFromISR(timQueues[0], &i, &shouldYield);
		} else {
			// Queue full, update counter to max value
			TIM_SetCounter(TIM1, i > 0 ? COUNTER_MAX - 1 : 0);
		}
	}
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);

	portYIELD_FROM_ISR(shouldYield);
}
#endif //TIM1_BASE

#if defined(TIM2_BASE)
_ENCODER_ISR(TIM2_IRQHandler)
{
	static int i;
	BaseType_t shouldYield = pdFALSE;

	if (timQueues[1] != nullptr) {
		if (TIM_GetCounter(TIM2) >= COUNTER_MAX / 2) {
			// Underflow
			i = -COUNTER_MAX;
		} else {
			// Overflow
			i = COUNTER_MAX;
		}
		if (xQueueIsQueueFullFromISR(timQueues[1]) == pdFALSE) {
			xQueueSendFromISR(timQueues[1], &i, &shouldYield);
		} else {
			// Queue full, update counter to max value
			TIM_SetCounter(TIM2, i > 0 ? COUNTER_MAX - 1 : 0);
		}
	}
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	portYIELD_FROM_ISR(shouldYield);
}

#endif //TIM2_BASE

#if defined(TIM3_BASE)
_ENCODER_ISR(TIM3_IRQHandler)
{
	static int i;
	BaseType_t shouldYield = pdFALSE;

	if (timQueues[2] != nullptr) {
		if (TIM_GetCounter(TIM3) >= COUNTER_MAX / 2) {
			// Underflow
			i = -COUNTER_MAX;
		} else {
			// Overflow
			i = COUNTER_MAX;
		}
		if (xQueueIsQueueFullFromISR(timQueues[2]) == pdFALSE) {
			xQueueSendFromISR(timQueues[2], &i, &shouldYield);
		} else {
			// Queue full, update counter to max value
			TIM_SetCounter(TIM3, i > 0 ? COUNTER_MAX - 1 : 0);
		}
	}
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);

	portYIELD_FROM_ISR(shouldYield);
}
#endif //TIM3_BASE

#if defined(TIM4_BASE)
_ENCODER_ISR(TIM4_IRQHandler)
{
	static int i;
	BaseType_t shouldYield = pdFALSE;

	if (timQueues[3] != nullptr) {
		if (TIM_GetCounter(TIM4) >= COUNTER_MAX / 2) {
			// Underflow
			i = -COUNTER_MAX;
		} else {
			// Overflow
			i = COUNTER_MAX;
		}
		if (xQueueIsQueueFullFromISR(timQueues[3]) == pdFALSE) {
			xQueueSendFromISR(timQueues[3], &i, &shouldYield);
		} else {
			// Queue full, update counter to max value
			TIM_SetCounter(TIM4, i > 0 ? COUNTER_MAX - 1 : 0);
		}
	}
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);

	portYIELD_FROM_ISR(shouldYield);
}
#endif //TIM4_BASE

}
