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
#include <PeripheralPins.h>
#include <pinmap.h>

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
static CH32VEncoder *timOwners[4] = {};

#ifdef TIM_MODULE_ENABLED
static bool matchesPin(PinName requested, PinName mapped) {
	if (requested == NC) {
		return false;
	}
	if (static_cast<uint32_t>(requested) & ALTX_MASK) {
		return requested == mapped;
	}
	return (static_cast<uint32_t>(requested) & ~ALTX_MASK)
		== (static_cast<uint32_t>(mapped) & ~ALTX_MASK);
}
#endif

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

	for (auto &owner : timOwners) {
		if (owner == this) {
			owner = nullptr;
		}
	}
	attached = false;
}

void CH32VEncoder::attach(uint32_t pinChannelA, uint32_t pinChannelB, encType et) {
	if (attached) {
		detach();
	}

#ifndef TIM_MODULE_ENABLED
	_Error_Handler("Encoder requires TIM_MODULE_ENABLED", 0);
	return;
#else
	const PinName pinA = digitalPinToPinName(pinChannelA);
	const PinName pinB = digitalPinToPinName(pinChannelB);
	const PinMap *mapA = nullptr;
	const PinMap *mapB = nullptr;
	for (const PinMap *a = PinMap_TIM; a->pin != NC && !mapA; ++a) {
		if (!matchesPin(pinA, a->pin) || CH_PIN_INVERTED(a->function)) {
			continue;
		}
		const uint32_t channelA = CH_PIN_CHANNEL(a->function);
		if (channelA != 1 && channelA != 2) {
			continue;
		}
		for (const PinMap *b = PinMap_TIM; b->pin != NC; ++b) {
			if (matchesPin(pinB, b->pin) && a->peripheral == b->peripheral
				&& !CH_PIN_INVERTED(b->function)
				&& CH_PIN_AFNUM(a->function) == CH_PIN_AFNUM(b->function)
				&& CH_PIN_CHANNEL(b->function) == 3 - channelA) {
				mapA = a;
				mapB = b;
				break;
			}
		}
	}
	if (!mapA) {
		_Error_Handler("Encoder pins require matching TIM CH1/CH2 routing", 0);
		return;
	}

	TIM_TypeDef *tim = static_cast<TIM_TypeDef *>(mapA->peripheral);
	int timerIndex = -1;
	if (tim == TIM1) timerIndex = 0;
	else if (tim == TIM2) timerIndex = 1;
	else if (tim == TIM3) timerIndex = 2;
	else if (tim == TIM4) timerIndex = 3;
	if (timerIndex < 0 || timOwners[timerIndex]) {
		_Error_Handler("Encoder timer unsupported or already in use", timerIndex);
		return;
	}
	if (!timQueues[timerIndex]) {
		timQueues[timerIndex] = xQueueCreate(10, sizeof(int));
	}
	if (!timQueues[timerIndex]) {
		_Error_Handler("Encoder queue allocation failed", timerIndex);
		return;
	}
	queue = timQueues[timerIndex];
	xQueueReset(queue);
	this->tim = tim;
	reversed = CH_PIN_CHANNEL(mapA->function) == 2;

	// Apply the board's AFIO routing without briefly driving the encoder pins.
	for (const PinMap *map : {mapA, mapB}) {
		pin_function(map->pin, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_FLOAT,
			NOPULL, CH_PIN_AFNUM(map->function)));
	}
	const uint32_t mode = useInternalWeakPullResistors == puType::up ? INPUT_PULLUP
		: useInternalWeakPullResistors == puType::down ? INPUT_PULLDOWN : INPUT;
	pinMode(pinChannelA, mode);
	pinMode(pinChannelB, mode);

	// Enable timer peripheral clock and pick IRQ based on timer instance
	uint32_t rcc_periph = 0;
	IRQn_Type irq = TIM1_UP_IRQn;
	if (tim == TIM1) {
		rcc_periph = RCC_APB2Periph_TIM1;
		irq = TIM1_UP_IRQn;
	} else if (tim == TIM2) {
		rcc_periph = RCC_APB1Periph_TIM2;
		irq = TIM2_IRQn;
	} else if (tim == TIM3) {
		rcc_periph = RCC_APB1Periph_TIM3;
		irq = TIM3_IRQn;
	} else if (tim == TIM4) {
		rcc_periph = RCC_APB1Periph_TIM4;
		irq = TIM4_IRQn;
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
		TIM_EncoderInterfaceConfig(tim, reversed ? TIM_EncoderMode_TI2 : TIM_EncoderMode_TI1,
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
	NVIC_SetPriority(irq, 0xA0);

	// start timer counting
	TIM_Cmd(tim, ENABLE);

	attached = true;
	timOwners[timerIndex] = this;
#endif
}

void CH32VEncoder::attachHalfQuad(uint32_t aPinNumber, uint32_t bPinNumber) {
	attach(aPinNumber, bPinNumber, encType::half);
}

void CH32VEncoder::attachSingleEdge(uint32_t aPinNumber, uint32_t bPinNumber) {
	attach(aPinNumber, bPinNumber, encType::single);
}

void CH32VEncoder::attachFullQuad(uint32_t aPinNumber, uint32_t bPinNumber) {
	attach(aPinNumber, bPinNumber, encType::full);
}

void CH32VEncoder::setCount(int64_t value) {
	overflow = (reversed ? -value : value) - COUNTER_MAX / 2;
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
	const int64_t count = pcnt_count + overflow;
	return reversed ? -count : count;
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
