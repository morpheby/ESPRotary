/*
 * ESP32Encoder.cpp
 *
 *  Created on: Oct 15, 2018
 *      Author: hephaestus
 */

#include <ESP32Encoder.h>
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <rom/gpio.h>
#define delay(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#endif

#include <soc/soc_caps.h>
#if SOC_PCNT_SUPPORTED
// Not all esp32 chips support the pcnt (notably the esp32c3 does not)
#include <soc/pcnt_struct.h>
#include "esp_log.h"
#include "esp_ipc.h"
#if ( defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3) )
	#include <freertos/FreeRTOS.h>
	#include <rom/gpio.h>
#endif

static const char* TAG_ENCODER = "ESP32Encoder";

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
#define _ENTER_CRITICAL() portENTER_CRITICAL_SAFE(&spinlock)
#define _EXIT_CRITICAL() portEXIT_CRITICAL_SAFE(&spinlock)

puType ESP32Encoder::useInternalWeakPullResistors = puType::none;

ESP32Encoder::ESP32Encoder():
	aPinNumber{(gpio_num_t) 0},
	bPinNumber{(gpio_num_t) 0},
	unit{NULL},
	count{0},
	attached{false}
{
    queue = (void *) xQueueCreate(10, sizeof(int));
}

ESP32Encoder::~ESP32Encoder() {
	detach();
	vQueueDelete(reinterpret_cast<QueueHandle_t>(queue));
}

static bool esp32encoder_pcnt_event_handler(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
	QueueHandle_t queue = reinterpret_cast<QueueHandle_t>(user_ctx);
    BaseType_t high_task_wakeup;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

void ESP32Encoder::detach(){
	ESP_ERROR_CHECK(pcnt_unit_stop(unit));
	ESP_ERROR_CHECK(pcnt_unit_enable(unit));
	ESP_ERROR_CHECK(pcnt_del_unit(unit));
	unit = NULL;
	attached = false;
}

void ESP32Encoder::attach(int a, int b, encType et) {
	if (attached) {
		ESP_LOGE(TAG_ENCODER, "attach: already attached");
		return;
	}

	this->aPinNumber = (gpio_num_t) a;
	this->bPinNumber = (gpio_num_t) b;

	//Set up the IO state of hte pin
	gpio_pad_select_gpio(aPinNumber);
	gpio_pad_select_gpio(bPinNumber);
	gpio_set_direction(aPinNumber, GPIO_MODE_INPUT);
	gpio_set_direction(bPinNumber, GPIO_MODE_INPUT);
	if(useInternalWeakPullResistors == puType::down){
		gpio_pulldown_en(aPinNumber);
		gpio_pulldown_en(bPinNumber);
	}
	if(useInternalWeakPullResistors == puType::up){
		gpio_pullup_en(aPinNumber);
		gpio_pullup_en(bPinNumber);
	}
	
	// Set up encoder PCNT configuration
    pcnt_unit_config_t unit_config = {
        .low_limit = _INT16_MIN,
        .high_limit = _INT16_MAX,
    };
	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit));
	
	// Configure channel 0
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = aPinNumber,
        .level_gpio_num = bPinNumber,
    };
	ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_a_config, &pcnt_chan_a));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, et != encType::single ? PCNT_CHANNEL_EDGE_ACTION_DECREASE : PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
	
	if (et == encType::full) {
		// Configure channel 1
		pcnt_channel_handle_t pcnt_chan_b = NULL;
		pcnt_chan_config_t chan_b_config = {
			.edge_gpio_num = bPinNumber,
			.level_gpio_num = aPinNumber,
		};
		ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_b_config, &pcnt_chan_b));

		ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
		ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
	}

	// Filter out bounces and noise
	setFilter(250);
	
	// Set watchpoints
    int watch_points[] = {_INT16_MIN, _INT16_MAX};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = esp32encoder_pcnt_event_handler,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(unit, &cbs, queue));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
	
	attached = true;
}

void ESP32Encoder::attachHalfQuad(int aPinNumber, int bPinNumber) {
	attach(aPinNumber, bPinNumber, encType::half);
}

void ESP32Encoder::attachSingleEdge(int aPinNumber, int bPinNumber) {
	attach(aPinNumber, bPinNumber, encType::single);
}

void ESP32Encoder::attachFullQuad(int aPinNumber, int bPinNumber) {
	attach(aPinNumber, bPinNumber, encType::full);
}

void ESP32Encoder::setCount(int64_t value) {
	count = value;
	ESP_ERROR_CHECK( pcnt_unit_clear_count(unit));
}

int64_t ESP32Encoder::getCount() {
    int pcnt_count = 0;
	while (xQueueReceive(reinterpret_cast<QueueHandle_t>(queue), &pcnt_count, 0)) {
		// Append overflowed steps to the counter
		count += pcnt_count;
	}
	ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &pcnt_count));
	count += pcnt_count;
	return count;
}

void ESP32Encoder::clearCount() {
	count = 0;
	ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
}

void ESP32Encoder::pauseCount() {
	ESP_ERROR_CHECK(pcnt_unit_stop(unit));
}

void ESP32Encoder::resumeCount() {
	ESP_ERROR_CHECK(pcnt_unit_start(unit));
}

void ESP32Encoder::setFilter(uint16_t value) {
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = value,
    };
	if (value == 0) {
		ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, NULL));
	} else {
    	ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_config));
	}
}
#else
#warning PCNT not supported on this SoC, this will likely lead to linker errors when using ESP32Encoder
#endif // SOC_PCNT_SUPPORTED
