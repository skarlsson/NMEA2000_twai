#include "NMEA2000_esp32.h"
#include "esp_log.h"

#define TAG "NMEA2000_esp32"

tNMEA2000_esp32::tNMEA2000_esp32(
    gpio_num_t TxPin,
    gpio_num_t RxPin,
    int twai_controller_id,
    CAN_speed_t can_speed) : tNMEA2000(),
                             is_open_(false),
                             error_monitor_task_handle_(nullptr),
                             should_stop_error_monitor_(false) {
    switch (can_speed) {
        case CAN_speed_t::CAN_SPEED_25KBPS:
            t_config_ = TWAI_TIMING_CONFIG_25KBITS();
            break;
        case CAN_speed_t::CAN_SPEED_50KBPS:
            t_config_ = TWAI_TIMING_CONFIG_50KBITS();
            break;
        case CAN_speed_t::CAN_SPEED_100KBPS:
            t_config_ = TWAI_TIMING_CONFIG_100KBITS();
            break;
        case CAN_speed_t::CAN_SPEED_125KBPS:
            t_config_ = TWAI_TIMING_CONFIG_125KBITS();
            break;
        case CAN_speed_t::CAN_SPEED_250KBPS:
            t_config_ = TWAI_TIMING_CONFIG_250KBITS();
            break;
        case CAN_speed_t::CAN_SPEED_500KBPS:
            t_config_ = TWAI_TIMING_CONFIG_500KBITS();
            break;
        case CAN_speed_t::CAN_SPEED_1000KBPS:
            t_config_ = TWAI_TIMING_CONFIG_1MBITS();
            break;
        default:
            t_config_ = TWAI_TIMING_CONFIG_250KBITS();
    }
    f_config_ = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    g_config_ = TWAI_GENERAL_CONFIG_DEFAULT(TxPin, RxPin, TWAI_MODE_NORMAL);
    g_config_.controller_id = twai_controller_id;
    g_config_.tx_queue_len = 40;
    g_config_.rx_queue_len = 40;

    // should be set using menuconfig - otherwise bad things happen when trying ota over can
#ifdef CONFIG_TWAI_ISR_IN_IRAM
    g_config_.intr_flags = ESP_INTR_FLAG_IRAM;
#else
    #warning "CONFIG_TWAI_ISR_IN_IRAM not set in menuconfig"
#endif
}

tNMEA2000_esp32::~tNMEA2000_esp32() {
    should_stop_error_monitor_ = true;
    if (error_monitor_task_handle_ != nullptr) {
        vTaskDelete(error_monitor_task_handle_);
    }
    CAN_deinit();
}

void tNMEA2000_esp32::SetCANBufferSize(uint16_t RxBufferSize, uint16_t TxBufferSize) {
    g_config_.rx_queue_len = RxBufferSize;
    g_config_.tx_queue_len = TxBufferSize;
}

void tNMEA2000_esp32::InitCANFrameBuffers() {
    // Set default buffer sizes if not set by user
    if (g_config_.rx_queue_len == 0) g_config_.rx_queue_len = 50;
    if (g_config_.tx_queue_len == 0) g_config_.tx_queue_len = 40;

    tNMEA2000::InitCANFrameBuffers();
}

bool tNMEA2000_esp32::CANOpen() {
    if (is_open_) return true;
    CAN_init();
    is_open_ = true;
    xTaskCreate(errorMonitorTask, "TWAI_errMonitor", 4096, this, 5, &error_monitor_task_handle_);
    return true;
}

void tNMEA2000_esp32::CAN_init() {
    ESP_LOGI(TAG, "Initializing TWAI driver");
    esp_err_t result = twai_driver_install_v2(&g_config_, &t_config_, &f_config_, &twai_handle_);
    if (result == ESP_OK) {
        result = twai_start_v2(twai_handle_);
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "TWAI driver started successfully");
        } else {
            ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(result));
        }
    } else {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(result));
    }
}

void tNMEA2000_esp32::CAN_deinit() {
    if (is_open_) {
        ESP_LOGI(TAG, "Stopping TWAI driver");
        twai_stop_v2(twai_handle_);
        twai_driver_uninstall_v2(twai_handle_);
        is_open_ = false;
    }
}

bool tNMEA2000_esp32::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent) {
    if (!is_open_)
        return false;

    twai_message_t message = {
        //        .extd = 1,
        //        .rtr = 0,
        //        .ss = 0,
        //        .self = 0,
        //        .dlc_non_comp = 0,
        //        .reserved = 0,
        // some compilers cant cope with union->struct above, setting the 32 bit flags directly below.
        .flags = 0x01,
        .identifier = id,
        .data_length_code = static_cast<uint8_t>(len > 8 ? 8 : len),
        .data = {0}
    };
    memcpy(message.data, buf, message.data_length_code);
    //todo this could use some love when doing microsleep
    esp_err_t result = twai_transmit_v2(twai_handle_, &message, wait_sent ? pdMS_TO_TICKS(100) : 0);
    if (result != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to transmit message: %s", esp_err_to_name(result));
    }
    return (result == ESP_OK);
}

bool tNMEA2000_esp32::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
    if (!is_open_)
        return false;
    twai_message_t message;
    if (twai_receive_v2(twai_handle_, &message, 0) == ESP_OK) {
        id = message.identifier;
        len = message.data_length_code;
        memcpy(buf, message.data, len);
        return true;
    }
    return false;
}

void tNMEA2000_esp32::errorMonitorTask(void *pvParameters) {
    auto *instance = static_cast<tNMEA2000_esp32 *>(pvParameters);
    twai_status_info_t status_info;

    while (!instance->should_stop_error_monitor_) {
        if (twai_get_status_info_v2(instance->twai_handle_, &status_info) == ESP_OK) {
            if (status_info.state == TWAI_STATE_BUS_OFF) {
                ESP_LOGE(TAG, "Bus-off condition detected");
                instance->handleBusError();
            } else if (status_info.tx_error_counter > 127 || status_info.rx_error_counter > 127) {
                ESP_LOGW(TAG, "High error counters detected: TX=%ld, RX=%ld",
                         status_info.tx_error_counter, status_info.rx_error_counter);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }

    vTaskDelete(nullptr);
}

void tNMEA2000_esp32::handleBusError() {
    ESP_LOGI(TAG, "Handling bus error: Reinitializing TWAI driver");
    CAN_deinit();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second before reinitializing
    CAN_init();
}
