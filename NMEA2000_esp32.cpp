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
                             should_stop_error_monitor_(false),
                             error_monitor_running_(false),
                             can_mutex_(xSemaphoreCreateMutex()),
                             not_open_reported_(false),
                             busoff_reported_(false)
{
    switch (can_speed)
    {
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

tNMEA2000_esp32::~tNMEA2000_esp32()
{
    should_stop_error_monitor_ = true;
    // Wait for the monitor task to exit on its own (it deletes itself when it
    // sees the stop flag) rather than force-deleting it, so it is never killed
    // while holding can_mutex_. Capped so a wedged task can't hang teardown.
    for (int i = 0; error_monitor_running_ && i < 500; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    CAN_deinit();
    if (can_mutex_ != nullptr)
    {
        vSemaphoreDelete(can_mutex_);
    }
}

void tNMEA2000_esp32::SetCANBufferSize(uint16_t RxBufferSize, uint16_t TxBufferSize)
{
    g_config_.rx_queue_len = RxBufferSize;
    g_config_.tx_queue_len = TxBufferSize;
}

void tNMEA2000_esp32::InitCANFrameBuffers()
{
    // Set default buffer sizes if not set by user
    if (g_config_.rx_queue_len == 0) g_config_.rx_queue_len = 50;
    if (g_config_.tx_queue_len == 0) g_config_.tx_queue_len = 40;

    tNMEA2000::InitCANFrameBuffers();
}

bool tNMEA2000_esp32::CANOpen()
{
    if (is_open_) return true;
    CAN_init();
    //is_open_ = true;
    if (xTaskCreate(errorMonitorTask, "TWAI_errMonitor", TWAI_ERROR_MONITOR_STACK_SIZE, this, 5, &error_monitor_task_handle_) == pdPASS)
    {
        error_monitor_running_ = true;
    }
    return true;
}

void tNMEA2000_esp32::CAN_init()
{
    xSemaphoreTake(can_mutex_, portMAX_DELAY);
    if (!busoff_reported_)
    {
        ESP_LOGI(TAG, "Initializing TWAI driver");
    }
    esp_err_t result = twai_driver_install_v2(&g_config_, &t_config_, &f_config_, &twai_handle_);
    if (result == ESP_OK)
    {
        result = twai_start_v2(twai_handle_);
        if (result == ESP_OK)
        {
            if (!busoff_reported_)
            {
                ESP_LOGI(TAG, "TWAI driver started successfully");
            }
            is_open_ = true;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(result));
        }
    }
    else
    {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(result));
    }
    xSemaphoreGive(can_mutex_);
}

void tNMEA2000_esp32::CAN_deinit()
{
    xSemaphoreTake(can_mutex_, portMAX_DELAY);
    if (is_open_)
    {
        // Mark closed before tearing down so a concurrent send/receive that
        // is waiting on the mutex sees the closed state once it acquires it.
        is_open_ = false;
        if (!busoff_reported_)
        {
            ESP_LOGI(TAG, "Stopping TWAI driver");
        }
        twai_stop_v2(twai_handle_);
        twai_driver_uninstall_v2(twai_handle_);
    }
    xSemaphoreGive(can_mutex_);
}

bool tNMEA2000_esp32::CANSendFrame(unsigned long id, unsigned char len, const unsigned char* buf, bool wait_sent)
{
    xSemaphoreTake(can_mutex_, portMAX_DELAY);
    if (!is_open_) {
        xSemaphoreGive(can_mutex_);
        if (!not_open_reported_) {
            ESP_LOGE(TAG, "CAN port not open; suppressing further messages until recovery");
            not_open_reported_ = true;
        }
        return false;
    }

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
    // Never block on transmit. The TWAI driver has its own TX queue for
    // buffering, and the NMEA2000 library maintains a send buffer above
    // this layer. Blocking here (e.g. 100ms per frame) causes catastrophic
    // event loop stalls when the CAN bus is faulty or unterminated and the
    // controller enters bus-off recovery.
    esp_err_t result = twai_transmit_v2(twai_handle_, &message, 0);
    xSemaphoreGive(can_mutex_);
    return (result == ESP_OK);
}

bool tNMEA2000_esp32::CANGetFrame(unsigned long& id, unsigned char& len, unsigned char* buf)
{
    xSemaphoreTake(can_mutex_, portMAX_DELAY);
    if (!is_open_) {
        xSemaphoreGive(can_mutex_);
        if (!not_open_reported_) {
            ESP_LOGE(TAG, "CAN port not open; suppressing further messages until recovery");
            not_open_reported_ = true;
        }
        return false;
    }
    twai_message_t message;
    bool received = (twai_receive_v2(twai_handle_, &message, 0) == ESP_OK);
    xSemaphoreGive(can_mutex_);

    if (!received)
    {
        return false;
    }

    // A received frame means the bus is alive again: re-arm the report-once
    // flags so a future outage is reported.
    if (busoff_reported_ || not_open_reported_) {
        ESP_LOGI(TAG, "CAN bus recovered");
        busoff_reported_ = false;
        not_open_reported_ = false;
    }
    id = message.identifier;
    len = message.data_length_code;
    memcpy(buf, message.data, len);
    return true;
}

void tNMEA2000_esp32::errorMonitorTask(void* pvParameters)
{
    auto* instance = static_cast<tNMEA2000_esp32*>(pvParameters);
    twai_status_info_t status_info;

    // Timestamp for the last high-error-counter warning (in milliseconds)
    uint32_t last_highcounter_log = 0;
    const uint32_t LOG_INTERVAL = 60000; // 1 minute in milliseconds

    while (!instance->should_stop_error_monitor_)
    {
        uint32_t current_time = pdTICKS_TO_MS(xTaskGetTickCount());

        if (twai_get_status_info_v2(instance->twai_handle_, &status_info) == ESP_OK)
        {
            if (status_info.state == TWAI_STATE_BUS_OFF)
            {
                // handleBusError reports the bus-off once and initiates recovery.
                instance->handleBusError();
            }
            else if (status_info.state == TWAI_STATE_STOPPED)
            {
                // Recovery has completed (driver returns to STOPPED); restart it
                // to resume normal operation. On a dead bus recovery never
                // completes, so the driver simply idles in RECOVERING instead.
                instance->resumeFromStopped();
            }
            else if (status_info.tx_error_counter > 127 || status_info.rx_error_counter > 127)
            {
                if (current_time - last_highcounter_log >= LOG_INTERVAL)
                {
                    ESP_LOGW(TAG, "High error counters detected: TX=%ld, RX=%ld",
                             status_info.tx_error_counter, status_info.rx_error_counter);
                    last_highcounter_log = current_time;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }

    instance->error_monitor_running_ = false;
    vTaskDelete(nullptr);
}

void tNMEA2000_esp32::handleBusError()
{
    // Report once per outage; the flag is re-armed on genuine recovery in
    // CANGetFrame (a received frame).
    if (!busoff_reported_) {
        ESP_LOGE(TAG, "Bus-off; initiating TWAI recovery, suppressing further messages until recovery");
        busoff_reported_ = true;
    }
    // Recover in place. twai_initiate_recovery_v2 transitions BUS_OFF ->
    // RECOVERING without uninstalling the driver; the monitor restarts it once
    // it returns to STOPPED. A full uninstall/reinstall here trips an ESP-IDF
    // assertion in vSemaphoreDeleteWithCaps (twai_driver_uninstall_v2) and
    // panics, so the driver must stay installed.
    xSemaphoreTake(can_mutex_, portMAX_DELAY);
    if (is_open_)
    {
        twai_initiate_recovery_v2(twai_handle_);
    }
    xSemaphoreGive(can_mutex_);
}

void tNMEA2000_esp32::resumeFromStopped()
{
    xSemaphoreTake(can_mutex_, portMAX_DELAY);
    if (is_open_)
    {
        twai_start_v2(twai_handle_);
    }
    xSemaphoreGive(can_mutex_);
}
