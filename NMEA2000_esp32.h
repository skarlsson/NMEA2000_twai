#pragma once
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "NMEA2000.h"

class tNMEA2000_esp32 : public tNMEA2000 {
public:
    enum class CAN_speed_t : uint32_t {
        CAN_SPEED_100KBPS = 100,
        CAN_SPEED_125KBPS = 125,
        CAN_SPEED_250KBPS = 250,
        CAN_SPEED_500KBPS = 500,
        CAN_SPEED_1000KBPS = 1000
    };

    tNMEA2000_esp32(gpio_num_t _TxPin, gpio_num_t _RxPin, CAN_speed_t = CAN_speed_t::CAN_SPEED_250KBPS);

    ~tNMEA2000_esp32();

    void SetCANBufferSize(uint16_t RxBufferSize, uint16_t TxBufferSize);

protected:
    bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent) override;

    bool CANOpen() override;

    bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) override;

    void InitCANFrameBuffers() override;

private:
    void CAN_init();

    void CAN_deinit();

    static void errorMonitorTask(void *pvParameters);

    void handleBusError();

    twai_timing_config_t t_config_;
    twai_filter_config_t f_config_;
    twai_general_config_t g_config_;
    bool is_open_;
    TaskHandle_t error_monitor_task_handle_;
    volatile bool should_stop_error_monitor_;
};
