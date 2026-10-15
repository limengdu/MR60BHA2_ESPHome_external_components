#pragma once
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include <map>

namespace esphome {
namespace seeed_mr60bha2 {
static const uint8_t FRAME_HEADER_BUFFER = 0x01;
static const uint16_t BREATH_RATE_TYPE_BUFFER = 0x0A14;
static const uint16_t PEOPLE_EXIST_TYPE_BUFFER = 0x0F09;
static const uint16_t HEART_RATE_TYPE_BUFFER = 0x0A15;
static const uint16_t DISTANCE_TYPE_BUFFER = 0x0A16;
static const uint16_t PRINT_CLOUD_BUFFER = 0x0A04;

class MR60BHA2Component : public Component,
                          public uart::UARTDevice {  // The class name must be the name defined by text_sensor.py
#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(has_target);
#endif
#ifdef USE_SENSOR
  SUB_SENSOR(breath_rate);
  SUB_SENSOR(heart_rate);
  SUB_SENSOR(distance);
  SUB_SENSOR(num_targets);
#endif

 public:
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }
  void dump_config() override;
  void loop() override;

#ifdef USE_SENSOR
  /**
   * Set how often breath/heart/distance/target-count sensors may publish.
   * 设置呼吸/心率/距离/目标数量传感器的发布间隔。
   *
   * @param interval_ms Interval in milliseconds; 0 publishes every new reading.
   *                    间隔（毫秒）；0 表示每次新读数都立即发布。
   */
  void set_sensor_update_interval(uint32_t interval_ms) { this->sensor_update_interval_ms_ = interval_ms; }
#endif

 protected:
  bool validate_message_();
  void process_frame_(uint16_t frame_id, uint16_t frame_type, const uint8_t *data, size_t length);

#ifdef USE_SENSOR
  /**
   * Publish a sensor reading, honoring update_interval unless force is set.
   * 发布传感器读数；除非 force 为 true，否则遵守 update_interval。
   *
   * @param sens Sensor to update / 要更新的传感器
   * @param value New reading / 新读数
   * @param last_publish_ms Last publish time for this sensor / 该传感器上次发布时间
   * @param force When true, publish immediately and reset the throttle
   *              为 true 时立即发布，并重置节流计时
   */
  void publish_sensor_state_(sensor::Sensor *sens, float value, uint32_t *last_publish_ms, bool force = false);
#endif

  std::vector<uint8_t> rx_message_;
#ifdef USE_SENSOR
  uint32_t sensor_update_interval_ms_{0};
  uint32_t last_breath_rate_publish_ms_{0};
  uint32_t last_heart_rate_publish_ms_{0};
  uint32_t last_distance_publish_ms_{0};
  uint32_t last_num_targets_publish_ms_{0};
#endif
};

}  // namespace seeed_mr60bha2
}  // namespace esphome
