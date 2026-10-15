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
static const uint16_t HEART_BREATH_PHASE_TYPE_BUFFER = 0x0A13;
static const uint16_t BREATH_RATE_TYPE_BUFFER = 0x0A14;
static const uint16_t PEOPLE_EXIST_TYPE_BUFFER = 0x0F09;
static const uint16_t HEART_RATE_TYPE_BUFFER = 0x0A15;
static const uint16_t DISTANCE_TYPE_BUFFER = 0x0A16;
static const uint16_t ALT_POSITION_TYPE_BUFFER = 0x0A17;
static const uint16_t PRINT_CLOUD_BUFFER = 0x0A04;
static const uint16_t POINT_CLOUD_DETECT_BUFFER = 0x0A08;
static const uint16_t MAX_FRAME_DATA_LENGTH = 256;

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

 protected:
  bool validate_message_();
  void process_frame_(uint16_t frame_id, uint16_t frame_type, const uint8_t *data, size_t length);
#ifdef USE_BINARY_SENSOR
  /**
   * Publish occupancy from human-detect and tracked-target reports.
   * 根据人体检测帧和跟踪目标数量发布占用状态。
   */
  void publish_occupancy_();
#endif
#ifdef USE_SENSOR
  /**
   * Set breath/heart/distance/target-count sensors to idle (0) when the scene is empty.
   * 场景为空时将呼吸/心率/距离/目标数量传感器置为 0。
   */
  void clear_idle_sensors_();
  /**
   * Publish a distance reading in centimetres.
   * 以厘米为单位发布距离读数。
   *
   * @param distance_cm Distance in centimetres / 距离（厘米）
   */
  void publish_distance_cm_(float distance_cm);
#endif

  std::vector<uint8_t> rx_message_;
  bool people_detected_{false};
  uint32_t tracked_targets_{0};
};

}  // namespace seeed_mr60bha2
}  // namespace esphome
