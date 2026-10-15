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
  SUB_SENSOR(target_x);
  SUB_SENSOR(target_y);
#endif

 public:
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }
  void dump_config() override;
  void loop() override;

 protected:
  bool validate_message_();
  void process_frame_(uint16_t frame_id, uint16_t frame_type, const uint8_t *data, size_t length);
  /**
   * True when human-detect, tracked cloud, detection cloud, or alt position is active.
   * 人体检测、跟踪点云、探测点云或备用坐标任一有效时为占用。
   */
  bool is_occupied_() const;
#ifdef USE_BINARY_SENSOR
  /**
   * Publish occupancy from human-detect and target-coordinate reports.
   * 根据人体检测帧和目标坐标报告发布占用状态。
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
  /**
   * Publish the nearest target X/Y in metres.
   * 以米为单位发布最近目标的 X/Y。
   *
   * @param x_m X coordinate in metres / X 坐标（米）
   * @param y_m Y coordinate in metres / Y 坐标（米）
   */
  void publish_target_xy_(float x_m, float y_m);
#endif
  /**
   * Apply a people-counting cloud payload (0x0A04 tracked or 0x0A08 detection).
   * 解析人数点云载荷（0x0A04 跟踪 或 0x0A08 探测）。
   *
   * @param slot Target-count field to update / 要更新的目标计数字段
   * @param data Frame payload / 帧载荷
   * @param length Payload length / 载荷长度
   * @param publish_count When true, update the num_targets sensor
   *                      为 true 时更新目标数量传感器
   */
  void apply_people_counting_(uint32_t *slot, const uint8_t *data, size_t length, bool publish_count);

  std::vector<uint8_t> rx_message_;
  bool people_detected_{false};
  bool alt_position_present_{false};
  uint32_t tracked_targets_{0};
  uint32_t detect_targets_{0};
};

}  // namespace seeed_mr60bha2
}  // namespace esphome
