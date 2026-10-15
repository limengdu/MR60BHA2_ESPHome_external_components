#include "seeed_mr60bha2.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cmath>
#include <utility>

namespace esphome {
namespace seeed_mr60bha2 {

static const char *const TAG = "seeed_mr60bha2";

static float decode_le_float_(const uint8_t *data) {
  uint32_t raw = encode_uint32(data[3], data[2], data[1], data[0]);
  float value;
  memcpy(&value, &raw, sizeof(float));
  return value;
}

/**
 * Convert a radar range value to centimetres.
 * 将雷达距离值转换为厘米。
 *
 * 0x0A16 payloads from shipped modules report centimetres. Some firmware
 * builds report metres (values in the 0.5–6.5 m operating window).
 * 量产模块的 0x0A16 载荷以厘米上报。部分固件在 0.5–6.5 m 工作窗口内以米上报。
 */
static float range_to_cm_(float value) {
  if (std::isfinite(value) && value > 0.0f && value < 10.0f) {
    return value * 100.0f;
  }
  return value;
}

static float hypot_to_cm_(float x_m, float y_m) { return std::hypot(x_m, y_m) * 100.0f; }

// Prints the component's configuration data. dump_config() prints all of the component's configuration
// items in an easy-to-read format, including the configuration key-value pairs.
void MR60BHA2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MR60BHA2:");
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR(" ", "People Exist Binary Sensor", this->has_target_binary_sensor_);
#endif
#ifdef USE_SENSOR
  LOG_SENSOR(" ", "Breath Rate Sensor", this->breath_rate_sensor_);
  LOG_SENSOR(" ", "Heart Rate Sensor", this->heart_rate_sensor_);
  LOG_SENSOR(" ", "Distance Sensor", this->distance_sensor_);
  LOG_SENSOR(" ", "Target Number Sensor", this->num_targets_sensor_);
#endif
}

// main loop
void MR60BHA2Component::loop() {
  uint8_t byte;

  // Is there data on the serial port
  while (this->available()) {
    this->read_byte(&byte);
    this->rx_message_.push_back(byte);
    if (!this->validate_message_()) {
      this->rx_message_.clear();
    }
  }
}

/**
 * @brief Calculate the checksum for a byte array.
 *
 * This function calculates the checksum for the provided byte array using an
 * XOR-based checksum algorithm.
 *
 * @param data The byte array to calculate the checksum for.
 * @param len The length of the byte array.
 * @return The calculated checksum.
 */
static uint8_t calculate_checksum(const uint8_t *data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  checksum = ~checksum;
  return checksum;
}

/**
 * @brief Validate the checksum of a byte array.
 *
 * This function validates the checksum of the provided byte array by comparing
 * it to the expected checksum.
 *
 * @param data The byte array to validate.
 * @param len The length of the byte array.
 * @param expected_checksum The expected checksum.
 * @return True if the checksum is valid, false otherwise.
 */
static bool validate_checksum(const uint8_t *data, size_t len, uint8_t expected_checksum) {
  return calculate_checksum(data, len) == expected_checksum;
}

bool MR60BHA2Component::validate_message_() {
  size_t at = this->rx_message_.size() - 1;
  auto *data = &this->rx_message_[0];

  if (at == 0) {
    return data[at] == FRAME_HEADER_BUFFER;
  }

  if (at <= 2) {
    return true;
  }

  if (at <= 4) {
    if (at == 4) {
      uint16_t length = encode_uint16(data[3], data[4]);
      if (length > MAX_FRAME_DATA_LENGTH) {
        ESP_LOGW(TAG, "FRAME_LEN ERROR: %u", length);
        return false;
      }
    }
    return true;
  }

  if (at <= 6) {
    return true;
  }

  uint16_t frame_type = encode_uint16(data[5], data[6]);
  uint8_t header_checksum = data[at];

  if (at == 7) {
    if (!validate_checksum(data, 7, header_checksum)) {
      ESP_LOGE(TAG, "HEAD_CKSUM_FRAME ERROR: 0x%02x", header_checksum);
      ESP_LOGV(TAG, "GET FRAME: %s", format_hex_pretty(data, 8).c_str());
      return false;
    }
    return true;
  }

  // Wait until all data is read. Extra firmware frame types (phase, raw
  // cloud, status, ...) are collected in full so the UART parser stays aligned.
  // 等完整帧收齐。固件额外帧（相位、点云、状态等）也完整收取，避免串口解析失步。
  uint16_t length = encode_uint16(data[3], data[4]);
  if (at - 8 < length) {
    return true;
  }

  uint8_t data_checksum = data[at];
  if (at == 8 + length) {
    if (!validate_checksum(data + 8, length, data_checksum)) {
      ESP_LOGE(TAG, "DATA_CKSUM_FRAME ERROR: 0x%02x", data_checksum);
      ESP_LOGV(TAG, "GET FRAME: %s", format_hex_pretty(data, 8 + length).c_str());
      return false;
    }
  }

  uint16_t frame_id = encode_uint16(data[1], data[2]);
  const uint8_t *frame_data = data + 8;
  ESP_LOGV(TAG, "Received Frame: ID: 0x%04x, Type: 0x%04x, Data: [%s] Raw Data: [%s]", frame_id, frame_type,
           format_hex_pretty(frame_data, length).c_str(), format_hex_pretty(this->rx_message_).c_str());
  this->process_frame_(frame_id, frame_type, data + 8, length);

  // Return false to reset rx buffer
  return false;
}

#ifdef USE_BINARY_SENSOR
void MR60BHA2Component::publish_occupancy_() {
  if (this->has_target_binary_sensor_ == nullptr) {
    return;
  }
  const bool occupied = this->people_detected_ || this->tracked_targets_ > 0;
  if (this->has_target_binary_sensor_->state == occupied) {
    return;
  }
  this->has_target_binary_sensor_->publish_state(occupied);
}
#endif

#ifdef USE_SENSOR
void MR60BHA2Component::clear_idle_sensors_() {
  if (this->people_detected_ || this->tracked_targets_ > 0) {
    return;
  }
  if (this->breath_rate_sensor_ != nullptr && this->breath_rate_sensor_->state != 0.0) {
    this->breath_rate_sensor_->publish_state(0.0);
  }
  if (this->heart_rate_sensor_ != nullptr && this->heart_rate_sensor_->state != 0.0) {
    this->heart_rate_sensor_->publish_state(0.0);
  }
  if (this->distance_sensor_ != nullptr && this->distance_sensor_->state != 0.0) {
    this->distance_sensor_->publish_state(0.0);
  }
  if (this->num_targets_sensor_ != nullptr && this->num_targets_sensor_->state != 0) {
    this->num_targets_sensor_->publish_state(0);
  }
}

void MR60BHA2Component::publish_distance_cm_(float distance_cm) {
  if (this->distance_sensor_ == nullptr || !std::isfinite(distance_cm) || distance_cm < 0.0f) {
    return;
  }
  if (this->distance_sensor_->state == distance_cm) {
    return;
  }
  this->distance_sensor_->publish_state(distance_cm);
}
#endif

void MR60BHA2Component::process_frame_(uint16_t frame_id, uint16_t frame_type, const uint8_t *data, size_t length) {
  switch (frame_type) {
    case BREATH_RATE_TYPE_BUFFER:
      if (!this->people_detected_ && this->tracked_targets_ == 0) {
        break;
      }
      if (this->breath_rate_sensor_ != nullptr && length >= 4) {
        uint32_t current_breath_rate_int = encode_uint32(data[3], data[2], data[1], data[0]);
        if (current_breath_rate_int != 0) {
          float breath_rate_float;
          memcpy(&breath_rate_float, &current_breath_rate_int, sizeof(float));
          if (this->breath_rate_sensor_->state == breath_rate_float) {
            break;
          }
          this->breath_rate_sensor_->publish_state(breath_rate_float);
        }
      }
      break;
    case PEOPLE_EXIST_TYPE_BUFFER:
      if (length >= 1) {
        this->people_detected_ = data[0] != 0;
#ifdef USE_BINARY_SENSOR
        this->publish_occupancy_();
#endif
#ifdef USE_SENSOR
        this->clear_idle_sensors_();
#endif
      }
      break;
    case HEART_RATE_TYPE_BUFFER:
      if (!this->people_detected_ && this->tracked_targets_ == 0) {
        break;
      }
      if (this->heart_rate_sensor_ != nullptr && length >= 4) {
        uint32_t current_heart_rate_int = encode_uint32(data[3], data[2], data[1], data[0]);
        if (current_heart_rate_int != 0) {
          float heart_rate_float;
          memcpy(&heart_rate_float, &current_heart_rate_int, sizeof(float));
          if (this->heart_rate_sensor_->state == heart_rate_float) {
            break;
          }
          this->heart_rate_sensor_->publish_state(heart_rate_float);
        }
      }
      break;
    case DISTANCE_TYPE_BUFFER:
#ifdef USE_SENSOR
      if (this->distance_sensor_ != nullptr) {
        bool range_valid = false;
        float distance_cm = 0.0f;
        if (length >= 8) {
          // Firmware 1.6.10+: uint32 range flag + float range.
          // 固件 1.6.10 起：uint32 有效标志 + float 距离。
          uint32_t range_flag = encode_uint32(data[3], data[2], data[1], data[0]);
          range_valid = range_flag != 0;
          distance_cm = range_to_cm_(decode_le_float_(data + 4));
        } else if (length >= 4) {
          uint32_t raw = encode_uint32(data[3], data[2], data[1], data[0]);
          if (raw != 0) {
            // Older 4-byte float payload used by some 1.6.4 frames.
            // 部分 1.6.4 帧使用 4 字节 float 载荷。
            float value;
            memcpy(&value, &raw, sizeof(float));
            range_valid = value != 0.0f;
            distance_cm = range_to_cm_(value);
          }
        }
        if (range_valid) {
          this->publish_distance_cm_(distance_cm);
        } else if (!this->people_detected_ && this->tracked_targets_ == 0) {
          this->publish_distance_cm_(0.0f);
        }
      }
#endif
      break;
    case PRINT_CLOUD_BUFFER:
      if (length >= 4) {
        uint32_t current_num_targets_int = encode_uint32(data[3], data[2], data[1], data[0]);
        this->tracked_targets_ = current_num_targets_int;
#ifdef USE_SENSOR
        if (this->num_targets_sensor_ != nullptr && this->num_targets_sensor_->state != current_num_targets_int) {
          this->num_targets_sensor_->publish_state(current_num_targets_int);
        }
        if (current_num_targets_int > 0 && length >= 12) {
          float x_m = decode_le_float_(data + 4);
          float y_m = decode_le_float_(data + 8);
          float distance_cm = hypot_to_cm_(x_m, y_m);
          if (distance_cm >= 5.0f) {
            this->publish_distance_cm_(distance_cm);
          }
        }
        this->clear_idle_sensors_();
#endif
#ifdef USE_BINARY_SENSOR
        this->publish_occupancy_();
#endif
      }
      break;
    case ALT_POSITION_TYPE_BUFFER:
#ifdef USE_SENSOR
      if (this->distance_sensor_ != nullptr && length >= 8) {
        float x_m = decode_le_float_(data);
        float y_m = decode_le_float_(data + 4);
        float distance_cm = hypot_to_cm_(x_m, y_m);
        if (distance_cm >= 5.0f) {
          this->publish_distance_cm_(distance_cm);
        }
      }
#endif
      break;
    case HEART_BREATH_PHASE_TYPE_BUFFER:
    case POINT_CLOUD_DETECT_BUFFER:
      break;
    default:
      break;
  }
}

}  // namespace seeed_mr60bha2
}  // namespace esphome
