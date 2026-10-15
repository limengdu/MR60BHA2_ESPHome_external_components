#include "seeed_mr60bha2.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <utility>

namespace esphome {
namespace seeed_mr60bha2 {

static const char *const TAG = "seeed_mr60bha2";

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
  if (this->sensor_update_interval_ms_ > 0) {
    ESP_LOGCONFIG(TAG, "  Sensor Update Interval: %.1fs", this->sensor_update_interval_ms_ / 1000.0f);
  }
#endif
}

#ifdef USE_SENSOR
void MR60BHA2Component::publish_sensor_state_(sensor::Sensor *sens, float value, uint32_t *last_publish_ms,
                                              bool force) {
  if (sens == nullptr) {
    return;
  }
  if (force) {
    *last_publish_ms = 0;
    sens->publish_state(value);
    return;
  }
  if (sens->state == value) {
    return;
  }
  uint32_t now = millis();
  if (this->sensor_update_interval_ms_ != 0 && *last_publish_ms != 0 &&
      now - *last_publish_ms < this->sensor_update_interval_ms_) {
    return;
  }
  *last_publish_ms = now;
  sens->publish_state(value);
}
#endif

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

  if (at <= 6) {
    return true;
  }

  uint16_t frame_type = encode_uint16(data[5], data[6]);

  if (frame_type != BREATH_RATE_TYPE_BUFFER && frame_type != HEART_RATE_TYPE_BUFFER &&
      frame_type != DISTANCE_TYPE_BUFFER && frame_type != PEOPLE_EXIST_TYPE_BUFFER &&
      frame_type != PRINT_CLOUD_BUFFER) {
    return false;
  }

  uint8_t header_checksum = data[at];

  if (at == 7) {
    if (!validate_checksum(data, 7, header_checksum)) {
      ESP_LOGE(TAG, "HEAD_CKSUM_FRAME ERROR: 0x%02x", header_checksum);
      ESP_LOGV(TAG, "GET FRAME: %s", format_hex_pretty(data, 8).c_str());
      return false;
    }
    return true;
  }

  // Wait until all data is read
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

void MR60BHA2Component::process_frame_(uint16_t frame_id, uint16_t frame_type, const uint8_t *data, size_t length) {
  if (this->has_target_binary_sensor_ != nullptr && !this->has_target_binary_sensor_->state &&
      frame_type != PEOPLE_EXIST_TYPE_BUFFER) {
    // Do not process other frames while people exists sensor is still false
    return;
  }
  switch (frame_type) {
    case BREATH_RATE_TYPE_BUFFER:
      if (this->breath_rate_sensor_ != nullptr && length >= 4) {
        uint32_t current_breath_rate_int = encode_uint32(data[3], data[2], data[1], data[0]);
        if (current_breath_rate_int != 0) {
          float breath_rate_float;
          memcpy(&breath_rate_float, &current_breath_rate_int, sizeof(float));
          this->publish_sensor_state_(this->breath_rate_sensor_, breath_rate_float,
                                      &this->last_breath_rate_publish_ms_);
        }
      }
      break;
    case PEOPLE_EXIST_TYPE_BUFFER:
      if (this->has_target_binary_sensor_ != nullptr && length >= 2) {
        uint16_t has_target_int = encode_uint16(data[1], data[0]);
        if (this->has_target_binary_sensor_->state == has_target_int) {
          break;
        }
        this->has_target_binary_sensor_->publish_state(has_target_int);
        if (has_target_int == 0) {
          if (this->breath_rate_sensor_ != nullptr && this->breath_rate_sensor_->state != 0.0) {
            this->publish_sensor_state_(this->breath_rate_sensor_, 0.0, &this->last_breath_rate_publish_ms_, true);
          }
          if (this->heart_rate_sensor_ != nullptr && this->heart_rate_sensor_->state != 0.0) {
            this->publish_sensor_state_(this->heart_rate_sensor_, 0.0, &this->last_heart_rate_publish_ms_, true);
          }
          if (this->distance_sensor_ != nullptr && this->distance_sensor_->state != 0.0) {
            this->publish_sensor_state_(this->distance_sensor_, 0.0, &this->last_distance_publish_ms_, true);
          }
          if (this->num_targets_sensor_ != nullptr && this->num_targets_sensor_->state != 0) {
            this->publish_sensor_state_(this->num_targets_sensor_, 0, &this->last_num_targets_publish_ms_, true);
          }
        }
      }
      break;
    case HEART_RATE_TYPE_BUFFER:
      if (this->heart_rate_sensor_ != nullptr && length >= 4) {
        uint32_t current_heart_rate_int = encode_uint32(data[3], data[2], data[1], data[0]);
        if (current_heart_rate_int != 0) {
          float heart_rate_float;
          memcpy(&heart_rate_float, &current_heart_rate_int, sizeof(float));
          this->publish_sensor_state_(this->heart_rate_sensor_, heart_rate_float, &this->last_heart_rate_publish_ms_);
        }
      }
      break;
    case DISTANCE_TYPE_BUFFER:
      if (data[0] != 0) {
        if (this->distance_sensor_ != nullptr && length >= 8) {
          uint32_t current_distance_int = encode_uint32(data[7], data[6], data[5], data[4]);
          float distance_float;
          memcpy(&distance_float, &current_distance_int, sizeof(float));
          this->publish_sensor_state_(this->distance_sensor_, distance_float, &this->last_distance_publish_ms_);
        }
      }
      break;
    case PRINT_CLOUD_BUFFER:
      if (this->num_targets_sensor_ != nullptr && length >= 4) {
        uint32_t current_num_targets_int = encode_uint32(data[3], data[2], data[1], data[0]);
        this->publish_sensor_state_(this->num_targets_sensor_, current_num_targets_int,
                                    &this->last_num_targets_publish_ms_);
      }
      break;
    default:
      break;
  }
}

}  // namespace seeed_mr60bha2
}  // namespace esphome
