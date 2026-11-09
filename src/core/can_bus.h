#pragma once

#include "can_protocol.h"

#include <cstdint>

namespace palpeur {

class CanBus {
 public:
  virtual ~CanBus() = default;
  virtual bool begin(std::uint32_t bitrate) = 0;
  virtual bool send(const CanFrame& frame) = 0;
  virtual bool receive(CanFrame& frame) = 0;
  virtual bool enabled() const = 0;
};

class DisabledCanBus final : public CanBus {
 public:
  bool begin(std::uint32_t bitrate) override;
  bool send(const CanFrame& frame) override;
  bool receive(CanFrame& frame) override;
  bool enabled() const override;
};

}  // namespace palpeur