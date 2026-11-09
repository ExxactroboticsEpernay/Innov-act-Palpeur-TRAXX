#pragma once

#include "measurement.h"

namespace palpeur {

class RowEstimator {
 public:
  virtual ~RowEstimator() = default;
  virtual PairEstimate estimate(const PairConfig& config,
                               const PairInput& input) const = 0;
};

class ContactPointEstimator final : public RowEstimator {
 public:
  PairEstimate estimate(const PairConfig& config,
                        const PairInput& input) const override;
};

class BisectorEstimator final : public RowEstimator {
 public:
  PairEstimate estimate(const PairConfig& config,
                        const PairInput& input) const override;
};

RowPose estimateRowPose(const VehicleGeometry& geometry,
                        const PairEstimate& rear,
                        const PairEstimate& front);

}  // namespace palpeur