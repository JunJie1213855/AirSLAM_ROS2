#include <gtest/gtest.h>
#include "imu.h"

namespace {
ImuDataList Samples(std::initializer_list<double> times) {
  ImuDataList values;
  for (double time : times) {
    ImuData value;
    value.timestamp = time;
    value.gyr = Eigen::Vector3d::Zero();
    value.acc = Eigen::Vector3d::Zero();
    values.push_back(value);
  }
  return values;
}
}
TEST(ImuPreintegration, OverlappingBatchesIntegrateEachIntervalOnce) {
  Preinteration pre;
  pre.SetNoiseAndWalk(1e-4, 1e-3, 1e-5, 1e-4);
  pre.AddBatchData(Samples({9.99, 10.01, 10.03, 10.05, 10.07}), 10.0, 10.052);
  pre.AddBatchData(Samples({10.03, 10.05, 10.07, 10.09, 10.11}), 10.0, 10.1);
  EXPECT_NEAR(0.1, pre.dT, 1e-12);
  EXPECT_NEAR(10.0, pre.start_time, 1e-12);
  EXPECT_NEAR(10.1, pre.end_time, 1e-12);
}
TEST(ImuPreintegration, RepeatedCompletedBatchAddsNoTime) {
  Preinteration pre;
  pre.SetNoiseAndWalk(1e-4, 1e-3, 1e-5, 1e-4);
  const auto samples = Samples({9.99, 10.01, 10.03, 10.05, 10.07});
  pre.AddBatchData(samples, 10.0, 10.05);
  pre.AddBatchData(samples, 10.0, 10.05);
  EXPECT_NEAR(0.05, pre.dT, 1e-12);
}
