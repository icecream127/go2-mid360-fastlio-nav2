#include <gtest/gtest.h>
#include "go2_real_localization/localization_state.hpp"
#include "go2_real_localization/registration_pipeline.hpp"
#include "go2_real_localization/registration_worker.hpp"
#include <chrono>
#include <future>
#include <random>
#include <thread>

using namespace go2_real_localization;
using namespace std::chrono_literals;

TEST(State, ZeroStampAndNewSeedCanProcessSameScan) {
  Config config; LocalizationState state(config);
  state.set_initial_pose(Transform::Identity()); state.observe_input(0);
  auto first = state.prepare(0, Transform::Identity(), 0);
  ASSERT_TRUE(first); EXPECT_TRUE(state.accept(*first, Transform::Identity(), 0));
  EXPECT_FALSE(state.prepare(0, Transform::Identity(), 0));
  state.set_initial_pose(Transform::Identity());
  EXPECT_TRUE(state.prepare(0, Transform::Identity(), 0));
}

TEST(State, OldResultCannotOverwriteNewInitialPose) {
  Config config; LocalizationState state(config);
  state.set_initial_pose(Transform::Identity()); state.observe_input(1000000000);
  auto old = state.prepare(1000000000, Transform::Identity(), 1000000000);
  ASSERT_TRUE(old);
  Transform new_seed = Transform::Identity(); new_seed(0, 3) = 4;
  state.set_initial_pose(new_seed);
  EXPECT_FALSE(state.accept(*old, Transform::Identity(), 1100000000));
  EXPECT_FALSE(state.valid());
  auto next = state.prepare(1000000000, Transform::Identity(), 1100000000);
  ASSERT_TRUE(next); EXPECT_FLOAT_EQ(next->map_T_body_guess(0, 3), 4);
}

TEST(State, CompletionChecksNegativeAndExpiredAge) {
  for (int64_t now : {0LL, 4000000000LL}) {
    Config config; LocalizationState state(config);
    state.set_initial_pose(Transform::Identity()); state.observe_input(1000000000);
    auto input = state.prepare(1000000000, Transform::Identity(), 1000000000);
    ASSERT_TRUE(input); EXPECT_FALSE(state.accept(*input, Transform::Identity(), now));
    EXPECT_FALSE(state.valid()); EXPECT_EQ(state.status(), "STALE_RESULT");
  }
}

TEST(State, InputAndClockResetInvalidateInflightWork) {
  Config config; LocalizationState state(config);
  state.set_initial_pose(Transform::Identity()); state.observe_input(1000000000);
  auto input = state.prepare(1000000000, Transform::Identity(), 1000000000);
  ASSERT_TRUE(input); ASSERT_TRUE(state.accept(*input, Transform::Identity(), 1000000000));
  state.observe_input(1100000000);
  auto inflight = state.prepare(1100000000, Transform::Identity(), 1100000000);
  ASSERT_TRUE(inflight);
  EXPECT_TRUE(state.expire(3200000000));
  EXPECT_FALSE(state.current(*inflight));
  EXPECT_TRUE(state.observe_input(0));
  EXPECT_FALSE(state.prepare(0, Transform::Identity(), 0));
  state.set_initial_pose(Transform::Identity());
  EXPECT_TRUE(state.prepare(0, Transform::Identity(), 0));
}

TEST(State, ResultCannotStartWithAlreadyExpiredTTL) {
  Config config; config.max_input_age = 5; config.transform_timeout = .5;
  LocalizationState state(config);
  state.set_initial_pose(Transform::Identity()); state.observe_input(0);
  auto input = state.prepare(0, Transform::Identity(), 0); ASSERT_TRUE(input);
  EXPECT_FALSE(state.accept(*input, Transform::Identity(), 600000000));
  EXPECT_EQ(state.status(), "STALE_RESULT"); EXPECT_FALSE(state.valid());
}

TEST(State, CompositionAndRosTimePause) {
  Config config; LocalizationState state(config);
  state.set_initial_pose(Transform::Identity()); state.observe_input(0);
  Transform odom_T_body = Transform::Identity(); odom_T_body(0, 3) = 2;
  auto input = state.prepare(0, odom_T_body, 0); ASSERT_TRUE(input);
  Transform map_T_body = Transform::Identity(); map_T_body(0, 3) = 5;
  ASSERT_TRUE(state.accept(*input, map_T_body, 0));
  EXPECT_FLOAT_EQ(state.map_T_odom()(0, 3), 3);
  for (int i = 0; i < 100; ++i) EXPECT_FALSE(state.expire(0));
  EXPECT_TRUE(state.expire(4000000000));
}

TEST(Config, RejectsNaNAndInvalidLimits) {
  Config config; config.voxel_size = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(config.validate(), std::invalid_argument);
  config = Config{}; config.min_inlier_ratio = 1.1;
  EXPECT_THROW(config.validate(), std::invalid_argument);
}

TEST(Worker, RemainsBoundedAndCallerCanHandleNewSeedWhileBusy) {
  std::promise<void> entered, release;
  auto gate = release.get_future().share();
  RegistrationWorker<int, int> worker([&](const int & number) {
    entered.set_value(); gate.wait(); return number;
  });
  EXPECT_TRUE(worker.submit(7));
  const auto started = entered.get_future().wait_for(2s);
  EXPECT_EQ(started, std::future_status::ready);
  EXPECT_TRUE(worker.busy()); EXPECT_FALSE(worker.submit(8)); EXPECT_FALSE(worker.take());
  Config config; LocalizationState state(config); state.set_initial_pose(Transform::Identity());
  EXPECT_EQ(state.status(), "INITIAL_POSE_RECEIVED");
  release.set_value();
  std::optional<RegistrationWorker<int, int>::Completion> result;
  const auto end = std::chrono::steady_clock::now() + 2s;
  while (!(result = worker.take()) && std::chrono::steady_clock::now() < end) std::this_thread::sleep_for(1ms);
  ASSERT_TRUE(result); ASSERT_TRUE(result->value); EXPECT_EQ(*result->value, 7); EXPECT_FALSE(worker.busy());
}

static Cloud::Ptr room(bool plane_only = false, bool corridor = false) {
  Cloud::Ptr cloud(new Cloud); std::mt19937 random(42); std::uniform_real_distribution<float> uniform(-2, 2);
  for (int axis = 0; axis < 3; ++axis) {
    if (plane_only && axis != 2) continue;
    if (corridor && axis == 0) continue;
    for (float side : {-2.f, 2.f}) {
      if (plane_only && side > 0) continue;
      for (int i = 0; i < 700; ++i) {
        Eigen::Vector3f point(uniform(random), uniform(random), uniform(random)); point[axis] = side;
        cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
      }
    }
  }
  return cloud;
}

TEST(Pipeline, RoomAcceptedAndFarSourceExcludedFromDenominator) {
  Config config; config.voxel_size = .1; config.local_radius = 5;
  auto map = room(); RegistrationPipeline pipeline(config, map);
  Cloud::Ptr scan(new Cloud(*map));
  for (int i = 0; i < 1000; ++i) scan->push_back(pcl::PointXYZ(100 + i, 0, 0));
  const auto result = pipeline.run(scan, Transform::Identity(), true);
  EXPECT_TRUE(result.accepted) << result.summary();
  EXPECT_LT(result.source_points, map->size());
  EXPECT_EQ(result.scan_points, map->size() + 1000);
  EXPECT_GT(result.inlier_ratio, .99); EXPECT_LT(result.rmse, .01);
  EXPECT_GT(result.observability_ratio, config.min_observability_ratio);
}

TEST(Pipeline, PlaneAndCorridorRejectedDespitePerfectAlignment) {
  for (bool corridor : {false, true}) {
    Config config; config.voxel_size = .1;
    auto map = room(!corridor, corridor); RegistrationPipeline pipeline(config, map);
    const auto result = pipeline.run(map, Transform::Identity(), true);
    EXPECT_FALSE(result.accepted) << result.summary();
    EXPECT_EQ(result.reason, "DEGENERATE") << result.summary();
    EXPECT_GT(result.inlier_ratio, .99);
  }
}
