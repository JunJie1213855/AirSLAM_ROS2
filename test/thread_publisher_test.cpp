#include <gtest/gtest.h>
#include "thread_publisher.h"

TEST(ThreadPublisher, ShutdownDrainsQueuedMessagesInOrder) {
  ThreadPublisher<int> publisher;
  std::vector<int> received;
  publisher.Register([&](const std::shared_ptr<const int>& value) { received.push_back(*value); });
  publisher.Start();
  for (int i = 0; i < 1000; ++i) publisher.Publish(std::make_shared<const int>(i));
  publisher.ShutDown();
  ASSERT_EQ(1000u, received.size());
  for (int i = 0; i < 1000; ++i) EXPECT_EQ(i, received[i]);
}
TEST(ThreadPublisher, ShutdownIsSafeBeforeStartAndCanBeRepeated) {
  ThreadPublisher<int> publisher;
  publisher.ShutDown();
  publisher.ShutDown();
}
