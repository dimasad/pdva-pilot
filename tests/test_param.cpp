#include "gtest/gtest.h"

TEST(Trivial, Success) {
  EXPECT_EQ(0, 0) << "This test is supposed to pass trivially";
}

TEST(Trivial, Failure) {
  EXPECT_EQ(0, 1) << "This test is supposed to fail trivially";
}
