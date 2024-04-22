#pragma once
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>

namespace test_space {
class TestClassInTestSpace {
public:
  // TestClassInTestSpace():global_a(0) {} 这样也编不过，不能这样赋值
  TestClassInTestSpace(int a) : a(a) {}
  int a = 0;
  static int global_a;
};
} // namespace test_space