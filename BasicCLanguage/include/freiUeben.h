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
class Animal {
public:
  Animal();
  virtual void Cry()=0;
  void TestFunc();

private:
  std::string classname;
  std::string objectname;
  std::string cryline;

  std::string teststring;
};

class Tiger : public Animal {
 public:
  Tiger(std::string objectname)
      : objectname(objectname), classname("tiger"), cryline("aww") {}
  virtual void Cry() override { std::cerr << cryline << std::endl; }
};
} // namespace test_space