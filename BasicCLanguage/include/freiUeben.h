#pragma once
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <functional>
#include <queue>

namespace test_space {
class TestClassInTestSpace {
public:
  // TestClassInTestSpace():global_a(0) {} 这样也编不过，不能这样赋值
  TestClassInTestSpace(int a) : a(a) {}
  int a = 0;
  static int global_a;
};
}  // namespace test_space
class Animal {
 public:
  Animal(std::string classname) : classname(classname) {
    std::cerr << "create this animal" << std::endl;
  };
  virtual ~Animal() { std::cerr << "delete this animal" << std::endl; }
  virtual void Cry() = 0;
  void TestFunc1() { std::cerr << "now TestFunc1Animal" << std::endl; }
  virtual void TestFunc2() { std::cerr << "now TestFunc2Animal" << std::endl; }
  virtual void TestFunc3() { std::cerr << "now TestFunc3Animal" << std::endl; }
  void TestFunc4() { std::cerr << "now TestFunc4Animal" << std::endl; }

 private:
  std::string classname;
  std::string teststring;
};

class Tiger : public Animal {
 public:
  Tiger(std::string objectname)
      : Animal("tiger"), objectname(objectname), cryline("aww!") {
    std::cerr << "create this tiger" << std::endl;
  }
  ~Tiger() { std::cerr << "delete this tiger" << std::endl; }
  void Cry() override { std::cerr << cryline << std::endl; }
  void TestFunc1() { std::cerr << "now TestFunc1Tiger" << std::endl; }
  void TestFunc2() { std::cerr << "now TestFunc2Tiger" << std::endl; }
  void TestFunc3() override {
    std::cerr << "now TestFunc3Tiger" << std::endl;
  }

 private:
  std::string objectname;
  std::string cryline;
};

void DeleteTiger(Tiger *a)
{
	delete a;
}

void DeleteAnimal(Animal *a)
{
	delete a;
}