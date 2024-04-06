#pragma once
#include <math.h>

#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <typeinfo>
// 注意，宏定义不受命名空间或者花括号的限制
// 在源文件定义时：其作用域为定义之后到源文件结束，所以通常会写在文件开头，以扩大其作用域。
// 在头文件中定义时：作用域是从包含该头文件的位置到文件结尾，同时 #undef
// 可终止宏定义的作用域。
#define PI 3.1415926
namespace MyMath {} // namespace MyMath
namespace HelloWorld {
void PrintHelloWorld();
}
namespace EigenLibs {
void EigenUebungs();
}
