#pragma once
#include <typeinfo>
#ifndef FMT_HEADER_ONLY
#define FMT_HEADER_ONLY
#endif
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "fmt/format.h"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include <iostream>

/* 添加按键控制循环结束功能 */
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>

int kbhit(void) {
  struct termios oldt, newt;
  int ch;
  int oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }
}