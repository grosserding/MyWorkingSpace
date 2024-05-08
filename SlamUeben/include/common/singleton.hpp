#pragma once

// Some Design Pattern is implemented here
// Current Finished Pattern:
// Singleton

#undef DISALLOW_COPY_AND_ASSIGN
#define DISALLOW_COPY_AND_ASSIGN(classname)         \
  classname(const classname &) = delete;            \
  classname &operator=(const classname &) = delete; \
  classname(classname &&) = delete;                 \
  classname &operator=(classname &&) = delete;

// Thread Safe guaranteed in C++ 11
#define DECLARE_SINGLETON(classname) \
 public:                             \
  static classname &GetInstance() {  \
    static classname instance;       \
    return instance;                 \
  }                                  \
                                     \
 private:                            \
  classname();                       \
                                     \
 public:                             \
  DISALLOW_COPY_AND_ASSIGN(classname)