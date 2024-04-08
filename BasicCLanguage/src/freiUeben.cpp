#include <iostream>
#include <memory>
#include <string>
#include <vector>
int main(int argc, char **argv) {
  // *ptrs
  {
    std::shared_ptr<int> sp_int_1(new int(1));
    std::cerr << "*sp_int_1 = " << *sp_int_1 << std::endl;
    {
      std::shared_ptr<int> sp_int_2(sp_int_1);
      std::cerr << "sp_int_1.use_count() 1st time = " << sp_int_1.use_count()
                << std::endl;
    }
    std::cerr << "sp_int_1.use_count() 2nd time = " << sp_int_1.use_count()
              << std::endl;
    std::unique_ptr<int> up_int_1(new int(1));
    std::unique_ptr<int> up_int_2(new int(2));
    up_int_1.reset(new int(3));
    std::cerr << "*up_int_1 = " << *up_int_1 << std::endl;
    up_int_1.reset(up_int_2.release());
    std::cerr << "*up_int_1 = " << *up_int_1 << std::endl;
    std::cerr << "up_int_2 is nullptr: " << !up_int_2 << std::endl;
  }

  return 0;
}