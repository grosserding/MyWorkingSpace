#include <iostream>
#include <memory>
#include <string>
#include <vector>
int main(int argc, char **argv) {
  // *ptrs
  {
    std::shared_ptr<int> sp_int_1(new int(1));
    // 这里注意和裸指针的区别：
    // 裸指针为：int *p = new int(1)
    // 智能指针为： std::shared_ptr<int> sp(new(1));
    std::cerr << "*sp_int_1 = " << *sp_int_1 << std::endl;
    {
      std::shared_ptr<int> sp_int_2(sp_int_1);
      std::cerr << "sp_int_1.use_count() 1st time = " << sp_int_1.use_count()
                << std::endl;
      // 这里sp_int_2就不存在了，因此sp_int_1指向的对象计数减一
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

  // *内存探索
  {
    int *p = new int;
    int t = 1;
    std::cerr << "p = " << p << ", &t = " << &t << std::endl;
    p = &t;
    std::cerr << "p = " << p << ", &t = " << &t << std::endl;
    *p = 2;
    std::cerr << "p = " << p << ", &t = " << &t << std::endl;
    int *q = &t;
    std::cerr << "p = " << p << ", q = " << q << std::endl;
    // **关于new
    {
      // ***1. 实例化一个对象
      std::shared_ptr<int> sp_int_1(new int(1));
      int *p_1 = new int(1);
      // ***2. 实例化一个数组
      int length = 10;
      int *p_2 = new int[length];
      int *p_3 = new int[length]{1, 2, 3};
      for (int i = 0; i < length; i++) {
        std::cerr << "p_3[" << i << "] = " << *(p_3 + i) << std::endl;
        std::cerr << "p_2[" << i << "] = " << *(p_2 + i) << std::endl;
        // 能看到，这里new完之后自动初始化为0了，同时注意这里指针遍历的方法
      }
      // ****2.1 智能指针怎样指向数组？
      // std::shared_ptr<int> sp_intgroup_wrong(new int[length]);
      // 这是不行的，因为释放时只会释放第一个
      std::shared_ptr<int[]> sp_intgroup(new int[length]);
      for (int i = 0; i < length; i++) {
        sp_intgroup[i] = i; // 这里竟然不需要用取内容符号*
      }
      for (int i = 0; i < length; i++) {
        std::cerr << "sp_intgroup[i] = " << sp_intgroup[i]
                  << std::endl; // 这里竟然不需要用取内容符号*
      }
      // std::shared_ptr<int> test_sp; test_sp = 1;
      // 这是不行的，但是智能指针数组却可以！
      // ***3. 定位new
      int buffer[100];
      int *p_buffer;
      p_buffer = new (buffer) int[20]{
          1, 2, 3}; // 把p_buffer重定位到原有的变量buffer上面，同时赋值
      // p_buffer = new (buffer) int[120]{1, 2, 3}; // 栈报错
      std::cerr << "&buffer = " << &buffer << std::endl;
      std::cerr << "p_buffer = " << p_buffer << std::endl;
      int *p_buffer_2;
      p_buffer_2 = new (buffer) int;
      std::cerr << "p_buffer_2 = " << p_buffer_2 << std::endl;
      p_buffer_2 = &(buffer[0]);
      std::cerr << "p_buffer_2 = " << p_buffer_2 << std::endl;
    }
  }

  // *vector相关、底层原理
  {
    std::vector<int> vec1{1, 2, 3, 4, 5};
    std::cerr << "vec1.capacity() = " << vec1.capacity() << std::endl;
    vec1 = std::vector<int>(vec1.begin(),
                            vec1.begin() +
                                2); // 这里直接复制了，连capacity也直接变为2
    std::cerr << "vec1.capacity() = " << vec1.capacity() << std::endl;
    vec1 = {1, 2, 3, 4, 5}; // 别忘了std::vector还能这样赋值
    std::cerr << "vec1.capacity() = " << vec1.capacity() << std::endl;
    std::cerr << "vec1 = ";
    for (auto tmp : vec1) {
      std::cerr << tmp << ", ";
    }
    vec1.erase(vec1.begin()); // 删除，后一个顶上。
    std::cerr << std::endl
              << "After erase vec1.begin(), vec1.capacity() = "
              << vec1.capacity() << std::endl;
    std::cerr << "vec1 = ";
    for (auto tmp : vec1) {
      std::cerr << tmp << ", ";
    }
    std::cerr << std::endl;

    std::vector<int> vec(3);
    // 这步已经完成初始化了其实，每个数都是0
    std::cerr << "--------------" << std::endl;
    for (auto num : vec) {
      std::cerr << num << std::endl;
    }
    std::cerr << "--------------" << std::endl;
    std::cerr << "vec[0] = " << vec[0] << std::endl;
    std::cerr << "vec.size() = " << vec.size() << std::endl;
    std::cerr << "vec.capacity() = " << vec.capacity() << std::endl;
    std::cerr << "vec.max_size() = " << vec.max_size() << std::endl;
    std::cerr << "&vec(1) = " << &(vec[0]) << std::endl;
    vec.emplace_back(1);
    vec.emplace_back(2);
    // 注意：这里压入的数，是直接扩容并赋值，而并非改变原来的第一个数和第二个数
    std::cerr << "--------------" << std::endl;
    for (auto num : vec) {
      std::cerr << num << std::endl;
    }
    std::cerr << "--------------" << std::endl;
    std::cerr << "vec[0] = " << vec[0] << std::endl;
    std::cerr << "vec.size() = " << vec.size() << std::endl;
    std::cerr << "vec.capacity() = " << vec.capacity() << std::endl;
    std::cerr << "vec.max_size() = " << vec.max_size() << std::endl;
    std::cerr << "&vec(1) = " << &(vec[0]) << std::endl;

    std::cerr << "--------------" << std::endl;
    std::cerr << "vec.size() = " << vec.size() << std::endl;
    std::cerr << "vec.capacity() = " << vec.capacity() << std::endl;
    vec.reserve(100); // 增加到100而非+100
    std::cerr << "vec.size() = " << vec.size() << std::endl;
    std::cerr << "vec.capacity() = " << vec.capacity() << std::endl;

    vec.reserve(10); // 小于原capacity，则不会改变！
    std::cerr << "vec.size() = " << vec.size() << std::endl;
    std::cerr << "vec.capacity() = " << vec.capacity() << std::endl;
  }
  return 0;
}