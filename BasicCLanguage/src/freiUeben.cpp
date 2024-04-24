#include "freiUeben.h"
int global = 0;
static int static_global = 1;

// {
// } 如果不在一个函数内，不能使用作用域

int Add_explicit(int a, int b) { return a + b; }
namespace test_space_2 {
class TestClass2 {
public:
  static void SetGlobalB1(int b) { global_b = b; }
  static void SetGlobalB2() { global_b = 0; }
  static int global_a;
  static int global_b;

private:
  static int global_c;
};

} // namespace test_space_2

int test_space_2::TestClass2::global_b = 211;
int test_space_2::TestClass2::global_a = 11;
int test_space_2::TestClass2::global_c =
    31; // *注意：这里不属于调用、访问，而是属于定义！因此和private访问权限无关！且因为是定义而不是简单的赋值，前面要加数据类型名int，这里很反直觉！
int test_space::TestClassInTestSpace::global_a = 10;

void buildTree(std::vector<int> &vec1) { return; }
int main(int argc, char **argv) {
  // *tools and test in need
  class TestStaticInMain {
    // static int a; // 还是不行，因为在main中
  };

  static int static_global_in_main = 2;
  // *ptrs
  std::cerr << "**********ptrs**********" << std::endl;
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

    // 双向迭代器
    std::vector<int> ls_1{1, 2, 3, 4, 5};
    std::vector<int>::iterator it_1 = ls_1.begin();
    std::vector<int>::iterator it_2 = ls_1.begin();
    std::cerr << "++it_1 = " << *(++it_2) << std::endl;
    *(it_1) = *(++it_1);
    *(it_1) = 2;
    std::cerr << "ls_1 after = " << ls_1[0] << "," << ls_1[1] << "..."
              << std::endl;
  }

  // *内存探索
  std::cerr << "**********内存探索**********" << std::endl;
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
  std::cerr << "**********vector相关、底层原理**********" << std::endl;
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

  // *vector和list对比
  std::cerr << "**********vector和list的迭代器对比**********" << std::endl;
  {
    // it vector
    std::vector<int> vec{1, 2, 3, 4, 5, 6, 7, 8};
    std::vector<int>::iterator it = vec.begin() + 4;
    std::cerr << "*it = " << *it << std::endl;
    std::cerr << "*(it+2) = " << *(it + 2) << std::endl;
    std::cerr << "*(it-2) = " << *(it - 2) << std::endl;
    std::cerr << "*(--it) = " << *(--it) << std::endl;
    std::cerr << "it > it-1 = " << (it > it - 1) << std::endl;
    // it list
    std::list<int> list_test{1, 2, 3, 4, 5, 6, 7, 8};
    std::list<int>::iterator it2 = list_test.begin();
    // it2 += 2; 编译不通过
    it2++;
    std::cerr << "*it2 = " << *it2 << std::endl;
    it2--;
    std::cerr << "*it2 = " << *it2 << std::endl;

    std::list<int> ls1{1, 2, 3};
    // 注意，pop_back(), pop_front()都返回void。auto a = ls1.pop_back();编译不过
  }
  // *unordered_map相关
  std::cerr << "**********unordered_map相关**********" << std::endl;
  {
    std::unordered_map<int, int> hash{{1, 11}, {2, 12}, {3, 13}};
    auto tryfind = hash.find(1);
    std::cerr << "tpye of tryfind = " << typeid(tryfind).name() << std::endl;
    if (tryfind == hash.end()) {
      std::cerr << "tryfind == hash.end()" << std::endl;
    } else {
      std::cerr << "tryfind->first = " << tryfind->first << std::endl;
      std::cerr << "tryfind->second = " << tryfind->second << std::endl;
    }
    // 别忘了unordered_map中find的用法，是unordered_map::iterator it =
    // hash.find(key);
  }
  // *static相关
  std::cerr << "**********static相关**********" << std::endl;
  {
    {
      static int static_global_in_scope =
          3; // 被限制在本作用域内，其他地方不能访问
    }
    {
      std::cerr << "global = " << global << std::endl;
      std::cerr << "static_global = " << static_global << std::endl;
      std::cerr << "static_global_in_main = " << static_global_in_main
                << std::endl;
      // std::cerr << "static_global_in_scope = " << static_global_in_scope <<
      // std::endl;  // 编译不通过
      class TestStatic {
      public:
        TestStatic(int a) : a(a) {}
        int a = 0;
        // static int global_a; 编译不通过
      private:
        // static int global_a; 仍然编不过！因为local
        // class不能包含静态数据成员，但可以包含静态函数成员
      };
      test_space::TestClassInTestSpace ts1(1);
      test_space::TestClassInTestSpace ts2(2);
      ts1.global_a = 100000;
      test_space::TestClassInTestSpace ts3(3);
      std::cerr << "ts2.global_a = " << ts2.global_a << std::endl;
      std::cerr << "ts3.global_a = " << ts3.global_a << std::endl;

      test_space_2::TestClass2 test1;
      test1.global_a = 21;
      test_space_2::TestClass2 test2;
      test2.global_a = 31;
      // std::cerr << "test1.global_c = " << test1.global_c
      //     << std::endl;
      //     这里才是访问，因为global_c是private，因此不能编译通过
      std::cerr << "test1.global_a = " << test1.global_a << std::endl;
      test_space_2::TestClass2::SetGlobalB1(311);
      std::cerr << "test1.global_b = " << test1.global_b << std::endl;
      test_space_2::TestClass2::SetGlobalB2();
      std::cerr << "test2.global_b = " << test2.global_b << std::endl;
    }
  }
  // *函数相关
  std::cerr << "**********函数相关**********" << std::endl;
  {
    std::vector<int> vec2{1, 2, 3};
    buildTree(vec2);
    // buildTree(std::vector<int>(vec2)); 这样就不行。参数为&
  }
  // *数学相关
  std::cerr << "**********数学相关**********" << std::endl;
  {
    double a = 4.0;
    auto b = sqrt(a);
    std::cerr << "b.type = " << typeid(b).name() << ", b = " << b << std::endl;
  }
  // *类、继承、多态
  std::cerr << "**********类、继承、多态**********" << std::endl;
  {
    Tiger xiaohu("huhu");
    xiaohu.Cry();
    Tiger *xiaohuzhizhen = new Tiger("huhuzhizhen");
    Animal *xiaohuzhizhen2 = new Tiger("huhuzhizhne2");
    xiaohuzhizhen2->TestFunc1();
    xiaohuzhizhen2->TestFunc2();
    xiaohuzhizhen2->TestFunc3();
    std::cerr << "-------------------------" << std::endl;
    DeleteAnimal(xiaohuzhizhen);
    std::cerr << "-------------------------" << std::endl;
    delete xiaohuzhizhen2;
    std::cerr << "-------------------------" << std::endl;
    std::cerr << "注意，如果基类的析构函数不定义为虚函数，则上面两个指针对象删"
                 "除时都只会调用基类的析构函数，而不会调用派生类的析构函数"
              << std::endl;
  }
  // *lambda
  std::cerr << "**********lambda**********" << std::endl;
  {
    std::function<int(int,int)> Add1 = [](int a, int b) -> int { return a + b; };
    auto Add2 = [](int a, int b) -> int { return a + b; };
    std::cerr << "Add(1, 2) = " << Add1(1, 2) << std::endl;
    std::cerr << "Add_explicit(1, 2) = " << Add_explicit(1, 2) << std::endl;
    std::cerr << "Add1.type = " << typeid(Add1).name() << std::endl;
    std::cerr << "Add2.type = " << typeid(Add2).name() << std::endl;
  }
  // *左右值和引用
  std::cerr << "**********左右值和引用**********" << std::endl;
  {
    // 左值定义
    int a = 3;
    int* pa = &a;
    int* pb = new int(5);
    std::cerr << "*pa = " << *pa << std::endl;
    std::cerr << "*pb = " << *pb << std::endl;

    // 左值引用
    int &ar = a;
    const int &car = a;
    int*& par = pa;
    std::cerr << "*par = " << *par << std::endl;
    a = 10;
    std::cerr << "ar = " << ar << std::endl;
    std::cerr << "car = " << car << std::endl;
    std::cerr << "*par = " << *par << std::endl;
    ar = 20;
    // car = 30; // 编译不通过，因为定义为const类型
    std::cerr << "car.type = " << typeid(car).name() << std::endl;
    std::cerr << "a = " << a << std::endl;
    std::cerr << "&a = " << &a << std::endl;
    std::cerr << "&ar = " << &ar << std::endl;
    std::cerr << "&car = " << &car << std::endl;

    // 右值引用
    int && rra = 10;
    const int && rrb = 20;
    std::cerr << "&rra = " << &rra << std::endl;
    std::cerr << "&rrb = " << &rrb << std::endl;

    // std::move
    int moved_a = 12;
    std::cerr << "&moved_a = " << &moved_a << std::endl;
    std::cerr << "moved_a = " << moved_a << std::endl;
    int &&rr_moved_a = std::move(moved_a);
    std::cerr << "&moved_a = " << &moved_a << std::endl;
    std::cerr << "moved_a = " << moved_a << std::endl;
    int another_moved_a = std::move(moved_a);
    std::cerr << "&moved_a = " << &moved_a << std::endl;
    std::cerr << "moved_a = " << moved_a << std::endl;
    std::string str_moved = "aaaaaa";
    std::cerr << "std_moved = " << str_moved << std::endl;
    std::cerr << "&std_moved = " << &str_moved << std::endl;
    std::string new_str = std::move(str_moved);
    std::cerr << "std_moved = " << str_moved << std::endl;
    std::cerr << "&std_moved = " << &str_moved << std::endl;
    str_moved = "bbbbbb";
    std::cerr << "std_moved = " << str_moved << std::endl;
    std::cerr << "&std_moved = " << &str_moved << std::endl;
    std::string && str_another = std::move(str_moved);
    std::cerr << "std_moved = " << str_moved << std::endl;
    std::cerr << "&std_moved = " << &str_moved << std::endl;

    int moved_num = 23;
    std::vector<int> vec;
    std::cerr << "moved_num = " << moved_num << std::endl;
    vec.push_back(std::move(moved_num));
    std::cerr << "moved_num = " << moved_num << std::endl;
    std::cerr << "&moved_num = " << &moved_num << std::endl;
    std::cerr << "&(vec[1]) = " << &(vec[1]) << std::endl;
  }
  // *右值引用、移动语义等
  std::cerr << "**********右值引用、移动语义等**********" << std::endl;
  {
    class MyClass {
      public:
       MyClass() : p(new int(0)) {
         std::cerr << "Default constructor" << std::endl;
       }
       MyClass(const MyClass &other) : p(new int(*(other.p))) {
         std::cerr << "Copy constructor" << std::endl;
       }
       MyClass(MyClass &&other) : p(new int(*(other.p))) {
         std::cerr << "Move constructor" << std::endl;
       }
       MyClass& operator=(const MyClass& other) {
          std::cerr << "Copy" << std::endl;
          this->p = new int(*(other.p));
       };
       MyClass& operator=(MyClass&& other) {
          std::cerr << "Copy" << std::endl;
          this->p = new int(*(other.p));
       };
      int* p;
    };
    MyClass mc1;
    MyClass mc2(mc1);
    MyClass mc3(std::move(mc1));
    std::cerr << "mc1.p = " << mc1.p << std::endl;
    std::cerr << "mc2.p = " << mc2.p << std::endl;
    std::cerr << "mc3.p = " << mc3.p << std::endl;

    class MyClassDefault {
     public:
      MyClassDefault() : p(new int(0)) {
        std::cerr << "Default constructor" << std::endl;
      }
      MyClassDefault(const MyClassDefault &other) : p(new int(*(other.p))) {
        std::cerr << "Copy constructor" << std::endl;
      }
      MyClassDefault(MyClassDefault &&other) : p(new int(*(other.p))) {
        std::cerr << "Move constructor" << std::endl;
      }
      ~MyClassDefault() {
        std::cerr << "Destructor" << std::endl;
      }
      int *p;
    };
    MyClassDefault mc1_d;
    MyClassDefault mc2_d(mc1_d);
    MyClassDefault mc3_d(std::move(mc1_d));
    std::cerr << "mc1_d.p = " << mc1_d.p << std::endl;
    std::cerr << "mc2_d.p = " << mc2_d.p << std::endl;
    std::cerr << "mc3_d.p = " << mc3_d.p << std::endl;

    MyClassDefault* test_new_and_delete = new MyClassDefault();
    delete test_new_and_delete;
  }
  return 0;
}