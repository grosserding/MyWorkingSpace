#include <iostream>
#include <vector>
// #include <cstdlib> // 生成随机数的库，包含rand() srand()
// #include <ctime> // 包含time()函数
#include <algorithm>  // sort()
#include <random>
#include <string>
#include <typeinfo>

//##1. zweite suchen
// standard loesung, 给一个值，如果查到返回idx，如果找不到，返回-1并插入。
int Search(std::vector<double>& list, double goal) {
  int right = list.size() - 1, left = 0;
  while (left <= right) {
    int mid = left + (right - left) / 2;
    if (list[mid] == goal) {
      return mid;
    }
    if (list[mid] < goal) {
      left = mid + 1;
    } else {
      right = mid - 1;
    }
  }
  list.insert(list.begin() + left, goal);
  return -1;
}

int main(int argc, char** argv) {
  std::cout << "# Allgemaine Fragen" << std::endl;
  //# Allgemaine Fragen
  {
    //##1. zweite suchen
    std::cout << "## zweite suchen." << std::endl;
    std::vector<double> list;
    std::random_device rd;
    for (int i = 0; i < 100; i++) {
      auto tmp_rd = rd();
      if (!i) {
        std::cout << typeid(tmp_rd).name() << std::endl;
        // 返回值为"j",表示该值为long或long long
      }
      double tmp = ((double)(tmp_rd / 100000)) / 100;
      std::cout << tmp << ", ";
      list.emplace_back(tmp);
    }
    std::cout << std::endl;
    sort(list.begin(), list.end());
    int n = 15;
    double goal1 = list[n];
    double goal2 = 50.0;
    std::cout << "before Search, list len = " << list.size() << std::endl;
    int idx1 = Search(list, goal1);
    std::cout << "after search 1st, size = " << list.size()
              << ", idx1 = " << idx1 << std::endl;
    int idx2 = Search(list, goal2);
    std::cout << "after search 2nd, size = " << list.size()
              << ", idx2 = " << idx2 << std::endl;
    // // 确定有的查找
    // int left = 0, right = list.size() - 1;
    // while (left <= right) {
    //   int mid = left + (right - left) / 2;
    //   if (list[mid] == goal) {
    //     std::cout << "goal idx = " << mid << std::endl;
    //     break;
    //   }
    //   if (list[mid] < goal) {
    //     left = mid;
    //   } else {
    //     right = mid;
    //   }
    // }
    // // 插入
    // double insert_value = 10000;
    // int goal_idx;
    // left = 0;
    // right = list.size() - 1;
    // bool inserted = false;
    // if (list[left] >= insert_value) {
    //   goal_idx = left;
    //   list.insert(list.begin() + goal_idx, insert_value);
    //   inserted = true;
    // } else if (list[right] <= insert_value) {
    //   goal_idx = list.size();
    //   list.emplace_back(insert_value);
    //   inserted = true;
    // } else {
    //   while (left != right - 1) {
    //     int mid = left + (right - left) / 2;
    //     if (list[mid] == insert_value) {
    //       goal_idx = mid;
    //       list.insert(list.begin() + goal_idx, insert_value);
    //       inserted = true;
    //       break;
    //     } else if (list[mid] < insert_value) {
    //       left = mid;
    //     } else {
    //       right = mid;
    //     }
    //   }
    // }
    // if (!inserted) {
    //   goal_idx = right;
    //   list.insert(list.begin() + goal_idx, insert_value);
    // }
    // std::cout << "inserted position = " << goal_idx << std::endl;
    // for (auto tmp : list) std::cout << tmp << ", ";
    // std::cout << std::endl;

    //** 关于typeid().name()的输出列表：
    // b:bool
    // c:char
    // d:double
    // e:long double
    // f:float
    // i:int32_t
    // j:uint32_t
    // s:int16_t(short)
    // t:uint16_t(unsigned short)
    // x:int64_t
    // y:uint64_t
    // NSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE:string
    // St6vectorI*SaI*EE:类型为 * 的 vector
    // St4pairI*^E：类型分别为 * ^ 的 pair
    // (省流：STL 找子串)
    // A*_-:... 类型的长度为 * 的数组
    // P-:... 的指针
    // PK-:const ... 的指针
  }

  {
    //## zweite suchen
    std::cout << "## zweite suchen." << std::endl;
  }
  return 0;
}