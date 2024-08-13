#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
using namespace std;

int minSubArrayLen(int target, std::vector<int> &nums) {
  int left = 0;
  int right = 1;
  int n = nums.size();
  int sum = nums[0];
  int len = n + 1;
  while (left < n && right <= n) {
    if (sum >= target) {
      len = std::min(len, right - left);
      std::cerr << "sum = " << sum << ", ";
      sum -= nums[left];
      left++;
      std::cerr << left << "------" << right << std::endl;
    } else {
      if (right == n)
        break;
      sum += nums[right];
      std::cerr << "sum = " << sum << ", ";
      right++;
      std::cerr << left << "------" << right << std::endl;
    }
  }
  return (len == n + 1) ? 0 : len;
}

std::vector<std::vector<int>> threeSum(std::vector<int> &nums) {
  int n = nums.size();
  sort(nums.begin(), nums.end());
  std::vector<std::vector<int>> res;
  for (int i = 0; i + 2 < nums.size(); i++) {
    int j = i + 1;
    int k = n - 1;
    std::cerr << i << ", " << j << ", " << k << std::endl;
    while (j < k) {
      if (nums[i] + nums[j] + nums[k] == 0) {
        std::vector<int> tmp = {nums[i], nums[j], nums[k]};
        res.emplace_back(tmp);
        while (j < k) {
          std::cerr << "while(j<k), j = " << j << ", k = " << k << std::endl;
          if (nums[j] == nums[j + 1]) {
            j++;
            std::cerr << i << ", " << j << ", " << k << std::endl;
          }
          if (nums[k] == nums[k - 1]) {
            k--;
            std::cerr << i << ", " << j << ", " << k << std::endl;
          }
        }
        j++;
        k--;
        std::cerr << i << ", " << j << ", " << k << std::endl;
      } else if (nums[i] + nums[j] + nums[k] < 0) {
        j++;
        std::cerr << i << ", " << j << ", " << k << std::endl;
      } else {
        k--;
        std::cerr << i << ", " << j << ", " << k << std::endl;
      }
    }
    while (i + 3 < nums.size()) {
      if (nums[i] == nums[i + 1]) {
        i++;
      } else {
        break;
      }
    }
  }
  return res;
}

int lengthOfLongestSubstring(std::string s) {
  int n = s.size();
  if (n == 0 || n == 1)
    return n;
  auto p = 1;
  auto q = p + 1;
  std::unordered_map<char, int> hash;
  int res = 1;
  hash[s[p - 1]] = p;
  while (true) {
    if (q > n)
      break;
    int flag = hash[s[q - 1]];
    if (flag == 0) {
      hash[s[q - 1]] = q;
      res = std::max(res, q - p + 1);
      std::cerr << "p = " << p << ", q = " << q << ", res = " << res
                << std::endl;
      q++;
    } else {
      while (p != flag) {
        hash[s[p - 1]] = 0;
        p++;
      }
      p++;
      hash[s[q - 1]] = q;
      q++;
    }
  }
  return res;
}

bool isIsomorphic(std::string s, std::string t) {
  unordered_map<char, char> hash;
  for (int i = 0; i < s.size(); i++) {
    auto it = hash.find(s[i]);
    if (it == hash.end()) {
      hash[s[i]] = hash[t[i]];
    } else {
      if (it->second != t[i])
        return false;
    }
  }
  return true;
}

int main(int argc, char **argv) {
  // *105. 前序中序构造二叉树
  {
    struct TreeNode {
      int val;
      TreeNode *left;
      TreeNode *right;
      TreeNode() : val(0), left(nullptr), right(nullptr) {}
      TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}
      TreeNode(int x, TreeNode *left, TreeNode *right)
          : val(x), left(left), right(right) {}
    };
    class Solution {
    public:
      TreeNode *buildTree(std::vector<int> &preorder,
                          std::vector<int> &inorder) {
        TreeNode *tree = new TreeNode(preorder[0]);
        int i = 0;
        for (; preorder[0] != inorder[i]; i++) {
        }
        count++;
        bool hasleft = false;
        bool hasright = false;
        if (i != 0) {
          hasleft = true;
        }
        if (i != preorder.size() - 1) {
          hasright = true;
        }
        std::cerr << "in the " << count << "th time, i = " << i
                  << ", root = " << preorder[0] << ", hasleft = " << hasleft
                  << ", hasright = " << hasright << std::endl;
        if (i != 0) {
          std::vector<int> preorder_left(preorder.begin() + 1,
                                         preorder.begin() + i + 1);
          std::vector<int> inorder_left(inorder.begin(), inorder.begin() + i);
          tree->left = buildTree(preorder_left, inorder_left);
          hasleft = true;
        }
        if (i != preorder.size() - 1) {
          std::vector<int> preorder_right(preorder.begin() + i + 1,
                                          preorder.end());
          std::vector<int> inorder_right(inorder.begin() + i + 1,
                                         inorder.end());
          tree->right = buildTree(preorder_right, inorder_right);
          hasright = true;
        }
        return tree;
      }

    private:
      int count = 0;
    };
    std::vector<int> preorder = {3, 9, 20, 15, 7};
    std::vector<int> inorder = {9, 3, 15, 20, 7};
    auto tree = Solution().buildTree(preorder, inorder);

    std::vector<int> vec1_temp = {1, 2, 3, 4};
    std::vector<int> vec2_tmp(vec1_temp.begin(), vec1_temp.begin() + 2);
    std::cerr << "vec1_tmp.end() == vec1_tmp.begin()+4?: "
              << (vec1_temp.end() == vec1_temp.begin() + 4) << std::endl;
    std::cerr << "vec2_tmp =" << vec2_tmp.size() << std::endl;
  }
  // *58. 最后一个单词长度（知识点：字符类型的判断）
  {
    char a = 'a';
    char b = 'B';
    char c = '1';
    char d = '*';
    std::cerr << "a = " << a << std::endl;
    std::cerr << "std::isalpha(a) = " << std::isalpha(a) << std::endl;
    std::cerr << "std::isupper(a) = " << std::isalpha(a) << std::endl;
    std::cerr << "std::islower(a) = " << std::isalpha(a) << std::endl;
    std::cerr << "std::isalnum(d) = " << std::isalnum(d) << std::endl;
    std::cerr << "std::islower(b) = " << std::islower(b) << std::endl;
    // 可见，isXXXX输出的是int，true时不一定为1，但是false一定为0
    auto A = std::toupper(a);
    char A_char = std::toupper(a);
    std::cerr << "A = " << A << ", type of A is " << typeid(A).name()
              << std::endl;
    std::cerr << "A_char = " << A_char << ", type of A_char is "
              << typeid(A_char).name() << std::endl;
    std::cerr << "std::tolower(b) = " << std::tolower(b) << std::endl;
    // 而toupper/tolower，默认也是int类型，但是int可以转成char，仍可以 char A =
    // std::toupper(A);
  }
  // *67 二进制求和（知识点：字符串的操作）
  {
    std::string a = "Aa";
    char b = 'b';
    a = b + a;
    std::cerr << "a = " << '"' << a << '"' << std::endl;
    char c = a[2];
    std::cerr << "c = " << c << std::endl;
    // 可以用索引访问，但不能用索引赋值。char和string操作时可以当string来用
    // insert相关：
    std::string origin_1 = "Ding";
    std::string insert_1 = "Zhi";
    origin_1.insert(1, insert_1);
    std::cerr << "origin_1 = " << origin_1 << std::endl;
    // origin_1.insert(2, 4, insert_1); 编译错误
    // origin_1.insert(2, 4, "insert_1"); 编译错误
    origin_1.insert(2, 4, '1'); // 设定次数的，只能加入字符，不能加入字符串
    std::cerr << "origin_1 = " << origin_1 << std::endl;
  }
  // *146 LRU缓存
  {
    class Node {
    public:
      Node *prev, *next;
      int key, value;
      Node(int key, int value) : key(key), value(value) {}
      Node() {}
    };

    class LRUCache {
    private:
      Node *dummy;
      int capacity;
      int size;
      std::unordered_map<int, Node *> key_to_node;

      void remove(Node *x) {
        x->prev->next = x->next;
        x->next->prev = x->prev; // 注意！这里双向链表，所以要两次链接！！！
        size--;
        key_to_node.erase(x->key);
      }

      void add(Node *x) {
        x->next = dummy->next;
        x->next->prev = x;
        dummy->next = x; // 这里也是两次链接！
        x->prev = dummy; // 这里也是两次链接！
        size++;
        key_to_node[x->key] = x;
      }

      Node *getNode(int key) {
        auto it = key_to_node.find(key);
        if (it == key_to_node.end())
          return nullptr;
        auto node = it->second;
        return node;
      }

    public:
      LRUCache(int capacity) : capacity(capacity), dummy(new Node()) {
        dummy->prev = dummy;
        dummy->next = dummy;
      }

      int get(int key) {
        auto node = getNode(key);
        if (!node) {
          return node->value;
        } else {
          return -1;
        }
      }

      void put(int key, int value) {
        auto node_new = new Node(key, value);
        auto node_get = getNode(key);
        if (!node_get) {
          add(node_new);
          if (size > capacity) {
            remove(dummy->prev);
          }
        } else {
          remove(node_get);
          add(node_new);
        }
      }
    };

    LRUCache cache = LRUCache(2);
    cache.put(1, 1);
    cache.put(2, 2);
    cache.put(3, 3);
    cache.put(2, 4);
    // std::cerr << "cache.get(1) = " << cache.get(1) << std::endl;
    // std::cerr << "cache.get(2) = " << cache.get(2) << std::endl;
    // std::cerr << "cache.get(3) = " << cache.get(3) << std::endl;
    // std::cerr << "cache.get(4) = " << cache.get(4) << std::endl;
  }
  // *88 合并两个有序数组
  {
    std::vector<int> vec1{0, 1, 3, 23, 3, 6, 9, 2};
    sort(vec1.begin(), vec1.end());
    std::cerr << "vec1 sort default:";
    for (auto val : vec1) {
      std::cerr << " " << val;
    }
    std::cerr << std::endl << "vec1 sort greater:";
    sort(vec1.begin(), vec1.end(), std::greater<int>());
    for (auto val : vec1) {
      std::cerr << " " << val;
    }
    std::cerr << std::endl << "vec1 sort less:";
    sort(vec1.begin(), vec1.end(), std::less<int>());
    for (auto val : vec1) {
      std::cerr << " " << val;
    }
    std::cerr << std::endl;
  }
  // *27 移除元素
  {
    std::cerr << "**No. 27 移除元素" << std::endl;
    std::vector<int> vec1{0, 1, 2, 3, 4, 5, 6, 7};
    std::cerr << "vec1.capacity = " << vec1.capacity() << std::endl;
    auto it1 = vec1.begin() + 3;
    auto it2 = it1 + 1;
    vec1.erase(it1);
    std::cerr << "vec1.capacity after = " << vec1.capacity() << std::endl;
    std::cerr << "vec1 after erase:";
    for (auto val : vec1) {
      std::cerr << " " << val;
    }
    std::cerr << std::endl;
    std::cerr << "it1 now = " << *it1 << std::endl;
    // **** experiment 1 **** //
    {
      std::vector<int> vec1{0};
      auto it1 = vec1.begin();
      auto it2 = it1 + 1;
      std::cerr << "it1 == vec1.end?: " << (it1 == vec1.end()) << std::endl;
      std::cerr << "it2 == vec1.end?: " << (it2 == vec1.end()) << std::endl;
      vec1.erase(it1);
      std::cerr << "it1 == vec1.end?: " << (it1 == vec1.end()) << std::endl;
      std::cerr << "it2 == vec1.end?: " << (it2 == vec1.end()) << std::endl;
    }
    // **** experiment 2 **** //
    {
      std::vector<int> vec1{1, 2};
      auto it1 = vec1.begin();
      std::cerr << "*it1 before erase = " << *it1 << std::endl;
      vec1.erase(it1);
      std::cerr << "*it1 after erase = " << *it1 << std::endl;
    }
    // ********* //
    // {
    //   // std::vector<int> nums{0, 1, 2, 3, 4, 5, 6, 7, 3, 5, 63, 3, 2};
    //   std::vector<int> nums{3, 2, 2, 3};
    //   int val = 3;
    //   int count = 0;
    //   int total = nums.size();
    //   auto it1 = nums.begin();
    //   auto it2 = it1;
    //   while (it1 != nums.end()) {
    //     if (*it1 == val) {
    //       it2 = it1 + 1;
    //       nums.erase(it1);
    //       count++;
    //       it1 = it2;
    //     } else {
    //       it1++;
    //     }
    //   }
    //   std::cerr << "left = " << total - count << std::endl;
    // }
  }

  // *15 三数之和
  {
    std::cerr << "**No. 15 三数之和" << std::endl;
    std::vector<int> tmp = {-4, -1, -1, 0, 1, 2};
    // threeSum(tmp);
  }

  // *209 长度最小的子数组
  {
    std::cerr << "**No. 209 长度最小的子数组" << std::endl;
    std::vector<int> tmp = {2, 3, 1, 2, 4, 3};
    int target = 7;
    auto res = minSubArrayLen(target, tmp);
    std::cerr << "result = " << res << std::endl;
  }

  // *3 无重复字符的最长子串
  {
    std::cerr << "**No. 3 无重复字符的最长子串" << std::endl;
    std::string str = "pwwkew";
    auto res = lengthOfLongestSubstring(str);
    std::cerr << "res = " << res << std::endl;
  }
  return 0;
}