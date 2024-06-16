#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
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
  return 0;
}