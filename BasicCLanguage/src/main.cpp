#include <iostream>
#include <string>
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
  // *58. 最后一个单词长度
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

  return 0;
}