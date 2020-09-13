/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node *left;
  Node *right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
  Node *root;

  KdTree() : root(NULL) {}

  void insert_at_node(Node *&node, std::vector<float> point, int id,
                      int depth) {
    if (node == NULL) {
      // insert root, or at the bottom level
      node = new Node(point, id);
    } else {
      if (depth % 2 == 0) {
        if (point[0] <= node->point[0]) {
          insert_at_node(node->left, point, id, depth + 1);
        } else {
          insert_at_node(node->right, point, id, depth + 1);
        }
      } else {
        if (point[1] <= node->point[1]) {
          insert_at_node(node->left, point, id, depth + 1);
        } else {
          insert_at_node(node->right, point, id, depth + 1);
        }
      }
    }
  }
  void insert(std::vector<float> point, int id) {
    insert_at_node(root, point, id, 0);
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    return ids;
  }
};
