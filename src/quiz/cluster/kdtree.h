/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <math.h>
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

  void search_at_node(Node *node, std::vector<float> &target,
                      std::vector<int> &ids, float distanceTol, int depth) {
    if (node == NULL) {
      return;
    }
    // calculate L2 distance
    float dis = sqrt(pow(target[0] - node->point[0], 2) + pow(target[1] - node->point[1], 2));
    if (dis <= distanceTol) {
      ids.push_back(node->id);
    }
    int index = depth % 2;
    float diff = target[index] - node->point[index];
    if (fabs(diff) <= distanceTol) {
      search_at_node(node->left, target, ids, distanceTol, depth + 1);
      search_at_node(node->right, target, ids, distanceTol, depth + 1);
    } else if (diff <= 0) {
      search_at_node(node->left, target, ids, distanceTol, depth + 1);
    } else {
      search_at_node(node->right, target, ids, distanceTol, depth + 1);
    }
  }

  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    search_at_node(root, target, ids, distanceTol, 0);
    return ids;
  }
};
