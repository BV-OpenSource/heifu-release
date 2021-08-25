#ifndef RTT_TREE_H
#define RTT_TREE_H

#include "Planner/Node.hpp"
#include "Planner/StateSpace.hpp"

namespace RRT3D {

    class RRT_Tree {
        public:
            RRT_Tree();

            void insert(Node *newNode, Node *leaf);

            void printTree();
            void printTreeAfterNode(Node *leaf);
            void printChilds(Node *leaf);

            Node *getNearestNeighbor(Node *randomNode, StateSpace *stateSpace, double &minDistance);

            Node *extendNode(Node *currentNode, Node *randomNode, double epsilon);

            std::vector<Node *> extractFullPath(Node *goal, Node *start, StateSpace *stateSpace);

            std::vector<Node *> processFinalRobotPath(std::vector<Node *>);

            void CleanTree(Node *leaf);

            Node *GetRoot();
            void SetRoot(Node *newRoot);
        private:
            Node *root;

            double getEuclideanDistance(Node *node1, Node *node2);
    };
}
#endif //RTT_TREE_H
