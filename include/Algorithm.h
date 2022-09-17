#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <iostream>
#include <list>
#include <math.h>
#include <vector>

// Define constant cost
const int move_straight = 10;
const int move_incline = 14;

struct RobotConfiguration
{
    int x, y; // Coordinate
};

struct Priority
{
    double g; // cost
    double h; // heuristic
    double f; // f = g+h
};

struct Node
{
    RobotConfiguration robot_config_;
    Priority priority_;
    Node *parent;

    // initialize node
    Node(RobotConfiguration config, Priority pri)
        : robot_config_(config), priority_(pri), parent(NULL) {}
};

class A_star
{
    public:
     // Get path API
     std::list<Node *> get_path(Node &start_node, Node &end_node,
                             bool isIgnoreCorner);

     // Initialize map
     void init_map(std::vector<std::vector<int>> &_map);

    private:
        // Find path from start_node to end_node
        Node *find_path(Node &start_node, Node &end_node, bool isIgnoreCorner);

        // Get Surrounding nodes
        std::vector<Node *> get_surrounding_nodes(const Node *current_node,
                                               bool isIgnoreCorner) const;

        // Justify whether node is free or is inside obstacle
        bool is_free(const Node *current_node, const Node *target_node,
                   bool isIgnoreCorner) const;

        // Justify whether node is in open/close list or not
        Node *is_in_list(const std::list<Node *> &list, const Node *node) const;

        // Get node with minimum cost
        Node *get_min_cost();

        // Calculate Priority
        double calculate_g(Node *parent_node, Node *current_node);
        double calculate_h(Node *current_node, Node *end_node);
        double calculate_f(Node *current_node);

    private:
        std::vector<std::vector<int>> map;
        std::list<Node *> openlist;
        std::list<Node *> closelist;
};

#endif // ALGORITHM_H
