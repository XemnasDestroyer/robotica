#pragma once
#include <map>
#include <set>
#include <string>
#include <vector>

enum class NodeType { ROOM, DOOR };

struct Node
{
    int id;
    NodeType type;
};

class Graph
{
public:
    void add_node(int id, NodeType type);
    void add_edge(int id1, int id2);

    const std::map<int, Node>& nodes() const { return nodes_; }
    const std::map<int, std::set<int>>& edges() const { return edges_; }

private:
    std::map<int, Node> nodes_;
    std::map<int, std::set<int>> edges_;
};
