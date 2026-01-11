#include "graph.h"

void Graph::add_node(int id, NodeType type)
{
    if (!nodes_.contains(id))
        nodes_[id] = Node{id, type};
}

void Graph::add_edge(int id1, int id2)
{
    edges_[id1].insert(id2);
    edges_[id2].insert(id1);   // no dirigido
}
