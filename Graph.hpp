//Mustafa Mannan U60366528


#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>
#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>

class Vertex {
public:
    std::string label;
    explicit Vertex(std::string label);
};

class Edge {
public:
    std::shared_ptr<Vertex> source, destination;
    unsigned long weight;
    Edge(std::shared_ptr<Vertex> src, std::shared_ptr<Vertex> dest, unsigned long wt);
};

class Graph {
private:
    std::unordered_map<std::string, std::shared_ptr<Vertex>> vertices;
    std::vector<std::shared_ptr<Edge>> edges;

public:
    Graph();
    void addVertex(const std::string& label);
    void removeVertex(const std::string& label);
    void addEdge(const std::string& label1, const std::string& label2, unsigned long weight);
    void removeEdge(const std::string& label1, const std::string& label2);
    unsigned long shortestPath(const std::string& startLabel, const std::string& endLabel, std::vector<std::string>& path);
};

#endif // GRAPH_HPP
