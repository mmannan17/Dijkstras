// undirected weighted Graph ADT implementation with shortest path Dijkstra's algorithm
// Mustafa Mannan U60366528

#include "Graph.hpp"


Vertex::Vertex(std::string label) : label(std::move(label)) {}


Edge::Edge(std::shared_ptr<Vertex> src, std::shared_ptr<Vertex> dest, unsigned long wt) 
    : source(std::move(src)), destination(std::move(dest)), weight(wt) {}


Graph::Graph() = default;

// Method to add a vertex to the graph
void Graph::addVertex(const std::string& label) {
    // Check if vertex already exists and add it if not
    if (vertices.find(label) == vertices.end()) {
        vertices[label] = std::make_shared<Vertex>(label);
    } else {
        // If vertex exists, print an error message
        std::cerr << "Vertex '" << label << "' already exists.\n";
    }
}

// Method to remove a vertex from the graph
void Graph::removeVertex(const std::string& label) {
    // Find the vertex to remove
    auto it = vertices.find(label);
    if (it != vertices.end()) {
        // Remove the vertex and its associated edges
        vertices.erase(it);
        edges.erase(std::remove_if(edges.begin(), edges.end(), 
            [&](const std::shared_ptr<Edge>& edge) {
                // Lambda to find edges connected to this vertex
                return edge->source->label == label || edge->destination->label == label;
            }), edges.end());
    } else {
        // If vertex not found, print an error message
        std::cerr << "Vertex '" << label << "' not found.\n";
    }
}

// Method to add an edge to the graph
void Graph::addEdge(const std::string& label1, const std::string& label2, unsigned long weight) {
    // Find the vertices to connect
    auto v1 = vertices.find(label1);
    auto v2 = vertices.find(label2);

    // Check if both vertices exist before adding an edge
    if (v1 == vertices.end() || v2 == vertices.end()) {
        std::cerr << "Both vertices must exist to add an edge.\n";
        return;
    }

    // Check if trying to add an edge to the same vertex
    if (label1 == label2) {
        std::cerr << "Cannot add an edge to the same vertex.\n";
        return;
    }

    
    edges.push_back(std::make_shared<Edge>(v1->second, v2->second, weight));
}

// Method to remove an edge from the graph
void Graph::removeEdge(const std::string& label1, const std::string& label2) {
    // Find the edge to remove
    auto it = std::find_if(edges.begin(), edges.end(), 
        [&](const std::shared_ptr<Edge>& edge) {
            // Lambda to match the edge in either direction
            return (edge->source->label == label1 && edge->destination->label == label2) ||
                   (edge->source->label == label2 && edge->destination->label == label1);
        });

    // Remove edge 
    if (it != edges.end()) {
        edges.erase(it);
    } else {
        // If edge not found, print an error message
        std::cerr << "Edge between '" << label1 << "' and '" << label2 << "' not found.\n";
    }
}


// Method to find the shortest path between two vertices using Dijkstra's algorithm
unsigned long Graph::shortestPath(const std::string& startLabel, const std::string& endLabel, std::vector<std::string>& path) {
    // Reset the path to be empty initially
    path.clear();

    // Check if start and end exist
    if (vertices.find(startLabel) == vertices.end() || vertices.find(endLabel) == vertices.end()) {
        
        return std::numeric_limits<unsigned long>::max(); // Start or end vertex not found
    }

    // Initialize all distances to infinity and the start vertex's distance to 0
    std::unordered_map<std::string, unsigned long> distances;
    for (const auto& vertexPair : vertices) {
        distances[vertexPair.first] = std::numeric_limits<unsigned long>::max();
    }
    distances[startLabel] = 0;

    
    std::unordered_map<std::string, std::string> predecessors;

    // Keep track of visited vertices to prevent re-processing
    std::unordered_set<std::string> visited;

    // priority queue to determine the next vertex with the smallest distance
    std::priority_queue<std::pair<unsigned long, std::string>, std::vector<std::pair<unsigned long, std::string>>, std::greater<>> pq;
    pq.emplace(0, startLabel);

    while (!pq.empty()) {
        std::string currentVertex = pq.top().second;
        pq.pop();

        // Skip  vertex if it;s already been visited
        if (visited.find(currentVertex) != visited.end()) continue;
        visited.insert(currentVertex);

        // If the current vertex is the destination, reconstruct and return shortest path
        if (currentVertex == endLabel) {
            std::string current = endLabel;
            while (current != startLabel) {
                path.insert(path.begin(), current);
                current = predecessors[current];
            }
            path.insert(path.begin(), startLabel);
            return distances[endLabel];
        }

       
        for (const auto& edge : edges) {
            std::shared_ptr<Vertex> neighbor = nullptr;

            // Check both directions of the edge 
            if (edge->source->label == currentVertex) {
                neighbor = edge->destination;
            } else if (edge->destination->label == currentVertex) {
                neighbor = edge->source;
            }

            // If neighbor is found, see if there's a shorter path 
            if (neighbor) {
                unsigned long newDist = distances[currentVertex] + edge->weight;
                if (newDist < distances[neighbor->label]) {
                    distances[neighbor->label] = newDist;
                    predecessors[neighbor->label] = currentVertex;
                    // Always push new distance into priority queue
                    pq.emplace(newDist, neighbor->label);
                }
            }
        }
    }

    // If no path was found return infinity
    if (predecessors.find(endLabel) == predecessors.end()) {
        return std::numeric_limits<unsigned long>::max();
    } else {
        // Reconstruct the shortest path
        std::vector<std::string> tempPath;
        std::string current = endLabel;
        while (current != startLabel) {
            tempPath.push_back(current);
            
            if (predecessors.find(current) == predecessors.end()) {
                return std::numeric_limits<unsigned long>::max(); 
            }
            current = predecessors[current];
        }
        tempPath.push_back(startLabel);
       
        path.assign(tempPath.rbegin(), tempPath.rend());
        return distances[endLabel];
    }
}
