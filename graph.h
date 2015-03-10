#include "vertex.h"
#include "global.h"

#include <unordered_map>

class Graph {
  public:
    Graph(float** adjacentMatrix, int N);
    ~Graph();

    void buildGraph();
    void addEdge(Vertex* v1, Vertex* v2, int dist);
    void printGraph();
    int makeMST();
    int makeTSP2();
    int makeTSP1p5();
    int totalWeight();

  private:
    std::unordered_map<int, Vertex*> vertices;  
    float** adjacentMatrix;
    int N;
};
