#include "edge.h"
#include <queue>
#include <unordered_map>
#include <vector>

class compareEdge {
  public:
    bool operator() (Edge& e1, Edge& e2) {
      return e1.getLength() < e2.getLength();
    }
};
class Vertex {
  friend class MST;
  public:
    Vertex(int label);
    ~Vertex();

    void addEdge(Vertex* to, int distance);
    bool wasVisited() const;
    int getLabel() const;

    void setVisited(bool visited);

    std::pair<int, Vertex*> getMinTo(); 
    bool isDeadEnd(); 

  private:
    int label;
    bool visited;
    std::unordered_map<int, Edge> edges;
    std::priority_queue<Edge, vector<Edge>, compareEdge> p_edges;
    std::vector<Edge> v_edges;
};

