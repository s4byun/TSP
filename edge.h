#include "global.h"

class Vertex;

class Edge {
  public:
    Edge(Vertex* from, Vertex* to, int length);
    Vertex* getFrom() const;
    Vertex* getTo() const;
    int getLength() const;
    bool operator<(const Edge& right) const;

    ~Edge() {}

  private:
    Vertex* from;
    Vertex* to;
    int length;
};
