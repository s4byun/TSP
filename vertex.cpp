#include "vertex.h"

Vertex::Vertex(int label) {
  this->label = label; 
}

Vertex::~Vertex() {}

void Vertex::addEdge(Vertex* to, int distance, int pm, int N) {
  Edge newEdge(this, to, distance);
  if(pm < 0) {
    std::pair<int, Edge> newPair(to->getLabel() + N, newEdge);
    edges.insert(newPair);
  }
  else {
    std::pair<int, Edge> newPair(to->getLabel(), newEdge);
    edges.insert(newPair);
  }
}

int Vertex::getMinTo() {
  int min = MAX_INT;
  int index;

  for(auto &it: edges) {
    if(min > it.second.getLength()) {
      min = it.second.getLength();
      index = it.first;
    }
  }

  return index;
}

bool Vertex::isDeadEnd() {
  for(auto &it: edges) {
    if(!(it.second.getTo()->wasVisited())) {
      return false;
    }
  }

  return true;
}

bool Vertex::wasVisited() const {
  return visited;
}

int Vertex::getLabel() const {
  return label;
}

void Vertex::setVisited(bool visited) {
  this->visited = visited;
}
