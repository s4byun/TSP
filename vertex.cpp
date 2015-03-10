#include "vertex.h"

Vertex::Vertex(int label) {
  this->label = label; 
}

Vertex::~Vertex() {}

void Vertex::addEdge(Vertex* to, int distance) {
  Edge newEdge(this, to, distance);
  std::pair<int, Edge> newPair(to->getLabel(), newEdge);
  edges.insert(newPair);
}

std::pair<int, Vertex*> Vertex::getMinTo() {
  int min = MAX_INT;
  Vertex* minVertex;

  for(auto &it: edges) {
    if(!(it.second.getTo()->wasVisited())) {
      if(min > it.second.getLength()) {
        min = it.second.getLength();
        minVertex = it.second.getTo();
      }
    }
  }

  std::pair<int, Vertex*> minPair(min, minVertex); 
  return minPair;
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
