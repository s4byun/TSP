#include "edge.h"

Edge::Edge(Vertex* from, Vertex* to, int length) {
  this->from = from;
  this->to = to;
  this->length = length;
}

Vertex* Edge::getFrom() const {
  return from;
}

Vertex* Edge::getTo() const {
  return to;
}

int Edge::getLength() const {
  return length;
}

bool Edge::operator<(const Edge& right) const {
  return length > right.length;
}
