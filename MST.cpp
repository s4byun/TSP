#include "MST.h"
#include <cmath>

MST::MST(float** input, int size) {
  adjacentMatrix = input;

  key = new int[size];   
  mstSet = new bool[size];  
  parent = new int[size];
  N = size;
}

MST::~MST() {
  for(int i=0; i<N; i++) {
    delete vertices[i];
  }
}

int MST::totalWeight() {
  int sum = 0;
  for(Vertex* v : vertices) {
    for(auto& it : v->edges) {
      sum += it.second.getLength();
    }
  }

  return sum/2;
}

//A utility function to find the vertex with minimum key value, from
// the set of vertices not yet included in MST
int MST::minKey(int key[], bool mstSet[])
{
  // Initialize min value
  int min = INT_MAX, min_index;

  for (int v = 0; v < N; v++)
    if (mstSet[v] == false && key[v] < min)
      min = key[v], min_index = v;

  return min_index;
}

void MST::buildGraph() {
  // Initialize all keys as INFINITE
  for (int i = 0; i < N; i++)
    key[i] = MAX_INT, mstSet[i] = false;

  // Always include first 1st vertex in MST.
  key[0] = 0;     // Make key 0 so that this vertex is picked as first vertex
  parent[0] = -1; // First node is always root of MST


  // The MST will have V vertices
  for (int count = 0; count < N-1; count++)
  {
    // Pick the minimum key vertex from the set of vertices
    // not yet included in MST
    int u = minKey(key, mstSet);
    mstSet[u] = true;
    for (int v = 0; v < N; v++)
      // mstSet[v] is false for vertices not yet included in MST
      // Update the key only if adjacentMatrix[u][v] is smaller than key[v]
      if (adjacentMatrix[u][v] && mstSet[v] == false && adjacentMatrix[u][v] <  key[v])
        parent[v]  = u, key[v] = adjacentMatrix[u][v];
  }


  for(int i=0; i<N; ++i) {
    Vertex* v = new Vertex(i);
    vertices.push_back(v);
  }

  for(int i=1; i<N; ++i) {
    Vertex* from = vertices[i];
    Vertex* to = vertices[parent[i]];
    int dist = adjacentMatrix[i][parent[i]];
    addEdge(from, to, dist, 1);
  }
  /*
     for(int i=0; i<N; i++) {
     Vertex* v = new Vertex(i);
     vertices.push_back(v);
     }

     for(int row = 0; row < N; ++row) {
     Vertex* v1 = vertices[row];
     for(int col = row+1; col < N; col++) {
     addEdge(v1, vertices[col], adjacentMatrix[row][col], 1);
     }
     }*/
}

void MST::addEdge(Vertex* v1, Vertex* v2, int dist, int pm) {
  v1->addEdge(v2, dist, pm, N);
  v2->addEdge(v1, dist, pm, N);
}

// TSP2
int MST::makeTSP2() {
  int sum = 0, counter = 0;
  bool fromDeadEnd = false;
  std::stack<Edge> s;
  Vertex* cur;

  // Initialization: set all visited values to false.
  for(Vertex* v : vertices) {
    v->setVisited(false);
  }

  // Starting vertex
  Vertex* start = vertices[0];
  start->setVisited(true);

  // Edges from starting vertex
  for(auto &it: start->edges) {
    s.push(it.second);
  }

  // There are N-1 edges excluding the one that goes back to the starting vertex
  while(counter < N-1) {
    Edge e = s.top();
    cur = e.getTo();
    s.pop();

    // Skip visited vertices
    if(cur->wasVisited()) {
      continue;
    }

    cur->setVisited(true);
    if(!fromDeadEnd) {
      sum += e.getLength();
    }

    ++counter;
    fromDeadEnd = false;

    if(cur->isDeadEnd()) {
      if(s.empty()) {
        break;
      }
      sum += adjacentMatrix[cur->getLabel()][s.top().getTo()->getLabel()];
      fromDeadEnd = true;
      continue;
    }

    for(auto &it: cur->edges) {
      if(!it.second.getTo()->wasVisited()) {
        s.push(it.second);
      }
    }
  }

  sum += adjacentMatrix[cur->getLabel()][start->getLabel()];
  return sum;
}

// Combine minimum weight matching with MST.
void MST::combine(PerfectMatching* pm, vector<int> oddVertices) {
  for(int i=0; i<oddVertices.size(); ++i) {
    int j = pm->GetMatch(i);
    if(i < j) {
      Vertex* from = vertices[oddVertices[i]];
      Vertex* to = vertices[oddVertices[j]];
      int dist = adjacentMatrix[oddVertices[i]][oddVertices[j]];
      addEdge(from, to, dist, -1);
    }
  }
}

vector<int> MST::makeTSP1_5() {
  std::stack<Vertex*> s;
  std::stack<Vertex*> c; // Euler circuit
  std::vector<int> route;
  Vertex* v = vertices[0];

  // Euler tour
  do{
    if(v->edges.empty()){
      c.push(v);
      v = s.top();
      s.pop();
    }
    else {
      s.push(v);
      Edge e = v->edges.begin()->second;
      int index = v->edges.begin()->first;

      Vertex* to = e.getTo(); 
      v->edges.erase(index);
      if(index > N)
        to->edges.erase(N + v->getLabel());
      else 
        to->edges.erase(v->getLabel());
      v = to;
    }
  } while(!s.empty());

  // Hamiltonian path
  for(Vertex* u : vertices) {
    u->setVisited(false);
  }

  while(!c.empty()) {
    if(c.top()->wasVisited()) {
      c.pop();
      continue;
    }
    Vertex* u = c.top();
    c.pop();
    u->setVisited(true);
    route.push_back(u->getLabel());
  }

  return route;
}

int MST::twoOptP(vector<int> route, int oldSum) {
  int improve = 0;
  int best_sum = oldSum;
  while(improve < 20) {
    for(int i=1; i<route.size()-2; ++i) {
      for(int j=i+1; j<route.size()-1; ++j) {
        int new_sum;
        if(j-i == 1) {
          new_sum = best_sum - adjacentMatrix[route[i-1]][route[i]] 
            - adjacentMatrix[route[j]][route[j+1]]
            + adjacentMatrix[route[i-1]][route[j]]
            + adjacentMatrix[route[i]][route[j+1]];
        }
        else {
          new_sum = best_sum - adjacentMatrix[route[i-1]][route[i]]
            - adjacentMatrix[route[i]][route[i+1]]
            - adjacentMatrix[route[j-1]][route[j]]
            - adjacentMatrix[route[j]][route[j+1]]
            + adjacentMatrix[route[j-1]][route[i]]
            + adjacentMatrix[route[j+1]][route[i]]
            + adjacentMatrix[route[i-1]][route[j]]
            + adjacentMatrix[route[i+1]][route[j]];
        }
        if(new_sum < best_sum) {
          best_sum = new_sum;
          route = twoOptPSwap(i, j, route);
          improve=0;
        }
      }
    }
    ++improve;
  }

  return best_sum;
}

vector<int> MST::twoOptPSwap(const int i, const int j, vector<int> route) {
  vector<int> new_route = route;

  new_route[i] = route[j];
  new_route[j] = route[i];

  return new_route;
}

int MST::twoOptE(vector<int> route, int oldSum) {
  int improve = 0;

  int best_sum = oldSum; 
  while(improve < 20) {
    for(int i=1; i<route.size()-2; ++i) {
      for(int j=i+1; j<route.size()-1; ++j) {
        int new_sum = best_sum - adjacentMatrix[route[i-1]][route[i]]
          - adjacentMatrix[route[j]][route[j+1]]
          + adjacentMatrix[route[i-1]][route[j]]
          + adjacentMatrix[route[i]][route[j+1]];
        if(new_sum < best_sum) {
          improve = 0;
          route = twoOptESwap(i, j, route);
          best_sum = new_sum;
        }
      }
    }
    ++improve;
  }

  return best_sum; 
}

vector<int> MST::twoOptESwap(const int i, const int j, vector<int> route) {
  vector<int> new_route = route;
  int k=0;

  for(int x=i; x < j+1 ; ++x) {
    new_route[x] = route[j-k];
    ++k;
  }

  return new_route;
}


vector<int> MST::getOddVertices() {
  vector<int> oddVertices; 

  for(Vertex* v : vertices) {
    if(v->edges.size() % 2 != 0) {
      oddVertices.push_back(v->getLabel());
    }
  }

  return oddVertices;
}

// A utility function to print the constructed MST stored in parent[]
void MST::printMST() {
  for(Vertex* v : vertices) { 
    cout << "vertex " << v->getLabel() << ": ";
    for(auto &it2: v->edges) { 
      cout << it2.second.getTo()->getLabel() << "(" << it2.second.getLength() << ") ";
    }
    cout << endl;
  }
}


