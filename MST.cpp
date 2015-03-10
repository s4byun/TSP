#include "MST.h"

MST::MST(float** input, int size) {
	adjacentMatrix = input;
	key = new int[size];   
  mstSet = new bool[size];  
	parent = new int[size];

	N = size;
}

MST::~MST() {

}

void MST::buildGraph() {
  for(int i=0; i<N; i++) {
    Vertex* v = new Vertex(i);
    vertices.push_back(v);
  }

  for(int row = 0; row < N; ++row) {
    Vertex* v1 = vertices[row];
    for(int col = row+1; col < N; col++) {
      addEdge(v1, vertices[col], adjacentMatrix[row][col]);
    }
  }
}

void MST::addEdge(Vertex* v1, Vertex* v2, int dist) {
  v1->addEdge(v2, dist);
  v2->addEdge(v1, dist);
}

int MST::makeMST() {
  priority_queue <Edge> pq;
  int sum=0;

  for(Vertex* v : vertices) {
    v->setVisited(false);
  }

  Vertex* start = vertices[0];
  start->setVisited(true);

  for(auto &it: start->edges) {
    pq.push(it.second);
  }

  while(!pq.empty()) {
    Edge edge = pq.top();
    pq.pop();

    Vertex* cur = edge.getTo();

    if(cur->wasVisited()) {
      cur->edges.erase(edge.getFrom()->getLabel());
      edge.getFrom()->edges.erase(cur->getLabel());
      continue;
    }

    sum += edge.getLength();
    cur->setVisited(true);

    for(auto &it: cur->edges) {
      if(!it.second.getTo()->wasVisited()) {
        pq.push(it.second);
      }
    }
  }

  return sum;
}

// A utility function to find the vertex with minimum key value, from
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

//calculate mean of all edges in the MST
float MST::calMean(int option) {
	float mean = 0.0;

	if(option == MST_1) {
		//calculate
	}else if(option == TSP2) {

	} else if(option == TSP1_5) {

	}

	return mean;
}

//calculate standard deviation of all edges in the MST
float MST::calStd(int option) {
	float std = 0.0;

	if(option == MST_1) {
		//calculate
	}else if(option == TSP2) {

	} else if(option == TSP1_5) {

	}

	return std;
}

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

void MST::makeTSP1_5() {
	
	//construct minimum-weight-matching for the given MST
	minimumMatching();

	//make all edges has even degree by combining mimimum-weight matching and MST
	combine();

	//calculate heuristic TSP cost 
}

void MST::minimumMatching() { //if you choose O(n^2)
	//find minimum-weight matching for the MST. 
	
	//you should carefully choose a matching algorithm to optimize the TSP cost.
}

void MST::combine() {
	//combine minimum-weight matching with the MST to get a multigraph which has vertices with even degree
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
