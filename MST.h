#include "common.h"
#include "vertex.h"
#include <vector>

#pragma once

class MST {
public:
	float** adjacentMatrix;
	int* parent; //Array to store constructed MST
	int* key; //Key values used to pick minimum weight edge in cut
	bool* mstSet; //To represent set of vertices not yet included in MST
	int N; //the size of pointset

	MST(float** adjacentMatrix, int size);
	~MST();

  void buildGraph();
  void addEdge(Vertex *v1, Vertex* v2, int dist);
  vector<int> getOddVertices();

	//deliverable a
	int makeMST();
	void printMST();

	//deliverable b
	int makeTSP2();

	//deliverable c
	void makeTSP1_5();
	
	float calMean(int option);
	float calStd(int option);

private:
  std::vector<Vertex*> vertices;
	void minimumMatching();
	void combine();
	int minKey(int key[], bool mstSet[]);

};
