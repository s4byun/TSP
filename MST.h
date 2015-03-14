#include "common.h"
#include "vertex.h"
#include <vector>
#include "Minmatching/PerfectMatching.h"

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
  void addEdge(Vertex *v1, Vertex* v2, int dist, int pm);
  vector<int> getOddVertices();
  void combine(PerfectMatching* pm, vector<int> oddVertices);

	//deliverable a
	int makeMST();
	void printMST();

	//deliverable b
	int makeTSP2();

	//deliverable c
  std::vector<int> makeTSP1_5();

  //EC1
	int twoOptE(vector<int> route, int oldSum);
  int twoOptP(vector<int> route, int oldSum);

	float calMean(int option);
	float calStd(int option);

private:
  std::vector<Vertex*> vertices;
	int minKey(int v_index); 
  vector<int> twoOptPSwap(const int i, const int j, vector<int> route);
  vector<int> twoOptESwap(const int i, const int j, vector<int> route);
  int totalDistance(vector<int> route);
};
