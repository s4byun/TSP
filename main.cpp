
#include "common.h"
#include "point.h"
#include "MST.h"
#include "Minmatching/PerfectMatching.h"

#include <math.h>
/*
   This project is a starter code and wrappers for CSE101W15 Implementation project.

   point.h - uniform random pointset generator

   MST.h - minimum spanning tree

   PerfectMatching.h - interface to min cost perfect matching code 

   -------------------------------------
   PerfectMatching is from the paper:

   Vladimir Kolmogorov. "Blossom V: A new implementation of a minimum cost perfect matching algorithm."
   In Mathematical Programming Computation (MPC), July 2009, 1(1):43-67.

sourcecode : pub.ist.ac.at/~vnk/software/blossom5-v2.05.src.tar.gz

*/

void LoadInput(int& node_num, int& edge_num, int*& edges, int*& weights, float** adjacentMatrix, vector<int>& oddVertices, int N) {
  int e = 0;
  node_num = N;
  edge_num = N*(N-1)/2 ; //complete graph

  edges = new int[2*edge_num];
  weights = new int[edge_num];

  for(int i = 0; i < N ; ++i) {
    for(int j = i+1 ; j< N ; ++j) {
      edges[2*e] = i;
      edges[2*e+1] = j;
      weights[e] = adjacentMatrix[oddVertices[i]][oddVertices[j]];
      e++;
    }
  }

  if (e != edge_num) { 
    cout<<"the number of edge is wrong"<<endl;

    exit(1); 
  }
}

void PrintMatching(int node_num, PerfectMatching* pm, vector<int> oddVertices) {
  int i, j;

  for (i=0; i<node_num; i++) {
    j = pm->GetMatch(i);
    if (i < j) printf("%d %d\n", oddVertices[i], oddVertices[j]);
  }
}

double calculateMean(std::vector<int> s, int trial) {
  int sum = 0;
  for(int i=0; i<trial; ++i) {
    sum += s[i];
  }
  return (double) sum/trial;
}

double calculateSD(std::vector<int> s, double mean) {
  double sum;

  for(int i=0; i<s.size(); i++) {
    sum += pow((mean - (double) s[i]), 2);
  }
    
  return sqrt(sum / (s.size() - 1));
}
int main() {
  set< pair<int,int> > generatedPointset;
  float** adjacentMatrix;
  int W, H, N;
  int trial;

  W = 13134;
  H = 11638;
  N = 8699;
  trial = 10;

  cout<<"W: "<<W<<" H: "<<H<<" N:"<<N<<endl;
  
  vector<int> mst_sum, tsp2_sum, tsp15_sum, twoOptP_sum, twoOptE_sum;

  for(int t=0; t < trial; ++t) {
    Point pointset;
    cout << "Trial " << t+1 << ": " << endl;
    pointset.generatePoint(W, H, N); //max(W,H,N) should be < 20000 because of memory limitation
    //pointset.printPointset();

    generatedPointset = pointset.getPointset();
    adjacentMatrix = pointset.getAdjacentMatrix();

    MST mst(adjacentMatrix, N);
    mst.buildGraph();

    //Deliverable A: From pointset and adjacentMatrix, you should construct MST with Prim or Kruskal
    int mstS = mst.totalWeight();
    mst_sum.push_back(mstS);
    cout << "MST Cost: " << mstS << endl;

    //Deliverable B: Find TSP2 path from the constructed MST
    int tsp2S = mst.makeTSP2();
    tsp2_sum.push_back(tsp2S);
    cout << "TSP2 Cost: " << tsp2S << endl;

    
    //Find the perfect minimum-weight matching 
    vector<int> oddVertices = mst.getOddVertices();
    int num = oddVertices.size(); 

    struct PerfectMatching::Options options;
    int i, e, node_num = num, edge_num = num*(num-1)/2;
    int* edges;
    int* weights;
    PerfectMatching *pm = new PerfectMatching(node_num, edge_num);

    LoadInput(node_num, edge_num, edges, weights, adjacentMatrix, oddVertices, num);

    for (e=0; e<edge_num; e++) {
      pm->AddEdge(edges[2*e], edges[2*e+1], weights[e]);
    }

    pm->options = options;
    pm->Solve();

    // find TSP 1p5 route
    mst.combine(pm, oddVertices);
    vector<int> route = mst.makeTSP1_5();

    // Total cost of TSP 1p5
    int tsp15S = 0;
    for(int i=0; i<N-1; ++i) {
      tsp15S += adjacentMatrix[route[i]][route[i+1]];
    }
    tsp15S += adjacentMatrix[route[0]][route[N-1]];
  
    //Deliverable C: Find TSP1.5 path from the constructed MST
    tsp15_sum.push_back(tsp15S);
    cout << "TSP 1p5 Cost: " << tsp15S << endl;
    
    // EC1
    int twoOptPS = mst.twoOptP(route, tsp15S);
    twoOptP_sum.push_back(twoOptPS);
    cout << "2OptP Cost: " << twoOptPS << endl;

    int twoOptES = mst.twoOptE(route, tsp15S);
    twoOptE_sum.push_back(twoOptES);
    cout << "2OptE Cost: " << twoOptES << endl;

    delete pm;
    delete [] edges;
    delete [] weights;
  }

  double mst_mean, tsp2_mean, tsp15_mean, twoP_mean, twoE_mean,
          mst_sd, tsp2_sd, tsp15_sd, twoP_sd, twoE_sd;

  mst_mean = calculateMean(mst_sum, trial);
  tsp2_mean = calculateMean(tsp2_sum, trial);
  tsp15_mean = calculateMean(tsp15_sum, trial);
  twoP_mean = calculateMean(twoOptP_sum, trial);
  twoE_mean = calculateMean(twoOptE_sum, trial);

  mst_sd = calculateSD(mst_sum, mst_mean);
  tsp2_sd = calculateSD(tsp2_sum, tsp2_mean);
  tsp15_sd = calculateSD(tsp15_sum, tsp15_mean);
  twoP_sd = calculateSD(twoOptP_sum, twoP_mean);
  twoE_sd = calculateSD(twoOptE_sum, twoE_mean);

  cout << endl;
  cout << "MST mean: " << mst_mean << endl;
  cout << "MST standard deviation: " << mst_sd << endl;

  cout << "TSP2 mean: " << tsp2_mean << endl;
  cout << "TSP2 standard deviation: " << tsp2_sd << endl;

  cout << "TSP15 mean: " << tsp15_mean << endl;
  cout << "TSP15 standard deviation: " << tsp15_sd << endl;

  cout << "twoOptP mean: " << twoP_mean << endl;
  cout << "twoOptP standard deviation: " << twoP_sd << endl;

  cout << "twoOptE mean: " << twoE_mean << endl;
  cout << "twoOptE standard deviation: " << twoE_sd << endl;

  
  return 0;
}

