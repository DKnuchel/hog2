//
//  DynamicPDBSearch.h
//  CMakeTest
//
//  Created by Damian Knuchel on 14.08.21.
//
#ifndef DynamicPDBSearch_h
#define DynamicPDBSearch_h

#include "Heuristic.h"
#include "Graph.h"

template<int width, int height, class state, class action, class environment>
class DynamicPDBSearch : public Heuristic <state>
{
public:
	DynamicPDBSearch(Heuristic<state> *hPairs, Heuristic<state> *hTriplets, bool wvc = false) : hPairs(hPairs), hTriplets(hTriplets), mm(!wvc), wvc(wvc) {};
	virtual ~DynamicPDBSearch() {}
	double** CreateAdjacencyMatrix(state &s1, state &s2);
	Graph CreateGraph(state &s1, state &s2); //Only Pairs
	
	virtual double HCost(const state &a, const state &b) const;

private:
	PermutationPDB<state, action, environment> *pdb;
	Heuristic<state> *hPairs;
	Heuristic<state> *hTriplets;
	double GetManDist(state &s1, state &s2);
	bool mm = true;
	bool wvc = false;
	
	Graph graph;
	
	double MaximumWeightMatching();
	double WeightedVertexCover();
	
protected:
};


/**
 Use Only for testing.
 This function got replaced by 'CreateGraph'
 */
template<int width, int height, class state, class action, class environment>
double** DynamicPDBSearch<width, height, state, action, environment>::CreateAdjacencyMatrix(state &s1, state &s2)
{
	assert(width==height);
	int n = width*height;
	int x = 0;
	double** graph = new double*[n];
	for (int i = 0; i < n; i++){
		graph[i] = new double[n];
		for (int j = 0; j < n; j++){
			if (j > i || i == 0 || j == 0)
			{
				graph[i][j] = 0;
			}
			else if (i == j)
			{
				graph[i][j] = GetManDist(s1, s2);
			}
			else
			{
				double hVal = hPairs->heuristics[x++]->HCost(s1, s2);
				graph[i][j] = hVal;
				graph[j][i] = hVal;
			}
		}
	}
	return graph;
}
/**
 TODO: Remove return statement
 */
template<int width, int height, class state, class action, class environment>
Graph DynamicPDBSearch<width, height, state, action, environment>::CreateGraph(state &s1, state &s2)
{
	Graph *g = new Graph();
	for (int i = 0; i < width*height; i++)
	{
		node *n;
		g->AddNode(n = new node(std::to_string(i).c_str()));
	}
	int x = 0;
	for (int i = 0; i < width*height; i++)
	{
		for (int j = 0; j < width*height; j++)
		{
			edge *e;
			if (j > i || i == 0 || j == 0)
			{
				continue;
			}
			else if (i == j)
			{
				g->AddEdge(e = new edge(i, j, GetManDist(s1, s2)));
				//e->SetLabelL(0, 0);
				//continue;
			}
			else
			{
				double hVal = hPairs->heuristics[x++]->HCost(s1, s2);
				g->AddEdge(e = new edge(i, j, hVal));
				//g->AddEdge(e = new edge(j, i, hVal));
				//e->SetLabelL(0, 1);
				//e->SetLabelL(1, x);
			}
//			int y = x;
//			for (int k = j; k < width*height; k++)
//			{
//				g->AddEdge(e = new edge(i, j, hTriplets->heuristics[y]->HCost(s1, s2)));
//				e->SetLabelL(0, 2);
//				e->SetLabelL(2, y);
//				g->AddEdge(e = new edge(i, k, hTriplets->heuristics[y]->HCost(s1, s2)));
//				e->SetLabelL(0, 2);
//				e->SetLabelL(2, y);
//				g->AddEdge(e = new edge(j, k, hTriplets->heuristics[y]->HCost(s1, s2)));
//				e->SetLabelL(0, 2);
//				e->SetLabelL(2, y);
//			}
		}
	}
	graph = *g;
	return *g;
}

template<int width, int height, class state, class action, class environment>
double DynamicPDBSearch<width, height, state, action, environment>::GetManDist(state &s1, state &s2)
{
	double hval = 0;
	double man_dist = 0;
	int xloc[width*height];
	int yloc[width*height];

	for (unsigned int x = 0; x < width; x++)
	{
		for (unsigned int y = 0; y < height; y++)
		{
			xloc[s2.puzzle[x + y*width]] = x;
			yloc[s2.puzzle[x + y*width]] = y;
		}
	}
	for (unsigned int x = 0; x < width; x++)
	{
		for (unsigned int y = 0; y < height; y++)
		{
			if (s1.puzzle[x + y*width] != 0)
			{
				double absDist = (abs((int)(xloc[s1.puzzle[x + y*width]] - x))
								 + abs((int)(yloc[s1.puzzle[x + y*width]] - y)));
				double movingTile = s1.puzzle[x + y*width];
				man_dist += absDist;
			}
		}
	}
	hval = std::max(hval, man_dist);
	return hval;
}
/**
 If the DynamicPDBSearch gets passed on as the heuristic, this function will be called if there is a lookup for the heuristic value between two specific states.
 */
template<int width, int height, class state, class action, class environment>
double DynamicPDBSearch<width, height, state, action, environment>::HCost(const state &s1, const state &s2) const
{
	// Implement Maximum weight matching and weighted vertex covera
	if(mm)
	{
		
	}else if(wvc)
	{
		
	}
	return 1.02;
}
/**
 TODO: https://en.wikipedia.org/wiki/Blossom_algorithm
 I know what to solve but I don't know how to solve it.
 */
template<int width, int height, class state, class action, class environment>
double DynamicPDBSearch<width, height, state, action, environment>::MaximumWeightMatching()
{
	int nEdge = graph.GetNumEdges();
	int nVertex = graph.GetNumNodes();
	double maxWeight = 0.0;
	//Find Max Weight
	for(int i = 0; i < nEdge; i++)
	{
		maxWeight = std::max(graph.GetEdge(i)->GetWeight(), maxWeight);
	}
	
	
	
}
/**
 TODO: https://www.coursera.org/lecture/approximation-algorithms/weighted-vertex-cover-3QEov
 I know what to solve but I don't know how to solve it.
 */
template<int width, int height, class state, class action, class environment>
double DynamicPDBSearch<width, height, state, action, environment>::WeightedVertexCover()
{
	return 1.02;
}


#endif /* DynamicPDBSearch_h */

