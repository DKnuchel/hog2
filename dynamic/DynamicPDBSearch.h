//
//  DynamicPDBSearch.h
//  CMakeTest
//
//  Created by Damian Knuchel on 14.08.21.
//
#ifndef DynamicPDBSearch_h
#define DynamicPDBSearch_h

#include "Heuristic.h"
template<int width, int height, class state, class action, class environment>
class DynamicPDBSearch
{
public:
	DynamicPDBSearch(Heuristic<state> *h){this->h = h;};
	double** CreateAdjacencyMatrix(state s1, state s2);

private:
	PermutationPDB<state, action, environment> *pdb;
	Heuristic<state> *h;
protected:
};



template<int width, int height, class state, class action, class environment>
double** DynamicPDBSearch<width, height, state, action, environment>::CreateAdjacencyMatrix(state s1, state s2){
	assert(width==height);
	int n = width*height;
	int x = 0;
	double** graph = new double*[n];
	for (int i = 0; i < n; i++){
		graph[i] = new double[n];
		for (int j = 0; j < n; j++){
			if (j <= i || i == 0)
			{
				graph[i][j] = 0;
			}
			else
			{
				graph[i][j] = h->heuristics[x++]->HCost(s1, s2);
			}
		}
	}
	return graph;
}

#endif /* DynamicPDBSearch_h */

