#include "Driver.h"
#include "MNPuzzle.h"
#include "IDAStar.h"
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"
#include "MR1PermutationPDB.h"
#include "STPInstances.h"


int main(){
 const int width = 4, height = 4;
    std::string path = "";
	MNPuzzle<4, 4> mnp;
	Heuristic<MNPuzzleState<4, 4> > h;
	MNPuzzleState<4, 4> start, goal;
	std::vector<slideDir> moves;
	std::vector<MNPuzzleState<4, 4> > statepath;
	goal.Reset();
	mnp.StoreGoal(goal);
		
	std::vector<int> p1 = {0, 1, 2, 3, 4, 5, 6, 7};
	std::vector<int> p2 = {0, 8, 9, 10, 11, 12, 13, 14, 15};
	int threads = std::thread::hardware_concurrency();
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height> > pdb1(&mnp, goal, p1);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height> > pdb2(&mnp, goal, p2);
	
	if (pdb1.Load(path.c_str()))
	{
		printf("Loaded successfully\n");
	}
	else {
		mnp.SetPattern(p1);
		pdb1.BuildAdditivePDB(goal, threads);
		pdb1.Save(path.c_str());
	}
	if (pdb2.Load(path.c_str()))
	{
		printf("Loaded successfully\n");
	}
	else {
		mnp.SetPattern(p2);
		pdb2.BuildAdditivePDB(goal, threads);
		pdb2.Save(path.c_str());
	}

	h.lookups.resize(0);
	h.lookups.push_back({kAddNode, 1, 2});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
			
	h.heuristics.resize(0);
	h.heuristics.push_back(&pdb1);
	h.heuristics.push_back(&pdb2);

	{
		IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
		ida.SetHeuristic(&h);
		std::vector<slideDir> path;
		for (int x = 0; x < 100; x++)
		{
			start = STP::GetKorfInstance(x);
			ida.GetPath(&mnp, start, goal, path);
			printf("ida\t%1.2f\t%llu\t%llu\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(), ida.GetNodesTouched());
		}
	}
}