#include "Driver.h"
#include "MNPuzzle.h"
#include "IDAStar.h"
#include "AStar.h"
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"
#include "MR1PermutationPDB.h"
#include "DynPermutationPDB.h"
#include "STPInstances.h"
#include "Timer.h"
#include "DynamicPDBSearch.h"
#include "Graph.h"


void ManhattanDistance(int iterationSteps, bool korfInstance = false);
void LinearConflict(int IterationSteps, bool korfInstance = false);
void SolveKorfAdditive(std::string path, int iterationSteps);
void Dynamic();
void DynamicLocal();

const int width = 4, height = 4;

int main()
{
	//DynamicLocal();
	Dynamic();
	//LinearConflict(1, true);
	//ManhattanDistance(1, true);
	//SolveKorfAdditive("./", 1);
}
//

void ManhattanDistance(int iterationSteps, bool korfInstance){
	printf("Heuristic: Manhattan Distance\n");
	MNPuzzle<width, height> mnp;
	MNPuzzleState<width, height> start, goal;
	std::vector<slideDir> path;
	mnp.Set_Use_Manhattan_Heuristic(true);
	Timer t;
	
	if (korfInstance)
	{
		t.StartTimer();
		for (int i = 0; i < iterationSteps; i++) {
			start = STP::GetKorfInstance(i);
			IDAStar<MNPuzzleState<width, height>, slideDir> ida;
			ida.GetPath(&mnp, start, goal, path);
			printf("ida\t%1.2f\t%llu\t%llu\t%1.2fs elapsed\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(), ida.GetNodesTouched(), t.EndTimer());
			
		}
		return;
	}
	
	for (int i = 0; i < iterationSteps; i++)
	{
		t.StartTimer();
		start = mnp.Generate_Random_Puzzle();
		goal.Reset();
		IDAStar<MNPuzzleState<width, height>, slideDir> ida;
		ida.GetPath(&mnp, start, goal, path);
		printf("ida\t%1.2f\t%llu\t%llu\t%1.2fs elapsed\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(), ida.GetNodesTouched(), t.EndTimer());	}
	return;
}

void LinearConflict(int iterationSteps, bool korfInstance)
{
	printf("Heuristic: Linear Conflict\n");
	MNPuzzle<width, height> mnp;
	MNPuzzleState<width, height> start, goal;
	std::vector<slideDir> path;
	mnp.Set_Use_Linear_Conflict_Heuristic(true);
	Timer t;
	
	if (korfInstance)
	{
		for (int i = 0; i < iterationSteps; i++) {
			t.StartTimer();
			start = STP::GetKorfInstance(i);
			IDAStar<MNPuzzleState<width, height>, slideDir> ida;
			ida.GetPath(&mnp, start, goal, path);
			printf("ida\t%1.2f\t%llu\t%llu\t%1.2fs elapsed\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(), ida.GetNodesTouched(), t.EndTimer());
		}
		return;
	}
	
	for (int i = 0; i < iterationSteps; i++)
	{
		t.StartTimer();
		start = mnp.Generate_Random_Puzzle();
		goal.Reset();
		IDAStar<MNPuzzleState<width, height>, slideDir> ida;
		ida.GetPath(&mnp, start, goal, path);
		printf("ida\t%1.2f\t%llu\t%llu\t%1.2fs elapsed\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(), ida.GetNodesTouched(), t.EndTimer());	}
	return;
}

void SolveKorfAdditive(std::string path, int iterationSteps)
{
	MNPuzzle<4, 4> mnp;
	Heuristic<MNPuzzleState<4, 4>> h;
	MNPuzzleState<4, 4> start, goal;
	std::vector<slideDir> moves;
	std::vector<MNPuzzleState<4, 4>> statepath;
	goal.Reset();
	mnp.StoreGoal(goal);
		
	std::vector<int> p1 = {0, 1, 2, 3, 4, 5, 6, 7};
	std::vector<int> p2 = {0, 8, 9, 10, 11, 12, 13, 14, 15};
	int threads = std::thread::hardware_concurrency();
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb1(&mnp, goal, p1);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb2(&mnp, goal, p2);
	
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
	
	//std::cout << "H-Size: " << h.heuristics.size() << std::endl;
	//std::cout << "Heuristic front:" << h.heuristics.front()->HCost(start, goal) << std::endl;

	{
		IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
		ida.SetHeuristic(&h);
		std::vector<slideDir> path;
		for (int x = 0; x < iterationSteps; x++)
		{
			start = STP::GetKorfInstance(x);
			ida.GetPath(&mnp, start, goal, path);
			printf("ida\t%1.2f\t%llu\t%llu\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(), ida.GetNodesTouched());
		}
	}
}

void Dynamic()
{
	MNPuzzle<width, height> mnp;
	Heuristic<MNPuzzleState<width, height>> h;
	MNPuzzleState<width,height> start, goal;
	std::vector<slideDir> moves;
	std::vector<MNPuzzleState<width, height>> statepath;
	goal.Reset();
	mnp.StoreGoal(goal);

	
	DynPermutationPDB<width, height, MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb(mnp);
	pdb.SetGoal(&goal);
		
	std::vector<LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> > pdbs = pdb.GetPDBs(std::string("./pairs/"));

	h.lookups.resize(0);
	h.lookups.push_back({kMaxNode, 1, 2});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	
	h.heuristics.resize(0);
	for(auto& i : pdbs)
	{
		h.heuristics.push_back(&i);
	}
	std::cout << "Heuristic front: " << h.heuristics.front()->HCost(start, goal) << std::endl;
	
	DynamicPDBSearch<width, height, MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> dyn(&h);
	double** graph = dyn.CreateAdjacencyMatrix(start, goal);
	{
		IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
		ida.SetHeuristic(&h);
		std::vector<slideDir> path;
		start = STP::GetKorfInstance(1);
		ida.GetPath(&mnp, start, goal, path);
		printf("ida\t%1.2f\t%llu\t%llu\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(), ida.GetNodesTouched());
	}
	
}

void DynamicLocal()
{
	std::string path = "./pairs/";
	int threads = std::thread::hardware_concurrency();
	
	MNPuzzle<width, height> mnp;
	Heuristic<MNPuzzleState<width, height>> h;
	MNPuzzleState<width,height> start, goal;
	std::vector<slideDir> moves;
	std::vector<MNPuzzleState<width, height>> statepath;
	goal.Reset();
	mnp.StoreGoal(goal);
	
	h.lookups.resize(0);
	h.lookups.push_back({kAddNode, 1, 2});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	
	h.heuristics.resize(0);
		
	static std::vector<std::vector<int>> patterns;
	static std::vector<LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>>* > pdbs;
	
	std::vector<int> pattern1 = {0, 1, 2};
	patterns.push_back(pattern1);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb1(&mnp, goal, pattern1);
	pdbs.push_back(&pdb1);
	std::vector<int> pattern2 = {0, 1, 3};
	patterns.push_back(pattern2);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb2(&mnp, goal, pattern2);
	pdbs.push_back(&pdb2);
	std::vector<int> pattern3 = {0, 1, 4};
	patterns.push_back(pattern3);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb3(&mnp, goal, pattern3);
	pdbs.push_back(&pdb3);
	std::vector<int> pattern4 = {0, 1, 5};
	patterns.push_back(pattern4);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb4(&mnp, goal, pattern4);
	pdbs.push_back(&pdb4);
	std::vector<int> pattern5 = {0, 1, 6};
	patterns.push_back(pattern5);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb5(&mnp, goal, pattern5);
	pdbs.push_back(&pdb5);
	std::vector<int> pattern6 = {0, 1, 7};
	patterns.push_back(pattern6);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb6(&mnp, goal, pattern6);
	pdbs.push_back(&pdb6);
	std::vector<int> pattern7 = {0, 1, 8};
	patterns.push_back(pattern7);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb7(&mnp, goal, pattern7);
	pdbs.push_back(&pdb7);
	std::vector<int> pattern8 = {0, 1, 9};
	patterns.push_back(pattern8);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb8(&mnp, goal, pattern8);
	pdbs.push_back(&pdb8);
	std::vector<int> pattern9 = {0, 1, 10};
	patterns.push_back(pattern9);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb9(&mnp, goal, pattern9);
	pdbs.push_back(&pdb9);
	std::vector<int> pattern10 = {0, 1, 11};
	patterns.push_back(pattern10);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb10(&mnp, goal, pattern10);
	pdbs.push_back(&pdb10);
	std::vector<int> pattern11 = {0, 1, 12};
	patterns.push_back(pattern11);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb11(&mnp, goal, pattern11);
	pdbs.push_back(&pdb11);
	std::vector<int> pattern12 = {0, 1, 13};
	patterns.push_back(pattern12);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb12(&mnp, goal, pattern12);
	pdbs.push_back(&pdb12);
	std::vector<int> pattern13 = {0, 1, 14};
	patterns.push_back(pattern13);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb13(&mnp, goal, pattern13);
	pdbs.push_back(&pdb13);
	std::vector<int> pattern14 = {0, 1, 15};
	patterns.push_back(pattern14);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb14(&mnp, goal, pattern14);
	pdbs.push_back(&pdb14);
	std::vector<int> pattern15 = {0, 2, 3};
	patterns.push_back(pattern15);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb15(&mnp, goal, pattern15);
	pdbs.push_back(&pdb15);
	std::vector<int> pattern16 = {0, 2, 4};
	patterns.push_back(pattern16);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb16(&mnp, goal, pattern16);
	pdbs.push_back(&pdb16);
	std::vector<int> pattern17 = {0, 2, 5};
	patterns.push_back(pattern17);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb17(&mnp, goal, pattern17);
	pdbs.push_back(&pdb17);
	std::vector<int> pattern18 = {0, 2, 6};
	patterns.push_back(pattern18);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb18(&mnp, goal, pattern18);
	pdbs.push_back(&pdb18);
	std::vector<int> pattern19 = {0, 2, 7};
	patterns.push_back(pattern19);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb19(&mnp, goal, pattern19);
	pdbs.push_back(&pdb19);
	std::vector<int> pattern20 = {0, 2, 8};
	patterns.push_back(pattern20);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb20(&mnp, goal, pattern20);
	pdbs.push_back(&pdb20);
	std::vector<int> pattern21 = {0, 2, 9};
	patterns.push_back(pattern21);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb21(&mnp, goal, pattern21);
	pdbs.push_back(&pdb21);
	std::vector<int> pattern22 = {0, 2, 10};
	patterns.push_back(pattern22);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb22(&mnp, goal, pattern22);
	pdbs.push_back(&pdb22);
	std::vector<int> pattern23 = {0, 2, 11};
	patterns.push_back(pattern23);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb23(&mnp, goal, pattern23);
	pdbs.push_back(&pdb23);
	std::vector<int> pattern24 = {0, 2, 12};
	patterns.push_back(pattern24);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb24(&mnp, goal, pattern24);
	pdbs.push_back(&pdb24);
	std::vector<int> pattern25 = {0, 2, 13};
	patterns.push_back(pattern25);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb25(&mnp, goal, pattern25);
	pdbs.push_back(&pdb25);
	std::vector<int> pattern26 = {0, 2, 14};
	patterns.push_back(pattern26);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb26(&mnp, goal, pattern26);
	pdbs.push_back(&pdb26);
	std::vector<int> pattern27 = {0, 2, 15};
	patterns.push_back(pattern27);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb27(&mnp, goal, pattern27);
	pdbs.push_back(&pdb27);
	std::vector<int> pattern28 = {0, 3, 4};
	patterns.push_back(pattern28);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb28(&mnp, goal, pattern28);
	pdbs.push_back(&pdb28);
	std::vector<int> pattern29 = {0, 3, 5};
	patterns.push_back(pattern29);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb29(&mnp, goal, pattern29);
	pdbs.push_back(&pdb29);
	std::vector<int> pattern30 = {0, 3, 6};
	patterns.push_back(pattern30);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb30(&mnp, goal, pattern30);
	pdbs.push_back(&pdb30);
	std::vector<int> pattern31 = {0, 3, 7};
	patterns.push_back(pattern31);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb31(&mnp, goal, pattern31);
	pdbs.push_back(&pdb31);
	std::vector<int> pattern32 = {0, 3, 8};
	patterns.push_back(pattern32);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb32(&mnp, goal, pattern32);
	pdbs.push_back(&pdb32);
	std::vector<int> pattern33 = {0, 3, 9};
	patterns.push_back(pattern33);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb33(&mnp, goal, pattern33);
	pdbs.push_back(&pdb33);
	std::vector<int> pattern34 = {0, 3, 10};
	patterns.push_back(pattern34);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb34(&mnp, goal, pattern34);
	pdbs.push_back(&pdb34);
	std::vector<int> pattern35 = {0, 3, 11};
	patterns.push_back(pattern35);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb35(&mnp, goal, pattern35);
	pdbs.push_back(&pdb35);
	std::vector<int> pattern36 = {0, 3, 12};
	patterns.push_back(pattern36);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb36(&mnp, goal, pattern36);
	pdbs.push_back(&pdb36);
	std::vector<int> pattern37 = {0, 3, 13};
	patterns.push_back(pattern37);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb37(&mnp, goal, pattern37);
	pdbs.push_back(&pdb37);
	std::vector<int> pattern38 = {0, 3, 14};
	patterns.push_back(pattern38);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb38(&mnp, goal, pattern38);
	pdbs.push_back(&pdb38);
	std::vector<int> pattern39 = {0, 3, 15};
	patterns.push_back(pattern39);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb39(&mnp, goal, pattern39);
	pdbs.push_back(&pdb39);
	std::vector<int> pattern40 = {0, 4, 5};
	patterns.push_back(pattern40);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb40(&mnp, goal, pattern40);
	pdbs.push_back(&pdb40);
	std::vector<int> pattern41 = {0, 4, 6};
	patterns.push_back(pattern41);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb41(&mnp, goal, pattern41);
	pdbs.push_back(&pdb41);
	std::vector<int> pattern42 = {0, 4, 7};
	patterns.push_back(pattern42);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb42(&mnp, goal, pattern42);
	pdbs.push_back(&pdb42);
	std::vector<int> pattern43 = {0, 4, 8};
	patterns.push_back(pattern43);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb43(&mnp, goal, pattern43);
	pdbs.push_back(&pdb43);
	std::vector<int> pattern44 = {0, 4, 9};
	patterns.push_back(pattern44);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb44(&mnp, goal, pattern44);
	pdbs.push_back(&pdb44);
	std::vector<int> pattern45 = {0, 4, 10};
	patterns.push_back(pattern45);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb45(&mnp, goal, pattern45);
	pdbs.push_back(&pdb45);
	std::vector<int> pattern46 = {0, 4, 11};
	patterns.push_back(pattern46);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb46(&mnp, goal, pattern46);
	pdbs.push_back(&pdb46);
	std::vector<int> pattern47 = {0, 4, 12};
	patterns.push_back(pattern47);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb47(&mnp, goal, pattern47);
	pdbs.push_back(&pdb47);
	std::vector<int> pattern48 = {0, 4, 13};
	patterns.push_back(pattern48);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb48(&mnp, goal, pattern48);
	pdbs.push_back(&pdb48);
	std::vector<int> pattern49 = {0, 4, 14};
	patterns.push_back(pattern49);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb49(&mnp, goal, pattern49);
	pdbs.push_back(&pdb49);
	std::vector<int> pattern50 = {0, 4, 15};
	patterns.push_back(pattern50);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb50(&mnp, goal, pattern50);
	pdbs.push_back(&pdb50);
	std::vector<int> pattern51 = {0, 5, 6};
	patterns.push_back(pattern51);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb51(&mnp, goal, pattern51);
	pdbs.push_back(&pdb51);
	std::vector<int> pattern52 = {0, 5, 7};
	patterns.push_back(pattern52);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb52(&mnp, goal, pattern52);
	pdbs.push_back(&pdb52);
	std::vector<int> pattern53 = {0, 5, 8};
	patterns.push_back(pattern53);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb53(&mnp, goal, pattern53);
	pdbs.push_back(&pdb53);
	std::vector<int> pattern54 = {0, 5, 9};
	patterns.push_back(pattern54);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb54(&mnp, goal, pattern54);
	pdbs.push_back(&pdb54);
	std::vector<int> pattern55 = {0, 5, 10};
	patterns.push_back(pattern55);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb55(&mnp, goal, pattern55);
	pdbs.push_back(&pdb55);
	std::vector<int> pattern56 = {0, 5, 11};
	patterns.push_back(pattern56);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb56(&mnp, goal, pattern56);
	pdbs.push_back(&pdb56);
	std::vector<int> pattern57 = {0, 5, 12};
	patterns.push_back(pattern57);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb57(&mnp, goal, pattern57);
	pdbs.push_back(&pdb57);
	std::vector<int> pattern58 = {0, 5, 13};
	patterns.push_back(pattern58);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb58(&mnp, goal, pattern58);
	pdbs.push_back(&pdb58);
	std::vector<int> pattern59 = {0, 5, 14};
	patterns.push_back(pattern59);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb59(&mnp, goal, pattern59);
	pdbs.push_back(&pdb59);
	std::vector<int> pattern60 = {0, 5, 15};
	patterns.push_back(pattern60);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb60(&mnp, goal, pattern60);
	pdbs.push_back(&pdb60);
	std::vector<int> pattern61 = {0, 6, 7};
	patterns.push_back(pattern61);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb61(&mnp, goal, pattern61);
	pdbs.push_back(&pdb61);
	std::vector<int> pattern62 = {0, 6, 8};
	patterns.push_back(pattern62);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb62(&mnp, goal, pattern62);
	pdbs.push_back(&pdb62);
	std::vector<int> pattern63 = {0, 6, 9};
	patterns.push_back(pattern63);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb63(&mnp, goal, pattern63);
	pdbs.push_back(&pdb63);
	std::vector<int> pattern64 = {0, 6, 10};
	patterns.push_back(pattern64);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb64(&mnp, goal, pattern64);
	pdbs.push_back(&pdb64);
	std::vector<int> pattern65 = {0, 6, 11};
	patterns.push_back(pattern65);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb65(&mnp, goal, pattern65);
	pdbs.push_back(&pdb65);
	std::vector<int> pattern66 = {0, 6, 12};
	patterns.push_back(pattern66);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb66(&mnp, goal, pattern66);
	pdbs.push_back(&pdb66);
	std::vector<int> pattern67 = {0, 6, 13};
	patterns.push_back(pattern67);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb67(&mnp, goal, pattern67);
	pdbs.push_back(&pdb67);
	std::vector<int> pattern68 = {0, 6, 14};
	patterns.push_back(pattern68);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb68(&mnp, goal, pattern68);
	pdbs.push_back(&pdb68);
	std::vector<int> pattern69 = {0, 6, 15};
	patterns.push_back(pattern69);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb69(&mnp, goal, pattern69);
	pdbs.push_back(&pdb69);
	std::vector<int> pattern70 = {0, 7, 8};
	patterns.push_back(pattern70);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb70(&mnp, goal, pattern70);
	pdbs.push_back(&pdb70);
	std::vector<int> pattern71 = {0, 7, 9};
	patterns.push_back(pattern71);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb71(&mnp, goal, pattern71);
	pdbs.push_back(&pdb71);
	std::vector<int> pattern72 = {0, 7, 10};
	patterns.push_back(pattern72);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb72(&mnp, goal, pattern72);
	pdbs.push_back(&pdb72);
	std::vector<int> pattern73 = {0, 7, 11};
	patterns.push_back(pattern73);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb73(&mnp, goal, pattern73);
	pdbs.push_back(&pdb73);
	std::vector<int> pattern74 = {0, 7, 12};
	patterns.push_back(pattern74);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb74(&mnp, goal, pattern74);
	pdbs.push_back(&pdb74);
	std::vector<int> pattern75 = {0, 7, 13};
	patterns.push_back(pattern75);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb75(&mnp, goal, pattern75);
	pdbs.push_back(&pdb75);
	std::vector<int> pattern76 = {0, 7, 14};
	patterns.push_back(pattern76);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb76(&mnp, goal, pattern76);
	pdbs.push_back(&pdb76);
	std::vector<int> pattern77 = {0, 7, 15};
	patterns.push_back(pattern77);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb77(&mnp, goal, pattern77);
	pdbs.push_back(&pdb77);
	std::vector<int> pattern78 = {0, 8, 9};
	patterns.push_back(pattern78);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb78(&mnp, goal, pattern78);
	pdbs.push_back(&pdb78);
	std::vector<int> pattern79 = {0, 8, 10};
	patterns.push_back(pattern79);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb79(&mnp, goal, pattern79);
	pdbs.push_back(&pdb79);
	std::vector<int> pattern80 = {0, 8, 11};
	patterns.push_back(pattern80);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb80(&mnp, goal, pattern80);
	pdbs.push_back(&pdb80);
	std::vector<int> pattern81 = {0, 8, 12};
	patterns.push_back(pattern81);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb81(&mnp, goal, pattern81);
	pdbs.push_back(&pdb81);
	std::vector<int> pattern82 = {0, 8, 13};
	patterns.push_back(pattern82);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb82(&mnp, goal, pattern82);
	pdbs.push_back(&pdb82);
	std::vector<int> pattern83 = {0, 8, 14};
	patterns.push_back(pattern83);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb83(&mnp, goal, pattern83);
	pdbs.push_back(&pdb83);
	std::vector<int> pattern84 = {0, 8, 15};
	patterns.push_back(pattern84);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb84(&mnp, goal, pattern84);
	pdbs.push_back(&pdb84);
	std::vector<int> pattern85 = {0, 9, 10};
	patterns.push_back(pattern85);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb85(&mnp, goal, pattern85);
	pdbs.push_back(&pdb85);
	std::vector<int> pattern86 = {0, 9, 11};
	patterns.push_back(pattern86);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb86(&mnp, goal, pattern86);
	pdbs.push_back(&pdb86);
	std::vector<int> pattern87 = {0, 9, 12};
	patterns.push_back(pattern87);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb87(&mnp, goal, pattern87);
	pdbs.push_back(&pdb87);
	std::vector<int> pattern88 = {0, 9, 13};
	patterns.push_back(pattern88);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb88(&mnp, goal, pattern88);
	pdbs.push_back(&pdb88);
	std::vector<int> pattern89 = {0, 9, 14};
	patterns.push_back(pattern89);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb89(&mnp, goal, pattern89);
	pdbs.push_back(&pdb89);
	std::vector<int> pattern90 = {0, 9, 15};
	patterns.push_back(pattern90);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb90(&mnp, goal, pattern90);
	pdbs.push_back(&pdb90);
	std::vector<int> pattern91 = {0, 10, 11};
	patterns.push_back(pattern91);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb91(&mnp, goal, pattern91);
	pdbs.push_back(&pdb91);
	std::vector<int> pattern92 = {0, 10, 12};
	patterns.push_back(pattern92);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb92(&mnp, goal, pattern92);
	pdbs.push_back(&pdb92);
	std::vector<int> pattern93 = {0, 10, 13};
	patterns.push_back(pattern93);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb93(&mnp, goal, pattern93);
	pdbs.push_back(&pdb93);
	std::vector<int> pattern94 = {0, 10, 14};
	patterns.push_back(pattern94);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb94(&mnp, goal, pattern94);
	pdbs.push_back(&pdb94);
	std::vector<int> pattern95 = {0, 10, 15};
	patterns.push_back(pattern95);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb95(&mnp, goal, pattern95);
	pdbs.push_back(&pdb95);
	std::vector<int> pattern96 = {0, 11, 12};
	patterns.push_back(pattern96);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb96(&mnp, goal, pattern96);
	pdbs.push_back(&pdb96);
	std::vector<int> pattern97 = {0, 11, 13};
	patterns.push_back(pattern97);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb97(&mnp, goal, pattern97);
	pdbs.push_back(&pdb97);
	std::vector<int> pattern98 = {0, 11, 14};
	patterns.push_back(pattern98);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb98(&mnp, goal, pattern98);
	pdbs.push_back(&pdb98);
	std::vector<int> pattern99 = {0, 11, 15};
	patterns.push_back(pattern99);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb99(&mnp, goal, pattern99);
	pdbs.push_back(&pdb99);
	std::vector<int> pattern100 = {0, 12, 13};
	patterns.push_back(pattern100);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb100(&mnp, goal, pattern100);
	pdbs.push_back(&pdb100);
	std::vector<int> pattern101 = {0, 12, 14};
	patterns.push_back(pattern101);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb101(&mnp, goal, pattern101);
	pdbs.push_back(&pdb101);
	std::vector<int> pattern102 = {0, 12, 15};
	patterns.push_back(pattern102);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb102(&mnp, goal, pattern102);
	pdbs.push_back(&pdb102);
	std::vector<int> pattern103 = {0, 13, 14};
	patterns.push_back(pattern103);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb103(&mnp, goal, pattern103);
	pdbs.push_back(&pdb103);
	std::vector<int> pattern104 = {0, 13, 15};
	patterns.push_back(pattern104);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb104(&mnp, goal, pattern104);
	pdbs.push_back(&pdb104);
	std::vector<int> pattern105 = {0, 14, 15};
	patterns.push_back(pattern105);
	static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb105(&mnp, goal, pattern105);
	pdbs.push_back(&pdb105);
	
//	int x = 1;
//	std::vector<int> pattern = {0, 0, 0};
//	for (int i = 1; i < width*height; i++)
//	{
//		for (int j = i+1; j < width*height; j++)
//		{
//			/*
//			pattern = {0, i, j};
//			patterns.push_back(pattern);
//			std::cout << "std::vector<int> pattern" << x << " = {0, " << i << ", " << j << "};" << std::endl;
//			std::cout << "patterns.push_back(pattern" << x << ");" << std::endl;
//			std::cout << "static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb" << x << "(&mnp, goal, pattern" << x << ");" << std::endl;
//			std::cout << "pdbs.push_back(&pdb" << x << ");" << std::endl;
//			x++;
//			 */
//			/*
//			static LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb (&mnp, goal, pattern);
//			if (!pdb.Load(path.c_str()))
//			{
//				mnp.SetPattern(pattern);
//				pdb.BuildAdditivePDB(goal, threads);
//				pdb.Save(path.c_str());
//			}
//			pdbs.push_back(pdb);
//			h.heuristics.push_back(&pdb);
//			 */
//		}
//	}
	
	for (auto i : pdbs)
	{
		h.heuristics.push_back(i);
	}
	
	DynamicPDBSearch<width, height, MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> dyn(&h);
	double** graph = dyn.CreateAdjacencyMatrix(start, goal);
	for(int i = 0; i < width*height; i++)
	{
		for(int j = 0; j < width*height; j++){
			std::cout << " " << graph[i][j] << "\t";
		}
		std::cout << std::endl;
	}
	
	std::cout << "HCost?: " << pdbs.front()->GetAbstractHash(start) << std::endl;
	std::cout << "H-Size: " << h.heuristics.size() << std::endl;
	std::cout << "Heuristic front:" << h.heuristics.front()->HCost(start, goal) << std::endl;

	{
		IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
		h.SetDynamic(true);
		ida.SetHeuristic(&h);
		std::vector<slideDir> path;
		start = STP::GetKorfInstance(1);
		ida.GetPath(&mnp, start, goal, path);
		printf("ida\t%1.2f\t%llu\t%llu\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(), ida.GetNodesTouched());
	}
}



/*
 Linear Conflict:
 +	Added in MNPuzzle.h before call for DefaultH(const state)

 
 DPA-PDB:
 -	If hashing and reading of the data can be ignored (same as of now) than we only need to check how to generate the patterns. (each pattern gets its own pdb, which then will be pushed into the heuristic and afterwards passed on to IDA*
 */
