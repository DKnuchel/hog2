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


#include "PhOHeuristic.h"
#include "../3rdparty/lp/lp_solver.h"

void ManhattanDistance(int iterationSteps, bool korfInstance = false);
void LinearConflict(int IterationSteps, bool korfInstance = false);
void SolveKorfAdditive(std::string path, int iterationSteps);
void Dynamic();
void PhO();
void LPTest();

const int width = 4, height = 4;

int main()
{
    //LPTest();
    std::cout << "PhO: " << std::endl;
	PhO();
	//Dynamic();
    std::cout << "Linear Conflict: " << std::endl;
    LinearConflict(1, true);
	//ManhattanDistance(1, true);
	//SolveKorfAdditive("./", 1);
}
//

void LPTest()
{
    lp::LPSolver *lps = new lp::LPSolver(lp::LPSolverType::CPLEX);
    named_vector::NamedVector<lp::LPVariable> vars;
    vars.emplace_back(-lps->get_infinity(), lps->get_infinity(), 0);
    vars.emplace_back(-lps->get_infinity(), lps->get_infinity(), 0);
    vars.emplace_back(3, lps->get_infinity(), 0);

    named_vector::NamedVector<lp::LPConstraint> constraints;
    lp::LPConstraint constraint1(32, lps->get_infinity());
    lp::LPConstraint constraint2(-4.32, lps->get_infinity());
    constraint1.insert(0, 1);
    constraint1.insert(1, 1);
    constraint2.insert(1, 1);
    constraint2.insert(2, 1);

    constraints.emplace_back(std::move(constraint1));
    constraints.emplace_back(std::move(constraint2));

    lp::LinearProgram lp(lp::LPObjectiveSense::MINIMIZE, std::move(vars), std::move(constraints));
    lps->load_problem(lp);
    lps->solve();
    std::vector<double> solutions = lps->extract_solution();
    for(auto i: solutions)
    {
        std::cout << i << " ";
        std::cout << std::endl;
    }
}

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
    /*
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

*/
	//for (int i = 0; i < iterationSteps; i++)
	{
		t.StartTimer();
		start = STP::GetKorfInstance(1);
		goal.Reset();
		IDAStar<MNPuzzleState<width, height>, slideDir> ida;
		ida.GetPath(&mnp, start, goal, path);
		printf("ida\t%1.2f\t%llu\t%llu\t%1.2fs elapsed\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(), ida.GetNodesTouched(), t.EndTimer());
    }
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
/*
void Dynamic()
{
	MNPuzzle<width, height> mnp;
	Heuristic<MNPuzzleState<width, height>> hPairs;
	Heuristic<MNPuzzleState<width, height>> hTriplets;
	MNPuzzleState<width,height> start, goal;
	std::vector<slideDir> moves;
	std::vector<MNPuzzleState<width, height>> statepath;
	goal.Reset();
	mnp.StoreGoal(goal);

	
	DynPermutationPDB<width, height, MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb(mnp);
	pdb.SetGoal(&goal);
		
	std::vector<LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> > pdbs = pdb.GetPairs(std::string("./pairs/"));
	std::vector<LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> > pdbs2 = pdb.GetTriplets(std::string("./triplets/"));

	hPairs.lookups.resize(0);
	hPairs.lookups.push_back({kMaxNode, 1, 2});
	hPairs.lookups.push_back({kLeafNode, 0, 0});
	hPairs.lookups.push_back({kLeafNode, 1, 1});
	
	hTriplets.lookups.resize(0);
	hTriplets.lookups.push_back({kMaxNode, 1, 2});
	hTriplets.lookups.push_back({kLeafNode, 0, 0});
	hTriplets.lookups.push_back({kLeafNode, 1, 1});
	
	hPairs.heuristics.resize(0);
	for(auto& i : pdbs)
	{
		hPairs.heuristics.push_back(&i);
	}
	for(auto& i: pdbs2)
	{
		hTriplets.heuristics.push_back(&i);
	}
		
	//Testing:
	start = STP::GetKorfInstance(1);
	goal = STP::GetKorfInstance(0);
	
	DynamicPDBSearch<width, height, MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> dyn(&hPairs, &hTriplets);
	double** graph = dyn.CreateAdjacencyMatrix(start, goal); //Only for comparison with the graph
	Graph g = dyn.CreateGraph(start, goal);
		
	*/
//	for(int i = 0; i < g.GetNumNodes(); i++)
//	{
//	std::cout << g.GetNode(i)->GetNumEdges() << " " << g.GetNode(i)->getNumIncomingEdges() << " " << g.GetNode(i)->getNumOutgoingEdges() << std::endl;
//	}
//
//	for(int i = 0; i < width*height; i++)
//	{
//		for(int j = i; j < g.GetNode(i)->GetNumEdges(); j++)
//		{
//			bool b = (g.GetNode(i)->getEdge(j)->GetWeight() == graph[i][j]);
//			std::cout << b << ": " << i << "-" << j << ": " << g.GetNode(i)->getEdge(j)->getFrom() << "-" << g.GetNode(i)->getEdge(j)->getTo() << " " <<g.GetNode(i)->getEdge(j)->GetWeight() << " " << graph[i][j] << std::endl;
//		}
//	}
/*
	std::cout << g.GetNode(1)->getEdge(0)->getFrom() << "-" << g.GetNode(1)->getEdge(0)->getTo() << std::endl; //verschiebung um 1 bei j = i-1
	std::cout << g.GetNode(1)->getEdge(1)->GetWeight() << "-" << g.GetNode(1)->getEdge(1)->getTo() << std::endl;
	
	
	
	std::cout << "Matrix: " << std::endl;
	for(int i = 0; i < width*height; i++)
	{
		for(int j = 0; j < width*height; j++){
			std::cout << " " << graph[i][j] << "\t";
		}
		std::cout << std::endl;
	}

	
	Heuristic<MNPuzzleState<width, height>> h;
	h.lookups.resize(0);
	h.lookups.push_back({kDynNode, 0, 0});
	h.heuristics.resize(0);
	h.heuristics.push_back(&dyn);
	
	{
		IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
		ida.SetHeuristic(&h);
		std::vector<slideDir> path;
		start = STP::GetKorfInstance(1);
		ida.GetPath(&mnp, start, goal, path);
		printf("ida\t%1.2f\t%llu\t%llu\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(), ida.GetNodesTouched());
	}
	
}
*/
void PhO()
{
	MNPuzzle<width, height> mnp;
	Heuristic<MNPuzzleState<width, height>> h1;
	MNPuzzleState<width,height> start, goal;
	std::vector<slideDir> moves;
	std::vector<MNPuzzleState<width, height>> statepath;
	goal.Reset();
	mnp.StoreGoal(goal);

	
	DynPermutationPDB<width, height, MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb(mnp);
	pdb.SetGoal(&goal);
		
	std::vector<LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> > pdbs = pdb.GetPDBs(std::string("./pdbs/"));

	h1.lookups.resize(0);
	h1.lookups.push_back({kMaxNode, 1, 2});
	h1.lookups.push_back({kLeafNode, 0, 0});
	h1.lookups.push_back({kLeafNode, 1, 1});
	
	h1.heuristics.resize(0);
	for(auto& i : pdbs)
	{
		h1.heuristics.push_back(&i);
	}
		
	//Testing:
	start = STP::GetKorfInstance(1);
	PhOHeuristic<width, height, MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pho(&h1, goal);

	Heuristic<MNPuzzleState<width, height>> h2;
	h2.lookups.resize(0);
	h2.lookups.push_back({kDynNode, 0, 0});
	h2.heuristics.resize(0);
	h2.heuristics.push_back(&pho);

    IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
    ida.SetHeuristic(&h2);
    std::vector<slideDir> path;
    start = STP::GetKorfInstance(1);
    ida.GetPath(&mnp, start, goal, path);
    printf("ida\t%1.2f\t%llu\t%llu\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(), ida.GetNodesTouched());
}



/*
 Linear Conflict:
 +	Added in MNPuzzle.h before call for DefaultH(const state)

 
 DPA-PDB:
 -	If hashing and reading of the data can be ignored (same as of now) than we only need to check how to generate the patterns. (each pattern gets its own pdb, which then will be pushed into the heuristic and afterwards passed on to IDA*
 */
