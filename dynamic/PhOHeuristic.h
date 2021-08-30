
#include "Heuristic.h"
#include "../3rdparty/lp/lp_solver.h"

template<int width, int height, class state, class action, class environment>
class PhOHeuristic: public Heuristic<state>
{
public:
	PhOHeuristic(Heuristic<state> *hPairs, Heuristic<state> *hTriplets, state goal) : hPairs(hPairs), hTriplets(hTriplets), goal(goal) {};
	virtual ~PhOHeuristic(){}
	virtual double HCost(const state &a, const state &b) const;

private:
	Heuristic<state> *hPairs;
	Heuristic<state> *hTriplets;
	state goal;
protected:
	
};

template<int width, int height, class state, class action, class environment>
double PhOHeuristic<width, height, state, action, environment>::HCost(const state &a, const state &b) const
{
	lp::LPSolver *lps = new lp::LPSolver(lp::LPSolverType::CPLEX);
	named_vector::NamedVector<lp::LPVariable> vars;
	vars.emplace_back(-lps->get_infinity(), lps->get_infinity(), 0);
	vars.emplace_back(-lps->get_infinity(), lps->get_infinity(), 0);
	vars.emplace_back(-lps->get_infinity(), lps->get_infinity(), 0);
	
	named_vector::NamedVector<lp::LPConstraint> constraints;
	lp::LPConstraint constraint1(10, lps->get_infinity());
	lp::LPConstraint constraint2(15, lps->get_infinity());
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
	return 0.1;

}
