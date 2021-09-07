
#include <langinfo.h>
#include "Heuristic.h"
#include "../3rdparty/lp/lp_solver.h"

template<int width, int height, class state, class action, class environment>
class PhOHeuristic: public Heuristic<state>
{
public:
	PhOHeuristic(Heuristic<state> *h, state goal) : h(h), goal(goal) {};
	virtual ~PhOHeuristic(){}
	virtual double HCost(const state &a, const state &b) const;


private:
    double GetTileManDist(int tile,const state &s1) const;
	Heuristic<state> *h;
	state goal;
protected:
	
};


template<int width, int height, class state, class action, class environment>
double PhOHeuristic<width, height, state, action, environment>::HCost(const state &a, const state &b) const
{
	//auto *lps = new lp::LPSolver(lp::LPSolverType::CPLEX);
    std::unique_ptr<lp::LPSolver> lps(new lp::LPSolver(lp::LPSolverType::CPLEX));
	named_vector::NamedVector<lp::LPVariable> vars;
    std::vector<int> pattern;
    named_vector::NamedVector<lp::LPConstraint> constraints;

    for(int i = 1; i < width*height; ++i)
    {
        vars.emplace_back(GetTileManDist(i, a), lps->get_infinity(), 0);
    }
    /*
    for(int i = 0; i < width*height-1; i++)
    {
        lp::LPConstraint constraint(GetTileManDist(i+1, a), lps->get_infinity());
        constraint.insert(i, 1);
        constraints.push_back(std::move(constraint));
    }
*/
    for(auto &tmph : h->heuristics)
    {
        pattern = dynamic_cast<PermutationPDB<state, action, environment> *>(tmph)->GetDistinct();
        //pattern.erase(pattern.begin());
        lp::LPConstraint constraint(tmph->HCost(a, goal), lps->get_infinity());
        for(int i = 1; i < pattern.size(); i++)
        {
            int tile = pattern [i];
            constraint.insert(tile-1, 1);
        }
        constraints.push_back(std::move(constraint));
    }

    lp::LinearProgram linearProgram(lp::LPObjectiveSense::MINIMIZE, std::move(vars), std::move(constraints));
    lps->load_problem(linearProgram);
    lps->solve();
    std::vector<double> operatorCosts = lps->extract_solution();
    double hVal = 0.0;
    for(auto cost : operatorCosts)
    {
        hVal += cost;
    }

    return hVal;
}

template<int width, int height, class state, class action, class environment>
double PhOHeuristic<width, height, state, action, environment>::GetTileManDist(int tile, const state &s1) const
{
    auto itS = std::distance(s1.puzzle.begin(), std::find(s1.puzzle.begin(), s1.puzzle.end(), tile));
    auto itG = std::distance(goal.puzzle.begin(), std::find(goal.puzzle.begin(), goal.puzzle.end(), tile));
    unsigned int xG = itG % width;
    unsigned int yG = std::floor(itG / width);
    unsigned int xS = itS % width;
    unsigned int yS = std::floor(itS / width);
    return (std::abs((int)(xG-xS)))+(std::abs((int)(yG-yS)));
}

