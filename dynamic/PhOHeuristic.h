
#include <langinfo.h>
#include "Heuristic.h"
#include "../3rdparty/lp/lp_solver.h"

//TODO: Remove action and environment from template
template<int width, int height, class state, class action, class environment>
class PhOHeuristic : public Heuristic<state> {

public:
    PhOHeuristic(Heuristic<state> *h, state goal, bool dual = false, bool is_integer = false) : h(h), goal(goal),
                                                                                                dual(dual), is_integer(
                    is_integer) {
        lps.reset(new lp::LPSolver(lp::LPSolverType::CPLEX));
    };

    virtual ~PhOHeuristic() {}

    virtual double HCost(const state &a, const state &b) const;

    void SetSortedPatterns();

private:
    double GetTileManDist(int tile, const state &s1) const;

    double GetPhOHeuristic(const state &a) const;

    void UpdatePhoHeuristic(const state &a) const;

    double GetPhOHDualHeuristic(const state &a) const;

    std::vector<std::vector<int>> sortedPatterns;
    Heuristic<state> *h;
    state goal;
    mutable bool firstRun = true;
    bool resolveLP = true;
    mutable std::unique_ptr<lp::LPSolver> lps;

    constexpr static int tiles = width * height - 1;

protected:
    bool is_integer = false;
    bool dual = false;

};


template<int width, int height, class state, class action, class environment>
double PhOHeuristic<width, height, state, action, environment>::HCost(const state &a, const state &b) const {
    //if(sortedPatterns.empty())
    //   SetSortedPatterns();
    if (!dual) {
        return GetPhOHeuristic(a);
    } else {
        return GetPhOHDualHeuristic(a);
    }
}

template<int width, int height, class state, class action, class environment>
double PhOHeuristic<width, height, state, action, environment>::GetTileManDist(int tile, const state &s1) const {
    auto itS = std::distance(s1.puzzle.begin(), std::find(s1.puzzle.begin(), s1.puzzle.end(), tile));
    auto itG = std::distance(goal.puzzle.begin(), std::find(goal.puzzle.begin(), goal.puzzle.end(), tile));
    unsigned int xG = itG % width;
    unsigned int yG = std::floor(itG / width);
    unsigned int xS = itS % width;
    unsigned int yS = std::floor(itS / width);
    return (std::abs((int) (xG - xS))) + (std::abs((int) (yG - yS)));
}
//TODO: Check with previous state für differences, only resolve if less than {X}, else reset the lps
template<int width, int height, class state, class action, class environment>
double PhOHeuristic<width, height, state, action, environment>::GetPhOHeuristic(const state &a) const {

    if (firstRun || !resolveLP) {
        firstRun = false;
        lps.reset(new lp::LPSolver(lp::LPSolverType::CPLEX));
        named_vector::NamedVector<lp::LPVariable> vars;
        named_vector::NamedVector<lp::LPConstraint> constraints;

        for (int i = 1; i < width * height; ++i) {
            vars.emplace_back(GetTileManDist(i, a), lps->get_infinity(), 1, is_integer);
        }

        std::vector<int> pattern;

        for (auto &tmph: h->heuristics) {
            pattern = dynamic_cast<PermutationPDB<state, action, environment> *>(tmph)->GetDistinct();
            lp::LPConstraint constraint(tmph->HCost(a, goal), lps->get_infinity());
            for (int i = 1; i < pattern.size(); i++) {
                int tile = pattern[i];
                constraint.insert(tile - 1, 1);
            }
            constraints.push_back(std::move(constraint));

        }

        lp::LinearProgram linearProgram(lp::LPObjectiveSense::MINIMIZE, std::move(vars), std::move(constraints));
        lps->load_problem(linearProgram);
    } else {
        UpdatePhoHeuristic(a);
    }
    lps->solve(); //X_1 + X_2 + X_4 >= h{1,2,4}
    std::vector<double> operatorCosts = lps->extract_solution();
    double hVal = 0.0;
    for (auto cost: operatorCosts) {
        hVal += cost;
    }
    return (int) hVal;
}

template<int width, int height, class state, class action, class environment>
void PhOHeuristic<width, height, state, action, environment>::UpdatePhoHeuristic(const state &a) const {
    // Update variables lower bound
    for (int i = 1; i < width * height; ++i) {
        this->lps->set_variable_lower_bound(i - 1, GetTileManDist(i, a));
    }
    std::vector<int> pattern;
    // Update constraints lower bound
    for (int i = 0; i < h->heuristics.size(); ++i) {
        this->lps->set_constraint_lower_bound(i, h->heuristics[i]->HCost(a, goal));
    }
}

template<int width, int height, class state, class action, class environment>
double PhOHeuristic<width, height, state, action, environment>::GetPhOHDualHeuristic(const state &a) const {
    //TODO: Write Update function for the dual problem
    named_vector::NamedVector<lp::LPVariable> vars;
    named_vector::NamedVector<lp::LPConstraint> constraints;
    int heuristicsSize = h->heuristics.size();

    //Add Y_i Variable for all pdbs, objective_coefficient = h{pdb->distincts}(s)
    for (int i = 0; i < heuristicsSize; ++i) {
        vars.emplace_back(0, lps->get_infinity(), h->heuristics[i]->HCost(a, goal), is_integer);
    }
    //Add Y_i for single tile heuristics, objective_coefficient = h{TMD}(s);
    for (int i = 0; i < tiles; ++i) {
        vars.emplace_back(0, lps->get_infinity(), GetTileManDist(i, a), is_integer);
    }
    //Generate constraints:
    //Iterate over tile-vectors
    for (int i = 0; i < sortedPatterns.size(); ++i) {
        lp::LPConstraint constraint(0, 1); //if all variables must be atleast 0, the constraint can't be less than 0.
        //Iterate over patterns which include tile i
        for (int j = 0; j < sortedPatterns[i].size(); ++j) {
            constraint.insert(sortedPatterns[i][j], 1);
        }
        constraint.insert(i + heuristicsSize, 1); //add single tile heuristics
        constraints.push_back(std::move(
                constraint)); //add constraint, which contains all variables of heuristics which contain tile i
    }

    lp::LinearProgram linearProgram(lp::LPObjectiveSense::MAXIMIZE, std::move(vars), std::move(constraints));
    lps->load_problem(linearProgram);
    lps->solve();
    //Debugging:
    //lps->print_failure_analysis();
    //lps->print_statistics();

    std::vector<double> operatorCosts = lps->extract_solution();
    double hVal = 0.0;
    for (int i = 0; i < heuristicsSize; ++i) {
        hVal += operatorCosts[i] * h->heuristics[i]->HCost(a, goal);
    }
    for (int i = 0; i < tiles; ++i) {
        hVal += operatorCosts[i + heuristicsSize] * GetTileManDist(i + 1, a);
    }
    return hVal;
}

template<int width, int height, class state, class action, class environment>
void PhOHeuristic<width, height, state, action, environment>::SetSortedPatterns() {
    std::vector<int> pattern;
    //sortedPatterns.reserve(tiles*h->heuristics.size());
    //create vector<int> for every tile
    for (int i = 0; i < tiles; ++i) {
        std::vector<int> _pattern;
        sortedPatterns.push_back(_pattern);
    }
    //Iterate over the pdbs
    for (int i = 0; i < h->heuristics.size(); ++i) {
        pattern = dynamic_cast<PermutationPDB<state, action, environment> *>(h->heuristics[i])->GetDistinct();
        //iterate over tiles in pattern
        for (int j = 1; j < pattern.size(); ++j) {
            sortedPatterns[pattern[j] - 1].push_back(
                    i); //enter pdb_index into tile-vector iff tile included in patterns
        }
    }
}


