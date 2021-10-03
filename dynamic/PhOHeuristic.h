
#include <langinfo.h>
#include "Heuristic.h"
#include "../3rdparty/lp/lp_solver.h"

//TODO: Remove action and environment from template
template<int width, int height, class state, class action, class environment>
class PhOHeuristic : public Heuristic<state> {

public:
    PhOHeuristic(Heuristic<state> *h, state goal, std::string pdbPath, bool dual = false, bool is_integer = false,
                 bool resolve = false)
            : heur(h), goal(goal),
              dual(dual), is_integer(
                    is_integer), resolveLP(resolve), path(pdbPath) {
        lps.reset(new lp::LPSolver(lp::LPSolverType::CPLEX));
    };

    virtual ~PhOHeuristic() {}

    virtual double HCost(const state &a, const state &b) const;

    void SetSortedPatterns();

    void setFiltered() {
        filter = true;
        resolveLP = false;
    };

    void setTest() {
        test = true;
    };

private:
    double GetTileManDist(int tile, const state &s1) const;

    double GetPhOHeuristic(const state &a) const;

    void UpdatePhoHeuristic(const state &a) const;

    double GetPhOHDualHeuristic(const state &a) const;

    void UpdatePhoDualHeuristic(const state &a) const;

    bool CheckPDB(LexPermutationPDB<state, action, environment> pdb, const state &s) const;

    std::vector<std::vector<std::vector<int>>> GetPartitions(std::vector<int> pattern, int m) const;

    std::vector<std::vector<int>> sortedPatterns;
    Heuristic<state> *heur;
    state goal;
    mutable bool firstRun = true;
    bool resolveLP = false;
    bool filter = false;
    bool test = false;
    std::string path;
    mutable std::unique_ptr<lp::LPSolver> lps;

    constexpr static int tiles = width * height - 1;

    const double epsilon = 0.00001;
    mutable environment mnp;

protected:
    bool is_integer = false;
    bool dual = false;

};


template<int width, int height, class state, class action, class environment>
double PhOHeuristic<width, height, state, action, environment>::HCost(const state &a, const state &b) const {
    //if(sortedPatterns.empty())
    //   SetSortedPatterns();
    if (!dual) {
        return std::floor(GetPhOHeuristic(a) + epsilon);
    } else {
        return std::floor(GetPhOHDualHeuristic(a) + epsilon);
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
        bool check = true;
        lps.reset(new lp::LPSolver(lp::LPSolverType::CPLEX));
        named_vector::NamedVector<lp::LPVariable> vars;
        named_vector::NamedVector<lp::LPConstraint> constraints;

        for (int i = 1; i < width * height; ++i) {
            vars.template emplace_back(0, lps->get_infinity(), 1,
                                       is_integer); //set variable from tile i with lower bound MD, because X_i >= h_i
        }
        std::vector<int> pattern;
        for (auto &tmph: heur->heuristics) {
            if(filter) {
                auto lex = dynamic_cast<LexPermutationPDB<state, action, environment> *>(tmph);
                check = CheckPDB(*lex, a);
            }
            if (check) {
                pattern = dynamic_cast<PermutationPDB<state, action, environment> *>(tmph)->GetDistinct();
                lp::LPConstraint constraint(tmph->HCost(a, goal), lps->get_infinity());
                for (int i = 1; i < pattern.size(); ++i) {
                    int tile = pattern[i];
                    constraint.insert(tile - 1, 1);
                }
                constraints.push_back(std::move(constraint));
            }
        }

        for (int i = 0; i < tiles; ++i) {
            lp::LPConstraint constraint(GetTileManDist(i + 1, a), lps->get_infinity());
            constraint.insert(i, 1);
            constraints.push_back(std::move(constraint));
        }

        lp::LinearProgram linearProgram(lp::LPObjectiveSense::MINIMIZE, std::move(vars), std::move(constraints));
        lps->load_problem(linearProgram);
    } else {
        UpdatePhoHeuristic(a);
    }
    lps->solve(); //X_1 + X_2 + X_4 >= heur{1,2,4}
    //return static_cast<int>(std::floor(lps->get_objective_value()));
    if(test) {
        lps->print_statistics();
        exit(0);
    }
    return lps->get_objective_value();
}

template<int width, int height, class state, class action, class environment>
void PhOHeuristic<width, height, state, action, environment>::UpdatePhoHeuristic(const state &a) const {
    // Update variables lower bound
    for (int i = 1; i < width * height; ++i) {
        this->lps->set_variable_lower_bound(i - 1, GetTileManDist(i, a));
    }
    std::vector<int> pattern;
    // Update constraints lower bound
    for (int i = 0; i < heur->heuristics.size(); ++i) {
        this->lps->set_constraint_lower_bound(i, heur->heuristics[i]->HCost(a, goal));
    }
}

template<int width, int height, class state, class action, class environment>
double PhOHeuristic<width, height, state, action, environment>::GetPhOHDualHeuristic(const state &a) const {
    //TODO: Write Update function for the dual problem
    if (firstRun || !resolveLP) {
        lps.reset(new lp::LPSolver(lp::LPSolverType::CPLEX));
        named_vector::NamedVector<lp::LPVariable> vars;
        named_vector::NamedVector<lp::LPConstraint> constraints;

        //Add Y_i Variable for all pdbs, objective_coefficient = heur{pdb->distincts}(s)
        for (int i = 0; i < heur->heuristics.size(); ++i) {
            vars.emplace_back(0, lps->get_infinity(), heur->heuristics[i]->HCost(a, goal), is_integer);
        }
        //Add Y_i for single tile heuristics, objective_coefficient = heur{TMD}(s);
        for (int i = 0; i < tiles; ++i) {
            vars.emplace_back(0, lps->get_infinity(), GetTileManDist(i + 1, a), is_integer);
        }
        //Generate constraints:
        //Iterate over tile-vectors
        for (int i = 0; i < sortedPatterns.size(); ++i) {
            lp::LPConstraint constraint(-lps->get_infinity(),
                                        1); //if all variables must be atleast 0, the constraint can't be less than 0.
            //Iterate over patterns which include tile i
            for (int j: sortedPatterns[i]) {
                constraint.insert(j, 1);
            }
            constraint.insert(i + heur->heuristics.size(), 1); //add single tile heuristics
            constraints.push_back(std::move(
                    constraint)); //add constraint, which contains all variables of heuristics which contain tile i
        }
        lp::LinearProgram linearProgram(lp::LPObjectiveSense::MAXIMIZE, std::move(vars), std::move(constraints));
        lps->load_problem(linearProgram);
    } else {
        UpdatePhoDualHeuristic(a);
    }
    lps->solve();
    //lps->print_statistics();
    return lps->get_objective_value();
}


template<int width, int height, class state, class action, class environment>
void PhOHeuristic<width, height, state, action, environment>::UpdatePhoDualHeuristic(const state &a) const {
    for (int i = 0; i < heur->heuristics.size(); ++i)
        this->lps->set_objective_coefficient(i, heur->heuristics[i]->HCost(a, goal));
    for (int i = 0; i < tiles; ++i)
        this->lps->set_objective_coefficient(i + heur->heuristics.size(), GetTileManDist(i + 1, a));
}


template<int width, int height, class state, class action, class environment>
void PhOHeuristic<width, height, state, action, environment>::SetSortedPatterns() {
    std::vector<int> pattern;
    //sortedPatterns.reserve(tiles*heur->heuristics.size());
    //create vector<int> for every tile
    for (int i = 0; i < tiles; ++i) {
        std::vector<int> _pattern;
        sortedPatterns.push_back(_pattern);
    }
    //Iterate over the pdbs
    for (int i = 0; i < heur->heuristics.size(); ++i) {
        pattern = dynamic_cast<PermutationPDB<state, action, environment> *>(heur->heuristics[i])->GetDistinct();
        //iterate over tiles in pattern
        for (int j = 1; j < pattern.size(); ++j) {
            sortedPatterns[pattern[j] - 1].push_back(
                    i); //enter pdb_index into tile-vector iff tile included in patterns
        }
    }
}

template<int width, int height, class state, class action, class environment>
bool
PhOHeuristic<width, height, state, action, environment>::CheckPDB(LexPermutationPDB<state, action, environment> pdb,
                                                                  const state &s) const {
    if (!filter) return true;
    state s2;
    s2.Reset();
    mnp.StoreGoal(s2);
    std::vector<int> distinct = pdb.GetDistinct();
    std::vector<std::vector<std::vector<int>>> partitions;
    double pdbCost = pdb.HCost(s, goal);
    for (int i = 2; i <= distinct.size(); i++) { //Partitiong into i sets
        std::vector<int> woz = {};
        woz.insert(woz.begin(), distinct.begin() + 1, distinct.end());
        partitions = GetPartitions(woz, i); //generate set of partitions
        for (auto &partition: partitions) { //for each possible partition
            double sum = 0.0;
            for (auto &tileSet: partition) { //for each set of tiles in the partition set
                if (tileSet.size() > 1) { //if HCost is found in the pdbs
                    std::vector<int> tmpPattern = {0};
                    tmpPattern.insert(tmpPattern.end(), tileSet.begin(), tileSet.end());
                    mnp.SetPattern(tmpPattern);
                    LexPermutationPDB<state, action, environment> tmpPDB(&mnp, goal,
                                                                         tmpPattern); //get pdb of pattern
                    if (tmpPDB.Load(path.c_str())) {
                        sum += tmpPDB.HCost(s, goal);
                    }
                } else if (tileSet.size() == 1) {
                    sum += GetTileManDist(tileSet[0], s);
                }
            }
            if (pdbCost <= sum) return false;
        }
    }
    return true;
}

template<int width, int height, class state, class action, class environment>
std::vector<std::vector<std::vector<int>>>
PhOHeuristic<width, height, state, action, environment>::GetPartitions(std::vector<int> pattern, int m) const {
    std::vector<std::vector<std::vector<int>>> ret;
    if (pattern.size() < m || m < 1) return ret;
    if (m == 1) {
        std::vector<std::vector<int>> partition;
        partition.push_back(pattern);
        ret.push_back(partition);
        return ret;
    }
    std::vector<std::vector<std::vector<int>>> prev1 = GetPartitions(
            std::vector<int>(pattern.begin(), pattern.end() - 1), m);
    for (int i = 0; i < prev1.size(); ++i) {
        for (int j = 0; j < prev1[i].size(); ++j) {
            std::vector<std::vector<int>> l;
            for (auto &inner: prev1[i]) l.push_back(inner);
            l[j].push_back(pattern[pattern.size() - 1]);
            ret.push_back(l);
        }

    }
    std::vector<int> set;
    set.push_back(pattern[pattern.size() - 1]);
    std::vector<std::vector<std::vector<int>>> prev2 = GetPartitions(
            std::vector<int>(pattern.begin(), pattern.end() - 1), m - 1);
    for (int i = 0; i < prev2.size(); ++i) {
        std::vector<std::vector<int>> l = prev2[i];
        l.push_back(set);
        ret.push_back(l);
    }
    return ret;
}


