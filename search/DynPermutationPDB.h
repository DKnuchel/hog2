//
//  DynPermutationPDB.h
//  CMakeTest
//
//  Created by Damian Knuchel on 07.08.21.
//

#ifndef DynPermutationPDB_h
#define DynPermutationPDB_h

#include "PermutationPDB.h"
#include "LexPermutationPDB.h"

template<const int width, const int height, class state, class action, class environment>
class DynPermutationPDB {
public:
    DynPermutationPDB(environment *env, int order);

    std::vector<LexPermutationPDB<state, action, environment>> GetPDBs(std::string path);

    void SetGoal(state *s) { goal = s; };
private:
    environment *env;
    state *goal;
    int order;

    void BuildPairsPDBs(const std::string &path);

    void BuildTripletsPDBs(const std::string &path);

    void BuildQuadrupletsPDBS(const std::string &path);

    std::vector<LexPermutationPDB<state, action, environment> > pdbs;
    std::vector<std::vector<int>> patterns;
    unsigned int threads = std::thread::hardware_concurrency();
};

template<int width, int height, class state, class action, class environment>
DynPermutationPDB<width, height, state, action, environment>::DynPermutationPDB(environment *env, int order) {
    this->env = env;
    this->order = order;
}

template<int width, int height, class state, class action, class environment>
void DynPermutationPDB<width, height, state, action, environment>::BuildPairsPDBs(const std::string &path) {
    std::vector<int> pattern = {0, 0, 0};
    for (int i = 1; i < width * height; i++) {
        for (int j = i + 1; j < width * height; j++) {
            pattern = {0, i, j};
            patterns.push_back(pattern);
            LexPermutationPDB<state, action, environment> pdb(env, *goal, patterns.back());
            if (!pdb.Load(path.c_str())) {
                env->SetPattern(pattern);
                pdb.BuildAdditivePDB(*goal, threads);
                pdb.Save(path.c_str());
            }
            pdbs.push_back(pdb);
        }
    }
}

template<int width, int height, class state, class action, class environment>
void DynPermutationPDB<width, height, state, action, environment>::BuildTripletsPDBs(const std::string &path) {
    std::vector<int> pattern = {0, 0, 0, 0};
    for (int i = 1; i < width * height; i++) {
        for (int j = i + 1; j < width * height; j++) {
            for (int k = j + 1; k < width * height; k++) {
                pattern = {0, i, j, k};
                patterns.push_back(pattern);
                LexPermutationPDB<state, action, environment> pdb(env, *goal, patterns.back());
                if (!pdb.Load(path.c_str())) {
                    env->SetPattern(pattern);
                    pdb.BuildAdditivePDB(*goal, threads);
                    pdb.Save(path.c_str());
                }
                pdbs.push_back(pdb);
            }
        }
    }
}


template<const int width, const int height, class state, class action, class environment>
void DynPermutationPDB<width, height, state, action, environment>::BuildQuadrupletsPDBS(const std::string &path) {
    std::vector<int> pattern = {0, 0, 0, 0, 0};
    for (int i = 1; i < width * height; i++) {
        for (int j = i + 1; j < width * height; j++) {
            for (int k = j + 1; k < width * height; k++) {
                for (int l = k + 1; l < width * height; l++) {
                    pattern = {0, i, j, k, l};
                    patterns.push_back(pattern);
                    LexPermutationPDB<state, action, environment> pdb(env, *goal, patterns.back());
                    if (!pdb.Load(path.c_str())) {
                        env->SetPattern(pattern);
                        pdb.BuildAdditivePDB(*goal, threads);
                        pdb.Save(path.c_str());
                    }
                    pdbs.push_back(pdb);
                }
            }
        }
    }
}

template<int width, int height, class state, class action, class environment>
std::vector<LexPermutationPDB<state, action, environment>>
DynPermutationPDB<width, height, state, action, environment>::GetPDBs(std::string path) {
    if (order > 1) BuildPairsPDBs(path);
    if (order > 2) BuildTripletsPDBs(path);
    if (order > 3) BuildQuadrupletsPDBS(path);
    return pdbs;
}


#endif /* DynPermutationPDB_h */
