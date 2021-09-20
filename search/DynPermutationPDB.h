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
#include <sstream>
#include <fstream>
#include <sys/stat.h>
#include <string>

template<const int width, const int height, class state, class action, class environment>
class DynPermutationPDB {
public:
    DynPermutationPDB(environment *env, int order);

    std::vector<LexPermutationPDB<state, action, environment>> GetPDBs(std::string path);

    void SetGoal(state *s) { goal = s; };

    void SetFiltered(bool b) { this->isFiltered = b; };

private:
    environment *env;
    state *goal;
    int order;

    void BuildPairsPDBs(const std::string &path);

    void BuildTripletsPDBs(const std::string &path);

    void BuildQuadrupletsPDBS(const std::string &path);

    bool YeetPDB(LexPermutationPDB<state, action, environment> pdb, const std::string &path);

    environment mnp;
    bool isFiltered = false;

    std::vector<std::vector<std::vector<int>>> GetPartitions(std::vector<int> pattern, int m);

    double GetTileManDist(int tile, const state &s1);

    std::vector<LexPermutationPDB<state, action, environment> > pdbs;
    std::vector<std::vector<int>> patterns;
    unsigned int threads = std::thread::hardware_concurrency();
};

template<int width, int height, class state, class action, class environment>
DynPermutationPDB<width, height, state, action, environment>::DynPermutationPDB(environment *env, int order) {
    this->env = env;
    this->order = order;
}

inline std::vector<std::string> splitter(const std::string &str, const std::string &del) {
    int start = 0;
    int end = static_cast<int>(str.find(del));
    std::vector<std::string> ret;
    while (end != -1) {
        ret.push_back(str.substr(start, end - start));
        start = end + static_cast<int>(del.size());
        end = static_cast<int>(str.find(del, start));
    }
    ret.push_back(str.substr(start, end - start));
    return ret;
}

inline std::vector<int> extractPattern(const std::string &str) {
    std::vector<std::string> strPattern = splitter(str, " ");
    std::vector<int> pattern = {0};
    for (auto &s: strPattern) {
        pattern.push_back(std::stoi(s));
        std::vector<int> vec = {0};
    }
    return pattern;
}

inline std::vector<std::vector<int>> getNonarbitraryPatterns(const std::string &path) {
    std::vector<std::vector<int>> patterns;
    std::ifstream file(path + "yeeted");
    std::string buf;
    while (std::getline(file, buf)) {
        patterns.push_back(extractPattern(buf));
    }
    file.close();
    return patterns;
}

template<int width, int height, class state, class action, class environment>
void
DynPermutationPDB<width, height, state, action, environment>::BuildPairsPDBs(const std::string &path) {
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
            if (isFiltered) { if (!YeetPDB(pdb, path)) pdbs.push_back(pdb); }
            else pdbs.push_back(pdb);
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
                if (isFiltered) { if (!YeetPDB(pdb, path)) pdbs.push_back(pdb); }
                else pdbs.push_back(pdb);
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
                    if (isFiltered) { if (!YeetPDB(pdb, path)) pdbs.push_back(pdb); }
                    else pdbs.push_back(pdb);
                }
            }
        }
    }
}

inline bool checkIfFileExists(const std::string &name) {
    struct stat buffer{};
    return (stat(name.c_str(), &buffer) == 0);
}

template<int width, int height, class state, class action, class environment>
std::vector<LexPermutationPDB<state, action, environment>>
DynPermutationPDB<width, height, state, action, environment>::GetPDBs(std::string path) {
    if (!isFiltered || !checkIfFileExists(path + "yeeted")) {
        if (order > 1) BuildPairsPDBs(path);
        if (order > 2) BuildTripletsPDBs(path);
        if (order > 3) BuildQuadrupletsPDBS(path);
    } else {
        for (auto &pattern: getNonarbitraryPatterns(path + "yeeted")) {
            LexPermutationPDB<state, action, environment> pdb(env, *goal, pattern);
            if (!pdb.Load(path.c_str())) {
                env->SetPattern(pattern);
                pdb.BuildAdditivePDB(*goal, threads);
                pdb.Save(path.c_str());
            }
            pdbs.push_back(pdb);
        }
    }
    return pdbs;
}

//TODO: Check for optimization of the yeet method, currently very slow.
//!TODO: Generate file with not yeeted patterns so we don't have to recalculate them every time.
template<int width, int height, class state, class action, class environment>
bool
DynPermutationPDB<width, height, state, action, environment>::YeetPDB(LexPermutationPDB<state, action, environment> p,
                                                                      const std::string &path) {
    mnp.StoreGoal(*goal);
    std::vector<int> distinct = p.GetDistinct();
    distinct.erase(distinct.begin());
    std::vector<std::vector<std::vector<int>>> partitions;
    state concreteState;
    for (int i = 2; i <= distinct.size(); i++) { //Partitiong into i sets
        partitions = GetPartitions(distinct, i); //generate set of partitions
        for (auto &partition: partitions) { //for each possible partition
            int compare = 0;
            for (int hash = 0; hash < p.PDB.Size(); ++hash) { //for each possible state
                p.GetStateFromPDBHash(hash, concreteState); //generate a concrete state from the abstract state (hash)
                double sum = 0.0;
                for (auto &tiles: partition) { //for each set of tiles in the partition set
                    if (tiles.size() > 1) { //if HCost is found in the pdbs
                        std::vector<int> tmpPattern = {0}; //add 0 element to the beginning of the pattern
                        tmpPattern.insert(tmpPattern.end(), tiles.begin(), tiles.end());
                        LexPermutationPDB<state, action, environment> pdb(&mnp, *goal, tmpPattern); //get pdb of pattern
                        if (pdb.Load(path.c_str())) sum += pdb.HCost(concreteState, *goal); //Add HCost from pdb to sum
                    } else if (tiles.size() == 1)
                        sum += GetTileManDist(tiles[0],
                                              concreteState); //If only one tile, get Manhattan Distance of tile and add to sum
                }
                if (p.HCost(concreteState, *goal) > sum) {
                    ++compare;
                    break;
                } //if there is even one case where p improves the heuristic compared to the partition, we can go on to the next partition.
            }
            if (compare == 0) {
                std::cout << "Removed - " << p.GetFileName(std::string(path).c_str()) << std::endl;
                return true;
            } //if the pdb was no improvement compared to the partition set
        }
    }
    //To reach this point, there exists no additive partitioning of the pattern with the same optimality.
    std::ofstream file(path + "yeeted", std::fstream::app);
    for (auto &t: distinct) file << t << " ";
    file << "\n";
    file.close();
    return false;
}


/*
 * https://newbedev.com/how-to-find-all-partitions-of-a-set
 */
template<int width, int height, class state, class action, class environment>
std::vector<std::vector<std::vector<int>>>
DynPermutationPDB<width, height, state, action, environment>::GetPartitions(std::vector<int> pattern, int m) {
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

template<const int width, const int height, class state, class action, class environment>
double DynPermutationPDB<width, height, state, action, environment>::GetTileManDist(int tile, const state &s1) {
    auto itS = std::distance(s1.puzzle.begin(), std::find(s1.puzzle.begin(), s1.puzzle.end(), tile));
    auto itG = std::distance(goal->puzzle.begin(), std::find(goal->puzzle.begin(), goal->puzzle.end(), tile));
    unsigned int xG = itG % width;
    unsigned int yG = std::floor(itG / width);
    unsigned int xS = itS % width;
    unsigned int yS = std::floor(itS / width);
    return (std::abs((int) (xG - xS))) + (std::abs((int) (yG - yS)));
}

#endif /* DynPermutationPDB_h */
