#include "Driver.h"
#include "MNPuzzle.h"
#include "IDAStar.h"
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"
#include "DynPermutationPDB.h"
#include "Timer.h"
#include "DynamicPDBSearch.h"
#include "Graph.h"
#include <iostream>
#include <fstream>

#include "PhOHeuristic.h"

#ifdef PUZZLE_SIZE
#if PUZZLE_SIZE == 4
#define SIZE 4
#include "STPInstances.h"
#elif PUZZLE_SIZE == 5
#define SIZE 5
#endif
#endif
//#define PUZZLE_SIZE @PUZZLE_SIZE@
const int width = SIZE, height = SIZE;
//const int width = 2, height = 3;

enum MODE {
    MD,
    LC,
    PDB,
    GENERATE
};
enum PDBMODE {
    STATIC,
    DYNAMIC,
    PHO
};
enum PHOMODE {
    PRIME,
    DUAL
};
typedef struct options {
    std::string puzzlePath;
    MODE mode;
    PDBMODE pdbMode;
    PHOMODE phoMode = PHOMODE::PRIME;
    std::vector<std::vector<int>> patterns;
    std::string pdbPath;
    int patternMaxOrder;
    bool isInteger = false;
    bool isFiltered = false;
    bool resolve = false;
    bool test = false;
    bool sevenTest = false;

    bool puzzlePathSet = false;
    bool patternsSet = false;
    bool patternMaxOrderSet = false;
    bool modeSet = false;
    bool pdbModeSet = false;
    bool pdbPathSet = false;

    bool checkOptions() const {
        if (!puzzlePathSet & (mode != MODE::GENERATE)) {
            std::cerr << "Missing puzzle! " << puzzlePath << std::endl;
            return false;
        }
        if (!modeSet) {
            std::cerr << "Missing algorithm!" << std::endl;
            return false;
        }
        if ((mode == MODE::PDB) & !pdbModeSet) {
            std::cerr << "Missing pdbMode!" << std::endl;
            return false;
        }
        if ((mode == MODE::PDB) & !patternsSet & !patternMaxOrderSet) {
            std::cerr << "Missing pattern or max order!" << std::endl;
            return false;
        }
        if ((mode == MODE::PDB) & !pdbPathSet) {
            std::cerr << "Missing pdb path!" << std::endl;
        }
        if (((mode == MODE::LC) || (mode == MODE::MD)) & (patternMaxOrderSet || patternsSet)) {
            std::cerr << "Ignoring patters" << std::endl;
            return false;
        }
        if ((mode == MODE::GENERATE) & (!puzzlePathSet || !pdbPathSet)) {
            std::cerr << "Missing paths" << std::endl;
        }
        if (pdbModeSet & (pdbMode == PDBMODE::STATIC) & !patternsSet) {
            std::cerr << "Missing Pattern!";
        }
        if (patternMaxOrderSet & (patternMaxOrder < 2)) {
            //std::cerr << "Order must be at least 2" << std::endl;
            //return false;
        }
        return true;
    };
} options;

void BasicHeuristics();

void StaticPDB();

void PhO(bool dual, bool is_integer, bool resolve);

void Generate();

void Test();

std::vector<std::vector<int>> extractPDBPattern(std::string str);

//std::vector<std::string> splitter(std::string str, std::string del);

options opt;

options getOptions(int argc, const char *const *argv);

int main(int argc, const char *argv[]) {
    //Test();
    //exit(0);
    std::string args = "";
    for (int i = 0; i < argc; ++i) {
        args += static_cast<std::string>(argv[i]) + " ";
    }
    //std::cerr << args << std::endl;
    //std::cout << "PUZZLE_SIZE: " << PUZZLE_SIZE << " Size: " << SIZE << std::endl;
    opt = getOptions(argc, argv);
    switch (opt.mode) {
        case MD:
            BasicHeuristics();
            break;
        case LC:
            BasicHeuristics();
            break;
        case PDB:
            switch (opt.pdbMode) {
                case STATIC:
                    StaticPDB();
                    break;
                case DYNAMIC:
                    PhO(true, true, opt.resolve);
                    break;
                case PHO:
                    switch (opt.phoMode) {
                        case PRIME:
                            PhO(false, opt.isInteger, opt.resolve);
                            break;
                        case DUAL:
                            PhO(true, opt.isInteger, opt.resolve);
                            break;
                    }
                    break;
            }
            break;
        case GENERATE:
            Generate();
            break;
    }
}

//TODO: Add option if a lp should be newly generated or resolved with updated values, bool already exists in {PhOHeuristic.h}
options getOptions(int argc, const char *const *argv) {
    options _opt;

    for (int i = 0; i < argc; ++i) {
        if (static_cast<std::string>(argv[i]) == "-i") {
            _opt.puzzlePath = static_cast<std::string>(argv[++i]);
            _opt.puzzlePathSet = true;
        } else if (static_cast<std::string>(argv[i]) == "-p") {
            if ((static_cast<std::string>(argv[++i])).size() > 4) {
                _opt.patterns = extractPDBPattern(static_cast<std::string>(argv[i]));
                _opt.patternsSet = true;
            } else {
                _opt.patternMaxOrder = std::stoi(argv[i]);
                _opt.patternMaxOrderSet = true;
            }
        } else if (static_cast<std::string>(argv[i]) == "-m") {
            if (static_cast<std::string>(argv[++i]) == "MD") {
                _opt.mode = MD;
                _opt.modeSet = true;
            } else if (static_cast<std::string>(argv[i]) == "LC") {
                _opt.mode = LC;
                _opt.modeSet = true;
            } else if (static_cast<std::string>(argv[i]) == "PDB") {
                _opt.mode = PDB;
                _opt.modeSet = true;
            } else if (static_cast<std::string>(argv[i]) == "GENERATE") {
                _opt.mode = GENERATE;
                _opt.modeSet = true;
            }
        } else if (static_cast<std::string>(argv[i]) == "--static") {
            _opt.pdbMode = STATIC;
            _opt.pdbModeSet = true;
        } else if (static_cast<std::string>(argv[i]) == "--dynamic") {
            _opt.pdbMode = DYNAMIC;
            _opt.pdbModeSet = true;
        } else if (static_cast<std::string>(argv[i]) == "--pho") {
            _opt.pdbMode = PHO;
            _opt.pdbModeSet = true;
        } else if (static_cast<std::string>(argv[i]) == "--dual") {
            _opt.phoMode = DUAL;
        } else if (static_cast<std::string>(argv[i]) == "--integer") {
            _opt.isInteger = true;
        } else if (static_cast<std::string>(argv[i]) == "-d") {
            _opt.pdbPath = static_cast<std::string>(argv[++i]);
            _opt.pdbPathSet = true;
        } else if (static_cast<std::string>(argv[i]) == "--filtered") {
            _opt.isFiltered = true;
        } else if (static_cast<std::string>(argv[i]) == "--resolve") {
            _opt.resolve = true;
        } else if (static_cast<std::string>(argv[i]) == "--test") {
            _opt.test = true;
        } else if (static_cast<std::string>(argv[i]) == "--seven") {
            _opt.sevenTest = true;
        }
    }
    if (!_opt.checkOptions())
        std::abort();
    return _opt;
}


void BasicHeuristics() {
    MNPuzzle<width, height> mnp;//({kRight, kLeft, kDown, kUp});
    MNPuzzleState<width, height> start, goal;
    std::vector<slideDir> path;
    mnp.Set_Use_Manhattan_Heuristic((opt.mode == MODE::MD));
    mnp.Set_Use_Linear_Conflict_Heuristic((opt.mode == MODE::LC));
    Timer t;
    {
        t.StartTimer();
        std::vector<MNPuzzleState<width, height>> _vec;
        MNPuzzle<width, height>::read_in_mn_puzzles(opt.puzzlePath.c_str(), false, 1, _vec);
        start = _vec[0];
        goal.Reset();
        IDAStar<MNPuzzleState<width, height>, slideDir> ida;
        ida.GetPath(&mnp, start, goal, path);
        printf("ida\tLength: %1.2f\tExpanded: %lu\tGenerated: %lu\tTime: %1.2f\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(),
               ida.GetNodesTouched(), t.EndTimer());
    }
}

void StaticPDB() {
    MNPuzzle<width, height> mnp;//({kRight, kLeft, kDown, kUp});
    Heuristic<MNPuzzleState<width, height>> h;
    MNPuzzleState<width, height> start, goal;
    std::vector<slideDir> moves;
    std::vector<MNPuzzleState<width, height>> statePath;
    goal.Reset();
    mnp.StoreGoal(goal);
    unsigned int threads = std::thread::hardware_concurrency();
    std::vector<LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>>> pdbs;
    for (auto &p: opt.patterns) {
        LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb(&mnp, goal, p);
        pdbs.push_back(pdb);
    }

    h.lookups.resize(0);
    h.lookups.push_back({kAddNode, 1, 3});
    h.lookups.push_back({kLeafNode, 0, 0});
    h.lookups.push_back({kLeafNode, 1, 1});
    if (opt.patterns.size() > 2) h.lookups.push_back({kLeafNode, 2, 2});

    h.heuristics.resize(0);
    for (auto &pdb: pdbs) {
        if (!pdb.Load(opt.pdbPath.c_str())) {
            mnp.SetPattern(pdb.GetDistinct());
            pdb.BuildAdditivePDB(goal, threads);
            pdb.Save(opt.pdbPath.c_str());
        }
        h.heuristics.push_back(&pdb);
    }

    Timer t;
    {
        t.StartTimer();
        std::vector<MNPuzzleState<width, height>> _vec;
        MNPuzzle<width, height>::read_in_mn_puzzles(opt.puzzlePath.c_str(), false, 1, _vec);
        start = _vec[0];
        IDAStar<MNPuzzleState<width, height>, slideDir> ida;
        ida.SetHeuristic(&h);
        std::vector<slideDir> path;
        ida.GetPath(&mnp, start, goal, path);
        printf("ida\tLength: %1.2f\tExpanded: %lu\tGenerated: %lu\tTime: %1.2f\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(),
               ida.GetNodesTouched(), t.EndTimer());
    }
}

void PhO(bool dual, bool is_integer, bool resolve) {
    MNPuzzle<width, height> mnp;//({kRight, kLeft, kDown, kUp});
    Heuristic<MNPuzzleState<width, height>> h1;
    MNPuzzleState<width, height> start, goal;
    std::vector<slideDir> moves;
    std::vector<MNPuzzleState<width, height>> statePath;
    goal.Reset();
    mnp.StoreGoal(goal);


    DynPermutationPDB<width, height, MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb(&mnp,
                                                                                                          opt.patternMaxOrder);
    pdb.SetFiltered(false);
    if (opt.sevenTest)
        pdb.SetPatterns({{0, 1, 2,  3,  4,  5,  6,  8},
                         {0, 7, 10, 11, 12, 13, 14, 15},
                         {0, 1, 2,  3,  4,  5,  6,  7},
                         {0, 2, 3,  4,  5,  6,  7,  8},
                         {0, 3, 4,  5,  6,  7,  8,  9},
                         //{0, 4, 5,  6,  7,  8,  9,  10},
                         //{0, 5, 6,  7,  8,  9,  10, 11},
                         {0, 6, 7,  8,  9,  10, 11, 12},
                         {0, 7, 8,  9,  10, 11, 12, 13},
                         {0, 8, 9,  10, 11, 12, 13, 14},
                         {0, 9, 10, 11, 12, 13, 14, 15}});
    pdb.SetGoal(&goal);

    std::vector<LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>>> pdbs = pdb.GetPDBs(
            std::string(opt.pdbPath));

    h1.lookups.resize(0);
    h1.lookups.push_back({kMaxNode, 1, 2});
    h1.lookups.push_back({kLeafNode, 0, 0});
    h1.lookups.push_back({kLeafNode, 1, 1});

    h1.heuristics.resize(0);
    for (auto &i: pdbs) {
        h1.heuristics.push_back(&i);
    }

    PhOHeuristic<width, height, MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pho(&h1, goal,
                                                                                                     std::move(
                                                                                                             std::string(
                                                                                                                     opt.pdbPath)),
                                                                                                     dual,
                                                                                                     is_integer,
                                                                                                     resolve);
    pho.SetSortedPatterns();
    if (opt.isFiltered) {
        std::cout << "Filtered" << std::endl;
        pho.setFiltered();
        pho.setTest();
    }
    if (opt.test) {
        pho.setTest();
    }

    Heuristic<MNPuzzleState<width, height>> h2;
    h2.lookups.resize(0);
    //h2.lookups.push_back({kMaxNode, 0, 1});
    h2.lookups.push_back({kDynNode, 0, 0});
    //h2.lookups.push_back({kLeafNode, 0, 0});
    h2.heuristics.resize(0);
    h2.heuristics.push_back(&pho);

    Timer t;
    {
        t.StartTimer();
        IDAStar<MNPuzzleState<width, height>, slideDir> ida;
        ida.SetHeuristic(&h2);
        std::vector<slideDir> path;
        std::vector<MNPuzzleState<width, height>> _vec;
        /*
        if (MNPuzzle<width, height>::read_in_mn_puzzles(opt.puzzlePath.c_str(), false, 1, _vec) == 1)
            std::cerr << "Failed to read puzzle!" << std::endl;
        for (int i = 0; i < _vec[0].puzzle.size(); ++i) {
            start.puzzle[i] = _vec[0].puzzle[i];
        }
         */
        MNPuzzle<width, height>::read_in_mn_puzzles(opt.puzzlePath.c_str(), false, 1, _vec);
        start = _vec[0];
        ida.GetPath(&mnp, start, goal, path);
        //std::vector<MNPuzzleState<width,height>> spath;
        //ida.GetPath(&mnp, start, goal, spath);
        printf("ida\tLength: %1.2f\tExpanded: %lu\tGenerated: %lu\tTime: %1.2f\n", mnp.GetPathLength(start, path), ida.GetNodesExpanded(),
               ida.GetNodesTouched(), t.EndTimer());
    }
}


std::vector<std::vector<int>> extractPDBPattern(std::string str) {
    std::vector<std::string> strPattern = splitter(str, ")(");
    std::vector<std::vector<int>> pattern;
    for (auto &s: strPattern) {
        s.erase(std::remove(s.begin(), s.end(), '('), s.end());
        s.erase(std::remove(s.begin(), s.end(), ')'), s.end());
        std::vector<int> vec = {0};
        for (auto &p: splitter(s, ",")) {
            vec.push_back(std::stoi(p));
        }
        pattern.push_back(vec);
    }
    return pattern;
}

/*
std::vector<std::string> splitter(std::string str, std::string del) {
    int start = 0;
    int end = str.find(del);
    std::vector<std::string> ret;
    while (end != -1) {
        ret.push_back(str.substr(start, end - start));
        start = end + del.size();
        end = str.find(del, start);
    }
    ret.push_back(str.substr(start, end - start));
    return ret;
}
*/
void Generate() {
    opt.patternMaxOrder = 4;
    MNPuzzle<width, height> mnp;
    MNPuzzleState<width, height> start, goal;

    //Generate 1000 puzzles
    std::vector<std::vector<int>> puzzles;
    for (int i = 0; i < 1000; ++i) {
        start = mnp.Generate_Random_Puzzle();
        std::vector<int> puzzle;
        puzzle.insert(puzzle.begin(), std::begin(start.puzzle), std::end(start.puzzle));
        puzzles.push_back(puzzle);
    }

    //Remove duplicate puzzles:
    std::sort(puzzles.begin(), puzzles.end());
    puzzles.erase(std::unique(puzzles.begin(), puzzles.end()), puzzles.end());
    assert(puzzles.size() == 1000 && "Duplicates detected!");

    for (int i = 0; i < 1000; ++i) {
        std::ofstream file(opt.puzzlePath + std::to_string(i) + ".pzl");
        for (auto &p: puzzles[i]) {
            file << p << " ";
        }
        file.close();
    }

    goal.Reset();
    DynPermutationPDB<width, height, MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb(&mnp,
                                                                                                          opt.patternMaxOrder);
    pdb.SetGoal(&goal);
    pdb.GetPDBs(std::string(opt.pdbPath));
    unsigned int threads = std::thread::hardware_concurrency();
    std::vector<LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>>> pdbs;
#ifdef PUZZLE_SIZE
#if PUZZLE_SIZE == 4
    for (int i = 0; i < 100; ++i) {
        std::ofstream file(opt.puzzlePath + "korf" + std::to_string(i) + ".pzl");
        for (auto &p: STP::GetKorfInstance(i).puzzle) {
            file << p << " ";
        }
        file.close();
    }
    opt.patterns = {
                    {0,  8,  9,  10, 11, 12, 13, 14, 15},
                    {0,  1,  2,  4,  5,  8},
                    {0,  3,  6,  7,  10, 11},
                    {0,  9,  12, 13, 14, 15},
                    {0,  1,  2,  4,  5,  8,  9},
                    {0,  3,  6,  7,  10, 11, 15},
                    {0,  12, 13, 14},
                    {0,  1,  2,  3,  4,  5,  6,  8},
                    {0,  7,  10, 11, 12, 13, 14, 15},
                    {0,  1,  2,  3,  4,  5,  6,  7},
                    {0,  2,  3,  4,  5,  6,  7,  8},
                    {0,  3,  4,  5,  6,  7,  8,  9},
                    {0,  4,  5,  6,  7,  8,  9,  10},
                    {0,  5,  6,  7,  8,  9,  10, 11},
                    {0,  6,  7,  8,  9,  10, 11, 12},
                    {0,  7,  8,  9,  10, 11, 12, 13},
                    {0,  8,  9,  10, 11, 12, 13, 14},
                    {0,  9,  10, 11, 12, 13, 14, 15}
                    };
#elif PUZZLE_SIZE == 5
    opt.patterns = {{0, 1,  5,  6,  10, 11, 12},
                    {0, 2,  3,  4,  7,  8,  9},
                    {0, 13, 14, 18, 19, 23, 24},
                    {0, 15, 16, 17, 20, 21, 22},
                    {0, 1,  2,  5,  6,  7,  12},
                    {0, 3,  4,  8,  9,  13, 14},
                    {0, 10, 11, 15, 16, 20, 21},
                    {0, 17, 18, 19, 22, 23, 24}};
#endif
#endif
    for (auto &p: opt.patterns) {
        LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>> pdb(&mnp, goal, p);
        pdbs.push_back(pdb);
    }

    for (auto &pdb: pdbs) {
        if (!pdb.Load(opt.pdbPath.c_str())) {
            mnp.SetPattern(pdb.GetDistinct());
            pdb.BuildAdditivePDB(goal, threads);
            pdb.Save(opt.pdbPath.c_str());
        }
    }
}
