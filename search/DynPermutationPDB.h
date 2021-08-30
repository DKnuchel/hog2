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
class DynPermutationPDB
{
public:
	DynPermutationPDB(environment &e);
	std::vector<LexPermutationPDB<state, action, environment>> GetPairs(std::string path);
	std::vector<LexPermutationPDB<state, action, environment>> GetTriplets(std::string path);
	void SetGoal(state *s){goal = s;};
private:
	environment env;
	state *goal;
	void BuildPairsPDBs(std::string path);
	void BuildTripletsPDBs(std::string path);
	std::vector<LexPermutationPDB<state, action, environment> > pairs;
	std::vector<LexPermutationPDB<state, action, environment> > triplets;
	std::vector<std::vector<int>> patterns;
	int threads = std::thread::hardware_concurrency();
};

template<int width, int height, class state, class action, class environment>
DynPermutationPDB<width, height, state, action, environment>::DynPermutationPDB(environment &e){
	this->env = env;
}

template<int width, int height, class state, class action, class environment>
void DynPermutationPDB<width, height, state, action, environment>::BuildPairsPDBs(std::string path)
{
	std::vector<int> pattern = {0, 0, 0};
	for (int i = 1; i < width*height; i++)
	{
		for (int j = i+1; j < width*height; j++)
		{
			pattern = {0, i, j};
			patterns.push_back(pattern);
			LexPermutationPDB<state, action, environment> pdb(&env, *goal, patterns.back());
			if (!pdb.Load(path.c_str()))
			{
				env.SetPattern(pattern);
				pdb.BuildAdditivePDB(*goal, threads);
				pdb.Save(path.c_str());
			}
			pairs.push_back(pdb);
		}
	}
}

template<int width, int height, class state, class action, class environment>
void DynPermutationPDB<width, height, state, action, environment>::BuildTripletsPDBs(std::string path)
{
	std::vector<int> pattern = {0, 0, 0, 0};
	for (int i = 1; i < width*height; i++)
	{
		for (int j = i+1; j < width*height; j++)
		{
			for (int k = j+1; k <width*height; k++){
				pattern = {0, i, j, k};
				patterns.push_back(pattern);
				LexPermutationPDB<state, action, environment> pdb(&env, *goal, patterns.back());
				if (!pdb.Load(path.c_str()))
				{
					env.SetPattern(pattern);
					pdb.BuildAdditivePDB(*goal, threads);
					pdb.Save(path.c_str());
				}
				triplets.push_back(pdb);
			}
		}
	}
}

template<int width, int height, class state, class action, class environment>
std::vector<LexPermutationPDB<state, action, environment>> DynPermutationPDB<width, height, state, action, environment>::GetPairs(std::string path)
{
	BuildPairsPDBs(path);
	return pairs;
}

template<int width, int height, class state, class action, class environment>
std::vector<LexPermutationPDB<state, action, environment>> DynPermutationPDB<width, height, state, action, environment>::GetTriplets(std::string path)
{
	BuildTripletsPDBs(path);
	return triplets;
}

#endif /* DynPermutationPDB_h */
