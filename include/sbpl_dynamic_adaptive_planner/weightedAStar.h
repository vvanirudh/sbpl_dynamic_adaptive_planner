/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**

   Author : Anirudh Vemula
   Date : 6th March 2016

   Taken from Venkat's code for ASIPP.

   Implements the interfaces for the weighted A* algorithm that will be used as the planner. Extends the generic SBPLPlanner from SBPL

 */


#ifndef __WEIGHTED_A_STAR_H_
#define __WEIGHTED_A_STAR_H_

// @avemula CHANGED included the file from the same package
#include <sbpl_dynamic_adaptive_planner/DiscreteSpaceTimeInformation.h>

//---configuration----

//control of EPS
#define WeightedAStar_DEFAULT_INITIAL_EPS	    5.0
#define WeightedAStar_DECREASE_EPS    5.0
//#define WeightedAStar_FINAL_EPS	    1.0


//---------------------

#define WeightedAStar_INCONS_LIST_ID 0

class CMDP;
class CMDPSTATE;
class CMDPACTION;
class CHeap;
class CList;


//-------------------------------------------------------------

//state structure
typedef class WEIGHTEDASTARSEARCHSTATEDATA : public AbstractSearchState
{
public:
	CMDPSTATE* MDPstate; //the MDP state itself
	//WeightedAStar* relevant data
	unsigned int v;
	unsigned int g;
	short unsigned int iterationclosed;
	short unsigned int callnumberaccessed;
	short unsigned int numofexpands;
	//best predecessor and the action from it, used only in forward searches
	CMDPSTATE *bestpredstate;
	//the next state if executing best action
	CMDPSTATE  *bestnextstate;
	unsigned int costtobestnextstate;
	int h;

	
public:
	WEIGHTEDASTARSEARCHSTATEDATA() {};	
	~WEIGHTEDASTARSEARCHSTATEDATA() {};
} WeightedAStarState;

//-------------------------------------------------------------
//Bubbles
typedef struct{
  WeightedAStarState* state;
  CKey key;
  bool done;
} HoldState_t;

typedef struct{
  bool active;
  vector<HoldState_t*> hold;
} BubbleData_t;
//-------------------------------------------------------------



//statespace
typedef struct WEIGHTEDASTARSEARCHSTATESPACE
{
	double eps;
    double eps_satisfied;
	CHeap* heap;
	CList* inconslist;
	short unsigned int searchiteration;
	short unsigned int callnumber;
	CMDPSTATE* searchgoalstate;
	CMDPSTATE* searchstartstate;
	
	CMDP searchMDP;

	bool bReevaluatefvals;
    bool bReinitializeSearchStateSpace;
	bool bNewSearchIteration;

} WeightedAStarSearchStateSpace_t;



//WeightedAStar* planner
class WeightedAStar : public SBPLPlanner
{

public:
	int replan(double allocated_time_secs, vector<int>* solution_stateIDs_V);
	int replan(double allocated_time_sec, vector<int>* solution_stateIDs_V, int* solcost);

    int set_goal(int goal_stateID);
    int set_start(int start_stateID);
    void costs_changed(StateChangeQuery const & stateChange);
    void costs_changed();
    int force_planning_from_scratch(); 

	int set_search_mode(bool bSearchUntilFirstSolution);


	virtual double get_solution_eps() const {return pSearchStateSpace_->eps_satisfied;};
    virtual int get_n_expands() const { return searchexpands; }
  void set_finalsolution_eps(double finalsolution_eps){WeightedAStar_FINAL_EPS = finalsolution_eps;};
	virtual void set_initialsolution_eps(double initialsolution_eps) {finitial_eps = initialsolution_eps; set_finalsolution_eps(initialsolution_eps);};
  void getSearchStats(bool* solutionFound, int* numExpands, int* solutionCost, double* searchTime);

	void print_searchpath(FILE* fOut);


	//constructors & destructors
    WeightedAStar(DiscreteSpaceTimeInformation* environment, bool bforwardsearch);
    ~WeightedAStar();



private:

  DiscreteSpaceTimeInformation *environment_;
  double WeightedAStar_FINAL_EPS;

	//member variables
	double finitial_eps;
	MDPConfig* MDPCfg_;

	bool bforwardsearch; //if true, then search proceeds forward, otherwise backward

	bool bsearchuntilfirstsolution; //if true, then search until first solution only (see planner.h for search modes)

    WeightedAStarSearchStateSpace_t* pSearchStateSpace_;

	unsigned int searchexpands;
	int MaxMemoryCounter;
	clock_t TimeStarted;
	FILE *fDeb;
  bool solfound;
  double searchtime;
  int finalsolcost;

  vector<BubbleData_t> bubbles;

	//member functions
	void Initialize_searchinfo(CMDPSTATE* state, WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	CMDPSTATE* CreateState(int stateID, WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	CMDPSTATE* GetState(int stateID, WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	int ComputeHeuristic(CMDPSTATE* MDPstate, WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	//initialization of a state
	void InitializeSearchStateInfo(WeightedAStarState* state, WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	//re-initialization of a state
	void ReInitializeSearchStateInfo(WeightedAStarState* state, WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	void DeleteSearchStateData(WeightedAStarState* state);

	//used for backward search
	void UpdatePreds(WeightedAStarState* state, WeightedAStarSearchStateSpace_t* pSearchStateSpace);


	//used for forward search
	void UpdateSuccs(WeightedAStarState* state, WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	int GetGVal(int StateID, WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
	int ImprovePath(WeightedAStarSearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs);

	void BuildNewOPENList(WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	void Reevaluatefvals(WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	//creates (allocates memory) search state space
	//does not initialize search statespace
	int CreateSearchStateSpace(WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	//deallocates memory used by SearchStateSpace
	void DeleteSearchStateSpace(WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	//debugging 
	void PrintSearchState(WeightedAStarState* state, FILE* fOut);


	//reset properly search state space
	//needs to be done before deleting states
	int ResetSearchStateSpace(WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	//initialization before each search
	void ReInitializeSearchStateSpace(WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	//very first initialization
	int InitializeSearchStateSpace(WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	int SetSearchGoalState(int SearchGoalStateID, WeightedAStarSearchStateSpace_t* pSearchStateSpace);


	int SetSearchStartState(int SearchStartStateID, WeightedAStarSearchStateSpace_t* pSearchStateSpace);

	//reconstruct path functions are only relevant for forward search
	int ReconstructPath(WeightedAStarSearchStateSpace_t* pSearchStateSpace);


	void PrintSearchPath(WeightedAStarSearchStateSpace_t* pSearchStateSpace, FILE* fOut);

	int getHeurValue(WeightedAStarSearchStateSpace_t* pSearchStateSpace, int StateID);

	//get path 
	vector<int> GetSearchPath(WeightedAStarSearchStateSpace_t* pSearchStateSpace, int& solcost);


	bool Search(WeightedAStarSearchStateSpace_t* pSearchStateSpace, vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs);


};


#endif



