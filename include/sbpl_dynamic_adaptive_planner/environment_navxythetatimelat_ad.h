/**

   Author : Anirudh Vemula
   Date : 8th March 2016

   Implements the interfaces for the (x,y,theta,time) adaptive space
   PART 2 (Why 2 parts?!!)

   Implements the non-exact version i.e. 3D - 4D adaptive dimensionality
   (Check once, not sure)

 */
#include <sbpl_dynamic_adaptive_planner/navviz.h>
#include <sbpl/headers.h>
#include <sbpl_dynamic_adaptive_planner/environment_navxythetatimelat.h>
#include <sbpl_dynamic_adaptive_planner/headers.h>


#ifndef __ENVIRONMENT_NAVXYTHETATIMELATAD_H_
#define __ENVIRONMENT_NAVXYTHETATIMELATAD_H_

#define TWO_PI 6.2831853
#define ENVNAV2D_MAXDIRS 16
#define MAX_HASH_ENTRIES 100000


// @avemula NOTE set VERBOSE to false for no SBPL output
#define VERBOSE true

/*#if VERBOSE
#define SBPL_printf printf
#define SBPL_INFO ROS_INFO
#define SBPL_ERROR ROS_ERROR
#endif*/

// @avemula ADDED dummy variable to imply low-dimension or something
//#define INVALID_TIME INFINTECOST

//#define DRAW_INFLATED_GRID
//#define ENV_USE_NO_HEUR
//#define ENV_USE_EUC_HEUR
#define ADD_INFLATION 0

//max transition radial length 0.2151
#define NEARTRESH2D 0.1500
#define NEARTRESH3D 0.2200

#define INVALID_THETA 16
#define NAVXYTHETALAT_THETADIRS 16
#define NAVXYTHETATIMELAT_THETADIRS 16

#define SHOWLASTSPHEREONLY
#define START_RADIUS 1.00

// @avemula DEFUNCT is used in lookup table where the index is obtained from (x,y,theta). In our case, it should be (x,y,theta,t)
#define XYTHETA2INDEX_AD(X,Y,THETA) (THETA + X*(NAVXYTHETALAT_THETADIRS+1) + Y*EnvNAVXYTHETALATCfg.EnvWidth_c*(NAVXYTHETALAT_THETADIRS+1))

#define XYTHETATIME2INDEX_AD(X,Y,THETA,TIME) (THETA + X*NAVXYTHETATIMELAT_THETADIRS + Y*EnvNAVXYTHETATIMELATCfg.EnvWidth_c*NAVXYTHETATIMELAT_THETADIRS + T*EnvNAVXYTHETATIMELATCfg.EnvWidth_c*NAVXYTHETATIMELAT_THETADIRS*EnvNAVXYTHETATIMELATCfg.EnvHeight_c)


// multiplier for 2-d vs 3-d costs
#define PENALTYVAL 2

//penalties for manuevers
#define PENALTY_STRAIGHT 1
#define PENALTY_TURN_45DEG 1
#define PENALTY_TURN_90DEG 1
#define PENALTY_TURN_135DEG 1
#define PENALTY_TURN_180DEG 1

// @avemula NOTADDED visualization related stuff
#define VISUALIZATIONS_ENABLED
#ifdef VISUALIZATIONS_ENABLED
	#define VISUALIZE_OBSTACLES
	#define VISUALIZE_INFLATED_OBSTACLES
        #define VISUALIZE_REGIONS
	//#define VISUALIZE_SPHERES
	//#define VISUALIZE_EXPANDED_STATES
	//#define VISUALIZE_STATEPROB
	//#define VISUALIZE_HEURISTIC
        #define VISUALIZE_DYNAMIC_OBSTACLES
#endif



typedef struct {
  float x;
  float y;
  float r;
} tSphere;

class EnvironmentNAVXYTHETATIMELATAD : public EnvironmentNAVXYTHETATIMELAT_AD
{

  int maxSphereIndex;
  int INVALID_TIME;

  int overcons_vizcount;
  int undercons_vizcount;
  /* -- start -- */
  /* taken from ENVNAV2D to help compute predecessors and successors of 2D states */
  
  // @avemula DOUBT Do we need to add dt_ as well?
  int dx_[ENVNAV2D_MAXDIRS];
  int dy_[ENVNAV2D_MAXDIRS];
  int dth_[ENVNAV2D_MAXDIRS];
  //the intermediate cells through which the actions go 
  int dxintersects_[ENVNAV2D_MAXDIRS][2];
  int dyintersects_[ENVNAV2D_MAXDIRS][2];
  //distances of transitions
  int dxy_distance_mm_[ENVNAV2D_MAXDIRS];
  /* -- end -- */

  float** disttoObs_incells; 
  float** disttoNonfree_incells;

  //SBPL2DGridSearch* grid2Dsearchfromstart; //computes h-values that estimate distances from start x,y to all cells
  //SBPL2DGridSearch* grid2Dsearchfromgoal;  //computes h-values that estimate distances to goal x,y from all cells
	
  FILE* statsFile;
  ros::NodeHandle rosHandle_;

 public:
  bool FORCE3DPLANNING; // @avemula NOTE this is used as FORCE4DPLANNING
  bool trackMode; // true - tracking, false - planning
  bool ShowMoreInfo;
  unsigned int ShowPathMode;

  std::vector<tSphere> spheres;

  // @avemula REMOVED visualization stuff
  NavVisualizer* navviz;
  int expands2D;
  int expands3D;
  int expands4D;
  int expandsN3D;
  int expandsN4D;
  int expandsTrack;
  int expands2D_tot;
  int expands3D_tot;
  int expands4D_tot;
  int expandsN3D_tot;
  int expandsN4D_tot;
  int expandsTrack_tot;
  int createdHashEntries;

  //TIMERS
  // @avemula NOTCHANGED did not account for 4D expansions and timers are not exactly useful now. Can be done later.
  double get2DSuccTime;
  double get3DSuccTime_HD;
  double get3DSuccTime_LD;
  double get4DSuccTime;
  double get2DPredTime;
  double get3DPredTime_HD;
  double get3DPredTime_LD;
  double get4DPredTime;
  double getPSuccTime;
  double getPPredTime;
  double getTSuccTime;
  double pCollCheckTime;
  double tCollCheckTime;

  // @avemula DUMMY variables
  double get3DSuccTime;
  double get3DPredTime;
  
  double iterTStartTime;
  double iterPStartTime;
  double iterPlanningTime;
  double iterTrackingTime;
  double vizTime;
  double planningTime;
  double trackingTime;
  double setTrackTime;
  double TRACKING_TIMEOUT;

  double LastBestTrackedUpdate;
  int BestTrackedStateID;
  int BestTrackedIndex;
  int MaxTrackIndex;
	
  bool lastSearchTimedOut;

  unsigned char** sphereStateGrid; //0 - in sphere, 1 - near sphere, 2 - outside sphere

  float** sphereDistGrid;
  char** stateProbGrid;
  int** sphereIndexGrid;
  unsigned char** inflatedGrid;
  unsigned char** expandsGrid;
	
  unsigned int** fvalField;

  const vector<int> *plan_solutionIDs_V;
  vector<int> P_costs_V;
	
  std::vector<int> prev_ADPlanIDV;

  ~EnvironmentNAVXYTHETATIMELATAD();
  EnvironmentNAVXYTHETATIMELATAD();
  void InitializeEnvironment();
  void initStorageSpace();

  // @avemula WHAT? is this function all about?
  inline bool trackedWell(){
    //return false;
    bool ret = (BestTrackedIndex > 20);//(1.0 * BestTrackedIndex >= 0.15 * MaxTrackIndex);
    return ret;
  }

  inline int compareADPlanVs(const std::vector<int> *plan1, const std::vector<int> *plan2){
    int size = min((int) plan1->size(), (int) plan2->size());
    for(int i = 0; i < size; i++){
      if(plan1->at(i) != plan2->at(i)){
	return i;
      }
    }
    return size;
  }

  bool stopTracking(int StateID);

  inline double GetStateProb(int x, int y) {
    if (stateProbGrid[x][y] >= 0)
      return stateProbGrid[x][y] / (double) INVALID_THETA;

    double p;
    int good = 0;
    int all = INVALID_THETA;

    for (int th = 0; th < all; th++) {
      if (IsValidConfiguration(x,y,th))
	good++;
    }

    p = good / (double) all;
    p = max(p, 0.0);
    p = min(p, 1.0);

    stateProbGrid[x][y] = good;
    return p;
  }

  
  inline double GetStateProb(int StateID){
    if(trackMode) return 1.0;
    
#ifndef USESTATEPROBABILITY
    return 1.0;
#endif
		
    EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[StateID];
    int x = HashEntry->X;
    int y = HashEntry->Y;
    int th = HashEntry->Theta;
    int t = HashEntry->T;
    if(th >= INVALID_THETA) return GetStateProb(x,y);

    double p;
		
    int good = 0;
    int all = INVALID_THETA;
    for(int th = 0; th < all; th++){
      if(IsValidConfiguration(x,y,th)){
	good++;
      }
    }
    p = good / (double) all;
		
    p = max(p, 0.0);
    p = min(p, 1.0);
		
    return p;
  }

  void PlannerExpandingState(int StateID, std::vector<int> *info);

  void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
  void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETATIMELATAction_t*>* actionindV=NULL);

  // 3D expansions
  void Get3DSuccs_HD(int SourceStateID, vector<int> *SuccIDV, vector<int> *CostV, vector<EnvNAVXYTHETATIMELATAction_t*> *actionV=NULL);
  void Get3DSuccs_LD(int SourceStateID, vector<int> *SuccIDV, vector<int> *CostV, vector<EnvNAVXYTHETATIMELATAction_t*> *actionV=NULL);
  
  void Get3DPreds_HD(int SourceStateID, vector<int> *PredIDV, vector<int> *CostV);
  void Get3DPreds_LD(int SourceStateID, vector<int> *PredIDV, vector<int> *CostV);

  void Get3DSuccs_HD(int x, int y, int th, vector<vector<int> > *SuccIDV, vector<int> *CostV);
  void Get3DSuccs_LD(int x, int y, int th, vector<vector<int> > *SuccIDV, vector<int> *CostV);

  void Get3DPreds_HD(int x, int y, int th, vector<vector<int> > *PredIDV, vector<int> *CostV);
  void Get3DPreds_LD(int x, int y, int th, vector<vector<int> > *PredIDV, vector<int> *CostV);

  void GetAll3DSuccs_HD(int x, int y, int th, std::vector<std::vector<int> > *Succs, std::vector<int> *costs);
  void GetAll3DSuccs_LD(int x, int y, int th, std::vector<std::vector<int> > *Succs, std::vector<int> *costs);
  
  void GetAll3DPreds_HD(int x, int y, int th, std::vector<std::vector<int> > *Preds, std::vector<int> *costs);
  void GetAll3DPreds_LD(int x, int y, int th, std::vector<std::vector<int> > *Preds, std::vector<int> *costs);
  
  // 4D expansions
  void Get4DSuccs(int SourceStateID, vector<int> *SuccIDV, vector<int> *CostV, vector<EnvNAVXYTHETATIMELATAction_t*> *actionV = NULL);
  void Get4DPreds(int SourceStateID, vector<int> *PredIDV, vector<int> *CostV);

  void Get4DSuccs(int x, int y, int th, int t, vector<vector<int> > *SuccIDV, vector<int> *CostV);
  void Get4DPreds(int x, int y, int th, int t, vector<vector<int> > *PredIDV, vector<int> *CostV);

  void GetAll4DSuccs(int x, int y, int th, int t, std::vector<std::vector<int> > *Succs, std::vector<int> *costs);
  void GetAll4DPreds(int x, int y, int th, int t, std::vector<std::vector<int> > *Preds, std::vector<int> *costs);
  
  // 2D expansions
  void Get2DSuccs(int SourceStateID, vector<int> *SuccIDV, vector<int> *CostV, vector<EnvNAVXYTHETATIMELATAction_t*> *actionV=NULL);
  void Get2DPreds(int SourceStateID, vector<int> *PredIDV, vector<int> *CostV);
  
  void Get2DSuccs(int x, int y, vector<vector<int> > *SuccIDV, vector<int> *CostV);
  void Get2DPreds(int x, int y, vector<vector<int> > *PredIDV, vector<int> *CostV);
  

  void GetSuccs_Plan_3D(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETATIMELATAction_t*>* actionindV=NULL);
  void GetSuccs_Plan_4D(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETATIMELATAction_t*>* actionindV=NULL);
  
  void GetSuccs_Track_3D(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETATIMELATAction_t*>* actionindV=NULL);
  void GetSuccs_Track_4D(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETATIMELATAction_t*>* actionindV=NULL);

  void ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<SBPL_4Dpt_t>* xythetaPath, int *cost=NULL, vector<int> *costs=NULL);

  void ComputeHeuristicValues();
  void EnsureHeuristicsUpdated(bool bGoalHeuristics);

  void addSphere(int StateID, double r, std::vector<int> *modStates=NULL);

  // @avemula ADDED grow all spheres function
  void growAllSpheres();
  
  void getTrackingModStates_3D(std::vector<int>* planModStates, std::vector<int>* trkModStates);
  void getTrackingModStates_4D(std::vector<int>* planModStates, std::vector<int>* trkModStates);

  bool isValidTrackingState(int StateID) {
    if(!trackMode) return false;
    int x, y, th, t;
    GetCoordFromState(StateID, x, y, th, t);
    if(th >= INVALID_THETA) return false;
    if (t == INVALID_TIME) return false;
    if(lookupCoords(x,y)!=0){
      return false;
    }
    return true;
  }

  void moveStartTo(int StateID, std::vector<int> *modStates=NULL);
  int getTrackingFailStateID();

  unsigned int lookupCoords(unsigned int x, unsigned int y);
  //returns 0 for inside some sphere
  //returns 1 for near some sphere
  //returns 2 for outside and far from all spheres
	
  void GetSuccs_Track(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
  void GetSuccs_Plan(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
  void GetPreds_Track(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
  void GetPreds_Plan(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
  void logStats();
  void logStat(char* info);
  void addSphereWhereTrackingFailed(double rad);
  int getExecutablePiece(std::vector<int>* path, double time_limit, std::vector<std::vector<double> > *traj, double* horizon);
  int  getTransitionCost(int sourceID, int targetID);

  void Computedxy();
	
  void SearchTimeoutCallback();


  // @avemula ADDED visualization stuff
  void draw();
  void visualizeEnvironment();
  void visualizePath(std::vector<int> *path, std::string name);
  void visualizeState(std::string name, std::vector<double> &coords, int col);
  void visualize3DState(int x, int y, int theta, int color, std::string name);
  void visualize3DState(std::string name, double x, double y, double theta, int color);
  void visualize2DState(int x, int y, int color, std::string name);
  void visualizeState(int stateID, int color, std::string name);
  void visualizeStates(std::vector<int> *stateIDs, int lcolor, int hcolor, std::string ns);
  void visualizeStates(std::vector<int> *stateIDs, std::vector<int> *colorsV, std::string ns);
  void visualizeStatePath(std::vector<int> *path, int scolor, int ecolor, std::string name);
  void visualizeTrajectory(std::vector<SBPL_4Dpt_t> *traj, int scolor, int ecolor, std::string name);
  void visualizeTrajectory(std::vector<std::vector<double> > *traj, int scolor, int ecolor, std::string name);
  void visualizeTransition(int sStateID, int tStateID, int color, std::string name);
  void visualizeSphere(tSphere* sphere, std::string name);

  // @avemula ADDED new visualize dynamic obstacle trajectory function
  //void visualizeDynamicObstacles();
  
  
  EnvNAVXYTHETATIMELATHashEntry_t* GetHashEntry_hash(int X, int Y, int Theta, int T);
  EnvNAVXYTHETATIMELATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta, int T, int TempT);
  EnvNAVXYTHETATIMELATHashEntry_t* GetHashEntry_lookup(int X, int Y, int Theta, int T);
  EnvNAVXYTHETATIMELATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta, int T, int TempT);

  //pointers to functions
  EnvNAVXYTHETATIMELATHashEntry_t* (EnvironmentNAVXYTHETATIMELATAD::*GetHashEntry)(int X, int Y, int Theta, int T);
  EnvNAVXYTHETATIMELATHashEntry_t* (EnvironmentNAVXYTHETATIMELATAD::*CreateNewHashEntry)(int X, int Y, int Theta, int T, int TempT);

  int GetStateFromCoord(int x, int y, int theta, int t);
  bool ExistsStateFromCoord(int x, int y, int theta, int t);
  int SetGoal(double x, double y, double theta);
  int SetStart(double x, double y, double theta, double t);

  int  getNearestState(int state_id, int start_ind, std::vector<int>* path);
  void processCostlyPath(std::vector<int>* planning_path, std::vector<int>* tracking_path);

  void ScaleEnv();

  void clearHashTable();
  void resetHashTable();
	
  void resetStateTable(int* new_start, int* goal);
	
  int GetGoalHeuristic(int stateID);
  int GetStartHeuristic(int stateID);

  void resetTrackingGrid();
  void updateTrackingGrid(const vector<int> *adplanIDV);

  inline void pause(){
    SBPL_WARN("Press enter to continue...");
    while (getchar() != '\n'){
		
    }
  }

  inline void PrintState(int StateID, bool bVerbose, FILE* fOut=stdout){
    fprintf(fOut, "Print state here! %d\n", StateID);
  }

  void setTrackMode(const vector<int> &adplanIDV, int cost, double tunnel_half_width, vector<int> *ModStates = NULL);
  void setPlanMode();

  bool IsValidCell2D(int X, int Y);
  int GetPathIndexForState(int SID);
	
  void resetExpandedStates();
  void expandingState(int StateID);
  void sendStateData(int StateID, int fVal, int gVal, int hVal);
  int  getBestSeenState();

  // @avemula ADDED new function to check if the given path is collision free with respect
  // to dynamic obstacles
  bool isCollisionFree(std::vector<int> *stateIDV, std::vector<int> *loc);
  
  inline bool isHighDPath(std::vector<int> *stateIDV){
    for(unsigned int i = 0; i < stateIDV->size(); i++){
      EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[stateIDV->at(i)];
      // @avemula CHANGED highD path would have INVALID_TIME
      if(HashEntry->Theta >= INVALID_THETA){
	return false;
      }
      // @avemula DOUBT can be changed by removing tracking entirely (and replacing it with a feasibility check)
      //if(HashEntry->T == INVALID_TIME && stateIDV->at(i)!=EnvNAVXYTHETATIMELAT.goalstateid) {
      //return false;
      //}
    }
    return true;
  }
	
  void reset();
	
  std::vector<std::vector<EnvNAVXYTHETATIMELATAction_t*> > revActionsV; //actions, indexed by ending theta

  inline void ComputeRevActions() {
    revActionsV.resize(INVALID_THETA);
    for(unsigned int th = 0; th < INVALID_THETA; th++){
      revActionsV[th].clear();
    }
    for(unsigned int th = 0; th < INVALID_THETA; th++){
      for (unsigned int aind = 0; aind < EnvNAVXYTHETATIMELATCfg.actionwidth; aind++) {
	EnvNAVXYTHETATIMELATAction_t* nav3daction = &EnvNAVXYTHETATIMELATCfg.ActionsV[(unsigned int)th][aind];
	revActionsV[nav3daction->endtheta].push_back(nav3daction);
      }
    }
  }
};

#endif
