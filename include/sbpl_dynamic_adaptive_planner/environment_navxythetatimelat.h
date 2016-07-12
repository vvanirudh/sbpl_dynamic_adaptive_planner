/**

   Author : Anirudh Vemula
   Date : 7th March 2016

   Implements the interfaces for the (x,y,theta,t) adaptive space
   PART 1 (Why 2 parts?!!)
*/

#ifndef __ENVIRONMENT_NAVXYTHETATIMELAT_AD_H_
#define __ENVIRONMENT_NAVXYTHETATIMELAT_AD_H_


#include <sbpl/headers.h>
#include <sbpl_dynamic_adaptive_planner/sbpl_dynamicObstacles.h>

//#include <sbpl_adaptive/sbpl_adaptive/headers.h>

#include <sbpl_dynamic_adaptive_planner/headers.h>


#include <sbpl_dynamic_adaptive_planner/AdaptiveDiscreteSpaceTimeInformation.h>

// @avemula NOTE set VERBOSE to false to supress all SBPL output
#define VERBOSE true

/*#if VERBOSE
#define SBPL_PRINTF printf
#define SBPL_INFO ROS_INFO
#endif*/


#define NAVXYTHETATIMELAT_DXYWIDTH 8
#define NAVXYTHETATIMELAT_THETADIRS 16

// @avemula WHY?
#define ENVNAVXYTHETATIMELAT_DEFAULTOBSTHRESH 254	//see explanation of the value below
#define ENVNAVXYTHETATIMELAT_DEFAULTDYNOBSTHRESH 254	//see explanation of the value below

#define NAVXYTHETATIMELAT_DEFAULT_ACTIONWIDTH 5 //decrease, increase, same angle while moving plus decrease, increase angle while standing.

#define NAVXYTHETATIMELAT_COSTMULT_MTOMM 1000

#define CONTTIME2DISC(X, CELLSIZE) (((X) >=0)?((int) ((X)/(CELLSIZE)+0.5)):((int)((X)/(CELLSIZE))-1))
#define DISCTIME2CONT(X, CELLSIZE) ((X)*(CELLSIZE))

#define FAIL 0
#define SUCCESS_WITH_TIME 1
#define SUCCESS_NO_TIME 2

typedef struct
{
	char starttheta;
	char dX;
	char dY;
	int dT;
  char endtheta;
	unsigned int cost;
  
  //all cells within footprint of polygon for all pose on trajectory
  //TODO:Mike using a larger struct than needed here!!!!!!! replace with an xyt struct!!!!!
	vector<SBPL_4Dcell_t> intersectingcellsV;
	
  //start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
  //points along centerpoint trajectory (continuous)
	vector<SBPL_4Dpt_t> intermptV;
	
  //start at 0,0,starttheta and end at endcell in discrete domain
  //centerpoint cells along all poses in trajectory
	vector<SBPL_4Dcell_t> interm4DcellsV;
} EnvNAVXYTHETATIMELATAction_t;

typedef struct 
{
  int stateID;
  int X;
  int Y;
  int Z; //optional Z parameter (for future extension)
  char Theta;
  int T;
  float TempT;
  int iteration;
} EnvNAVXYTHETATIMELATHashEntry_t;

typedef struct
{
  int motprimID;
  unsigned char starttheta_c;
  int additionalactioncostmult;
  SBPL_4Dcell_t endcell;
  //intermptV start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
  vector<SBPL_4Dpt_t> intermptV; 
}SBPL_xythetatime_mprimitive;

typedef struct
{
  
  int startstateid;
  int goalstateid;
  
  bool bInitialized;
  
  //any additional variables
  
}EnvironmentNAVXYTHETATIMELAT_t;

//configuration parameters
typedef struct ENV_NAVXYTHETATIMELAT_CONFIG
{
  int EnvWidth_c;
  int EnvHeight_c;
  int EnvDepth_c; // optional Z parameter
  int StartX_c;
  int StartY_c;
  int StartZ_c; // optional Z parameter
  int StartTheta;
  int StartTime;
  int EndX_c;
  int EndY_c;
  int EndZ_c; // optional Z parameter
  int EndTheta;
  unsigned char** Grid2D; // must replace with Grid3D incase of Z parameter extension

  //the value at which and above which cells are obstacles in the maps sent from outside
  //the default is defined above
  unsigned char obsthresh; 

  //the value at which and above which until obsthresh (not including it) cells have the nearest obstacle at distance smaller than or equal to 
  //the inner circle of the robot. In other words, the robot is definitely colliding with the obstacle, independently of its orientation
  //if no such cost is known, then it should be set to obsthresh (if center of the robot collides with obstacle, then the whole robot collides with it
  //independently of its rotation)
  unsigned char cost_inscribed_thresh; 

  //the value at which and above which until cost_inscribed_thresh (not including it) cells 
  //**may** have a nearest osbtacle within the distance that is in between the robot inner circle and the robot outer circle
  //any cost below this value means that the robot will NOT collide with any obstacle, independently of its orientation
  //if no such cost is known, then it should be set to 0 or -1 (then no cell cost will lower than it, and therefore the robot's footprint will always be checked)
  int cost_possibly_circumscribed_thresh; //it has to be integer, because -1 means that it is not provided.

  //theshold for how close we get to a dynamic obstacle before it is called a collision
  unsigned char dynamic_obstacle_collision_cost_thresh;

  double nominalvel_mpersecs;
  double timetoturn45degsinplace_secs;
  double cellsize_m;
  double timeResolution;

  int dXY[NAVXYTHETATIMELAT_DXYWIDTH][2]; // replace with dXYZ for Z parameter extension

  EnvNAVXYTHETATIMELATAction_t** ActionsV; //array of actions, ActionsV[i][j] - jth action for sourcetheta = i
  vector<EnvNAVXYTHETATIMELATAction_t*>* PredActionsV; //PredActionsV[i] - vector of pointers to the actions that result in a state with theta = i

  int actionwidth; //number of motion primitives
  vector<SBPL_xythetatime_mprimitive> mprimV;

  vector<sbpl_2Dpt_t> FootprintPolygon; // footprint would be 3D in case of Z parameter extension
  double robotRadius;
} EnvNAVXYTHETATIMELATConfig_t;


class SBPL2DGridSearch;

class EnvironmentNAVXYTHETATIMELATTICE_AD : public AdaptiveDiscreteSpaceTimeInformation
{

 public:
  EnvironmentNAVXYTHETATIMELATTICE_AD();

  bool InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV, const char* sMotPrimFile, const char* sDynObsFile);
  bool InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV, const char* sMotPrimFile);
  bool InitializeEnv(const char* sEnvFile);

  virtual bool SetEnvParameter(const char* parameter, int value);
  virtual int GetEnvParameter(const char* parameter);
  bool InitializeMDPCfg(MDPConfig *MDPCfg);
  virtual int  GetFromToHeuristic(int FromStateID, int ToStateID) = 0;
  virtual int  GetGoalHeuristic(int stateID) = 0;
  virtual int  GetStartHeuristic(int stateID) = 0;
  virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) = 0;
  virtual void SetAllPreds(CMDPSTATE* state);
  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);

  virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) = 0;


  void PrintEnv_Config(FILE* fOut);

  // @avemula NEEDED? two extra initialize environment functions when we are just going to use the file prototype
  bool InitializeEnv(int width, int height,
		     /** if mapdata is NULL the grid is initialized to all freespace */
		     const unsigned char* mapdata,
		     double startx, double starty, double starttheta, double startTime,
		     double goalx, double goaly, double goaltheta,
		     double goaltol_x, double goaltol_y, double goaltol_theta,
		     const vector<sbpl_2Dpt_t> & perimeterptsV,
		     double cellsize_m, double timeResolution,
		     double nominalvel_mpersecs, double timetoturn45degsinplace_secs, 
		     unsigned char obsthresh, unsigned char dynobsthresh, const char* sMotPrimFile,
		     vector<SBPL_DynamicObstacle_t> & dynObs);

  bool InitializeEnv(int width, int height,
		     /** if mapdata is NULL the grid is initialized to all freespace */
		     const unsigned char* mapdata,
		     double startx, double starty, double starttheta, double startTime,
		     double goalx, double goaly, double goaltheta,
		     double goaltol_x, double goaltol_y, double goaltol_theta,
		     const vector<sbpl_2Dpt_t> & perimeterptsV,
		     double cellsize_m, double timeResolution,
		     double nominalvel_mpersecs, double timetoturn45degsinplace_secs, 
		     unsigned char obsthresh, unsigned char dynobsthresh, const char* sMotPrimFile);

  bool UpdateCost(int x, int y, unsigned char newcost);

  virtual void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV) = 0;
  virtual void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV) = 0;


  bool IsObstacle(int x, int y);
  bool IsValidConfiguration(int X, int Y, int Theta);

  int getStartID();
  int getGoalID();

  void GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double* starttheta, double* startTime, double* goalx, double* goaly, double* goaltheta,
		   double* cellsize_m, double* timeResolution, double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs, unsigned char* obsthresh,
		   unsigned char* dyn_obs_thresh, vector<SBPL_xythetatime_mprimitive>* motionprimitiveV);

  const EnvNAVXYTHETATIMELATConfig_t* GetEnvNavConfig();

  ~EnvironmentNAVXYTHETATIMELATTICE_AD();

  void PrintTimeStat(FILE* fOut);
  
  unsigned char GetMapCost(int x, int y);
  bool IsWithinMapCell(int X, int Y);

  /** Transform a pose into discretized form. The angle 'pth' is
      considered to be valid if it lies between -2pi and 2pi (some
      people will prefer 0<=pth<2pi, others -pi<pth<=pi, so this
      compromise should suit everyone).
      
      \note Even if this method returns false, you can still use the
      computed indices, for example to figure out how big your map
      should have been.
      
      \return true if the resulting indices lie within the grid bounds
      and the angle was valid.
  */
  //bool PoseContToDisc(double px, double py, double pth,
  //int &ix, int &iy, int &ith) const;
  bool PoseContToDisc(double px, double py, double pth, double pt,
		      int &ix, int &iy, int &ith, int &it) const;
  
  /** Transform grid indices into a continuous pose. The computed
      angle lies within 0<=pth<2pi.
      
      \note Even if this method returns false, you can still use the
      computed indices, for example to figure out poses that lie
      outside of your current map.
      
      \return true if all the indices are within grid bounds.
  */
  //bool PoseDiscToCont(int ix, int iy, int ith,
  //double &px, double &py, double &pth) const;
  bool PoseDiscToCont(int ix, int iy, int ith, int it,
		      double &px, double &py, double &pth, double &pt) const;

  virtual void PrintVars(){};

 protected:
  virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, int SourceTime, EnvNAVXYTHETATIMELATAction_t* action, bool isHD=true);
  // @avemula ADDED a new getactioncost function that supports 3D expansions
  virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETATIMELATAction_t* action);
  
  void InitializeTimelineMap();
  bool UpdateTimelineMap();
  void FillInBubble(int CenterX, int CenterY, int T, int rad);
  void FillInBubbleColumn(int x, int topY, int botY, int T);
  void FillInBubbleCell(int x, int y, int T);

  void intervals2Timeline(int x, int y);
  int getInterval(int x, int y, int t);
  virtual EnvNAVXYTHETATIMELATHashEntry_t* getEntryFromID(int id) = 0;
  static bool pairCompare (pair<int,int> i, pair<int,int> j) { return (i.first<j.first); };

  unsigned char getDynamicObstacleCost(SBPL_4Dcell_t cell);
  double dynObsPtSqrDist(SBPL_4Dcell_t cell, SBPL_Traj_Pt_t p);

  // member data
  EnvNAVXYTHETATIMELATConfig_t EnvNAVXYTHETATIMELATCfg;
  EnvironmentNAVXYTHETATIMELAT_t EnvNAVXYTHETATIMELAT;
  vector<SBPL_4Dcell_t> affectedsuccstatesV; //arrays of states whose outgoing actions cross cell 0,0
  vector<SBPL_4Dcell_t> affectedpredstatesV; //arrays of states whose incoming actions cross cell 0,0
  int iteration;
  vector<SBPL_DynamicObstacle_t> dynamicObstacles;
  vector< vector< vector<int> > > timelineMap;
  vector< vector< vector< pair<int,int> > > > intervalMap;
  int temporal_padding;

  //2D search for heuristic computations
  bool bNeedtoRecomputeStartHeuristics; //set whenever grid2Dsearchfromstart needs to be re-executed
  bool bNeedtoRecomputeGoalHeuristics; //set whenever grid2Dsearchfromgoal needs to be re-executed
  SBPL2DGridSearch* grid2Dsearchfromstart; //computes h-values that estimate distances from start x,y to all cells
  SBPL2DGridSearch* grid2Dsearchfromgoal;  //computes h-values that estimate distances to goal x,y from all cells

  //load dynamic obstacles
  void ReadDynamicObstacles(FILE* fDynObs);

  virtual void ReadConfiguration(FILE* fCfg);

  void InitializeEnvConfig(vector<SBPL_xythetatime_mprimitive>* motionprimitiveV);


  bool CheckQuant(FILE* fOut);

  void SetConfiguration(int width, int height,
			/** if mapdata is NULL the grid is initialized to all freespace */
			const unsigned char* mapdata,
			int startx, int starty, int starttheta, int startTime,
			int goalx, int goaly, int goaltheta,
			double cellsize_m, double timeResolution,
			double nominalvel_mpersecs, double timetoturn45degsinplace_secs, const vector<sbpl_2Dpt_t> & robot_perimeterV);
	
  bool InitGeneral( vector<SBPL_xythetatime_mprimitive>* motionprimitiveV);
  void PrecomputeActionswithBaseMotionPrimitive(vector<SBPL_xythetatime_mprimitive>* motionprimitiveV);
  void PrecomputeActionswithCompleteMotionPrimitive(vector<SBPL_xythetatime_mprimitive>* motionprimitiveV);
  void PrecomputeActions();

  void CreateStartandGoalStates();

  virtual void InitializeEnvironment() = 0;

  void ComputeHeuristicValues();

  bool IsValidCell(int X, int Y);

  void CalculateFootprintForPose(SBPL_4Dpt_t pose, vector<SBPL_4Dcell_t>* footprint);
  void RemoveSourceFootprint(SBPL_4Dpt_t sourcepose, vector<SBPL_4Dcell_t>* footprint);

  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETATIMELATAction_t*>* actionindV=NULL) = 0;

  double EuclideanDistance_m(int X1, int Y1, int X2, int Y2);

  void ComputeReplanningData();
  void ComputeReplanningDataforAction(EnvNAVXYTHETATIMELATAction_t* action);

  bool ReadMotionPrimitives(FILE* fMotPrims);
  bool ReadinMotionPrimitive(SBPL_xythetatime_mprimitive* pMotPrim, FILE* fIn);
  int ReadinCell(SBPL_4Dcell_t* cell, char* fIn);
  int ReadinPose(SBPL_4Dpt_t* pose, char* fIn);

  void PrintHeuristicValues();

};


class EnvironmentNAVXYTHETATIMELAT_AD : public EnvironmentNAVXYTHETATIMELATTICE_AD
{
  
 public:
  ~EnvironmentNAVXYTHETATIMELAT_AD();

  int SetStart(double x, double y, double theta, double startTime);
  int SetGoal(double x, double y, double theta);
  void SetGoalTolerance(double tol_x, double tol_y, double tol_theta) { /**< not used yet */ }
  bool setDynamicObstacles(vector<SBPL_DynamicObstacle_t> dynObs, bool reset_states = true);
  EnvNAVXYTHETATIMELATHashEntry_t* getEntryFromID(int id);

  //void GetCoordFromState(int stateID, int& x, int& y, int& theta) const;
  void GetCoordFromState(int stateID, int& x, int& y, int& theta, int& t) const;
  void GetCoordFromState(int stateID, int& x, int& y, int& theta, int& t, float& tempT) const;
  //int GetStateFromCoord(int x, int y, int theta);
  int GetStateFromCoord(int x, int y, int theta, int t);

  void ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<SBPL_4Dpt_t>* xythetaPath); 
  bool AreEquivalent(int StateID1, int StateID2){return StateID1 == StateID2;}
  void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);

  virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETATIMELATAction_t*>* actionindV=NULL);

  void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV);
  void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV);

  virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

  virtual int  GetFromToHeuristic(int FromStateID, int ToStateID);
  virtual int  GetGoalHeuristic(int stateID);
  virtual int  GetStartHeuristic(int stateID);

  virtual int	 SizeofCreatedEnv();

  virtual void PrintVars(){};

 protected:
  //hash table of size x_size*y_size. Maps from coords to stateId	
  int HashTableSize;
  vector<EnvNAVXYTHETATIMELATHashEntry_t*>* Coord2StateIDHashTable;
  //vector that maps from stateID to coords	
  vector<EnvNAVXYTHETATIMELATHashEntry_t*> StateID2CoordTable;

  EnvNAVXYTHETATIMELATHashEntry_t** Coord2StateIDHashTable_lookup;
  
  unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Theta, unsigned int T);

  EnvNAVXYTHETATIMELATHashEntry_t* GetHashEntry_hash(int X, int Y, int Theta, int T);
  EnvNAVXYTHETATIMELATHashEntry_t* CreateNewHashEntry_hash(int X, int Y, int Theta, int T);
  EnvNAVXYTHETATIMELATHashEntry_t* GetHashEntry_lookup(int X, int Y, int Theta, int T);
  EnvNAVXYTHETATIMELATHashEntry_t* CreateNewHashEntry_lookup(int X, int Y, int Theta, int T);

  //pointers to functions
  EnvNAVXYTHETATIMELATHashEntry_t* (EnvironmentNAVXYTHETATIMELAT_AD::*GetHashEntry)(int X, int Y, int Theta, int T);
  EnvNAVXYTHETATIMELATHashEntry_t* (EnvironmentNAVXYTHETATIMELAT_AD::*CreateNewHashEntry)(int X, int Y, int Theta, int T);


  virtual void InitializeEnvironment();

  void PrintHashTableHist(FILE* fOut);

  // @avemula CHECK how these data members need to be modified to account dynamic obstacles
  unsigned char** inflatedGrid;
  float** disttoObs_incells; 
  float** disttoNonfree_incells;

};

#endif
