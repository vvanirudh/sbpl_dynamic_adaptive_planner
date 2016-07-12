/**

   Author : Anirudh Vemula
   Date : 7th March 2016

   Extends the DiscreteSpaceTimeInformation class to the adaptive setting

*/


#ifndef ADAPTIVE_DISCRETE_SPACE_TIME_INFORMATION_H
#define ADAPTIVE_DISCRETE_SPACE_TIME_INFORMATION_H

#include <sbpl/headers.h>
#include <sbpl_dynamic_adaptive_planner/sbpl_dynamicObstacles.h>
#include <sbpl_dynamic_adaptive_planner/DiscreteSpaceTimeInformation.h>
#include <sbpl_dynamic_adaptive_planner/SCVStat.h>

#include <iostream>
#include <memory>

using namespace std;

//enum AdaptiveStateType_t : unsigned char {LD=0, NearHD=1, HD=2};

class AdaptiveDiscreteSpaceTimeInformation : virtual public DiscreteSpaceTimeInformation {

 public:
  std::vector<int> modifiedStates;
  std::vector<int> invalidStates;
  
  virtual bool isHighDPath(std::vector<int> *stateIDV){
    SBPL_ERROR("isHighDPath not implemented for this environment");
    return false;
  }
  
  virtual bool isGoalReached() {
    SBPL_ERROR("isGoalReached not implemented for this environment");
    return false;
  }
  
  virtual int getFutureStateEstimate(std::vector<std::vector<double> > *traj) {
    SBPL_ERROR("getFutureStateEstimate not implemented for this environment");
    return -1;
  }
  
  virtual void resetExpandedStates() {
    SBPL_ERROR("resetExpandedStates not implemented!");
    throw SBPL_Exception();
  };
  
  virtual void expandingState(int StateID) {
    SBPL_ERROR("expandingState not implemented!");
    throw SBPL_Exception();
  };
  
  virtual bool stopTracking(int StateID) {
    return false;
  }
  
  virtual int getBestSeenState() {
    SBPL_ERROR("getBestSeenState() not implemented for this environment! Cannot construct a partial path!");
    throw SBPL_Exception();
    return -1;
  };
  
  virtual void SearchTimeoutCallback(){
    SBPL_ERROR("SearchTimeoutCallback() not implemented for this environment");
  }


  // @avemula ADDED two new functions
  virtual void growAllSpheres() {
    SBPL_ERROR("growAllSpheres is not implemented");
  }
  
  virtual bool isCollisionFree(std::vector<int> *stateIDV, std::vector<int> *loc) {
    SBPL_ERROR("isCollisionFree is not implemented");
    return false;
  }

  // @avemula ADDED a helper print function
  virtual void printStatePath(std::vector<int> *StateIDV) {
    SBPL_ERROR("printStatePath is not implemented");
  }
  
  /** \brief sets the environment in adaptive planning mode
   */
  virtual void setPlanMode() = 0;
  
  virtual void visualizeStatePath(std::vector<int> *path, int scolor, int ecolor, std::string name) {
    SBPL_ERROR("visualizeStatePath not implemented!");
  };
  
  virtual void visualizeTrajectory(std::vector<std::vector<double> > &traj, int scolor, int ecolor, std::string name) {
    SBPL_ERROR("visualizeTrajectory not implemented!");
  };
  
  /** \brief constructs a tunnel of width tunnelWidth around the path specified by stateIDs_V
      and sets the environment in tracking mode
  */
  virtual void setTrackMode(const std::vector<int> &stateIDs_V, int cost, double tunnelWidth, std::vector<int> *ModStates = NULL) =0;
  
  /** \brief adds a new sphere of radius rad at the state coordinates specified by StateID
   */
  virtual void addSphere(int StateID, double rad, std::vector<int> *modifiedStates=NULL) = 0;

  
  virtual void addSphere(int StateID, double rad, int &first_mod_step) {
    SBPL_ERROR("addSphere(int StateID, double rad, int &first_mod_step) Not implemented for this environment!");
    throw SBPL_Exception();
  }
  
  virtual void getTrackingModStates(std::vector<int>* planModStates, std::vector<int>* trkModStates){ SBPL_ERROR("getTrackingModStates not implemented for this environment!"); };

  virtual void moveStartTo(int StateID, std::vector<int> *modStates=NULL) { SBPL_ERROR("moveStartTo(int stateID) not implemented for this environment!"); };

  virtual int getTrackingFailStateID() {
    SBPL_ERROR("getTrackingFailStateID not implemented!");
    throw SBPL_Exception();
  };
  virtual void addSphereWhereTrackingFailed(double rad) {
    SBPL_ERROR("addSphereWhereTrackingFailed not implemented!");
    throw SBPL_Exception();
  };
  /** \brief destructor
   */
  ~AdaptiveDiscreteSpaceTimeInformation() {
    spheres_.clear();
  }

  virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) {
    if(trackMode){
      GetPreds_Track(TargetStateID, PredIDV, CostV);
    } else {
      GetPreds_Plan(TargetStateID, PredIDV, CostV);
    }
  };
  
  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV) {
    if(trackMode){
      GetSuccs_Track(SourceStateID, SuccIDV, CostV);
    } else {
      GetSuccs_Plan(SourceStateID, SuccIDV, CostV);
    }
  };

  /** \brief constructor - specify the high-dim size and low-dim size
   */
  AdaptiveDiscreteSpaceTimeInformation(int HDIM, int LDIM)
    {
      trackMode = false;
      StartStateID = -1;
      GoalStateID = -1;
      getPredTime = 0;
      getSuccTime = 0;
      SBPL_ERROR("Trying to use default AdaptiveDiscreteSpaceTimeInformation constructor!");
      throw new SBPL_Exception();
    }
  
  AdaptiveDiscreteSpaceTimeInformation(){
    trackMode = false;
    StartStateID = -1;
    GoalStateID = -1;
    getPredTime = 0;
    getSuccTime = 0;
  }
  
  virtual void logStats() {
    SBPL_ERROR("showStats not implemented for this environment!");
  }
  
  virtual void logStat(char* info) {
    SBPL_ERROR("logStat not implemented for this environment!");
  }
  
  virtual void visualizePath(std::vector<int>* path){
    SBPL_ERROR("visualizePath not implemented for this environment!");
  }
  
  virtual void visualizeStates(std::vector<int> *stateIDs, int lcolor, int hcolor, std::string ns){
    
  }
  
  virtual void visualizeStates(std::vector<int> *stateIDs, std::vector<int> *colorsV, std::string ns){
    
  }
  
  virtual void visualizeState(std::string name, std::vector<double> &coords, int col){
    
  }
  
  virtual void visualizeEnvironment(){
    
  }
  
  virtual bool trackedWell() {
    return true;
  }

  virtual int getExecutablePiece(std::vector<int>* path, double time_limit, std::vector<std::vector<double> > *traj, double* horizon=NULL){
    SBPL_ERROR("getExecutablePiece not implemented for this environment!");
    return 0;
  }
  
  virtual void processCostlyPath(const std::vector<int> &planning_path, const std::vector<int> &tracking_path, std::vector<int>* new_sphere_locations) {
    SBPL_ERROR("processCostlyPath not implemented for this environment!");
    throw new SBPL_Exception();
  }
  
  /** resets the environment to its original state - no spheres, etc. */
  virtual void reset() {
    SBPL_ERROR("reset not implemented!");
    throw SBPL_Exception();
  };
  
  void pause(){
    printf("Enter to continue...");
    char inp;
    do {
      inp = getchar();
    } while (inp != '\n');
  }
  
  virtual bool stateIDPathToTraj(std::vector<int> stateIDV, std::vector<std::vector<double> > *traj){
    SBPL_ERROR("stateIDPathToTraj not implemented for this environment!");
    return false;
  }
  
  virtual bool GetInterpolatedPath(const std::vector<int> &stateIDV, std::vector<int> &new_stateIDV){
    SBPL_ERROR("GetInterpolatedPath not implemented for this environment!");
    return false;
  }
  
  virtual void setRPYMode(std::vector<int> &path){
    SBPL_ERROR("setRPYMode not implemented for this environment!");
    throw new SBPL_Exception();
  }
  
  virtual void showSummary(FILE* fptr=stdout){
    SBPL_ERROR("showSummary not implemented for this environment!");
  }
  
  virtual void getPlanningData(std::vector<std::string> &names, std::vector<std::string> &values){
    SBPL_ERROR("getPlanningData not implemented for this environment!");
  }
  
  virtual void getDistancesTraveled(std::vector<int> *path, double* ee, double* eb, double* ft){
    SBPL_ERROR("getDistancesTraveled not implemented for this environment!");
	}
  
  inline void setStat(AdaptivePlannerCSVStat_c* stat){
    stat_.reset(stat);
    }
  
  int StartStateID;
  int GoalStateID;

 protected:

  /** NOTES:
      the environment is always in tracking or planning mode. use setTrackMode and setPlanMode to change the mode of the environment
      the SBPL AdaptivePlanner will use setTrackMode and setPlanMode to switch between environment modes
      getSuccs and getPreds functions should take into account the environment mode when generating successor or predecessor states for the planner
  */
  bool trackMode; // true - tracking, false - planning
  double getSuccTime; //time spent generating successors - for debugging purposes
  double getPredTime; //time spent generating predecessors - for debugging purposes
  std::vector<std::vector<double> > spheres_; //the list of spheres added to the environment,
  //each sphere is a vector of doubles of size LowDim + 1
  //LowDim components specify the sphere position coordinates
  //1 component specifies the sphere radius
  
  std::shared_ptr<AdaptivePlannerCSVStat_c> stat_;
  
  /** \brief gets successors for tracking mode -- this should be specified by the user when creating their adaptive_environment file
   */
  virtual void GetSuccs_Track(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV) {
    SBPL_ERROR("GetSuccs_Track not implemented for this environment");
    throw SBPL_Exception();
  }
  
  /** \brief gets successors for planning mode -- this should be specified by the user when creating their adaptive_environment file
   */
  virtual void GetSuccs_Plan(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV) {
    SBPL_ERROR("GetSuccs_Plan not implemented for this environment");
    throw SBPL_Exception();
  }
  
  /** \brief gets predecessors for tracking mode -- this should be specified by the user when creating their adaptive_environment file
   */
  virtual void GetPreds_Track(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) {
    SBPL_ERROR("GetPreds_Track not implemented for this environment");
    throw SBPL_Exception();
  }
  
  /** \brief gets predecessors for planning mode -- this should be specified by the user when creating their adaptive_environment file
   */
  virtual void GetPreds_Plan(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) {
    SBPL_ERROR("GetPreds_Plan not implemented for this environment");
    throw SBPL_Exception();
  }
  
};


#endif
