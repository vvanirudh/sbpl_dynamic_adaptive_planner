/**
   Author : Anirudh Vemula
   Date: 6th March 2016
   
   Taken from Venkat's code for ASIPP

   The way I understand, this implements the (x,y,theta,t) space. Extends the generic discretespaceinformation class from SBPL.
 */

#ifndef DISCRETE_SPACE_TIME_INFORMATION_H
#define DISCRETE_SPACE_TIME_INFORMATION_H

#include <sbpl/headers.h>
#include <sbpl_dynamic_adaptive_planner/sbpl_dynamicObstacles.h>

class DiscreteSpaceTimeInformation : public DiscreteSpaceInformation{
public:
  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV) = 0;
  virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<int>* stateBubbles, vector<int>* bubbleCollisions){GetSuccs(SourceStateID, SuccIDV, CostV);}
  virtual void Relax(int sourceID, int targetID){}
  virtual int getNumBubbles(){return 0;}
  virtual bool setDynamicObstacles(vector<SBPL_DynamicObstacle_t> dynObs, bool reset_states=true) = 0;
  virtual bool UpdateCost(int x, int y, unsigned char newcost) = 0;
  virtual int SetStart(double x, double y, double theta, double startTime) = 0;
  virtual int SetGoal(double x, double y, double theta) = 0;
	virtual unsigned char GetMapCost(int x, int y) = 0;
  virtual void GetCoordFromState(int stateID, int& x, int& y, int& theta, int& t) const = 0;
  virtual void ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<SBPL_4Dpt_t>* xythetaPath) = 0;
  virtual void getExpansions(vector<SBPL_4Dpt_t>* p){}
  virtual bool InitializeEnv(int width, int height,
                     /** if mapdata is NULL the grid is initialized to all freespace */
                     const unsigned char* mapdata,
                     double startx, double starty, double starttheta, double startTime,
                     double goalx, double goaly, double goaltheta,
                     double goaltol_x, double goaltol_y, double goaltol_theta,
                     const vector<sbpl_2Dpt_t> & perimeterptsV,
                     double cellsize_m, double timeResolution,
                     double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                     unsigned char obsthresh, unsigned char dynobsthresh, const char* sMotPrimFile,
                     vector<SBPL_DynamicObstacle_t> & dynObs) = 0;
};

#endif
