/**

   Author : Anirudh Vemula
   Date : 7th March 2016

   Implements the functionality for the (x,y,theta,t) adaptive space.
   PART 1 

 */
#include <sbpl/headers.h>
#include <sbpl_dynamic_adaptive_planner/environment_navxythetatimelat.h>

#define DRAW_MAP 0
#define DEFAULT_TEMPORAL_PADDING 1

using namespace std;

static long int checks = 0;

// @avemula NOTNEEDED used in lookup table. Must index (x,y,theta,t) in our case
#define XYTHETA2INDEX(X,Y,THETA) (THETA + X*NAVXYTHETALAT_THETADIRS + Y*EnvNAVXYTHETALATCfg.EnvWidth_c*NAVXYTHETALAT_THETADIRS)

#define XYTHETATIME2INDEX(X,Y,THETA,TIME) (THETA + X*NAVXYTHETATIMELAT_THETADIRS + Y*EnvNAVXYTHETATIMELATCfg.EnvWidth_c*NAVXYTHETATIMELAT_THETADIRS + T*EnvNAVXYTHETATIMELATCfg.EnvWidth_c*NAVXYTHETATIMELAT_THETADIRS*EnvNAVXYTHETATIMELATCfg.EnvHeight_c)

EnvironmentNAVXYTHETATIMELATTICE_AD::EnvironmentNAVXYTHETATIMELATTICE_AD() {

  EnvNAVXYTHETATIMELATCfg.obsthresh = ENVNAVXYTHETATIMELAT_DEFAULTOBSTHRESH;
  EnvNAVXYTHETATIMELATCfg.cost_inscribed_thresh = EnvNAVXYTHETATIMELATCfg.obsthresh; //the value that pretty much makes it disabled
  EnvNAVXYTHETATIMELATCfg.cost_possibly_circumscribed_thresh = -1; //the value that pretty much makes it disabled

  grid2Dsearchfromstart = NULL;
  grid2Dsearchfromgoal = NULL;
  bNeedtoRecomputeStartHeuristics = true;
  bNeedtoRecomputeGoalHeuristics = true;
  iteration = 0;
  EnvNAVXYTHETATIMELATCfg.Grid2D = NULL;

  EnvNAVXYTHETATIMELAT.bInitialized = false;

  EnvNAVXYTHETATIMELATCfg.actionwidth = NAVXYTHETATIMELAT_DEFAULT_ACTIONWIDTH;

}

EnvironmentNAVXYTHETATIMELATTICE_AD::~EnvironmentNAVXYTHETATIMELATTICE_AD() {

  printf("destroying XYTHETALATTICE\n");
  if(grid2Dsearchfromstart != NULL)
    delete grid2Dsearchfromstart;
  grid2Dsearchfromstart = NULL;

  if(grid2Dsearchfromgoal != NULL)
    delete grid2Dsearchfromgoal;
  grid2Dsearchfromgoal = NULL;

  if(EnvNAVXYTHETATIMELATCfg.Grid2D != NULL)
    {	
      for (int x = 0; x < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; x++) 
	delete [] EnvNAVXYTHETATIMELATCfg.Grid2D[x];
      delete [] EnvNAVXYTHETATIMELATCfg.Grid2D;
      EnvNAVXYTHETATIMELATCfg.Grid2D = NULL;
    }

  //delete actions
  if(EnvNAVXYTHETATIMELATCfg.ActionsV != NULL)
    {
      for(int tind = 0; tind < NAVXYTHETATIMELAT_THETADIRS; tind++)
	delete [] EnvNAVXYTHETATIMELATCfg.ActionsV[tind];
      delete [] EnvNAVXYTHETATIMELATCfg.ActionsV;
      EnvNAVXYTHETATIMELATCfg.ActionsV = NULL;
    }
  if(EnvNAVXYTHETATIMELATCfg.PredActionsV != NULL)
    {
      delete [] EnvNAVXYTHETATIMELATCfg.PredActionsV;
      EnvNAVXYTHETATIMELATCfg.PredActionsV = NULL;
    }

}

static unsigned int inthash(unsigned int key)
{
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}

void EnvironmentNAVXYTHETATIMELATTICE_AD::ReadDynamicObstacles(FILE* fDynObs){
	char sTemp[1024], sTemp1[1024];
  int iTemp;

  SBPL_PRINTF("Reading Dynamic Obstacles...\n");

  //get the number of dynamic obstacles in the file
	if(fscanf(fDynObs, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "NumberOfDynamicObstacles:");
	if(strcmp(sTemp1, sTemp) != 0){
		SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		throw new SBPL_Exception();
	}
	if(fscanf(fDynObs, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
    throw new SBPL_Exception();
  }
	int numObs = atoi(sTemp);

  //for each dynamic obstacle
  for(int i=0; i < numObs; i++){

    //check that the ID matches i
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
      throw new SBPL_Exception();
    }
    strcpy(sTemp1, "DynamicObstacleID:");
    if(strcmp(sTemp1, sTemp) != 0){
      SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
      SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
      throw new SBPL_Exception();
    }
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
      throw new SBPL_Exception();
    }
    iTemp = atoi(sTemp);
    if(iTemp != i){
      SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
      SBPL_PRINTF("Expected %d got %d\n", i, iTemp);
      throw new SBPL_Exception();
    }
    SBPL_DynamicObstacle_t obs;

    //Read in the obstacle's radius
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
      throw new SBPL_Exception();
    }
    strcpy(sTemp1, "ObstacleRadius:");
    if(strcmp(sTemp1, sTemp) != 0){
      SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
      SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
      throw new SBPL_Exception();
    }
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
      throw new SBPL_Exception();
    }
    obs.radius = atof(sTemp) + EnvNAVXYTHETATIMELATCfg.robotRadius;

    //read the number of trajectories for this obstacle
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
      throw new SBPL_Exception();
    }
    strcpy(sTemp1, "NumberOfTrajectories:");
    if(strcmp(sTemp1, sTemp) != 0){
      SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
      SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
      throw new SBPL_Exception();
    }
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
      throw new SBPL_Exception();
    }
    int numTraj = atoi(sTemp);

    //for each trajectory
    double trajProbSum = 0;
    for(int j=0; j < numTraj; j++){
       
      //check that the ID matches j
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      strcpy(sTemp1, "TrajectoryID:");
      if(strcmp(sTemp1, sTemp) != 0){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      iTemp = atoi(sTemp);
      if(iTemp != j){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("Expected %d got %d\n", j, iTemp);
        throw new SBPL_Exception();
      }
      SBPL_Trajectory_t traj;

      //read in this trajectory's probability
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      strcpy(sTemp1, "TrajectoryProbability:");
      if(strcmp(sTemp1, sTemp) != 0){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      traj.prob = atof(sTemp);
      if(traj.prob < 0 || traj.prob > 1){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("Expected TrajectoryProbability on the interval [0,1] but got %f\n", traj.prob);
        throw new SBPL_Exception();
      }
      trajProbSum += traj.prob;

      //read the number of intermediate points are given for the trajectory
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      strcpy(sTemp1, "NumberOfPoints:");
      if(strcmp(sTemp1, sTemp) != 0){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      int numPoints = atoi(sTemp);

      //for each point
      int prev_t = 0;
      for(int k=0; k < numPoints; k++){
        //fill in the point
        SBPL_Traj_Pt_t pt;
        if(fscanf(fDynObs, "%s", sTemp) != 1){
          SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
          throw new SBPL_Exception();
        }
        pt.x = atof(sTemp);
        if(fscanf(fDynObs, "%s", sTemp) != 1){
          SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
          throw new SBPL_Exception();
        }
        pt.y = atof(sTemp);
        if(fscanf(fDynObs, "%s", sTemp) != 1){
          SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
          throw new SBPL_Exception();
        }
        pt.t = CONTTIME2DISC(atof(sTemp),EnvNAVXYTHETATIMELATCfg.timeResolution);

        if(prev_t > pt.t && k != 0){
          SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
          SBPL_PRINTF("dynamic obstacle trajectory times can't decrease!\n");
          throw new SBPL_Exception();
        }
        prev_t = pt.t;

        if(fscanf(fDynObs, "%s", sTemp) != 1){
          SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
          throw new SBPL_Exception();
        }
        pt.std_dev = atof(sTemp);

        //store the point in the trajectory
        traj.points.push_back(pt);
      }

      //check if the obstacle should "disappear" after it has finished its trajectory
      //or if it sits in the configuration from the last frame of the trajectory forever
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      strcpy(sTemp1, "ObstacleExistsAfterTrajectory:");
      if(strcmp(sTemp1, sTemp) != 0){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        SBPL_ERROR("ERROR: ran out of dynamic obstacle file early\n");
        throw new SBPL_Exception();
      }
      traj.existsAfter = atoi(sTemp);
      if(traj.existsAfter != 0 && traj.existsAfter != 1){
        SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
        SBPL_PRINTF("ObstacleExistsAfterTrajectory is a boolean and needs to be 0 or 1\n");
        throw new SBPL_Exception();
      }

      //store the trajectory in the dynamic obstacle
      obs.trajectories.push_back(traj);
    }

    //check that the trajectory probabilities sum to 1
    if(fabs(trajProbSum - 1.0) > ERR_EPS){
      SBPL_ERROR("ERROR: dynamic obstacle file has incorrect format\n");
      SBPL_PRINTF("Probabilities for trajectories of dynamic obstacle %d sum to %f instead of 1\n", i, trajProbSum);
      throw new SBPL_Exception();
    }

    //store the dynamic obstacle into the dynamic obstacle vector
    dynamicObstacles.push_back(obs);

  }

  SBPL_PRINTF("Done Reading Dynamic Obstacles\n");
}


void EnvironmentNAVXYTHETATIMELATTICE_AD::InitializeTimelineMap(){
  SBPL_PRINTF("Initializing interval and timeline maps\n");
  //create a clean intervalMap for dynamic obstacles
  intervalMap.resize(EnvNAVXYTHETATIMELATCfg.EnvWidth_c);
  timelineMap.resize(EnvNAVXYTHETATIMELATCfg.EnvWidth_c);
  for (int x = 0; x < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; x++){
    intervalMap[x].resize(EnvNAVXYTHETATIMELATCfg.EnvHeight_c);
    timelineMap[x].resize(EnvNAVXYTHETATIMELATCfg.EnvHeight_c);
  }

  UpdateTimelineMap();
}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::UpdateTimelineMap(){
  SBPL_PRINTF("clearing interval map\n");
  //clear the bubble map
  for(unsigned int x=0; x<intervalMap.size(); x++){
    for(unsigned int y=0; y<intervalMap[x].size(); y++){
      intervalMap[x][y].clear();
      timelineMap[x][y].clear();
    }
  }

  SBPL_PRINTF("filling in new interval map\n");
  //fill in new interval map
  for(unsigned int i=0; i<dynamicObstacles.size(); i++){
    for(unsigned int j=0; j<dynamicObstacles[i].trajectories.size(); j++){
      //TODO:Mike handle trajectory probabilities for now I'll assume all probabilities are 1
      for(unsigned int k=0; k<dynamicObstacles[i].trajectories[j].points.size(); k++){
        SBPL_Traj_Pt_t p = dynamicObstacles[i].trajectories[j].points[k];
        //TODO:The +1 on the next line is a hack, to make sure all points on the continuous circle are contained in the discrete circle.
        //This makes sure we will not be in collision with a dynamic obstacle but it is too conservative and the padding will be to big
        //on some parts of the circle.
        int bubble_cell_rad = (int)ceil((dynamicObstacles[i].radius + p.std_dev*3)/EnvNAVXYTHETATIMELATCfg.cellsize_m)+1;
        FillInBubble(CONTXY2DISC(p.x, EnvNAVXYTHETATIMELATCfg.cellsize_m), CONTXY2DISC(p.y, EnvNAVXYTHETATIMELATCfg.cellsize_m), p.t, bubble_cell_rad);
      }
    }
  }

  SBPL_PRINTF("creating timeline map\n");
  for(unsigned int x=0; x<timelineMap.size(); x++){
    for(unsigned int y=0; y<timelineMap[x].size(); y++){
      intervals2Timeline(x,y);
    }
  }

#if DRAW_MAP  
  SBPL_PRINTF("intervalMap\n");
  //draw bubble map
	for(int y = 0; y < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; y++){
		for(int x = 0; x < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; x++){
      SBPL_PRINTF("(");
      for(unsigned int i=0; i<intervalMap[x][y].size(); i++)
        SBPL_PRINTF("%d %d,",intervalMap[x][y][i].first,intervalMap[x][y][i].second);
      SBPL_PRINTF(") ");
		}
    SBPL_PRINTF("\n");
  }
  SBPL_PRINTF("timelineMap\n");
  //draw bubble map
	for(int y = 0; y < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; y++){
		for(int x = 0; x < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; x++){
      SBPL_PRINTF("(");
      for(unsigned int i=0; i<timelineMap[x][y].size(); i++)
        SBPL_PRINTF("%d,",timelineMap[x][y][i]);
      SBPL_PRINTF(") ");
		}
    SBPL_PRINTF("\n");
  }
#endif
  
  SBPL_PRINTF("done updating timeline map\n");
  //envIntervalLatHashEntry_t* HashEntry = StateID2CoordTable[envIntervalLat.startstateid];

	if(EnvNAVXYTHETATIMELAT.bInitialized){
    EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = getEntryFromID(EnvNAVXYTHETATIMELAT.startstateid);
    int interval = getInterval(HashEntry->X, HashEntry->Y, HashEntry->T);
    return interval >= 0;
  }
  return true;
}

//Bresenham Circle Algorithm for finding edge of the circle so I can mark all cells inside
void EnvironmentNAVXYTHETATIMELATTICE_AD::FillInBubble(int CenterX, int CenterY, int T, int rad){
  //SBPL_PRINTF("begin circle %d at (%d,%d,%d)  with radius %d\n",ID,CenterX,CenterY,T,rad);
  int d = 3 - 2*rad;
  int x = 0;
  int y = rad;

  while(x <= y){
    //mark pixels in 4 columns
    FillInBubbleColumn(CenterX-y, CenterY+x, CenterY-x, T);
    FillInBubbleColumn(CenterX-x, CenterY+y, CenterY-y, T);
    FillInBubbleColumn(CenterX+x, CenterY+y, CenterY-y, T);
    FillInBubbleColumn(CenterX+y, CenterY+x, CenterY-x, T);
    if(d <= 0)
      d += 4*x + 6;
    else{
      d += 4*(x-y) + 10;
      y--;
    }
    x++;
  }
  //SBPL_PRINTF("end circle\n");
}

void EnvironmentNAVXYTHETATIMELATTICE_AD::FillInBubbleColumn(int x, int topY, int botY, int T){
  //SBPL_PRINTF("begin column %d from %d to %d\n", x, topY, botY);
  //return if column is outside map
  if(x < 0 || x >= (int)intervalMap.size())
    return;
  //snap topY and botY to edge of map if they are off of it
  if(topY >= (int)intervalMap[x].size())
    topY = intervalMap[x].size()-1;
  if(botY < 0)
    botY = 0;
  //return if column is outside map
  if(topY < 0 || botY >= (int)intervalMap[x].size())
    return;

  //SBPL_PRINTF("fill in top\n");
  //fill in top of column
  FillInBubbleCell(x, topY, T);
  
  //SBPL_PRINTF("fill in bottom\n");
  //fill in bottom of column
  FillInBubbleCell(x, botY, T);

  //SBPL_PRINTF("check if there is more column\n");
  //check if the rest of this column has already been filled in and if so we are done

  if(topY-botY <= 1)
    return;

  //otherwise fill in the column
  for(int y = topY-1; y>botY; y--){
    FillInBubbleCell(x, y, T);
  }

  //SBPL_PRINTF("end column\n");
}

void EnvironmentNAVXYTHETATIMELATTICE_AD::FillInBubbleCell(int x, int y, int T){
  int minT = T - temporal_padding;
  int maxT = T + temporal_padding;
  intervalMap[x][y].push_back(pair<int,int>(minT,maxT));
}

void EnvironmentNAVXYTHETATIMELATTICE_AD::intervals2Timeline(int x, int y){
  if(intervalMap[x][y].empty())
    return;
  sort(intervalMap[x][y].begin(), intervalMap[x][y].end(), pairCompare);
  timelineMap[x][y].push_back(intervalMap[x][y][0].first);
  timelineMap[x][y].push_back(intervalMap[x][y][0].second + 1);
  for(unsigned int i=1; i<intervalMap[x][y].size(); i++){
    if(intervalMap[x][y][i].first <= timelineMap[x][y].back() && intervalMap[x][y][i].second >= timelineMap[x][y].back())
      timelineMap[x][y].back() = intervalMap[x][y][i].second + 1;
    else if(intervalMap[x][y][i].first > timelineMap[x][y].back()){
      timelineMap[x][y].push_back(intervalMap[x][y][i].first);
      timelineMap[x][y].push_back(intervalMap[x][y][i].second + 1);
    }
  }
}

int EnvironmentNAVXYTHETATIMELATTICE_AD::getInterval(int x, int y, int t){
  int i = distance(timelineMap[x][y].begin(), upper_bound(timelineMap[x][y].begin(), timelineMap[x][y].end(), t));
  if(i%2)
    return -1;
  return i/2;
}

void EnvironmentNAVXYTHETATIMELATTICE_AD::SetConfiguration(int width, int height,
					const unsigned char* mapdata,
					int startx, int starty, int starttheta, int startTime,
					int goalx, int goaly, int goaltheta,
					double cellsize_m, double timeResolution, double nominalvel_mpersecs, double timetoturn45degsinplace_secs, 
          const vector<sbpl_2Dpt_t> & robot_perimeterV) {
  EnvNAVXYTHETATIMELATCfg.EnvWidth_c = width;
  EnvNAVXYTHETATIMELATCfg.EnvHeight_c = height;
  EnvNAVXYTHETATIMELATCfg.StartX_c = startx;
  EnvNAVXYTHETATIMELATCfg.StartY_c = starty;
  EnvNAVXYTHETATIMELATCfg.StartTheta = starttheta;
  EnvNAVXYTHETATIMELATCfg.StartTime = startTime;
 
  if(EnvNAVXYTHETATIMELATCfg.StartX_c < 0 || EnvNAVXYTHETATIMELATCfg.StartX_c >= EnvNAVXYTHETATIMELATCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }
  if(EnvNAVXYTHETATIMELATCfg.StartY_c < 0 || EnvNAVXYTHETATIMELATCfg.StartY_c >= EnvNAVXYTHETATIMELATCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal start coordinates\n");
    throw new SBPL_Exception();
  }
  if(EnvNAVXYTHETATIMELATCfg.StartTheta < 0 || EnvNAVXYTHETATIMELATCfg.StartTheta >= NAVXYTHETATIMELAT_THETADIRS) {
    SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
    throw new SBPL_Exception();
  }
  
  EnvNAVXYTHETATIMELATCfg.EndX_c = goalx;
  EnvNAVXYTHETATIMELATCfg.EndY_c = goaly;
  EnvNAVXYTHETATIMELATCfg.EndTheta = goaltheta;

  if(EnvNAVXYTHETATIMELATCfg.EndX_c < 0 || EnvNAVXYTHETATIMELATCfg.EndX_c >= EnvNAVXYTHETATIMELATCfg.EnvWidth_c) {
    SBPL_ERROR("ERROR: illegal goal coordinates\n");
    throw new SBPL_Exception();
  }
  if(EnvNAVXYTHETATIMELATCfg.EndY_c < 0 || EnvNAVXYTHETATIMELATCfg.EndY_c >= EnvNAVXYTHETATIMELATCfg.EnvHeight_c) {
    SBPL_ERROR("ERROR: illegal goal coordinates\n");
    throw new SBPL_Exception();
  }
  if(EnvNAVXYTHETATIMELATCfg.EndTheta < 0 || EnvNAVXYTHETATIMELATCfg.EndTheta >= NAVXYTHETATIMELAT_THETADIRS) {
    SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
    throw new SBPL_Exception();
  }

  EnvNAVXYTHETATIMELATCfg.FootprintPolygon = robot_perimeterV;

  EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs = nominalvel_mpersecs;
  EnvNAVXYTHETATIMELATCfg.cellsize_m = cellsize_m;
  EnvNAVXYTHETATIMELATCfg.timeResolution = timeResolution;
  EnvNAVXYTHETATIMELATCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;

  // unallocate the 2D environment
  /*if(EnvNAVXYTHETATIMELATCfg.Grid2D != NULL) {
    for (int x=0; x < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; x++) {
      delete[] EnvNAVXYTHETATIMELATCfg.Grid2D[x];
    }
    delete[] EnvNAVXYTHETATIMELATCfg.Grid2D;
    EnvNAVXYTHETATIMELATCfg.Grid2D = NULL;
    }*/
  

  //allocate the 2D environment
  EnvNAVXYTHETATIMELATCfg.Grid2D = new unsigned char* [EnvNAVXYTHETATIMELATCfg.EnvWidth_c];
  for (int x = 0; x < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; x++) {
    EnvNAVXYTHETATIMELATCfg.Grid2D[x] = new unsigned char [EnvNAVXYTHETATIMELATCfg.EnvHeight_c];
  }
  
  //environment:
  if (0 == mapdata) {
    for (int y = 0; y < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; x++) {
	EnvNAVXYTHETATIMELATCfg.Grid2D[x][y] = 0;
      }
    }
  }
  else {
    for (int y = 0; y < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; x++) {
			EnvNAVXYTHETATIMELATCfg.Grid2D[x][y] = mapdata[x+y*width];
      }
    }
  }
}

void EnvironmentNAVXYTHETATIMELATTICE_AD::ReadConfiguration(FILE* fCfg)
{
	//read in the configuration of environment and initialize  EnvNAVXYTHETATIMELATCfg structure
	char sTemp[1024], sTemp1[1024];
	int dTemp;
	int x, y;

	//discretization(cells)
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "discretization(cells):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.EnvWidth_c = atoi(sTemp);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.EnvHeight_c = atoi(sTemp);

	// @avemula ADDED extra fscanf to account for the Z variable
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	
	//obsthresh: 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "obsthresh:");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.obsthresh = atoi(sTemp);
	SBPL_PRINTF("obsthresh = %d\n", EnvNAVXYTHETATIMELATCfg.obsthresh);

	//cost_inscribed_thresh: 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "cost_inscribed_thresh:");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.cost_inscribed_thresh = atoi(sTemp);
	SBPL_PRINTF("cost_inscribed_thresh = %d\n", EnvNAVXYTHETATIMELATCfg.cost_inscribed_thresh);


	//cost_possibly_circumscribed_thresh: 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "cost_possibly_circumscribed_thresh:");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.cost_possibly_circumscribed_thresh = atoi(sTemp);
	SBPL_PRINTF("cost_possibly_circumscribed_thresh = %d\n", EnvNAVXYTHETATIMELATCfg.cost_possibly_circumscribed_thresh);

	//dynamic_obstacle_collision_cost_thresh: 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "dynamic_obstacle_collision_cost_thresh:");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.dynamic_obstacle_collision_cost_thresh = atoi(sTemp);
	SBPL_PRINTF("dynamic_obstacle_collision_cost_thresh = %d\n", EnvNAVXYTHETATIMELATCfg.dynamic_obstacle_collision_cost_thresh);
	
	//cellsize
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "cellsize(meters):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.cellsize_m = atof(sTemp);
	
	//timeResolution
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "timeResolution(seconds):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.timeResolution = atof(sTemp);

  //temporal_padding
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "temporal_padding(seconds):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	temporal_padding = (int)ceil(atof(sTemp)/EnvNAVXYTHETATIMELATCfg.timeResolution);
	SBPL_PRINTF("temporal_padding = %d\n", temporal_padding);

	//speeds
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "nominalvel(mpersecs):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs = atof(sTemp);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	strcpy(sTemp1, "timetoturn45degsinplace(secs):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		SBPL_ERROR("ERROR: configuration file has incorrect format\n");
		SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
		throw new SBPL_Exception();
	}
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.timetoturn45degsinplace_secs = atof(sTemp);


	//start(meters,rads): 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.StartX_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETATIMELATCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.StartY_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETATIMELATCfg.cellsize_m);
	
	// @avemula ADDED extra fscanf to account for the extra Z variable
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	
	EnvNAVXYTHETATIMELATCfg.StartTheta = ContTheta2Disc(atof(sTemp), NAVXYTHETATIMELAT_THETADIRS);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.StartTime = CONTTIME2DISC(atof(sTemp),EnvNAVXYTHETATIMELATCfg.timeResolution);


	if(EnvNAVXYTHETATIMELATCfg.StartX_c < 0 || EnvNAVXYTHETATIMELATCfg.StartX_c >= EnvNAVXYTHETATIMELATCfg.EnvWidth_c)
	{
		SBPL_ERROR("ERROR: illegal start coordinates\n");
		throw new SBPL_Exception();
	}
	if(EnvNAVXYTHETATIMELATCfg.StartY_c < 0 || EnvNAVXYTHETATIMELATCfg.StartY_c >= EnvNAVXYTHETATIMELATCfg.EnvHeight_c)
	{
		SBPL_ERROR("ERROR: illegal start coordinates\n");
		throw new SBPL_Exception();
	}
	if(EnvNAVXYTHETATIMELATCfg.StartTheta < 0 || EnvNAVXYTHETATIMELATCfg.StartTheta >= NAVXYTHETATIMELAT_THETADIRS) {
		SBPL_ERROR("ERROR: illegal start coordinates for theta\n");
		throw new SBPL_Exception();
	}

	//end(meters,rads): 
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.EndX_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETATIMELATCfg.cellsize_m);
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.EndY_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETATIMELATCfg.cellsize_m);

	// @avemula ADDED an extra fscanf to account for Z variable
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
  }
	EnvNAVXYTHETATIMELATCfg.EndTheta = ContTheta2Disc(atof(sTemp), NAVXYTHETATIMELAT_THETADIRS);;

	if(EnvNAVXYTHETATIMELATCfg.EndX_c < 0 || EnvNAVXYTHETATIMELATCfg.EndX_c >= EnvNAVXYTHETATIMELATCfg.EnvWidth_c)
	{
		SBPL_ERROR("ERROR: illegal end coordinates\n");
		throw new SBPL_Exception();
	}
	if(EnvNAVXYTHETATIMELATCfg.EndY_c < 0 || EnvNAVXYTHETATIMELATCfg.EndY_c >= EnvNAVXYTHETATIMELATCfg.EnvHeight_c)
	{
		SBPL_ERROR("ERROR: illegal end coordinates\n");
		throw new SBPL_Exception();
	}
	if(EnvNAVXYTHETATIMELATCfg.EndTheta < 0 || EnvNAVXYTHETATIMELATCfg.EndTheta >= NAVXYTHETATIMELAT_THETADIRS) {
		SBPL_ERROR("ERROR: illegal goal coordinates for theta\n");
		throw new SBPL_Exception();
	}

	//unallocate the 2D environment
	if(EnvNAVXYTHETATIMELATCfg.Grid2D != NULL) {
	  for (int x=0; x < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; x++) {
	    delete[] EnvNAVXYTHETATIMELATCfg.Grid2D[x];
	  }
	  delete[] EnvNAVXYTHETATIMELATCfg.Grid2D;
	  EnvNAVXYTHETATIMELATCfg.Grid2D = NULL;
	}
	
	//allocate the 2D environment
	if(fscanf(fCfg, "%s", sTemp) != 1){
    SBPL_ERROR("ERROR: ran out of env file early\n");
    throw new SBPL_Exception();
	}

	//printf("The pointer value should be null %p\n", (void*)EnvNAVXYTHETATIMELATCfg.Grid2D);
	
	EnvNAVXYTHETATIMELATCfg.Grid2D = new unsigned char* [EnvNAVXYTHETATIMELATCfg.EnvWidth_c];
	for (x = 0; x < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; x++)
	{
		EnvNAVXYTHETATIMELATCfg.Grid2D[x] = new unsigned char [EnvNAVXYTHETATIMELATCfg.EnvHeight_c];
	}

	//environment:
	for (y = 0; y < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; y++){
		for (x = 0; x < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; x++)
		{
			if(fscanf(fCfg, "%d", &dTemp) != 1)
			{
				SBPL_ERROR("ERROR: incorrect format of config file (%d,%d)\n",y,x);
				throw new SBPL_Exception();
			}
			EnvNAVXYTHETATIMELATCfg.Grid2D[x][y] = dTemp;
      //SBPL_PRINTF("%d ",dTemp);
		}
  //SBPL_PRINTF("\n");
  }

}

int EnvironmentNAVXYTHETATIMELATTICE_AD::ReadinCell(SBPL_4Dcell_t* cell, char* fIn)
{
  char* temp;
  char* sptr;
  
  if((temp = strtok_r(fIn, " ", &sptr)) == NULL)
    return FAIL;
  cell->x = atoi(temp);
  
  if((temp = strtok_r(NULL, " ", &sptr)) == NULL)
    return FAIL;
  cell->y = atoi(temp);
  
  if((temp = strtok_r(NULL, " ", &sptr)) == NULL)
    return FAIL;
  cell->theta = atoi(temp);
  //normalize the angle
  cell->theta = NORMALIZEDISCTHETA(cell->theta, NAVXYTHETATIMELAT_THETADIRS);
  
  //This is an optional param
  if((temp = strtok_r(NULL, " ", &sptr)) != NULL){
    cell->t = atoi(temp);
    return SUCCESS_WITH_TIME;
  }

  return SUCCESS_NO_TIME;
}

int EnvironmentNAVXYTHETATIMELATTICE_AD::ReadinPose(SBPL_4Dpt_t* pose, char* fIn)
{
  char* temp;
  char* sptr;

	if((temp = strtok_r(fIn, " ", &sptr)) == NULL)
	   return FAIL;
	pose->x = atof(temp);

	if((temp = strtok_r(NULL, " ", &sptr)) == NULL)
	   return FAIL;
	pose->y = atof(temp);

	if((temp = strtok_r(NULL, " ", &sptr)) == NULL)
	   return FAIL;
	pose->theta = atof(temp);
	pose->theta = normalizeAngle(pose->theta);

  //This is an optional param
	if((temp = strtok_r(NULL, " ", &sptr)) != NULL){
    pose->t = atof(temp);
    return SUCCESS_WITH_TIME;
  }

	return SUCCESS_NO_TIME;
}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::ReadinMotionPrimitive(SBPL_xythetatime_mprimitive* pMotPrim, FILE* fIn)
{
  int bufSize = 1024;
  char sTemp[bufSize];
	int dTemp;
  char sExpected[bufSize];
  int numofIntermPoses;
  bool timeGiven = true;
  int timeGivenTemp;

  //read in actionID
  strcpy(sExpected, "primID:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fIn, "%d", &pMotPrim->motprimID) != 1)
    return false;

  //read in start angle
  strcpy(sExpected, "startangle_c:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fIn, "%d", &dTemp) == 0)
  {
    SBPL_ERROR("ERROR reading startangle\n");
    return false;	
  }
  pMotPrim->starttheta_c = dTemp;
   
  //read in end pose
  strcpy(sExpected, "endpose_c:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }

  if(fgets(sTemp, bufSize, fIn) == NULL)
    return false;
  timeGivenTemp = ReadinCell(&pMotPrim->endcell, sTemp);
  if(timeGivenTemp == FAIL){
    SBPL_ERROR("ERROR: failed to read in endsearchpose\n");
    return false;
  }
  else if(timeGivenTemp == SUCCESS_NO_TIME)
    timeGiven = false;
  
 
  //read in action cost
  strcpy(sExpected, "additionalactioncostmult:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fIn, "%d", &dTemp) != 1)
    return false;
	pMotPrim->additionalactioncostmult = dTemp;
    
  //read in intermediate poses
  strcpy(sExpected, "intermediateposes:");
  if(fscanf(fIn, "%s", sTemp) == 0)
    return false;
  if(strcmp(sTemp, sExpected) != 0){
    SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
    return false;
  }
  if(fscanf(fIn, "%d", &numofIntermPoses) != 1)
    return false;

	//all intermposes should be with respect to 0,0 as starting pose since it will be added later and should be done 
	//after the action is rotated by initial orientation
  if(fgets(sTemp, bufSize, fIn) == NULL)//this gets rid of the new line character from the previous line
    return false;
  for(int i = 0; i < numofIntermPoses; i++){
    SBPL_4Dpt_t intermpose;
    if(fgets(sTemp, bufSize, fIn) == NULL)
      return false;
    timeGivenTemp = ReadinPose(&intermpose, sTemp);
    if(timeGivenTemp == FAIL){
      SBPL_ERROR("ERROR: failed to read in intermediate pose %d\n",i);
      return false;
    }
    else if(timeGivenTemp == SUCCESS_NO_TIME)
      timeGiven = false;
    pMotPrim->intermptV.push_back(intermpose);
  }

  //compute time intervals if they were not given
  // @avemula ASK
  if(!timeGiven){
    double x = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x;
    double y = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y;
    double theta = fabs(computeMinUnsignedAngleDiff(pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta,
                                                    pMotPrim->intermptV[0].theta));
    double timeFromTranslation = sqrt(x*x + y*y)/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs;
    // @avemula REPLACED time 
    //double timeFromRotation = theta/(PI_CONST/4.0)*EnvNAVXYTHETATIMELATCfg.timetoturn45degsinplace_secs;
    //double t = __max(timeFromTranslation, timeFromRotation);
    double t = timeFromTranslation;
    if(t == 0)
      t = EnvNAVXYTHETATIMELATCfg.timeResolution;

    pMotPrim->endcell.t = (int)ceil(t/EnvNAVXYTHETATIMELATCfg.timeResolution);
    t = pMotPrim->endcell.t*EnvNAVXYTHETATIMELATCfg.timeResolution;
    double step = t/(pMotPrim->intermptV.size()-1);
    double temp = 0;
    for(unsigned int i = 0; i < pMotPrim->intermptV.size(); i++){
      pMotPrim->intermptV[i].t = temp;
      temp += step;
    }
  }
  //TODO:Mike if times were given at least make sure they are increasing...

	//check that the last pose corresponds correctly to the last pose
	SBPL_4Dpt_t sourcepose;
	sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	sourcepose.theta = DiscTheta2Cont(pMotPrim->starttheta_c, NAVXYTHETATIMELAT_THETADIRS);
  sourcepose.t = 0;
	double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x;
	double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y;
	double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta;				
	double mp_endt = pMotPrim->intermptV[pMotPrim->intermptV.size()-1].t;				
	int endx_c = CONTXY2DISC(mp_endx_m, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	int endy_c = CONTXY2DISC(mp_endy_m, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	int endtheta_c = ContTheta2Disc(mp_endtheta_rad, NAVXYTHETATIMELAT_THETADIRS);
	int endt_c = CONTTIME2DISC(mp_endt, EnvNAVXYTHETATIMELATCfg.timeResolution);
	if(endx_c != pMotPrim->endcell.x || endy_c != pMotPrim->endcell.y || endtheta_c != pMotPrim->endcell.theta || endt_c != pMotPrim->endcell.t)
	{	
		SBPL_ERROR("ERROR: incorrect primitive %d with startangle=%d last interm point %f %f %f %f does not match end pose %d %d %d %d\n", 
    pMotPrim->motprimID, pMotPrim->starttheta_c,
    pMotPrim->intermptV[pMotPrim->intermptV.size()-1].x, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].y, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].theta, pMotPrim->intermptV[pMotPrim->intermptV.size()-1].t,
    pMotPrim->endcell.x, pMotPrim->endcell.y,pMotPrim->endcell.theta,pMotPrim->endcell.t);	
    return false;
	}
  
  return true;
}


bool EnvironmentNAVXYTHETATIMELATTICE_AD::ReadMotionPrimitives(FILE* fMotPrims)
{
    char sTemp[1024], sExpected[1024];
    float fTemp;
    int dTemp;
    int totalNumofActions = 0;

    SBPL_PRINTF("Reading in motion primitives...");
    
    //read in the resolution
    strcpy(sExpected, "resolution_m:");
    if(fscanf(fMotPrims, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if(fscanf(fMotPrims, "%f", &fTemp) == 0)
        return false;
    printf("Environment cellsize is %f and motprim cellsize is %f\n", EnvNAVXYTHETATIMELATCfg.cellsize_m, fTemp); fflush(stdout);
    if(fabs(fTemp-EnvNAVXYTHETATIMELATCfg.cellsize_m) > ERR_EPS){
        SBPL_ERROR("ERROR: invalid grid resolution %f (instead of %f) in the dynamics file\n", 
               fTemp, EnvNAVXYTHETATIMELATCfg.cellsize_m);
        return false;
    }

    //read in time resolution
    strcpy(sExpected, "timeResolution:");
    if(fscanf(fMotPrims, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if(fscanf(fMotPrims, "%f", &fTemp) == 0)
        return false;
    if(fabs(fTemp-EnvNAVXYTHETATIMELATCfg.timeResolution) > ERR_EPS){
        SBPL_ERROR("ERROR: invalid time resolution %f (instead of %f) in the dynamics file\n", 
               fTemp, EnvNAVXYTHETATIMELATCfg.timeResolution);
        return false;
    }

    //read in the angular resolution
    strcpy(sExpected, "numberofangles:");
    if(fscanf(fMotPrims, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if(fscanf(fMotPrims, "%d", &dTemp) == 0)
        return false;
    if(dTemp != NAVXYTHETATIMELAT_THETADIRS){
        SBPL_ERROR("ERROR: invalid angular resolution %d angles (instead of %d angles) in the motion primitives file\n", 
               dTemp, NAVXYTHETATIMELAT_THETADIRS);
        return false;
    }


    //read in the total number of actions
    strcpy(sExpected, "totalnumberofprimitives:");
    if(fscanf(fMotPrims, "%s", sTemp) == 0)
        return false;
    if(strcmp(sTemp, sExpected) != 0){
        SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
        return false;
    }
    if(fscanf(fMotPrims, "%d", &totalNumofActions) == 0){
        return false;
    }

    for(int i = 0; i < totalNumofActions; i++){
		SBPL_xythetatime_mprimitive motprim;

		if(EnvironmentNAVXYTHETATIMELATTICE_AD::ReadinMotionPrimitive(&motprim, fMotPrims) == false)
			return false;

		EnvNAVXYTHETATIMELATCfg.mprimV.push_back(motprim);

	}
    SBPL_PRINTF("done ");

    return true;
}

void EnvironmentNAVXYTHETATIMELATTICE_AD::ComputeReplanningDataforAction(EnvNAVXYTHETATIMELATAction_t* action)
{
	int j;

	//iterate over all the cells involved in the action
	SBPL_4Dcell_t startcell4d, endcell4d;
	for(int i = 0; i < (int)action->intersectingcellsV.size(); i++)
	{

		//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
		startcell4d.theta = action->starttheta;
		startcell4d.x = - action->intersectingcellsV.at(i).x;
		startcell4d.y = - action->intersectingcellsV.at(i).y;
    startcell4d.t = - action->intersectingcellsV.at(i).t;

		//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
		endcell4d.theta = NORMALIZEDISCTHETA(action->endtheta, NAVXYTHETATIMELAT_THETADIRS); 
		endcell4d.x = startcell4d.x + action->dX; 
		endcell4d.y = startcell4d.y + action->dY;
		endcell4d.t = startcell4d.t + action->dT;

		//store the cells if not already there
		for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
		{
			if(affectedsuccstatesV.at(j) == endcell4d)
				break;
		}
		if (j == (int)affectedsuccstatesV.size())
			affectedsuccstatesV.push_back(endcell4d);

		for(j = 0; j < (int)affectedpredstatesV.size(); j++)
		{
			if(affectedpredstatesV.at(j) == startcell4d)
				break;
		}
		if (j == (int)affectedpredstatesV.size())
			affectedpredstatesV.push_back(startcell4d);

  }//over intersecting cells

	

	//add the centers since with h2d we are using these in cost computations
	//---intersecting cell = origin
	//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
	startcell4d.theta = action->starttheta;
	startcell4d.x = - 0;
	startcell4d.y = - 0;
	startcell4d.t = - 0;

	//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
	endcell4d.theta = NORMALIZEDISCTHETA(action->endtheta, NAVXYTHETATIMELAT_THETADIRS); 
	endcell4d.x = startcell4d.x + action->dX; 
	endcell4d.y = startcell4d.y + action->dY;
	endcell4d.t = startcell4d.y + action->dT;

	//store the cells if not already there
	for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
	{
		if(affectedsuccstatesV.at(j) == endcell4d)
			break;
	}
	if (j == (int)affectedsuccstatesV.size())
		affectedsuccstatesV.push_back(endcell4d);

	for(j = 0; j < (int)affectedpredstatesV.size(); j++)
	{
		if(affectedpredstatesV.at(j) == startcell4d)
			break;
	}
	if (j == (int)affectedpredstatesV.size())
		affectedpredstatesV.push_back(startcell4d);


	//---intersecting cell = outcome state
	//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
	startcell4d.theta = action->starttheta;
	startcell4d.x = - action->dX;
	startcell4d.y = - action->dY;
	startcell4d.t = - action->dT;

	//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
	endcell4d.theta = NORMALIZEDISCTHETA(action->endtheta, NAVXYTHETATIMELAT_THETADIRS); 
	endcell4d.x = startcell4d.x + action->dX; 
	endcell4d.y = startcell4d.y + action->dY;
	endcell4d.t = startcell4d.t + action->dT;

	for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
	{
		if(affectedsuccstatesV.at(j) == endcell4d)
			break;
	}
	if (j == (int)affectedsuccstatesV.size())
		affectedsuccstatesV.push_back(endcell4d);

	for(j = 0; j < (int)affectedpredstatesV.size(); j++)
	{
		if(affectedpredstatesV.at(j) == startcell4d)
			break;
	}
	if (j == (int)affectedpredstatesV.size())
		affectedpredstatesV.push_back(startcell4d);


}

//computes all the 4D states whose outgoing actions are potentially affected when cell (0,0) changes its status
//it also does the same for the 4D states whose incoming actions are potentially affected when cell (0,0) changes its status
void EnvironmentNAVXYTHETATIMELATTICE_AD::ComputeReplanningData()
{

    //iterate over all actions
	//orientations
	for(int tind = 0; tind < NAVXYTHETATIMELAT_THETADIRS; tind++)
    {        
        //actions
		for(int aind = 0; aind < EnvNAVXYTHETATIMELATCfg.actionwidth; aind++)
		{
            //compute replanning data for this action 
			ComputeReplanningDataforAction(&EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind]);
		}
	}
}

//here motionprimitivevector contains actions only for 0 angle
void EnvironmentNAVXYTHETATIMELATTICE_AD::PrecomputeActionswithBaseMotionPrimitive(vector<SBPL_xythetatime_mprimitive>* motionprimitiveV)
{

	SBPL_PRINTF("Pre-computing action data using base motion primitives...\n");
	EnvNAVXYTHETATIMELATCfg.ActionsV = new EnvNAVXYTHETATIMELATAction_t* [NAVXYTHETATIMELAT_THETADIRS];
	EnvNAVXYTHETATIMELATCfg.PredActionsV = new vector<EnvNAVXYTHETATIMELATAction_t*> [NAVXYTHETATIMELAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;

	//iterate over source angles
	for(int tind = 0; tind < NAVXYTHETATIMELAT_THETADIRS; tind++)
	{
		SBPL_PRINTF("pre-computing action %d out of %d actions\n", tind, NAVXYTHETATIMELAT_THETADIRS);
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind] = new EnvNAVXYTHETATIMELATAction_t[motionprimitiveV->size()];

		//compute sourcepose
		SBPL_4Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETATIMELATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETATIMELATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETATIMELAT_THETADIRS);
    sourcepose.t = DISCTIME2CONT(0, EnvNAVXYTHETATIMELATCfg.timeResolution);

		//iterate over motion primitives
		for(size_t aind = 0; aind < motionprimitiveV->size(); aind++)
		{
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].starttheta = tind;
			double mp_endx_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].x;
			double mp_endy_m = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].y;
			double mp_endtheta_rad = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].theta;
			double mp_endt = motionprimitiveV->at(aind).intermptV[motionprimitiveV->at(aind).intermptV.size()-1].t;
			
			double endx = sourcepose.x + (mp_endx_m*cos(sourcepose.theta) - mp_endy_m*sin(sourcepose.theta));
			double endy = sourcepose.y + (mp_endx_m*sin(sourcepose.theta) + mp_endy_m*cos(sourcepose.theta));
			
			int endx_c = CONTXY2DISC(endx, EnvNAVXYTHETATIMELATCfg.cellsize_m);
			int endy_c = CONTXY2DISC(endy, EnvNAVXYTHETATIMELATCfg.cellsize_m);

			
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta = ContTheta2Disc(mp_endtheta_rad+sourcepose.theta, NAVXYTHETATIMELAT_THETADIRS);
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX = endx_c;
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY = endy_c;
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT = CONTTIME2DISC(mp_endt + sourcepose.t, EnvNAVXYTHETATIMELATCfg.timeResolution);
      /*
			if(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY != 0 || EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX != 0) //if we translate
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EnvNAVXYTHETATIMELATCfg.cellsize_m/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs*
								sqrt((double)(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX + 
								EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY))));
			else if(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].starttheta != EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta)//else if we turn in place
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*
						EnvNAVXYTHETATIMELATCfg.timetoturn45degsinplace_secs*fabs(computeMinUnsignedAngleDiff(mp_endtheta_rad,0))/(PI_CONST/4.0));
      else //else (we stand still)
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*DISCTIME2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.timeResolution));
      */
      //EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*DISCTIME2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.timeResolution));
      EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT*EnvNAVXYTHETATIMELATCfg.timeResolution);


			//compute and store interm points as well as intersecting cells
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV.clear();
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].interm4DcellsV.clear();
			SBPL_4Dcell_t previnterm4Dcell;
			previnterm4Dcell.theta = previnterm4Dcell.x = previnterm4Dcell.y = previnterm4Dcell.t = 0;
			for (int pind = 0; pind < (int)motionprimitiveV->at(aind).intermptV.size(); pind++)
			{
				SBPL_4Dpt_t intermpt = motionprimitiveV->at(aind).intermptV[pind];
		
				//rotate it appropriately
				double rotx = intermpt.x*cos(sourcepose.theta) - intermpt.y*sin(sourcepose.theta);
				double roty = intermpt.x*sin(sourcepose.theta) + intermpt.y*cos(sourcepose.theta);
				intermpt.x = rotx;
				intermpt.y = roty;
				intermpt.theta = normalizeAngle(sourcepose.theta + intermpt.theta);
        //don't need to set time since the intermpt.t is already correct since sourcepose.t is always 0

				//store it (they are with reference to 0,0,stattheta (not sourcepose.x,sourcepose.y,starttheta (that is, half-bin))
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

				//now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y
				SBPL_4Dpt_t pose;
				pose = intermpt;
				pose.x += sourcepose.x;
				pose.y += sourcepose.y;
        pose.t += sourcepose.t;
				CalculateFootprintForPose(pose, &EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV);

				//now also store the intermediate discretized cell if not there already
				SBPL_4Dcell_t interm4Dcell;
				interm4Dcell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETATIMELATCfg.cellsize_m);
				interm4Dcell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
				interm4Dcell.theta = ContTheta2Disc(pose.theta, NAVXYTHETATIMELAT_THETADIRS); 
				interm4Dcell.t = CONTXY2DISC(pose.t, EnvNAVXYTHETATIMELATCfg.timeResolution);
				if(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].interm4DcellsV.size() == 0 || 
					previnterm4Dcell.theta != interm4Dcell.theta || previnterm4Dcell.x != interm4Dcell.x || previnterm4Dcell.y != interm4Dcell.y || previnterm4Dcell.t != interm4Dcell.t)
				{
					EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].interm4DcellsV.push_back(interm4Dcell);
				}
				previnterm4Dcell = interm4Dcell;

			}

			//now remove the source footprint
			RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			SBPL_FPRINTF(fDeb, "action tind=%d aind=%d: dX=%d dY=%d dT=%d endtheta=%d (%.2f degs -> %.2f degs) cost=%d (mprim: %.2f %.2f %.2f %.2f)\n",
				tind, aind, 			
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT,
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, sourcepose.theta*180/PI_CONST, 
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV[EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV.size()-1].theta*180/PI_CONST,	
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost,
				mp_endx_m, mp_endy_m, mp_endt, mp_endtheta_rad);
#endif

			//add to the list of backward actions
			int targettheta = EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + NAVXYTHETATIMELAT_THETADIRS;
			 EnvNAVXYTHETATIMELATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind]));

		}
	}

	//set number of actions
	EnvNAVXYTHETATIMELATCfg.actionwidth = motionprimitiveV->size();


	//now compute replanning data
	ComputeReplanningData();

	SBPL_PRINTF("done pre-computing action data based on motion primitives\n");


}

//here motionprimitivevector contains actions for all angles
void EnvironmentNAVXYTHETATIMELATTICE_AD::PrecomputeActionswithCompleteMotionPrimitive(vector<SBPL_xythetatime_mprimitive>* motionprimitiveV)
{

	SBPL_PRINTF("Pre-computing action data using motion primitives for every angle...\n");
	EnvNAVXYTHETATIMELATCfg.ActionsV = new EnvNAVXYTHETATIMELATAction_t* [NAVXYTHETATIMELAT_THETADIRS];
	EnvNAVXYTHETATIMELATCfg.PredActionsV = new vector<EnvNAVXYTHETATIMELATAction_t*> [NAVXYTHETATIMELAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;

	if(motionprimitiveV->size()%NAVXYTHETATIMELAT_THETADIRS != 0)
	{
		SBPL_ERROR("ERROR: motionprimitives should be uniform across actions\n");
		throw new SBPL_Exception();
	}

	EnvNAVXYTHETATIMELATCfg.actionwidth = ((int)motionprimitiveV->size())/NAVXYTHETATIMELAT_THETADIRS;

	//iterate over source angles
	int maxnumofactions = 0;
	for(int tind = 0; tind < NAVXYTHETATIMELAT_THETADIRS; tind++)
	{
		SBPL_PRINTF("pre-computing action %d out of %d actions\n", tind, NAVXYTHETATIMELAT_THETADIRS);

		EnvNAVXYTHETATIMELATCfg.ActionsV[tind] = new EnvNAVXYTHETATIMELATAction_t[EnvNAVXYTHETATIMELATCfg.actionwidth];  

		//compute sourcepose
		SBPL_4Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETATIMELATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETATIMELATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETATIMELAT_THETADIRS);
    sourcepose.t = DISCTIME2CONT(0, EnvNAVXYTHETATIMELATCfg.timeResolution);


		//iterate over motion primitives
		int numofactions = 0;
		int aind = -1;
		for(int mind = 0; mind < (int)motionprimitiveV->size(); mind++)
		{
			//find a motion primitive for this angle
			if(motionprimitiveV->at(mind).starttheta_c != tind)
				continue;
			
			aind++;
			numofactions++;

			//start angle
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].starttheta = tind;

			//compute dislocation
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta = motionprimitiveV->at(mind).endcell.theta;
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX = motionprimitiveV->at(mind).endcell.x;
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY = motionprimitiveV->at(mind).endcell.y;


			// @avemula NOTE using time computed from the readinmotionprimitive function (accounting for both rotation and translation)
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT = motionprimitiveV->at(mind).endcell.t;

			// @avemula DEBUG
			/*printf("Action takes time : %d\n", EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT); fflush(stdout);*/

      /*
			//compute cost (if translating)
			if(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY != 0 || EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX != 0)
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EnvNAVXYTHETATIMELATCfg.cellsize_m/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs*
								sqrt((double)(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX + 
								EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY))));
			else if(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].starttheta != EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta)//cost (if turning in place)
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*
						EnvNAVXYTHETATIMELATCfg.timetoturn45degsinplace_secs*
						fabs(computeMinUnsignedAngleDiff(DiscTheta2Cont(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETATIMELAT_THETADIRS),
														DiscTheta2Cont(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].starttheta, NAVXYTHETATIMELAT_THETADIRS)))/(PI_CONST/4.0));
      else //cost (if standing still)
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*DISCTIME2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.timeResolution));
      */

      //EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*DISCTIME2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.timeResolution));

			// @avemula ASK
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT*EnvNAVXYTHETATIMELATCfg.timeResolution);

			// @avemula REPLACING cost 
			/*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EnvNAVXYTHETATIMELATCfg.cellsize_m/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs*sqrt((double)(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX +EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY))));*/

			//use any additional cost multiplier (if the action has any rotation)
			//if(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].starttheta != EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta)
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost *= motionprimitiveV->at(mind).additionalactioncostmult;
			
			//compute and store interm points as well as intersecting cells
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV.clear();
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].interm4DcellsV.clear();
			SBPL_4Dcell_t previnterm4Dcell;
			previnterm4Dcell.theta = 0; previnterm4Dcell.x = 0; previnterm4Dcell.y = previnterm4Dcell.t = 0;
			for (int pind = 0; pind < (int)motionprimitiveV->at(mind).intermptV.size(); pind++)
			{
				SBPL_4Dpt_t intermpt = motionprimitiveV->at(mind).intermptV[pind];
		
				//store it (they are with reference to 0,0,stattheta (not sourcepose.x,sourcepose.y,starttheta (that is, half-bin))
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV.push_back(intermpt);

				//now compute the intersecting cells (for this pose has to be translated by sourcepose.x,sourcepose.y
				SBPL_4Dpt_t pose;
				pose = intermpt;
				pose.x += sourcepose.x;
				pose.y += sourcepose.y;
        pose.t += sourcepose.t;
				CalculateFootprintForPose(pose, &EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV);
			
				//now also store the intermediate discretized cell if not there already
				SBPL_4Dcell_t interm4Dcell;
				interm4Dcell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETATIMELATCfg.cellsize_m);
				interm4Dcell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
				interm4Dcell.theta = ContTheta2Disc(pose.theta, NAVXYTHETATIMELAT_THETADIRS); 
				interm4Dcell.t = CONTTIME2DISC(pose.t, EnvNAVXYTHETATIMELATCfg.timeResolution);
				if(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].interm4DcellsV.size() == 0 || 
					previnterm4Dcell.theta != interm4Dcell.theta || previnterm4Dcell.x != interm4Dcell.x || previnterm4Dcell.y != interm4Dcell.y || previnterm4Dcell.t != interm4Dcell.t)
				{
					EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].interm4DcellsV.push_back(interm4Dcell);
				}
				previnterm4Dcell = interm4Dcell;
			}

			//now remove the source footprint
			RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			SBPL_FPRINTF(fDeb, "action tind=%d aind=%d: dX=%d dY=%d dT=%d endtheta=%d (%.2f degs -> %.2f degs) cost=%d (mprimID %d: %d %d %d %d)\n",
				tind, aind, 			
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT,
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, 
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV[0].theta*180/PI_CONST, 
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV[EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV.size()-1].theta*180/PI_CONST,	
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost,
				motionprimitiveV->at(mind).motprimID, 
				motionprimitiveV->at(mind).endcell.x, motionprimitiveV->at(mind).endcell.y, motionprimitiveV->at(mind).endcell.t, motionprimitiveV->at(mind).endcell.theta);
#endif

			//add to the list of backward actions
			int targettheta = EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + NAVXYTHETATIMELAT_THETADIRS;
			 EnvNAVXYTHETATIMELATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind]));

		}

		if(maxnumofactions < numofactions)
			maxnumofactions = numofactions;
	}



	//at this point we don't allow nonuniform number of actions
	if(motionprimitiveV->size() != (size_t)(NAVXYTHETATIMELAT_THETADIRS*maxnumofactions))
	{
		SBPL_ERROR("ERROR: nonuniform number of actions is not supported (maxnumofactions=%d while motprims=%d thetas=%d\n",
				maxnumofactions, (int)motionprimitiveV->size(), NAVXYTHETATIMELAT_THETADIRS);
		throw new SBPL_Exception();
	}

	//now compute replanning data
	ComputeReplanningData();

	SBPL_PRINTF("done pre-computing action data based on motion primitives\n");

}

void EnvironmentNAVXYTHETATIMELATTICE_AD::PrecomputeActions()
{

	//construct list of actions
	SBPL_PRINTF("Pre-computing action data using the motion primitives for a 4D kinematic planning...\n");
	EnvNAVXYTHETATIMELATCfg.ActionsV = new EnvNAVXYTHETATIMELATAction_t* [NAVXYTHETATIMELAT_THETADIRS];
	EnvNAVXYTHETATIMELATCfg.PredActionsV = new vector<EnvNAVXYTHETATIMELATAction_t*> [NAVXYTHETATIMELAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;
	//iterate over source angles
	for(int tind = 0; tind < NAVXYTHETATIMELAT_THETADIRS; tind++)
	{
		SBPL_PRINTF("processing angle %d\n", tind);
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind] = new EnvNAVXYTHETATIMELATAction_t[EnvNAVXYTHETATIMELATCfg.actionwidth];

		//compute sourcepose
		SBPL_4Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETATIMELATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETATIMELATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETATIMELAT_THETADIRS);
		sourcepose.t = DISCTIME2CONT(0, EnvNAVXYTHETATIMELATCfg.timeResolution);

		//the construction assumes that the robot first turns and then goes along this new theta
		int aind = 0;
		for(; aind < 3; aind++)
		{
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].starttheta = tind;
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta = (tind + aind - 1)%NAVXYTHETATIMELAT_THETADIRS; //-1,0,1
			double angle = DiscTheta2Cont(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETATIMELAT_THETADIRS);
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX = (int)(cos(angle) + 0.5*(cos(angle)>0?1:-1));
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY = (int)(sin(angle) + 0.5*(sin(angle)>0?1:-1));
      //TODO:Mike we are ignoring the turn time in dT and in the cost, it is easily computable here 
      //and we know the robot rotates before moving so it seems like it should be added....
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT = (int)(ceil(EnvNAVXYTHETATIMELATCfg.cellsize_m/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs*sqrt((double)(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX + EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY))));
      EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EnvNAVXYTHETATIMELATCfg.cellsize_m/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs*sqrt((double)(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX + EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY))));

			//compute intersecting cells
			SBPL_4Dpt_t pose;
			pose.x = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETATIMELATCfg.cellsize_m);
			pose.y = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETATIMELATCfg.cellsize_m);
			pose.theta = angle;
			pose.t = DISCTIME2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.timeResolution);
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV.clear();
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			CalculateFootprintForPose(pose, &EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV);
			RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d dT=%d cost=%d\n",
				tind, aind, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, angle, 
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY,
				EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost);
#endif

			//add to the list of backward actions
			int targettheta = EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta;
			if (targettheta < 0)
				targettheta = targettheta + NAVXYTHETATIMELAT_THETADIRS;
			 EnvNAVXYTHETATIMELATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind]));

		}

		//decrease and increase angle without movement
		aind = 3;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].starttheta = tind;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta = tind-1; //TODO:Mike ask Max if this should be modulo the number of angle directions like below
		if(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta < 0) EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta += NAVXYTHETATIMELAT_THETADIRS;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX = 0;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY = 0;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT = (int)(EnvNAVXYTHETATIMELATCfg.timetoturn45degsinplace_secs);
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EnvNAVXYTHETATIMELATCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		SBPL_4Dpt_t pose;
		pose.x = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETATIMELATCfg.cellsize_m);
		pose.y = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETATIMELATCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETATIMELAT_THETADIRS);
		pose.t = DISCTIME2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.timeResolution);
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV.clear();
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
		SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d dT=%d cost=%d\n",
			tind, aind, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETATIMELAT_THETADIRS),
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY,
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		int targettheta = EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + NAVXYTHETATIMELAT_THETADIRS;
		 EnvNAVXYTHETATIMELATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind]));


		aind = 4;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].starttheta = tind;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta = (tind + 1)%NAVXYTHETATIMELAT_THETADIRS; 
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX = 0;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY = 0;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT = (int)(EnvNAVXYTHETATIMELATCfg.timetoturn45degsinplace_secs);
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EnvNAVXYTHETATIMELATCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		pose.x = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETATIMELATCfg.cellsize_m);
		pose.y = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETATIMELATCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETATIMELAT_THETADIRS);
		pose.t = DISCTIME2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.timeResolution);
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV.clear();
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV);


#if DEBUG
		SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d dT=%d cost=%d\n",
			tind, aind, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETATIMELAT_THETADIRS),
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY,
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		targettheta = EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + NAVXYTHETATIMELAT_THETADIRS;
		 EnvNAVXYTHETATIMELATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind]));

    //add one action for standing still
		aind = 5;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].starttheta = tind;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta = tind;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX = 0;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY = 0;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT = 1;
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*DISCTIME2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.timeResolution));

		//compute intersecting cells
		pose.x = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETATIMELATCfg.cellsize_m);
		pose.y = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETATIMELATCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETATIMELAT_THETADIRS);
		pose.t = DISCTIME2CONT(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.timeResolution);
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intermptV.clear();
		EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].intersectingcellsV);


#if DEBUG
		SBPL_PRINTF("action tind=%d aind=%d: endtheta=%d (%f) dX=%d dY=%d dT=%d cost=%d\n",
			tind, aind, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, DiscTheta2Cont(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta, NAVXYTHETATIMELAT_THETADIRS),
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dY,
			EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].dT, EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		targettheta = EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind].endtheta;
		if (targettheta < 0)
			targettheta = targettheta + NAVXYTHETATIMELAT_THETADIRS;
		 EnvNAVXYTHETATIMELATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETATIMELATCfg.ActionsV[tind][aind]));
	}

	//now compute replanning data
	ComputeReplanningData();

	SBPL_PRINTF("done pre-computing action data\n");


}


void EnvironmentNAVXYTHETATIMELATTICE_AD::InitializeEnvConfig(vector<SBPL_xythetatime_mprimitive>* motionprimitiveV)
{
	//aditional to configuration file initialization of EnvNAVXYTHETATIMELATCfg if necessary

	//dXY dirs
	EnvNAVXYTHETATIMELATCfg.dXY[0][0] = -1;
	EnvNAVXYTHETATIMELATCfg.dXY[0][1] = -1;
	EnvNAVXYTHETATIMELATCfg.dXY[1][0] = -1;
	EnvNAVXYTHETATIMELATCfg.dXY[1][1] = 0;
	EnvNAVXYTHETATIMELATCfg.dXY[2][0] = -1;
	EnvNAVXYTHETATIMELATCfg.dXY[2][1] = 1;
	EnvNAVXYTHETATIMELATCfg.dXY[3][0] = 0;
	EnvNAVXYTHETATIMELATCfg.dXY[3][1] = -1;
	EnvNAVXYTHETATIMELATCfg.dXY[4][0] = 0;
	EnvNAVXYTHETATIMELATCfg.dXY[4][1] = 1;
	EnvNAVXYTHETATIMELATCfg.dXY[5][0] = 1;
	EnvNAVXYTHETATIMELATCfg.dXY[5][1] = -1;
	EnvNAVXYTHETATIMELATCfg.dXY[6][0] = 1;
	EnvNAVXYTHETATIMELATCfg.dXY[6][1] = 0;
	EnvNAVXYTHETATIMELATCfg.dXY[7][0] = 1;
	EnvNAVXYTHETATIMELATCfg.dXY[7][1] = 1;


	SBPL_4Dpt_t temppose;
	temppose.x = 0.0;
	temppose.y = 0.0;
	temppose.theta = 0.0;
	temppose.t = 0.0;
	vector<SBPL_4Dcell_t> footprint;
	CalculateFootprintForPose(temppose, &footprint);
	SBPL_PRINTF("number of cells in footprint of the robot = %d\n", (int)footprint.size());

#if DEBUG
	SBPL_FPRINTF(fDeb, "footprint cells (size=%d):\n", footprint.size());
	for(int i = 0; i < (int) footprint.size(); i++)
	{
		SBPL_FPRINTF(fDeb, "%d %d (cont: %.3f %.3f)\n", footprint.at(i).x, footprint.at(i).y, 
			DISCXY2CONT(footprint.at(i).x, EnvNAVXYTHETATIMELATCfg.cellsize_m), 
			DISCXY2CONT(footprint.at(i).y, EnvNAVXYTHETATIMELATCfg.cellsize_m));
	}
#endif


	if(motionprimitiveV == NULL)
		PrecomputeActions();
	else
		PrecomputeActionswithCompleteMotionPrimitive(motionprimitiveV);


}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::IsValidCell(int X, int Y)
{
	return (X >= 0 && X < EnvNAVXYTHETATIMELATCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAVXYTHETATIMELATCfg.EnvHeight_c && 
		EnvNAVXYTHETATIMELATCfg.Grid2D[X][Y] < EnvNAVXYTHETATIMELATCfg.obsthresh);
}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::IsWithinMapCell(int X, int Y)
{
	return (X >= 0 && X < EnvNAVXYTHETATIMELATCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAVXYTHETATIMELATCfg.EnvHeight_c);
}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::IsValidConfiguration(int X, int Y, int Theta)
{
	vector<SBPL_4Dcell_t> footprint;
	SBPL_4Dpt_t pose;

	//compute continuous pose
	pose.x = DISCXY2CONT(X, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	pose.y = DISCXY2CONT(Y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	pose.theta = DiscTheta2Cont(Theta, NAVXYTHETATIMELAT_THETADIRS);

	//compute footprint cells
	CalculateFootprintForPose(pose, &footprint);

	//iterate over all footprint cells
	for(int find = 0; find < (int)footprint.size(); find++)
	{
		int x = footprint.at(find).x;
		int y = footprint.at(find).y;

		if (x < 0 || x >= EnvNAVXYTHETATIMELATCfg.EnvWidth_c ||
			y < 0 || Y >= EnvNAVXYTHETATIMELATCfg.EnvHeight_c ||		
			EnvNAVXYTHETATIMELATCfg.Grid2D[x][y] >= EnvNAVXYTHETATIMELATCfg.obsthresh)
		{
			return false;
		}
	}

	return true;
}


// @avemula NOTE change the collision checks according to the dimensionality of the point considered
int EnvironmentNAVXYTHETATIMELATTICE_AD::GetActionCost(int SourceX, int SourceY, int SourceTheta, int SourceT, EnvNAVXYTHETATIMELATAction_t* action, bool isHD)
{
	SBPL_4Dcell_t cell;
	SBPL_4Dcell_t interm4Dcell;
	int i;

	//TODO - go over bounding box (minpt and maxpt) to test validity and skip testing boundaries below, also order intersect cells so that the four farthest pts go first
	
	if(!IsValidCell(SourceX, SourceY))
		return INFINITECOST;
	if(!IsValidCell(SourceX + action->dX, SourceY + action->dY))
		return INFINITECOST;

	//need to iterate over discretized center cells and compute cost based on them
	unsigned char maxcellcost = 0;
	for(i = 0; i < (int)action->interm4DcellsV.size(); i++)
	{
		interm4Dcell = action->interm4DcellsV.at(i);
		interm4Dcell.x = interm4Dcell.x + SourceX;
		interm4Dcell.y = interm4Dcell.y + SourceY;
		interm4Dcell.t = interm4Dcell.t + SourceT;
		
		if(interm4Dcell.x < 0 || interm4Dcell.x >= EnvNAVXYTHETATIMELATCfg.EnvWidth_c ||
			interm4Dcell.y < 0 || interm4Dcell.y >= EnvNAVXYTHETATIMELATCfg.EnvHeight_c)
			return INFINITECOST;

		maxcellcost = __max(maxcellcost, EnvNAVXYTHETATIMELATCfg.Grid2D[interm4Dcell.x][interm4Dcell.y]);
		//check that the robot is NOT in the cell at which there is no valid orientation
		if(maxcellcost >= EnvNAVXYTHETATIMELATCfg.cost_inscribed_thresh)

		  return INFINITECOST;

		// @avemula NOTE don't do the following check for LD states
		// @avemula ASK
		if(isHD) {
		  if(getDynamicObstacleCost(interm4Dcell) > EnvNAVXYTHETATIMELATCfg.dynamic_obstacle_collision_cost_thresh){
		    
		    /*SBPL_PRINTF("dynObs collision x=%f y=%f t=%f\n", 
		       DISCXY2CONT(interm4Dcell.x, EnvNAVXYTHETATIMELATCfg.cellsize_m),
		       DISCXY2CONT(interm4Dcell.y, EnvNAVXYTHETATIMELATCfg.cellsize_m),
		       DISCTIME2CONT(interm4Dcell.t, EnvNAVXYTHETATIMELATCfg.timeResolution));*/
		    
		    
		    return INFINITECOST;
		  }
		}

	}


	//check collisions that for the particular footprint orientation along the action
	if(EnvNAVXYTHETATIMELATCfg.FootprintPolygon.size() > 1 && (int)maxcellcost >= EnvNAVXYTHETATIMELATCfg.cost_possibly_circumscribed_thresh)
	{
		checks++;

		for(i = 0; i < (int)action->intersectingcellsV.size(); i++) 
		{
			//get the cell in the map
			cell = action->intersectingcellsV.at(i);
			cell.x = cell.x + SourceX;
			cell.y = cell.y + SourceY;
			cell.t = cell.t + SourceT;
			
			//check validity
			if(!IsValidCell(cell.x, cell.y)/*|| getDynamicObstacleCost(cell) > EnvNAVXYTHETATIMELATCfg.dynamic_obstacle_collision_cost_thresh*/) 
				return INFINITECOST;

			//if(EnvNAVXYTHETATIMELATCfg.Grid2D[cell.x][cell.y] > currentmaxcost) //cost computation changed: cost = max(cost of centers of the robot along action)
			//	currentmaxcost = EnvNAVXYTHETATIMELATCfg.Grid2D[cell.x][cell.y];	//intersecting cells are only used for collision checking
		}
	}

	//to ensure consistency of h2D:
	maxcellcost = __max(maxcellcost, EnvNAVXYTHETATIMELATCfg.Grid2D[SourceX][SourceY]);
	int currentmaxcost = (int)__max(maxcellcost, EnvNAVXYTHETATIMELATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);

	return action->cost*(currentmaxcost+1); //use cell cost as multiplicative factor
 
}

// @avemula ADDED new getactioncost function that supports 3D expansions
int EnvironmentNAVXYTHETATIMELATTICE_AD::GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETATIMELATAction_t* action)
{
	SBPL_4Dcell_t cell;
	SBPL_4Dcell_t interm4Dcell;
	int i;

	//TODO - go over bounding box (minpt and maxpt) to test validity and skip testing boundaries below, also order intersect cells so that the four farthest pts go first
	
	if(!IsValidCell(SourceX, SourceY))
		return INFINITECOST;
	if(!IsValidCell(SourceX + action->dX, SourceY + action->dY))
		return INFINITECOST;

	//need to iterate over discretized center cells and compute cost based on them
	unsigned char maxcellcost = 0;
	for(i = 0; i < (int)action->interm4DcellsV.size(); i++)
	{
		interm4Dcell = action->interm4DcellsV.at(i);
		interm4Dcell.x = interm4Dcell.x + SourceX;
		interm4Dcell.y = interm4Dcell.y + SourceY;
		//interm4Dcell.t = interm4Dcell.t + SourceT;
		
		if(interm4Dcell.x < 0 || interm4Dcell.x >= EnvNAVXYTHETATIMELATCfg.EnvWidth_c ||
			interm4Dcell.y < 0 || interm4Dcell.y >= EnvNAVXYTHETATIMELATCfg.EnvHeight_c)
			return INFINITECOST;

		maxcellcost = __max(maxcellcost, EnvNAVXYTHETATIMELATCfg.Grid2D[interm4Dcell.x][interm4Dcell.y]);
		//check that the robot is NOT in the cell at which there is no valid orientation
		if(maxcellcost >= EnvNAVXYTHETATIMELATCfg.cost_inscribed_thresh)
			return INFINITECOST;

		// @avemula REMOVED cannot check dynamic obstacle collision without time
		/*if(getDynamicObstacleCost(interm4Dcell) > EnvNAVXYTHETATIMELATCfg.dynamic_obstacle_collision_cost_thresh){
      
      return INFINITECOST;
    }*/

	}


	//check collisions that for the particular footprint orientation along the action
	if(EnvNAVXYTHETATIMELATCfg.FootprintPolygon.size() > 1 && (int)maxcellcost >= EnvNAVXYTHETATIMELATCfg.cost_possibly_circumscribed_thresh)
	{
		checks++;

		for(i = 0; i < (int)action->intersectingcellsV.size(); i++) 
		{
			//get the cell in the map
			cell = action->intersectingcellsV.at(i);
			cell.x = cell.x + SourceX;
			cell.y = cell.y + SourceY;
			//cell.t = cell.t + SourceT;
			
			//check validity
			if(!IsValidCell(cell.x, cell.y)/*|| getDynamicObstacleCost(cell) > EnvNAVXYTHETATIMELATCfg.dynamic_obstacle_collision_cost_thresh*/) 
				return INFINITECOST;

			//if(EnvNAVXYTHETATIMELATCfg.Grid2D[cell.x][cell.y] > currentmaxcost) //cost computation changed: cost = max(cost of centers of the robot along action)
			//	currentmaxcost = EnvNAVXYTHETATIMELATCfg.Grid2D[cell.x][cell.y];	//intersecting cells are only used for collision checking
		}
	}

	//to ensure consistency of h2D:
	maxcellcost = __max(maxcellcost, EnvNAVXYTHETATIMELATCfg.Grid2D[SourceX][SourceY]);
	int currentmaxcost = (int)__max(maxcellcost, EnvNAVXYTHETATIMELATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);

	return action->cost*(currentmaxcost+1); //use cell cost as multiplicative factor
 
}


unsigned char EnvironmentNAVXYTHETATIMELATTICE_AD::getDynamicObstacleCost(SBPL_4Dcell_t cell){
  
  return (getInterval(cell.x,cell.y,cell.t) >= 0 ? 0 : 255);
}

double EnvironmentNAVXYTHETATIMELATTICE_AD::dynObsPtSqrDist(SBPL_4Dcell_t cell, SBPL_Traj_Pt_t p){
  double x = DISCXY2CONT(cell.x, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  double y = DISCXY2CONT(cell.y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  return (x-p.x)*(x-p.x)+(y-p.y)*(y-p.y);
}


double EnvironmentNAVXYTHETATIMELATTICE_AD::EuclideanDistance_m(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2));
    return EnvNAVXYTHETATIMELATCfg.cellsize_m*sqrt((double)sqdist);

}

//adds points to it (does not clear it beforehand)
void EnvironmentNAVXYTHETATIMELATTICE_AD::CalculateFootprintForPose(SBPL_4Dpt_t pose, vector<SBPL_4Dcell_t>* footprint)
{  
	int pind;

#if DEBUG
//  SBPL_PRINTF("---Calculating Footprint for Pose: %f %f %f---\n",
//	 pose.x, pose.y, pose.theta);
#endif

  //handle special case where footprint is just a point
  if(EnvNAVXYTHETATIMELATCfg.FootprintPolygon.size() <= 1){
    SBPL_4Dcell_t cell;
    cell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETATIMELATCfg.cellsize_m);
    cell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETATIMELATCfg.cellsize_m);

	for(pind = 0; pind < (int)footprint->size(); pind++)
	{
		if(cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y)
			break;
	}
	if(pind == (int)footprint->size()) footprint->push_back(cell);
    return;
  }

  vector<sbpl_2Dpt_t> bounding_polygon;
  unsigned int find;
  double max_x = -INFINITECOST, min_x = INFINITECOST, max_y = -INFINITECOST, min_y = INFINITECOST;
  sbpl_2Dpt_t pt = {0,0};
  for(find = 0; find < EnvNAVXYTHETATIMELATCfg.FootprintPolygon.size(); find++){
    
    //rotate and translate the corner of the robot
    pt = EnvNAVXYTHETATIMELATCfg.FootprintPolygon[find];
    
    //rotate and translate the point
    sbpl_2Dpt_t corner;
    corner.x = cos(pose.theta)*pt.x - sin(pose.theta)*pt.y + pose.x;
    corner.y = sin(pose.theta)*pt.x + cos(pose.theta)*pt.y + pose.y;
    bounding_polygon.push_back(corner);
#if DEBUG
//    SBPL_PRINTF("Pt: %f %f, Corner: %f %f\n", pt.x, pt.y, corner.x, corner.y);
#endif
    if(corner.x < min_x || find==0){
      min_x = corner.x;
    }
    if(corner.x > max_x || find==0){
      max_x = corner.x;
    }
    if(corner.y < min_y || find==0){
      min_y = corner.y;
    }
    if(corner.y > max_y || find==0){
      max_y = corner.y;
    }
    
  }

#if DEBUG
//  SBPL_PRINTF("Footprint bounding box: %f %f %f %f\n", min_x, max_x, min_y, max_y);
#endif
  //initialize previous values to something that will fail the if condition during the first iteration in the for loop
  int prev_discrete_x = CONTXY2DISC(pt.x, EnvNAVXYTHETATIMELATCfg.cellsize_m) + 1; 
  int prev_discrete_y = CONTXY2DISC(pt.y, EnvNAVXYTHETATIMELATCfg.cellsize_m) + 1;
  int prev_inside = 0;
  int discrete_x;
  int discrete_y;

  for(double x=min_x; x<=max_x; x+=EnvNAVXYTHETATIMELATCfg.cellsize_m/3){
    for(double y=min_y; y<=max_y; y+=EnvNAVXYTHETATIMELATCfg.cellsize_m/3){
      pt.x = x;
      pt.y = y;
      discrete_x = CONTXY2DISC(pt.x, EnvNAVXYTHETATIMELATCfg.cellsize_m);
      discrete_y = CONTXY2DISC(pt.y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
      
      //see if we just tested this point
      if(discrete_x != prev_discrete_x || discrete_y != prev_discrete_y || prev_inside==0){

#if DEBUG
//		SBPL_PRINTF("Testing point: %f %f Discrete: %d %d\n", pt.x, pt.y, discrete_x, discrete_y);
#endif
	
		if(IsInsideFootprint(pt, &bounding_polygon)){
		//convert to a grid point

#if DEBUG
//			SBPL_PRINTF("Pt Inside %f %f\n", pt.x, pt.y);
#endif

			SBPL_4Dcell_t cell;
			cell.x = discrete_x;
			cell.y = discrete_y;

			//insert point if not there already
			int pind = 0;
			for(pind = 0; pind < (int)footprint->size(); pind++)
			{
				if(cell.x == footprint->at(pind).x && cell.y == footprint->at(pind).y)
					break;
			}
			if(pind == (int)footprint->size()) footprint->push_back(cell);

			prev_inside = 1;

#if DEBUG
//			SBPL_PRINTF("Added pt to footprint: %f %f\n", pt.x, pt.y);
#endif
		}
		else{
			prev_inside = 0;
		}

      }
	  else
	  {
#if DEBUG
		//SBPL_PRINTF("Skipping pt: %f %f\n", pt.x, pt.y);
#endif
      }
      
      prev_discrete_x = discrete_x;
      prev_discrete_y = discrete_y;

    }//over x_min...x_max
  }
}

void EnvironmentNAVXYTHETATIMELATTICE_AD::RemoveSourceFootprint(SBPL_4Dpt_t sourcepose, vector<SBPL_4Dcell_t>* footprint)
{  
	vector<SBPL_4Dcell_t> sourcefootprint;

	//compute source footprint
	CalculateFootprintForPose(sourcepose, &sourcefootprint);

	//now remove the source cells from the footprint
	for(int sind = 0; sind < (int)sourcefootprint.size(); sind++)
	{
		for(int find = 0; find < (int)footprint->size(); find++)
		{
			if(sourcefootprint.at(sind).x == footprint->at(find).x && sourcefootprint.at(sind).y == footprint->at(find).y)
			{
				footprint->erase(footprint->begin() + find);
				break;
			}
		}//over footprint
	}//over source
}


//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentNAVXYTHETATIMELATTICE_AD::ComputeHeuristicValues()
{
	//whatever necessary pre-computation of heuristic values is done here 
	SBPL_PRINTF("Precomputing heuristics...\n");
	fflush(stdout);
	
	//allocated 2D grid searches
	grid2Dsearchfromstart = new SBPL2DGridSearch(EnvNAVXYTHETATIMELATCfg.EnvWidth_c, EnvNAVXYTHETATIMELATCfg.EnvHeight_c, (float)EnvNAVXYTHETATIMELATCfg.cellsize_m);
	grid2Dsearchfromgoal = new SBPL2DGridSearch(EnvNAVXYTHETATIMELATCfg.EnvWidth_c, EnvNAVXYTHETATIMELATCfg.EnvHeight_c, (float)EnvNAVXYTHETATIMELATCfg.cellsize_m);

	SBPL_PRINTF("done\n");

}

//------------debugging functions---------------------------------------------
bool EnvironmentNAVXYTHETATIMELATTICE_AD::CheckQuant(FILE* fOut) 
{

  for(double theta  = -10; theta < 10; theta += 2.0*PI_CONST/NAVXYTHETATIMELAT_THETADIRS*0.01)
    {
		int nTheta = ContTheta2Disc(theta, NAVXYTHETATIMELAT_THETADIRS);
		double newTheta = DiscTheta2Cont(nTheta, NAVXYTHETATIMELAT_THETADIRS);
		int nnewTheta = ContTheta2Disc(newTheta, NAVXYTHETATIMELAT_THETADIRS);

		SBPL_FPRINTF(fOut, "theta=%f(%f)->%d->%f->%d\n", theta, theta*180/PI_CONST, nTheta, newTheta, nnewTheta);

        if(nTheta != nnewTheta)
        {
            SBPL_ERROR("ERROR: invalid quantization\n");                     
            return false;
        }
    }

  return true;
}

//-----------------------------------------------------------------------------

//-----------interface with outside functions-----------------------------------
bool EnvironmentNAVXYTHETATIMELATTICE_AD::InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV, const char* sMotPrimFile, const char* sDynObsFile){
  //load the environment and the motion primitives
  InitializeEnv(sEnvFile, perimeterptsV, sMotPrimFile);

  //load the dynamic obstacles
  FILE* fDynObs = fopen(sDynObsFile, "r");
  if(fDynObs == NULL){
    SBPL_PRINTF("Error: unable to open %s\n", sDynObsFile);
    throw new SBPL_Exception();
  }
  ReadDynamicObstacles(fDynObs);
  fclose(fDynObs);
  return UpdateTimelineMap();
}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::InitializeEnv(const char* sEnvFile, const vector<sbpl_2Dpt_t>& perimeterptsV, const char* sMotPrimFile)
{
  //find the radius of the robot (smallest circle that contains the robot) squared
  //this will be used to pad dynamic obstacles for faster collision checking
  
  
  double max_sqr_radius = 0;
  for(unsigned int i=0; i<perimeterptsV.size(); i++){
    double x = perimeterptsV.at(i).x;
    double y = perimeterptsV.at(i).y;
    double d = x*x + y*y;
    if(d > max_sqr_radius)
      max_sqr_radius = d;
  }
  
  EnvNAVXYTHETATIMELATCfg.robotRadius = sqrt(max_sqr_radius);
  
  SBPL_PRINTF("robotRadius=%f\n", EnvNAVXYTHETATIMELATCfg.robotRadius);
  fflush(stdout);

  EnvNAVXYTHETATIMELATCfg.FootprintPolygon = perimeterptsV;

  FILE* fCfg = fopen(sEnvFile, "r");
  if(fCfg == NULL)
    {
      SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
      throw new SBPL_Exception();
    }
  ReadConfiguration(fCfg);
  fclose(fCfg);

  temporal_padding = DEFAULT_TEMPORAL_PADDING;
  InitializeTimelineMap();

  if(sMotPrimFile != NULL)
    {
      FILE* fMotPrim = fopen(sMotPrimFile, "r");
      if(fMotPrim == NULL)
	{
	  SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
	  throw new SBPL_Exception();
	}
      if(ReadMotionPrimitives(fMotPrim) == false)
	{
	  SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
	  throw new SBPL_Exception();
	}
      fclose(fMotPrim);
      
      InitGeneral(&EnvNAVXYTHETATIMELATCfg.mprimV);
      //printf("coord2state is %x\n", Coord2StateIDHashTable);
    }
  else
    InitGeneral(NULL);


  return true;
}


bool EnvironmentNAVXYTHETATIMELATTICE_AD::InitializeEnv(const char* sEnvFile)
{

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
		throw new SBPL_Exception();
	}
	ReadConfiguration(fCfg);
  fclose(fCfg);

	InitGeneral(NULL);


	return true;
}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::InitializeEnv(int width, int height,
					const unsigned char* mapdata,
					double startx, double starty, double starttheta, double startTime,
					double goalx, double goaly, double goaltheta,
				  double goaltol_x, double goaltol_y, double goaltol_theta,
					const vector<sbpl_2Dpt_t> & perimeterptsV,
					double cellsize_m, double timeResolution,
          double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
					unsigned char obsthresh, unsigned char dynobsthresh,  const char* sMotPrimFile,
          vector<SBPL_DynamicObstacle_t> & dynObs)
{

	SBPL_PRINTF("env: initialize with width=%d height=%d start=%.3f %.3f %.3f %.3f goalx=%.3f %.3f %.3f cellsize=%.3f timeResolution=%.3f nomvel=%.3f timetoturn=%.3f, obsthresh=%d\n",
		width, height, startx, starty, starttheta, startTime, goalx, goaly, goaltheta, cellsize_m, timeResolution, nominalvel_mpersecs, timetoturn45degsinplace_secs, obsthresh);

	SBPL_PRINTF("perimeter has size=%d\n", (int)perimeterptsV.size());

	for(int i = 0; i < (int)perimeterptsV.size(); i++)
	{
		SBPL_PRINTF("perimeter(%d) = %.4f %.4f\n", i, perimeterptsV.at(i).x, perimeterptsV.at(i).y);
	}


	EnvNAVXYTHETATIMELATCfg.obsthresh = obsthresh;
	EnvNAVXYTHETATIMELATCfg.dynamic_obstacle_collision_cost_thresh = dynobsthresh;

  //Set up the dynamic obstacles...
  double max_sqr_radius = 0;
  for(unsigned int i=0; i<perimeterptsV.size(); i++){
    double x = perimeterptsV.at(i).x;
    double y = perimeterptsV.at(i).y;
    double d = x*x + y*y;
    if(d > max_sqr_radius)
      max_sqr_radius = d;
  }
  EnvNAVXYTHETATIMELATCfg.robotRadius = sqrt(max_sqr_radius);
  dynamicObstacles.clear();
  for(unsigned int i=0; i<dynObs.size(); i++){
    SBPL_DynamicObstacle_t obs;
    obs.radius = dynObs[i].radius + EnvNAVXYTHETATIMELATCfg.robotRadius;
    for(unsigned int j=0; j<dynObs[i].trajectories.size(); j++){
      SBPL_Trajectory_t traj;
      traj.prob = dynObs[i].trajectories[j].prob;
      traj.existsAfter = dynObs[i].trajectories[j].existsAfter;
      for(unsigned int k=0; k<dynObs[i].trajectories[j].points.size(); k++){
        SBPL_Traj_Pt_t p;
        p.x = dynObs[i].trajectories[j].points[k].x;
        p.y = dynObs[i].trajectories[j].points[k].y;
        p.t = dynObs[i].trajectories[j].points[k].t;
        p.std_dev = dynObs[i].trajectories[j].points[k].std_dev;
        traj.points.push_back(p);
      }
      obs.trajectories.push_back(traj);
    }
    dynamicObstacles.push_back(obs);
  }

	//TODO - need to set the tolerance as well

	SetConfiguration(width, height,
					mapdata,
					CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m), ContTheta2Disc(starttheta, NAVXYTHETATIMELAT_THETADIRS), CONTTIME2DISC(startTime, timeResolution),
					CONTXY2DISC(goalx, cellsize_m), CONTXY2DISC(goaly, cellsize_m), ContTheta2Disc(goaltheta, NAVXYTHETATIMELAT_THETADIRS),
					cellsize_m, timeResolution, nominalvel_mpersecs, timetoturn45degsinplace_secs, perimeterptsV);

	if(sMotPrimFile != NULL)
	{
		FILE* fMotPrim = fopen(sMotPrimFile, "r");
		if(fMotPrim == NULL)
		{
			SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
			throw new SBPL_Exception();
		}

		if(ReadMotionPrimitives(fMotPrim) == false)
		{
			SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
			throw new SBPL_Exception();
		}
    fclose(fMotPrim);
	}

  temporal_padding = DEFAULT_TEMPORAL_PADDING;
  InitializeTimelineMap();

	if(EnvNAVXYTHETATIMELATCfg.mprimV.size() != 0)
	{
		InitGeneral(&EnvNAVXYTHETATIMELATCfg.mprimV);
	}
	else
		InitGeneral(NULL);

	return true;
}


bool EnvironmentNAVXYTHETATIMELATTICE_AD::InitializeEnv(int width, int height,
					const unsigned char* mapdata,
					double startx, double starty, double starttheta, double startTime,
					double goalx, double goaly, double goaltheta,
				  double goaltol_x, double goaltol_y, double goaltol_theta,
					const vector<sbpl_2Dpt_t> & perimeterptsV,
					double cellsize_m, double timeResolution,
          double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
					unsigned char obsthresh, unsigned char dynobsthresh,  const char* sMotPrimFile)
{

	SBPL_PRINTF("env: initialize with width=%d height=%d start=%.3f %.3f %.3f %.3f goalx=%.3f %.3f %.3f cellsize=%.3f timeResolution=%.3f nomvel=%.3f timetoturn=%.3f, obsthresh=%d\n",
		width, height, startx, starty, starttheta, startTime, goalx, goaly, goaltheta, cellsize_m, timeResolution, nominalvel_mpersecs, timetoturn45degsinplace_secs, obsthresh);

	SBPL_PRINTF("perimeter has size=%d\n", (int)perimeterptsV.size());

	for(int i = 0; i < (int)perimeterptsV.size(); i++)
	{
		SBPL_PRINTF("perimeter(%d) = %.4f %.4f\n", i, perimeterptsV.at(i).x, perimeterptsV.at(i).y);
	}


	EnvNAVXYTHETATIMELATCfg.obsthresh = obsthresh;
	EnvNAVXYTHETATIMELATCfg.dynamic_obstacle_collision_cost_thresh = dynobsthresh;

	//TODO - need to set the tolerance as well

	SetConfiguration(width, height,
					mapdata,
					CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m), ContTheta2Disc(starttheta, NAVXYTHETATIMELAT_THETADIRS), CONTTIME2DISC(startTime, timeResolution),
					CONTXY2DISC(goalx, cellsize_m), CONTXY2DISC(goaly, cellsize_m), ContTheta2Disc(goaltheta, NAVXYTHETATIMELAT_THETADIRS),
					cellsize_m, timeResolution, nominalvel_mpersecs, timetoturn45degsinplace_secs, perimeterptsV);

	if(sMotPrimFile != NULL)
	{
		FILE* fMotPrim = fopen(sMotPrimFile, "r");
		if(fMotPrim == NULL)
		{
			SBPL_ERROR("ERROR: unable to open %s\n", sMotPrimFile);
			throw new SBPL_Exception();
		}

		if(ReadMotionPrimitives(fMotPrim) == false)
		{
			SBPL_ERROR("ERROR: failed to read in motion primitive file\n");
			throw new SBPL_Exception();
		}
    fclose(fMotPrim);
	}

	if(EnvNAVXYTHETATIMELATCfg.mprimV.size() != 0)
	{
		InitGeneral(&EnvNAVXYTHETATIMELATCfg.mprimV);
	}
	else
		InitGeneral(NULL);

	return true;
}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::InitGeneral(vector<SBPL_xythetatime_mprimitive>* motionprimitiveV) {


  //Initialize other parameters of the environment
  InitializeEnvConfig(motionprimitiveV);
  
  //initialize Environment
  InitializeEnvironment();
  
  //pre-compute heuristics
  ComputeHeuristicValues();

  return true;
}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::InitializeMDPCfg(MDPConfig *MDPCfg)
{
	//initialize MDPCfg with the start and goal ids	
	MDPCfg->goalstateid = EnvNAVXYTHETATIMELAT.goalstateid;
	MDPCfg->startstateid = EnvNAVXYTHETATIMELAT.startstateid;

	return true;
}

void EnvironmentNAVXYTHETATIMELATTICE_AD::PrintHeuristicValues()
{
	FILE* fHeur = fopen("heur.txt", "w");
	SBPL2DGridSearch* grid2Dsearch = NULL;
	
	for(int i = 0; i < 2; i++)
	{
		if(i == 0 && grid2Dsearchfromstart != NULL)
		{
			grid2Dsearch = grid2Dsearchfromstart;
			SBPL_FPRINTF(fHeur, "start heuristics:\n");
		}
		else if(i == 1 && grid2Dsearchfromgoal != NULL)
		{
			grid2Dsearch = grid2Dsearchfromgoal;
			SBPL_FPRINTF(fHeur, "goal heuristics:\n");
		}
		else
			continue;

		for (int y = 0; y < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; y++) {
			for (int x = 0; x < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; x++) {
				if(grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y) < INFINITECOST)
					SBPL_FPRINTF(fHeur, "%5d ", grid2Dsearch->getlowerboundoncostfromstart_inmm(x, y));
			else
				SBPL_FPRINTF(fHeur, "XXXXX ");
			}
			SBPL_FPRINTF(fHeur, "\n");
		}
	}
	fclose(fHeur);
}

void EnvironmentNAVXYTHETATIMELATTICE_AD::SetAllPreds(CMDPSTATE* state)
{
	//implement this if the planner needs access to predecessors
	
	SBPL_ERROR("ERROR in EnvNAVXYTHETATIMELAT... function: SetAllPreds is undefined\n");
	throw new SBPL_Exception();
}


void EnvironmentNAVXYTHETATIMELATTICE_AD::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
	GetSuccs(SourceStateID, SuccIDV, CostV, NULL);
}





const EnvNAVXYTHETATIMELATConfig_t* EnvironmentNAVXYTHETATIMELATTICE_AD::GetEnvNavConfig() {
  return &EnvNAVXYTHETATIMELATCfg;
}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::UpdateCost(int x, int y, unsigned char newcost)
{

#if DEBUG
	SBPL_FPRINTF(fDeb, "Cost updated for cell %d %d from old cost=%d to new cost=%d\n", x,y,EnvNAVXYTHETATIMELATCfg.Grid2D[x][y], newcost);
#endif

    EnvNAVXYTHETATIMELATCfg.Grid2D[x][y] = newcost;

	bNeedtoRecomputeStartHeuristics = true;
	bNeedtoRecomputeGoalHeuristics = true;

    return true;
}


void EnvironmentNAVXYTHETATIMELATTICE_AD::PrintEnv_Config(FILE* fOut)
{

	//implement this if the planner needs to print out EnvNAVXYTHETATIMELAT. configuration
	
	SBPL_ERROR("ERROR in EnvNAVXYTHETATIMELAT... function: PrintEnv_Config is undefined\n");
	throw new SBPL_Exception();

}

void EnvironmentNAVXYTHETATIMELATTICE_AD::PrintTimeStat(FILE* fOut)
{

#if TIME_DEBUG
    SBPL_FPRINTF(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f\n",
            time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC, 
            time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::IsObstacle(int x, int y)
{

#if DEBUG
	SBPL_FPRINTF(fDeb, "Status of cell %d %d is queried. Its cost=%d\n", x,y,EnvNAVXYTHETATIMELATCfg.Grid2D[x][y]);
#endif


	return (EnvNAVXYTHETATIMELATCfg.Grid2D[x][y] >= EnvNAVXYTHETATIMELATCfg.obsthresh); 

}

int EnvironmentNAVXYTHETATIMELATTICE_AD::getStartID(){
  return EnvNAVXYTHETATIMELAT.startstateid;
}

int EnvironmentNAVXYTHETATIMELATTICE_AD::getGoalID(){
  return EnvNAVXYTHETATIMELAT.goalstateid;
}

void EnvironmentNAVXYTHETATIMELATTICE_AD::GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double*starttheta, double* startTime, double* goalx, double* goaly, double* goaltheta, double* cellsize_m, double* timeResolution, double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs, unsigned char* obsthresh, unsigned char* dyn_obs_thresh, vector<SBPL_xythetatime_mprimitive>* mprimitiveV)
{
	*size_x = EnvNAVXYTHETATIMELATCfg.EnvWidth_c;
	*size_y = EnvNAVXYTHETATIMELATCfg.EnvHeight_c;

	*startx = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	*starty = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.StartY_c, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	*starttheta = DiscTheta2Cont(EnvNAVXYTHETATIMELATCfg.StartTheta, NAVXYTHETATIMELAT_THETADIRS);
	*startTime = DISCTIME2CONT(EnvNAVXYTHETATIMELATCfg.StartTime, EnvNAVXYTHETATIMELATCfg.timeResolution);
	*goalx = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	*goaly = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.EndY_c, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	*goaltheta = DiscTheta2Cont(EnvNAVXYTHETATIMELATCfg.EndTheta, NAVXYTHETATIMELAT_THETADIRS);

	*cellsize_m = EnvNAVXYTHETATIMELATCfg.cellsize_m;
	*timeResolution = EnvNAVXYTHETATIMELATCfg.timeResolution;
	*nominalvel_mpersecs = EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs;
	*timetoturn45degsinplace_secs = EnvNAVXYTHETATIMELATCfg.timetoturn45degsinplace_secs;

	*dyn_obs_thresh = EnvNAVXYTHETATIMELATCfg.dynamic_obstacle_collision_cost_thresh;
	*obsthresh = EnvNAVXYTHETATIMELATCfg.obsthresh;

	*mprimitiveV = EnvNAVXYTHETATIMELATCfg.mprimV;
}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::PoseContToDisc(double px, double py, double pth, double pt,int &ix, int &iy, int &ith, int &it) const
{
  ix = CONTXY2DISC(px, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  iy = CONTXY2DISC(py, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  ith = ContTheta2Disc(pth, NAVXYTHETATIMELAT_THETADIRS); // ContTheta2Disc() normalizes the angle
  it = CONTTIME2DISC(pt, EnvNAVXYTHETATIMELATCfg.timeResolution);
  return (pth >= -2*PI_CONST) && (pth <= 2*PI_CONST)
    && (ix >= 0) && (ix < EnvNAVXYTHETATIMELATCfg.EnvWidth_c)
    && (iy >= 0) && (iy < EnvNAVXYTHETATIMELATCfg.EnvHeight_c);
}

bool EnvironmentNAVXYTHETATIMELATTICE_AD::PoseDiscToCont(int ix, int iy, int ith, int it,double &px, double &py, double &pth, double &pt) const
{
  px = DISCXY2CONT(ix, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  py = DISCXY2CONT(iy, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  pth = normalizeAngle(DiscTheta2Cont(ith, NAVXYTHETATIMELAT_THETADIRS));
  pt = DISCTIME2CONT(it, EnvNAVXYTHETATIMELATCfg.timeResolution);
  return (ith >= 0) && (ith < NAVXYTHETATIMELAT_THETADIRS)
    && (ix >= 0) && (ix < EnvNAVXYTHETATIMELATCfg.EnvWidth_c)
    && (iy >= 0) && (iy < EnvNAVXYTHETATIMELATCfg.EnvHeight_c);
}

unsigned char EnvironmentNAVXYTHETATIMELATTICE_AD::GetMapCost(int x, int y)
{
	return EnvNAVXYTHETATIMELATCfg.Grid2D[x][y];
}



bool EnvironmentNAVXYTHETATIMELATTICE_AD::SetEnvParameter(const char* parameter, int value)
{

	if(EnvNAVXYTHETATIMELAT.bInitialized == true)
	{
		SBPL_ERROR("ERROR: all parameters must be set before initialization of the environment\n");
		return false;
	}

	SBPL_PRINTF("setting parameter %s to %d\n", parameter, value);

	if(strcmp(parameter, "cost_inscribed_thresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		EnvNAVXYTHETATIMELATCfg.cost_inscribed_thresh = (unsigned char)value;
	}
	else if(strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		EnvNAVXYTHETATIMELATCfg.cost_possibly_circumscribed_thresh = value;
	}
	else if(strcmp(parameter, "cost_obsthresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		EnvNAVXYTHETATIMELATCfg.obsthresh = (unsigned char)value;
	}
	else if(strcmp(parameter, "cost_dyn_obs_thresh") == 0)
	{
		if(value < 0 || value > 255)
		{
		  SBPL_ERROR("ERROR: invalid value %d for parameter %s\n", value, parameter);
			return false;
		}
		EnvNAVXYTHETATIMELATCfg.dynamic_obstacle_collision_cost_thresh = (unsigned char)value;
	}
	else
	{
		SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
		return false;
	}

	return true;
}

int EnvironmentNAVXYTHETATIMELATTICE_AD::GetEnvParameter(const char* parameter)
{

	if(strcmp(parameter, "cost_inscribed_thresh") == 0)
	{
		return (int) EnvNAVXYTHETATIMELATCfg.cost_inscribed_thresh;
	}
	else if(strcmp(parameter, "cost_possibly_circumscribed_thresh") == 0)
	{
		return (int) EnvNAVXYTHETATIMELATCfg.cost_possibly_circumscribed_thresh;
	}
	else if(strcmp(parameter, "cost_obsthresh") == 0)
	{
		return (int) EnvNAVXYTHETATIMELATCfg.obsthresh;
	}
	else if(strcmp(parameter, "cost_dyn_obs_thresh") == 0)
	{
		return (int) EnvNAVXYTHETATIMELATCfg.dynamic_obstacle_collision_cost_thresh;
	}
	else
	{
		SBPL_ERROR("ERROR: invalid parameter %s\n", parameter);
		throw new SBPL_Exception();
	}

}


//-----------------XYTHETATIME Environment (child) class------------------------

EnvironmentNAVXYTHETATIMELAT_AD::~EnvironmentNAVXYTHETATIMELAT_AD()
{
	printf("destroying XYTHETALAT\n");

	//delete the states themselves first
	for (int i = 0; i < (int)StateID2CoordTable.size(); i++)
	{
		delete StateID2CoordTable.at(i);
		StateID2CoordTable.at(i) = NULL;
	}
	StateID2CoordTable.clear();

	//delete hashtable
	if(Coord2StateIDHashTable != NULL)
	{
		delete [] Coord2StateIDHashTable;
		Coord2StateIDHashTable = NULL;
	}	
	if(Coord2StateIDHashTable_lookup != NULL)
	{
		delete [] Coord2StateIDHashTable_lookup;
		Coord2StateIDHashTable_lookup = NULL;
	}

}

void EnvironmentNAVXYTHETATIMELAT_AD::GetCoordFromState(int stateID, int& x, int& y, int& theta, int& t) const {
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
  t = HashEntry->T;
}

void EnvironmentNAVXYTHETATIMELAT_AD::GetCoordFromState(int stateID, int& x, int& y, int& theta, int& t, float& tempT) const {
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
  t = HashEntry->T;
  tempT = HashEntry->TempT;
}

int EnvironmentNAVXYTHETATIMELAT_AD::GetStateFromCoord(int x, int y, int theta, int t) {

   EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
   if((OutHashEntry = (this->*GetHashEntry)(x, y, theta, t)) == NULL){
        //have to create a new entry
     OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta, t);
    }
    return OutHashEntry->stateID;
}

void EnvironmentNAVXYTHETATIMELAT_AD::ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, vector<SBPL_4Dpt_t>* xythetaPath)
{
  vector<EnvNAVXYTHETATIMELATAction_t*> actionV;
  vector<int> CostV;
  vector<int> SuccIDV;
  int targetx_c, targety_c, targettheta_c;
  int sourcex_c, sourcey_c, sourcetheta_c;
  int targett_c, sourcet_c;

  //SBPL_PRINTF("checks=%ld\n", checks);

  xythetaPath->clear();

#if DEBUG
  SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:\n");
#endif

  for(int pind = 0; pind < (int)(stateIDPath->size())-1; pind++)
    {
      int sourceID = stateIDPath->at(pind);
      int targetID = stateIDPath->at(pind+1);

#if DEBUG
      GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c);
#endif


      //get successors and pick the target via the cheapest action
      SuccIDV.clear();
      CostV.clear();
      actionV.clear();
      GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);
		
      int bestcost = INFINITECOST;
      int bestsind = -1;

#if DEBUG
      GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c);
      GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targett_c);
      SBPL_FPRINTF(fDeb, "looking for %d %d %d %d -> %d %d %d %d (numofsuccs=%d)\n", sourcex_c, sourcey_c, sourcetheta_c, sourcet_c, targetx_c, targety_c, targettheta_c, targett_c, SuccIDV.size()); 

#endif

      for(int sind = 0; sind < (int)SuccIDV.size(); sind++)
	{

#if DEBUG
	  int x_c, y_c, theta_c, t_c;
	  GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c, t_c);
	  SBPL_FPRINTF(fDeb, "succ: %d %d %d %d\n", x_c, y_c, theta_c, t_c); 
#endif

	  if(SuccIDV[sind] == targetID && CostV[sind] <= bestcost)
	    {
	      bestcost = CostV[sind];
	      bestsind = sind;
	    }
	}
      if(bestsind == -1)
	{
	  SBPL_ERROR("ERROR: successor not found for transition:\n");
	  GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c);
	  GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targett_c);
	  SBPL_PRINTF("%d %d %d %d -> %d %d %d %d\n", sourcex_c, sourcey_c, sourcetheta_c, sourcet_c,targetx_c, targety_c, targettheta_c, targett_c); 
	  throw new SBPL_Exception();
	}

      //now push in the actual path
      int sourcex_c, sourcey_c, sourcetheta_c;
      int sourcet_c;
      GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c);
      double sourcex, sourcey, sourcet;
      sourcex = DISCXY2CONT(sourcex_c, EnvNAVXYTHETATIMELATCfg.cellsize_m);
      sourcey = DISCXY2CONT(sourcey_c, EnvNAVXYTHETATIMELATCfg.cellsize_m);
      sourcet = DISCTIME2CONT(sourcet_c, EnvNAVXYTHETATIMELATCfg.timeResolution);
      //TODO - when there are no motion primitives we should still print source state
      for(int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size())-1; ipind++) 
	{
	  //translate appropriately
	  SBPL_4Dpt_t intermpt = actionV[bestsind]->intermptV[ipind];
	  intermpt.x += sourcex;
	  intermpt.y += sourcey;
	  intermpt.t += sourcet;

#if DEBUG
	  int nx = CONTXY2DISC(intermpt.x, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	  int ny = CONTXY2DISC(intermpt.y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	  int nt = CONTTIME2DISC(intermpt.t, EnvNAVXYTHETATIMELATCfg.timeResolution);
	  SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f %.3f (%d %d %d cost=%d) ", 
		       intermpt.x, intermpt.y, intermpt.theta, intermpt.t
		       nx, ny, 
		       ContTheta2Disc(intermpt.theta, NAVXYTHETATIMELAT_THETADIRS), nt, EnvNAVXYTHETATIMELATCfg.Grid2D[nx][ny]);
	  if(ipind == 0) SBPL_FPRINTF(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
	  else SBPL_FPRINTF(fDeb, "\n");
#endif

	  //store
	  xythetaPath->push_back(intermpt);
	}
    }
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAVXYTHETATIMELAT_AD::SetGoal(double x_m, double y_m, double theta_rad){

	int x = CONTXY2DISC(x_m, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	int theta = ContTheta2Disc(theta_rad, NAVXYTHETATIMELAT_THETADIRS);

	SBPL_PRINTF("env: setting goal to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

	if(!IsWithinMapCell(x,y))
	{
		SBPL_ERROR("ERROR: trying to set a goal cell %d %d that is outside of map\n", x,y);
		return -1;
	}

    if(!IsValidConfiguration(x,y,theta))
	{
		SBPL_PRINTF("WARNING: goal configuration is invalid\n");
	}

    EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
    if((OutHashEntry = (this->*GetHashEntry)(x, y, theta, INFINITECOST)) == NULL){
        //have to create a new entry
      OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta, INFINITECOST);
    }

	//need to recompute start heuristics?
	//if(EnvNAVXYTHETATIMELAT.goalstateid != OutHashEntry->stateID)
  int oldGoalX, oldGoalY, oldGoalTheta;
  int oldGoalT;
  GetCoordFromState(EnvNAVXYTHETATIMELAT.goalstateid, oldGoalX, oldGoalY, oldGoalTheta, oldGoalT);
  if(oldGoalX != x || oldGoalY != y || oldGoalTheta != theta)
	{
		bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
		bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
	}



    EnvNAVXYTHETATIMELAT.goalstateid = OutHashEntry->stateID;

	EnvNAVXYTHETATIMELATCfg.EndX_c = x;
	EnvNAVXYTHETATIMELATCfg.EndY_c = y;
	EnvNAVXYTHETATIMELATCfg.EndTheta = theta;


    return EnvNAVXYTHETATIMELAT.goalstateid;    

}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAVXYTHETATIMELAT_AD::SetStart(double x_m, double y_m, double theta_rad, double startTime){

  int x = CONTXY2DISC(x_m, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  int y = CONTXY2DISC(y_m, EnvNAVXYTHETATIMELATCfg.cellsize_m); 
  int theta = ContTheta2Disc(theta_rad, NAVXYTHETATIMELAT_THETADIRS);
  int t = CONTTIME2DISC(startTime, EnvNAVXYTHETATIMELATCfg.timeResolution);

  if(!IsWithinMapCell(x,y))
    {
      SBPL_ERROR("ERROR: trying to set a start cell %d %d that is outside of map\n", x,y);
      return -1;
    }

  SBPL_PRINTF("env: setting start to %.3f %.3f %.3f %.3f (%d %d %d %d)\n", x_m, y_m, theta_rad, startTime, x, y, theta, t);

  if(!IsValidConfiguration(x,y,theta))
    {
      SBPL_PRINTF("WARNING: start configuration %d %d %d is invalid\n", x,y,theta);
    }

  EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
  if((OutHashEntry = (this->*GetHashEntry)(x, y, theta, t)) == NULL){
    //have to create a new entry
    OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta, t);
  }

  //need to recompute start heuristics?
  //if(EnvNAVXYTHETATIMELAT.startstateid != OutHashEntry->stateID)
  int oldStartX, oldStartY, oldStartTheta;
  int oldStartT;
  GetCoordFromState(EnvNAVXYTHETATIMELAT.startstateid, oldStartX, oldStartY, oldStartTheta, oldStartT);
  if(oldStartX != x || oldStartY != y || oldStartTheta != theta)
    {
      bNeedtoRecomputeStartHeuristics = true;
      bNeedtoRecomputeGoalHeuristics = true; //because termination condition can be not all states TODO - make it dependent on term. condition
    }

  //set start
  EnvNAVXYTHETATIMELAT.startstateid = OutHashEntry->stateID;
  EnvNAVXYTHETATIMELATCfg.StartX_c = x;
  EnvNAVXYTHETATIMELATCfg.StartY_c = y;
  EnvNAVXYTHETATIMELATCfg.StartTheta = theta;
  EnvNAVXYTHETATIMELATCfg.StartTime = t;

  return EnvNAVXYTHETATIMELAT.startstateid;    

}

bool EnvironmentNAVXYTHETATIMELAT_AD::setDynamicObstacles(vector<SBPL_DynamicObstacle_t> dynObs, bool reset_states){
  dynamicObstacles.clear();
  for(unsigned int i=0; i<dynObs.size(); i++){
    SBPL_DynamicObstacle_t obs;
    obs.radius = dynObs[i].radius + EnvNAVXYTHETATIMELATCfg.robotRadius;
    for(unsigned int j=0; j<dynObs[i].trajectories.size(); j++){
      SBPL_Trajectory_t traj;
      traj.prob = dynObs[i].trajectories[j].prob;
      traj.existsAfter = dynObs[i].trajectories[j].existsAfter;
      for(unsigned int k=0; k<dynObs[i].trajectories[j].points.size(); k++){
        SBPL_Traj_Pt_t p;
        p.x = dynObs[i].trajectories[j].points[k].x;
        p.y = dynObs[i].trajectories[j].points[k].y;
        p.t = dynObs[i].trajectories[j].points[k].t;
        p.std_dev = dynObs[i].trajectories[j].points[k].std_dev;
        traj.points.push_back(p);
      }
      obs.trajectories.push_back(traj);
    }
    dynamicObstacles.push_back(obs);
  }
  SBPL_PRINTF("we have %d dynamic obstacles\n", (int)dynamicObstacles.size());
  SBPL_Traj_Pt_t p1 = dynamicObstacles[0].trajectories[0].points[0];
  SBPL_Traj_Pt_t p2 = dynamicObstacles[0].trajectories[0].points[dynamicObstacles[0].trajectories[0].points.size()-1];
  SBPL_PRINTF("start=(%f, %f, %d) end=(%f, %f, %d)\n", p1.x, p1.y, p1.t, p2.x, p2.y, p2.t);

  return UpdateTimelineMap();
}

EnvNAVXYTHETATIMELATHashEntry_t* EnvironmentNAVXYTHETATIMELAT_AD::getEntryFromID(int id){
  return StateID2CoordTable[id];
}

void EnvironmentNAVXYTHETATIMELAT_AD::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
  if(stateID >= (int)StateID2CoordTable.size())
    {
      SBPL_ERROR("ERROR in EnvNAVXYTHETATIMELAT... function: stateID illegal (2)\n");
      throw new SBPL_Exception();
    }
#endif

  if(fOut == NULL)
    fOut = stdout;

  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  EnvNAVXYTHETATIMELATHashEntry_t* GoalHashEntry = StateID2CoordTable[EnvNAVXYTHETATIMELAT.goalstateid];

  if(HashEntry->X == GoalHashEntry->X && HashEntry->Y == GoalHashEntry->Y && HashEntry->Theta == GoalHashEntry->Theta && bVerbose)
    {
      SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

  if(bVerbose)
    SBPL_FPRINTF(fOut, "X=%d Y=%d Theta=%d T=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T);
  else
    SBPL_FPRINTF(fOut, "%.3f %.3f %.3f %.3f\n", DISCXY2CONT(HashEntry->X, EnvNAVXYTHETATIMELATCfg.cellsize_m), DISCXY2CONT(HashEntry->Y,EnvNAVXYTHETATIMELATCfg.cellsize_m), 
		 DiscTheta2Cont(HashEntry->Theta, NAVXYTHETATIMELAT_THETADIRS), DISCTIME2CONT(HashEntry->T, EnvNAVXYTHETATIMELATCfg.timeResolution));

}

EnvNAVXYTHETATIMELATHashEntry_t* EnvironmentNAVXYTHETATIMELAT_AD::GetHashEntry_lookup(int X, int Y, int Theta, int T)
{

  int index = XYTHETATIME2INDEX(X,Y,Theta, T);	
  return Coord2StateIDHashTable_lookup[index];

}

EnvNAVXYTHETATIMELATHashEntry_t* EnvironmentNAVXYTHETATIMELAT_AD::GetHashEntry_hash(int X, int Y, int Theta, int T)
{

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

  //T = 0;
	int binid = GETHASHBIN(X, Y, Theta, T);

#if DEBUG
	if ((int)Coord2StateIDHashTable[binid].size() > 500)
	  {
	    SBPL_PRINTF("WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n", 
			binid, X, Y, Coord2StateIDHashTable[binid].size());
	    
	    PrintHashTableHist();		
	  }
#endif

	//iterate over the states in the bin and select the perfect match
	for(int ind = 0; ind < (int)Coord2StateIDHashTable[binid].size(); ind++)
	  {
		if( Coord2StateIDHashTable[binid][ind]->X == X 
		    && Coord2StateIDHashTable[binid][ind]->Y == Y
		    && Coord2StateIDHashTable[binid][ind]->Theta == Theta
		    && Coord2StateIDHashTable[binid][ind]->T == T)
		  {
#if TIME_DEBUG
		    time_gethash += clock()-currenttime;
#endif
		    return Coord2StateIDHashTable[binid][ind];
		  }
	  }
		    
#if TIME_DEBUG	
		    time_gethash += clock()-currenttime;
#endif

	return NULL;	  
}

 EnvNAVXYTHETATIMELATHashEntry_t* EnvironmentNAVXYTHETATIMELAT_AD::CreateNewHashEntry_lookup(int X, int Y, int Theta, int T) 
{
  int i;

#if TIME_DEBUG	
  clock_t currenttime = clock();
#endif

  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = new EnvNAVXYTHETATIMELATHashEntry_t;

  HashEntry->X = X;
  HashEntry->Y = Y;
  HashEntry->Theta = Theta;
  HashEntry->T = T;
  HashEntry->iteration = 0;

  HashEntry->stateID = StateID2CoordTable.size();

  //insert into the tables
  StateID2CoordTable.push_back(HashEntry);

  int index = XYTHETATIME2INDEX(X,Y,Theta, T);

#if DEBUG
  if(Coord2StateIDHashTable_lookup[index] != NULL)
    {
      printf("ERROR: creating hash entry for non-NULL hashentry\n");
      system("pause");
      exit(1);
    }
#endif

  Coord2StateIDHashTable_lookup[index] = HashEntry;

  //insert into and initialize the mappings
  int* entry = new int [NUMOFINDICES_STATEID2IND]; 
  StateID2IndexMapping.push_back(entry);
  for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
    {
      StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

  if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
    {
      printf("ERROR in Env... function: last state has incorrect stateID\n");
      int returnPointer = system("pause");
      exit(1);	
    }

#if TIME_DEBUG
  time_createhash += clock()-currenttime;
#endif

  return HashEntry;
}

EnvNAVXYTHETATIMELATHashEntry_t* EnvironmentNAVXYTHETATIMELAT_AD::CreateNewHashEntry_hash(int X, int Y, int Theta, int T) 
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = new EnvNAVXYTHETATIMELATHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;
	HashEntry->Theta = Theta;
  HashEntry->T = T;
	HashEntry->iteration = 0;

	HashEntry->stateID = StateID2CoordTable.size();

	//insert into the tables
	StateID2CoordTable.push_back(HashEntry);


	//get the hash table bin
	i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T); 

	//insert the entry into the bin
  Coord2StateIDHashTable[i].push_back(HashEntry);

	//insert into and initialize the mappings
	int* entry = new int [NUMOFINDICES_STATEID2IND];
	StateID2IndexMapping.push_back(entry);
	for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
	{
		StateID2IndexMapping[HashEntry->stateID][i] = -1;
	}

	if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
	{
		SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
		throw new SBPL_Exception();	
	}

#if TIME_DEBUG
	time_createhash += clock()-currenttime;
#endif

	return HashEntry;
}

void EnvironmentNAVXYTHETATIMELAT_AD::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETATIMELATAction_t*>* actionV /*=NULL*/)
{
  int aind;

#if TIME_DEBUG
  clock_t currenttime = clock();
#endif

  //clear the successor array
  SuccIDV->clear();
  CostV->clear();
  SuccIDV->reserve(EnvNAVXYTHETATIMELATCfg.actionwidth); 
  CostV->reserve(EnvNAVXYTHETATIMELATCfg.actionwidth);
	if(actionV != NULL)
	{
		actionV->clear();
		actionV->reserve(EnvNAVXYTHETATIMELATCfg.actionwidth);
	}

	//get X, Y for the state
	EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
	EnvNAVXYTHETATIMELATHashEntry_t* GoalHashEntry = StateID2CoordTable[EnvNAVXYTHETATIMELAT.goalstateid];
	
  //SBPL_PRINTF("stateid=%d x=%d, y=%d, theta=%d, t=%d\n",SourceStateID,HashEntry->X,HashEntry->Y,HashEntry->Theta,HashEntry->T);
  if(SourceStateID == EnvNAVXYTHETATIMELAT.goalstateid){
    //SBPL_PRINTF("whoa! there went the goal!\n");
    return;
  }

	//iterate through actions
  bool bTestBounds = false;
  if(HashEntry->X == 0 || HashEntry->X == EnvNAVXYTHETATIMELATCfg.EnvWidth_c-1 ||  //TODO - need to modify to take robot perimeter into account
     HashEntry->Y == 0 || HashEntry->Y == EnvNAVXYTHETATIMELATCfg.EnvHeight_c-1)
      bTestBounds = true;

	for (aind = 0; aind < EnvNAVXYTHETATIMELATCfg.actionwidth; aind++)
	{
    //compute new pose from this aind action (successor pose)
		EnvNAVXYTHETATIMELATAction_t* nav4daction = &EnvNAVXYTHETATIMELATCfg.ActionsV[(int)HashEntry->Theta][aind];
    int newX = HashEntry->X + nav4daction->dX;
		int newY = HashEntry->Y + nav4daction->dY;
    int newT = HashEntry->T + nav4daction->dT;
		int newTheta = NORMALIZEDISCTHETA(nav4daction->endtheta, NAVXYTHETATIMELAT_THETADIRS);	

    //skip the invalid cells
		if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
      if(!IsValidCell(newX, newY)) 
        continue;
    }

		//get cost
		int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav4daction);
    if(cost >= INFINITECOST)
      continue;

    EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
    //TODO:Mike let this have some close enough threshold
    if(newX == GoalHashEntry->X &&
       newY == GoalHashEntry->Y &&
       newTheta == GoalHashEntry->Theta){
      //SBPL_PRINTF("the goal has been spotted!\n");
      OutHashEntry = GoalHashEntry;
    }
    else if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
      {
	//have to create a new entry
	OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT);
      }
    
    SuccIDV->push_back(OutHashEntry->stateID);
    //SBPL_PRINTF("    stateID:=%d x=%d y=%d theta=%d t=%d h=%d cost=%d\n", OutHashEntry->stateID, newX, newY, newTheta, newT, GetGoalHeuristic(OutHashEntry->stateID), cost);
    CostV->push_back(cost);
    if(actionV != NULL)
      actionV->push_back(nav4daction);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif

}

void EnvironmentNAVXYTHETATIMELAT_AD::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{

	//TODO- to support tolerance, need: a) generate preds for goal state based on all possible goal state variable settings,
	//b) change goal check condition in gethashentry c) change getpredsofchangedcells and getsuccsofchangedcells functions

    int aind;

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(EnvNAVXYTHETATIMELATCfg.actionwidth); 
    CostV->reserve(EnvNAVXYTHETATIMELATCfg.actionwidth);

	//get X, Y for the state
	EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[TargetStateID];

	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAVXYTHETATIMELATCfg.EnvWidth_c-1 || //TODO - need to modify to take robot perimeter into account
       HashEntry->Y == 0 || HashEntry->Y == EnvNAVXYTHETATIMELATCfg.EnvHeight_c-1)
      bTestBounds = true;

	for (aind = 0; aind < (int)EnvNAVXYTHETATIMELATCfg.PredActionsV[(int)HashEntry->Theta].size(); aind++)
	  {
	    
	    EnvNAVXYTHETATIMELATAction_t* nav4daction = EnvNAVXYTHETATIMELATCfg.PredActionsV[(int)HashEntry->Theta].at(aind);
	    
	    int predX = HashEntry->X - nav4daction->dX;
	    int predY = HashEntry->Y - nav4daction->dY;
	    int predTheta = nav4daction->starttheta;	
	    int predT = HashEntry->T - nav4daction->dT;
	    
	    
	    //skip the invalid cells
	    if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
	      if(!IsValidCell(predX, predY))
                continue;
	    }
	    
	    //get cost
	    int cost = GetActionCost(predX, predY, predTheta, predT, nav4daction);
	    if(cost >= INFINITECOST)
	      continue;
	    
	    EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
	    if((OutHashEntry = (this->*GetHashEntry)(predX, predY, predTheta, predT)) == NULL)
	      {
		//have to create a new entry
		OutHashEntry = (this->*CreateNewHashEntry)(predX, predY, predTheta, predT);
	      }
	    
	    PredIDV->push_back(OutHashEntry->stateID);
	    CostV->push_back(cost);
	  }

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif


}

void EnvironmentNAVXYTHETATIMELAT_AD::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{

	int cost;

#if DEBUG
	if(state->StateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in Env... function: stateID illegal\n");
		throw new SBPL_Exception();
	}

	if((int)state->Actions.size() != 0)
	{
		SBPL_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
		throw new SBPL_Exception();
	}
#endif
	

	//get X, Y for the state
	EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[state->StateID];
	EnvNAVXYTHETATIMELATHashEntry_t* GoalHashEntry = StateID2CoordTable[EnvNAVXYTHETATIMELAT.goalstateid];

	//goal state should be absorbing
  if(HashEntry->X == GoalHashEntry->X && HashEntry->Y == GoalHashEntry->Y && HashEntry->Theta == GoalHashEntry->Theta)
		return;
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAVXYTHETATIMELATCfg.EnvWidth_c-1 || //TODO - modify based on robot's size
       HashEntry->Y == 0 || HashEntry->Y == EnvNAVXYTHETATIMELATCfg.EnvHeight_c-1)
        bTestBounds = true;
	for (int aind = 0; aind < EnvNAVXYTHETATIMELATCfg.actionwidth; aind++)
	{
		EnvNAVXYTHETATIMELATAction_t* nav4daction = &EnvNAVXYTHETATIMELATCfg.ActionsV[(int)HashEntry->Theta][aind];
    int newX = HashEntry->X + nav4daction->dX;
		int newY = HashEntry->Y + nav4daction->dY;
		int newTheta = NORMALIZEDISCTHETA(nav4daction->endtheta, NAVXYTHETATIMELAT_THETADIRS);	
    int newT = HashEntry->T + nav4daction->dT;

        //skip the invalid cells
        if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(newX, newY))
                continue;
        }

		//get cost
		cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav4daction);
        if(cost >= INFINITECOST)
            continue;

		//add the action
		CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

    	EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
	if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
		{
			//have to create a new entry
		  OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT);
		}
		action->AddOutcome(OutHashEntry->stateID, cost, 1.0); 

#if TIME_DEBUG
		time3_addallout += clock()-currenttime;
#endif

	}
}

void EnvironmentNAVXYTHETATIMELAT_AD::GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV)
{
	nav2dcell_t cell;
	SBPL_4Dcell_t affectedcell;
	EnvNAVXYTHETATIMELATHashEntry_t* affectedHashEntry;

	//increment iteration for processing savings
	iteration++;

	for(int i = 0; i < (int)changedcellsV->size(); i++) 
	{
		cell = changedcellsV->at(i);
			
		//now iterate over all states that could potentially be affected
		for(int sind = 0; sind < (int)affectedpredstatesV.size(); sind++)
		{
			affectedcell = affectedpredstatesV.at(sind);

			//translate to correct for the offset
			affectedcell.x = affectedcell.x + cell.x;
			affectedcell.y = affectedcell.y + cell.y;

			//insert only if it was actually generated
			affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta, affectedcell.t);
			if(affectedHashEntry != NULL && affectedHashEntry->iteration < iteration)
			{
				preds_of_changededgesIDV->push_back(affectedHashEntry->stateID);
				affectedHashEntry->iteration = iteration; //mark as already inserted
			}
		}
	}
}

void EnvironmentNAVXYTHETATIMELAT_AD::GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV)
{
	nav2dcell_t cell;
	SBPL_4Dcell_t affectedcell;
	EnvNAVXYTHETATIMELATHashEntry_t* affectedHashEntry;

	SBPL_ERROR("ERROR: getsuccs is not supported currently\n");
	throw new SBPL_Exception();

	//increment iteration for processing savings
	iteration++;

	//TODO - check
	for(int i = 0; i < (int)changedcellsV->size(); i++) 
	{
		cell = changedcellsV->at(i);
			
		//now iterate over all states that could potentially be affected
		for(int sind = 0; sind < (int)affectedsuccstatesV.size(); sind++)
		{
			affectedcell = affectedsuccstatesV.at(sind);

			//translate to correct for the offset
			affectedcell.x = affectedcell.x + cell.x;
			affectedcell.y = affectedcell.y + cell.y;

			//insert only if it was actually generated
			affectedHashEntry = (this->*GetHashEntry)(affectedcell.x, affectedcell.y, affectedcell.theta, affectedcell.t);
			if(affectedHashEntry != NULL && affectedHashEntry->iteration < iteration)
			{
				succs_of_changededgesIDV->push_back(affectedHashEntry->stateID);
				affectedHashEntry->iteration = iteration; //mark as already inserted
			}
		}
	}
}

void EnvironmentNAVXYTHETATIMELAT_AD::InitializeEnvironment()
{
	EnvNAVXYTHETATIMELATHashEntry_t* HashEntry;

	inflatedGrid = (unsigned char**) malloc (EnvNAVXYTHETATIMELATCfg.EnvWidth_c * sizeof(unsigned char*));
	for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
		inflatedGrid[i] = (unsigned char*) malloc (EnvNAVXYTHETATIMELATCfg.EnvHeight_c * sizeof(unsigned char));
	}

	disttoObs_incells = (float**) malloc (EnvNAVXYTHETATIMELATCfg.EnvWidth_c * sizeof(float*));
	for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
		disttoObs_incells[i] = (float*) malloc (EnvNAVXYTHETATIMELATCfg.EnvHeight_c * sizeof(float));
	}

	disttoNonfree_incells = (float**) malloc (EnvNAVXYTHETATIMELATCfg.EnvWidth_c * sizeof(float*));
	for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
		disttoNonfree_incells[i] = (float*) malloc (EnvNAVXYTHETATIMELATCfg.EnvHeight_c * sizeof(float));
	}

	int maxsize = EnvNAVXYTHETATIMELATCfg.EnvWidth_c*EnvNAVXYTHETATIMELATCfg.EnvHeight_c*(NAVXYTHETALAT_THETADIRS);

	// @avemula NOTE Making sure that hashtable is always used and not lookup since time dimension is unbounded
	if(maxsize <= SBPL_XYTHETALAT_MAXSTATESFORLOOKUP && false)
	{
		printf("environment stores states in lookup table\n");

		Coord2StateIDHashTable_lookup = new EnvNAVXYTHETATIMELATHashEntry_t*[maxsize]; 
		for(int i = 0; i < maxsize; i++)
			Coord2StateIDHashTable_lookup[i] = NULL;
		GetHashEntry = &EnvironmentNAVXYTHETATIMELAT_AD::GetHashEntry_lookup;
		CreateNewHashEntry = &EnvironmentNAVXYTHETATIMELAT_AD::CreateNewHashEntry_lookup;
		
		//not using hash table
		HashTableSize = 0;
		Coord2StateIDHashTable = NULL;
	}
	else
	{		
		printf("environment stores states in hashtable\n");

		printf("Should NOT come here!\n");

		//initialize the map from Coord to StateID
		HashTableSize = 4*1024*1024; //should be power of two 
		Coord2StateIDHashTable = new vector<EnvNAVXYTHETATIMELATHashEntry_t*>[HashTableSize]; 
		GetHashEntry = &EnvironmentNAVXYTHETATIMELAT_AD::GetHashEntry_hash;
		CreateNewHashEntry = &EnvironmentNAVXYTHETATIMELAT_AD::CreateNewHashEntry_hash;

		//not using hash
		Coord2StateIDHashTable_lookup = NULL;
	}


	//initialize the map from StateID to Coord
	StateID2CoordTable.clear();

	//create start state 
	HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c, EnvNAVXYTHETATIMELATCfg.StartTheta, EnvNAVXYTHETATIMELATCfg.StartTime);
	EnvNAVXYTHETATIMELAT.startstateid = HashEntry->stateID;

	//create goal state 
	HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c, EnvNAVXYTHETATIMELATCfg.EndTheta, INFINITECOST);
	EnvNAVXYTHETATIMELAT.goalstateid = HashEntry->stateID;

	//inflate obstacles for 2D planning
	computeDistancestoNonfreeAreas(EnvNAVXYTHETATIMELATCfg.Grid2D, EnvNAVXYTHETATIMELATCfg.EnvWidth_c, EnvNAVXYTHETATIMELATCfg.EnvHeight_c, EnvNAVXYTHETATIMELATCfg.obsthresh, disttoObs_incells, disttoNonfree_incells);

	float rad = min( fabs(EnvNAVXYTHETATIMELATCfg.FootprintPolygon[0].x - EnvNAVXYTHETATIMELATCfg.FootprintPolygon[2].x) / 2.0, 
		fabs(EnvNAVXYTHETATIMELATCfg.FootprintPolygon[0].y - EnvNAVXYTHETATIMELATCfg.FootprintPolygon[2].y) / 2.0) + EnvNAVXYTHETATIMELATCfg.cellsize_m / 2.0;
	rad = rad / EnvNAVXYTHETATIMELATCfg.cellsize_m;
	for(int w = 0; w < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; w++){
		for(int h = 0; h < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; h++){
			if(disttoObs_incells[w][h] >= rad){
				inflatedGrid[w][h] = 0;
			} else {
				inflatedGrid[w][h] = EnvNAVXYTHETATIMELATCfg.obsthresh;
			}
		}
	}

	//initialized
	EnvNAVXYTHETATIMELAT.bInitialized = true;

}

unsigned int EnvironmentNAVXYTHETATIMELAT_AD::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta, unsigned int T)
{

  return inthash(inthash(X1)+(inthash(X2)<<1)+(inthash(Theta)<<2)+(inthash(T)<<3)) & (HashTableSize-1);
}

void EnvironmentNAVXYTHETATIMELAT_AD::PrintHashTableHist(FILE* fOut)
{
	int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

	for(int  j = 0; j < HashTableSize; j++)
	{
	  if((int)Coord2StateIDHashTable[j].size() == 0)
			s0++;
		else if((int)Coord2StateIDHashTable[j].size() < 50)
			s1++;
		else if((int)Coord2StateIDHashTable[j].size() < 100)
			s50++;
		else if((int)Coord2StateIDHashTable[j].size() < 200)
			s100++;
		else if((int)Coord2StateIDHashTable[j].size() < 300)
			s200++;
		else if((int)Coord2StateIDHashTable[j].size() < 400)
			s300++;
		else
			slarge++;
	}
	SBPL_PRINTF("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n",
		s0,s1, s50, s100, s200,s300,slarge);
}

int EnvironmentNAVXYTHETATIMELAT_AD::GetFromToHeuristic(int FromStateID, int ToStateID)
{

#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(FromStateID >= (int)StateID2CoordTable.size() 
		|| ToStateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in EnvNAVXYTHETATIMELAT... function: stateID illegal\n");
		throw new SBPL_Exception();
	}
#endif

	//get X, Y for the state
	EnvNAVXYTHETATIMELATHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
	EnvNAVXYTHETATIMELATHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];
	
	//TODO - check if one of the gridsearches already computed and then use it.
	

	return (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y)/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs);	

}

int EnvironmentNAVXYTHETATIMELAT_AD::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		SBPL_ERROR("ERROR in EnvNAVXYTHETATIMELAT... function: stateID illegal\n");
		throw new SBPL_Exception();
	}
#endif

	if(bNeedtoRecomputeGoalHeuristics)
	{
		grid2Dsearchfromgoal->search(EnvNAVXYTHETATIMELATCfg.Grid2D, EnvNAVXYTHETATIMELATCfg.cost_inscribed_thresh, 
			EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c, EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c,  
			SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
		bNeedtoRecomputeGoalHeuristics = false;
		SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c)
			/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs));

#if DEBUG
		PrintHeuristicValues();
#endif

	}

	EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y); //computes distances from start state that is grid2D, so it is EndX_c EndY_c 
	int hEuclid = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EuclideanDistance_m(HashEntry->X, HashEntry->Y, EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c));
		

#if DEBUG
	SBPL_FPRINTF(fDeb, "h2D = %d hEuclid = %d\n", h2D, hEuclid);
#endif

	//define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D,hEuclid))/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs); 

}

int EnvironmentNAVXYTHETATIMELAT_AD::GetStartHeuristic(int stateID)
{


#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(stateID >= (int)StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
		system("pause");
		exit(1);
	}
#endif


	if(bNeedtoRecomputeStartHeuristics)
	{
		grid2Dsearchfromstart->search(inflatedGrid, EnvNAVXYTHETATIMELATCfg.cost_inscribed_thresh, 
			EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c, EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c, 
			SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH); 
		bNeedtoRecomputeStartHeuristics = false;
		printf("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c)
			/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs));

#if DEBUG
		//PrintHeuristicValues();
#endif

	}

	EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
	int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
	int hEuclid = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EuclideanDistance_m(EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c, HashEntry->X, HashEntry->Y));
		
	//define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D,hEuclid))/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs); 

}

int EnvironmentNAVXYTHETATIMELAT_AD::SizeofCreatedEnv()
{
	return (int)StateID2CoordTable.size();
	
}
