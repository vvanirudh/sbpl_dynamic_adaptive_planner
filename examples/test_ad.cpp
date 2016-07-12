/**

   Author : Anirudh Vemula
   Date : 10th March 2016

   A sample run file to test the dynamic AD planner functionality to check
   if the imlementation works without any bugs and stuff 

*/

#include <iostream>
#include <string.h>
#include <math.h>
#include <vector>
using namespace std;

#include <sbpl/headers.h>
#include <sbpl_dynamic_adaptive_planner/headers.h>

#define PRINT_SPREAD_STATS 0
#define TEMPORAL_PADDING 1

// @avemula ADDED the following parameters from AD
#define DRAWGRID
#define HALFWIDTH 0.025 * 15
#define HALFLENGTH 0.025 * 30
#define DRAWTRANSITIONS
#define AD_PLAN_EPS 1.5
#define AD_TRACK_EPS 1
#define AD_PLAN_TIMELIMIT 10.0
#define AD_TRACK_TIMELIMIT 20.0
#define TUNNEL_HALFWIDTH 0.10
#define F3D_EPS 1.5
#define DEFAULT_TIMEOUT 300.0

//#define RADIUS 0.4102 //make radius twice the longest 3d transition
//#define RADIUS 1.0204
#define RADIUS 0.4102

// @avemula ADDED the following parameters from SIPP
double halfwidth = 0;//0.025;
double halflength = 0;//0.025;
double allocated_time_secs = 300.0; //in seconds
bool bsearchuntilfirstsolution = true;
double initialEps = 1.0;
double dec_eps = 1.0;
bool bforwardsearch = true;
double timeRes = 0.1;


EnvironmentNAVXYTHETATIMELATAD *environment_navxythetatimelatad;

int TIME = 0;
int elapsed_time = 0;

int size_x, size_y;
double startx_d, starty_d, starttheta_d, starttime_d, goalx_d, goaly_d, goaltheta_d, cellsize, timeResolution, nominalvel_d, timetoturn45degsinplace_d;
unsigned char obsthresh_uc, dynobsthresh_uc;

std::vector<SBPL_4Dpt_t> tr_path;
std::vector<SBPL_4Dpt_t> ad_path;

std::vector<SBPL_4Dpt_t> exec_traj;
bool exec_traj_lock;
int exec_traj_idx;

// @avemula ADDED all the prototypes here
int getNumDynObsCollisions(vector<SBPL_DynamicObstacle_t> dynObs, double px, double py, int pt);
vector<SBPL_DynamicObstacle_t> ReadDynamicObstacles(char* dynObs_filename);
int planxythetatimelat_ad(int argc, char *argv[], vector<SBPL_DynamicObstacle_t> dynObs);
float computeXYTPathCost(std::vector<SBPL_4Dpt_t > *xythetaPath);
float pointToLineDist(SBPL_4Dpt_t p, SBPL_4Dpt_t q1, SBPL_4Dpt_t q2);
// @avemula NOTE didn't add planCallback function which had just visualizing stuff

void PrintUsage(char *argv[])
{
	printf("USAGE: %s <map file> <mprim file> [robot width] [robot length] [subopt bound] [time per iteration limit]\n", argv[0]);
}

float eucDist(float x1, float y1, float x2, float y2){
	return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}


int getNumDynObsCollisions(vector<SBPL_DynamicObstacle_t> dynObs, double px, double py, int pt){
  int collisions = 0;
  for(unsigned int i=0; i < dynObs.size(); i++){
    SBPL_DynamicObstacle_t obs = dynObs.at(i);
    for(unsigned int j=0; j < obs.trajectories.size(); j++){
      SBPL_Trajectory_t traj = obs.trajectories.at(j);
      unsigned int k = 0;
      for(; k < traj.points.size(); k++){
        if(traj.points.at(k).t > pt)
          break;
      }

      if(k==0){
        //This trajectory hasn't even started yet because all of the points in the come after t
        //we assume since we were specifically given a trajectory that starts after the present
        //that the obstacle is irrelevant and for all practical purposes "doesn't exist yet"
        continue;
      }
      else if(k == traj.points.size()){
        //This dynamic obstacle has already finished its trajectory before time t.
        //Here we check if the user wanted to assume the obstacle remains at rest
        //in its final configuration or if it "doesn't exist" after its trajectory.
        if(!traj.existsAfter)
          continue;

        SBPL_Traj_Pt_t p = traj.points.at(k-1);
        double dist = (p.x-px)*(p.x-px) + (p.y-py)*(p.y-py);
        if(sqrt(dist) <= obs.radius + 3*p.std_dev){
          printf("collide rad=%f from k=%d at (%d %d %d)\n", obs.radius, k, CONTXY2DISC(traj.points.at(k).x, 0.025), CONTXY2DISC(traj.points.at(k).y, 0.025), traj.points.at(k).t);
          collisions++;
        }
      }
      else{
        //p1 and p2 are the points before and after cell (in terms of time) respectively
        bool hit = false;
        SBPL_Traj_Pt_t p1, p2;
        p1 = traj.points.at(k-1);
        double dist = (p1.x-px)*(p1.x-px) + (p1.y-py)*(p1.y-py);
        if(abs(traj.points.at(k-1).t-pt) <= TEMPORAL_PADDING && sqrt(dist) <= obs.radius + 3*p1.std_dev)
          hit = true;
        p2 = traj.points.at(k);
        dist = (p2.x-px)*(p2.x-px) + (p2.y-py)*(p2.y-py);
        if(abs(traj.points.at(k).t-pt) <= TEMPORAL_PADDING && sqrt(dist) <= obs.radius + 3*p2.std_dev)
          hit = true;

        if(hit){
          printf("collide rad=%f from k=%d at (%d %d %d), robot at (%d %d %d)\n", 
                 obs.radius, k, 
                 CONTXY2DISC(traj.points.at(k).x, 0.025), 
                 CONTXY2DISC(traj.points.at(k).y, 0.025), 
                 traj.points.at(k).t,
                 CONTXY2DISC(px, 0.025), 
                 CONTXY2DISC(py, 0.025), 
                 pt);
          collisions++;
        }
      }
      
    }
  }
  return collisions;
}

vector<SBPL_DynamicObstacle_t> ReadDynamicObstacles(char* dynObs_filename){
  FILE* fDynObs = fopen(dynObs_filename, "r");
	char sTemp[1024], sTemp1[1024];
  int iTemp;
  vector<SBPL_DynamicObstacle_t> dynamicObstacles;

  //printf("Reading Dynamic Obstacles...\n");

  //get the number of dynamic obstacles in the file
	if(fscanf(fDynObs, "%s", sTemp) != 1){
    printf("ERROR: ran out of dynamic obstacle file early\n");
    exit(1);
  }
	strcpy(sTemp1, "NumberOfDynamicObstacles:");
	if(strcmp(sTemp1, sTemp) != 0){
		printf("ERROR: dynamic obstacle file has incorrect format\n");
		printf("Expected %s got %s\n", sTemp1, sTemp);
		exit(1);
	}
	if(fscanf(fDynObs, "%s", sTemp) != 1){
    printf("ERROR: ran out of dynamic obstacle file early\n");
    exit(1);
  }
	int numObs = atoi(sTemp);

  //for each dynamic obstacle
  for(int i=0; i < numObs; i++){

    //check that the ID matches i
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      printf("ERROR: ran out of dynamic obstacle file early\n");
      exit(1);
    }
    strcpy(sTemp1, "DynamicObstacleID:");
    if(strcmp(sTemp1, sTemp) != 0){
      printf("ERROR: dynamic obstacle file has incorrect format\n");
      printf("Expected %s got %s\n", sTemp1, sTemp);
      exit(1);
    }
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      printf("ERROR: ran out of dynamic obstacle file early\n");
      exit(1);
    }
    iTemp = atoi(sTemp);
    if(iTemp != i){
      printf("ERROR: dynamic obstacle file has incorrect format\n");
      printf("Expected %d got %d\n", i, iTemp);
      exit(1);
    }
    SBPL_DynamicObstacle_t obs;

    //Read in the obstacle's radius
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      printf("ERROR: ran out of dynamic obstacle file early\n");
      exit(1);
    }
    strcpy(sTemp1, "ObstacleRadius:");
    if(strcmp(sTemp1, sTemp) != 0){
      printf("ERROR: dynamic obstacle file has incorrect format\n");
      printf("Expected %s got %s\n", sTemp1, sTemp);
      exit(1);
    }
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      printf("ERROR: ran out of dynamic obstacle file early\n");
      exit(1);
    }
    //obs.radiusSquared = atof(sTemp) + EnvNAVXYTHETATIMELATCfg.robotRadius;
    obs.radius = atof(sTemp) + sqrt(halfwidth*halfwidth + halflength*halflength);

    //read the number of trajectories for this obstacle
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      printf("ERROR: ran out of dynamic obstacle file early\n");
      exit(1);
    }
    strcpy(sTemp1, "NumberOfTrajectories:");
    if(strcmp(sTemp1, sTemp) != 0){
      printf("ERROR: dynamic obstacle file has incorrect format\n");
      printf("Expected %s got %s\n", sTemp1, sTemp);
      exit(1);
    }
    if(fscanf(fDynObs, "%s", sTemp) != 1){
      printf("ERROR: ran out of dynamic obstacle file early\n");
      exit(1);
    }
    int numTraj = atoi(sTemp);

    //for each trajectory
    double trajProbSum = 0;
    for(int j=0; j < numTraj; j++){
       
      //check that the ID matches j
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      strcpy(sTemp1, "TrajectoryID:");
      if(strcmp(sTemp1, sTemp) != 0){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("Expected %s got %s\n", sTemp1, sTemp);
        exit(1);
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      iTemp = atoi(sTemp);
      if(iTemp != j){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("Expected %d got %d\n", j, iTemp);
        exit(1);
      }
      SBPL_Trajectory_t traj;

      //read in this trajectory's probability
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      strcpy(sTemp1, "TrajectoryProbability:");
      if(strcmp(sTemp1, sTemp) != 0){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("Expected %s got %s\n", sTemp1, sTemp);
        exit(1);
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      traj.prob = atof(sTemp);
      if(traj.prob < 0 || traj.prob > 1){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("Expected TrajectoryProbability on the interval [0,1] but got %f\n", traj.prob);
        exit(1);
      }
      trajProbSum += traj.prob;

      //read the number of intermediate points are given for the trajectory
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      strcpy(sTemp1, "NumberOfPoints:");
      if(strcmp(sTemp1, sTemp) != 0){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("Expected %s got %s\n", sTemp1, sTemp);
        exit(1);
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      int numPoints = atoi(sTemp);

      //for each point
      int prev_t = 0;
      for(int k=0; k < numPoints; k++){
        //fill in the point
        SBPL_Traj_Pt_t pt;
        if(fscanf(fDynObs, "%s", sTemp) != 1){
          printf("ERROR: ran out of dynamic obstacle file early\n");
          exit(1);
        }
        pt.x = atof(sTemp);
        if(fscanf(fDynObs, "%s", sTemp) != 1){
          printf("ERROR: ran out of dynamic obstacle file early\n");
          exit(1);
        }
        pt.y = atof(sTemp);

	// @avemula ADDED an extra fscanf to account for the dummy variable in the dynObstFile - Z variable
	if(fscanf(fDynObs, "%s", sTemp) != 1){
          printf("ERROR: ran out of dynamic obstacle file early\n");
          exit(1);
        }
	
        if(fscanf(fDynObs, "%s", sTemp) != 1){
          printf("ERROR: ran out of dynamic obstacle file early\n");
          exit(1);
        }
        //pt.t = CONTTIME2DISC(atof(sTemp),EnvNAVXYTHETATIMELATCfg.timeResolution);
        pt.t = CONTTIME2DISC(atof(sTemp),timeRes);

        if(prev_t > pt.t && k != 0){
          printf("ERROR: dynamic obstacle file has incorrect format\n");
          printf("dynamic obstacle trajectory times can't decrease!\n");
          exit(1);
        }
        prev_t = pt.t;

        if(fscanf(fDynObs, "%s", sTemp) != 1){
          printf("ERROR: ran out of dynamic obstacle file early\n");
          exit(1);
        }
        pt.std_dev = atof(sTemp);

        //store the point in the trajectory
        traj.points.push_back(pt);
      }

      //check if the obstacle should "disappear" after it has finished its trajectory
      //or if it sits in the configuration from the last frame of the trajectory forever
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      strcpy(sTemp1, "ObstacleExistsAfterTrajectory:");
      if(strcmp(sTemp1, sTemp) != 0){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("Expected %s got %s\n", sTemp1, sTemp);
        exit(1);
      }
      if(fscanf(fDynObs, "%s", sTemp) != 1){
        printf("ERROR: ran out of dynamic obstacle file early\n");
        exit(1);
      }
      traj.existsAfter = atoi(sTemp);
      if(traj.existsAfter != 0 && traj.existsAfter != 1){
        printf("ERROR: dynamic obstacle file has incorrect format\n");
        printf("ObstacleExistsAfterTrajectory is a boolean and needs to be 0 or 1\n");
        exit(1);
      }

      //store the trajectory in the dynamic obstacle
      obs.trajectories.push_back(traj);
    }

    //check that the trajectory probabilities sum to 1
    if(fabs(trajProbSum - 1.0) > ERR_EPS){
      printf("ERROR: dynamic obstacle file has incorrect format\n");
      printf("Probabilities for trajectories of dynamic obstacle %d sum to %f instead of 1\n", i, trajProbSum);
      exit(1);
    }

    //store the dynamic obstacle into the dynamic obstacle vector
    dynamicObstacles.push_back(obs);

  }
/*
  for(int i=0; i<dynamicObstacles.size(); i++){
    printf("obs %d: radiusSquared=%f\n",i,dynamicObstacles[i].radiusSquared);
    for(int j=0; j<dynamicObstacles[i].trajectories.size(); j++){
      printf("  traj %d: prob=%f\n",j,dynamicObstacles[i].trajectories[j].prob);
      for(int k=0; k<dynamicObstacles[i].trajectories[j].points.size(); k++){
        SBPL_Traj_Pt_t p = dynamicObstacles[i].trajectories[j].points[k];
        printf("    point %d: x=%f y=%f t=%d std_dev=%f\n",k,p.x,p.y,p.t,p.std_dev);
      }
    }
  }
  */
  

  //printf("Done Reading Dynamic Obstacles\n");
  fclose(fDynObs);
  return dynamicObstacles;
}


float computeXYTPathCost(std::vector<SBPL_4Dpt_t > *xythetaPath){
	float cost = 0;
	for(unsigned int i = 1; i < xythetaPath->size(); i++){
		cost += eucDist(xythetaPath->at(i-1).x , xythetaPath->at(i-1).y, xythetaPath->at(i).x, xythetaPath->at(i).y);
	}
	return cost;
}

float pointToLineDist(SBPL_4Dpt_t p, SBPL_4Dpt_t q1, SBPL_4Dpt_t q2){
	//find the line through the qs : Ax + By + C = 0 | B = -1
	if (q2.x != q1.x) {
		float A = (q2.y - q1.y) / (q2.x - q1.x);
		float C = q2.y - A * q2.x;
		float dist = fabs(A * p.x - p.y + C) / sqrt(A*A + 1);
		return dist;
	} else if (q2.y !=q1.y) {
		//swap x and y axis
		float A = (q2.x - q1.x) / (q2.y - q1.y);
		float C = q2.x - A * q2.y;
		float dist = fabs(A * p.y - p.x + C) / sqrt(A*A + 1);
		return dist;
	} else {
		//the two q's are the same
		//so return the distance from p to q1
		return sqrt((p.x - q1.x)*(p.x - q1.x) + (p.y - q1.y)*(p.y - q1.y));
	}
}

int planxythetatimelat_ad(int argc, char *argv[], vector<SBPL_DynamicObstacle_t> dynObs) {

  AdaptivePlanner *planner;

  // Initializing environment and planner
  environment_navxythetatimelatad = new EnvironmentNAVXYTHETATIMELATAD();
  MDPConfig MDPCfg;

  printf("Creating robot perimeter..."); fflush(stdout);
  std::vector<sbpl_2Dpt_t> perimeterptsV;
  sbpl_2Dpt_t pt_m;
  //double halfwidth;
  //double halflength;
  float ad_plan_eps;
  float ad_plan_time_alloc;

  if(argc != 8) {
    halfwidth = 0.025;
    halflength = 0.025 * 2;
    ad_plan_eps = AD_PLAN_EPS;
    ad_plan_time_alloc = AD_PLAN_TIMELIMIT;
  } else {
    halfwidth = atof(argv[4]) * 0.025 / 2.0;
    halflength = atof(argv[5]) * 0.025 / 2.0;
    ad_plan_eps = atof(argv[6]);
    ad_plan_time_alloc = atof(argv[7]);
  }
  pt_m.x = -halflength;
  pt_m.y = -halfwidth;
  perimeterptsV.push_back(pt_m);
  pt_m.x = halflength;
  pt_m.y = -halfwidth;
  perimeterptsV.push_back(pt_m);
  pt_m.x = halflength;
  pt_m.y = halfwidth;
  perimeterptsV.push_back(pt_m);
  pt_m.x = -halflength;
  pt_m.y = halfwidth;
  perimeterptsV.push_back(pt_m);
  //perimeterptsV.clear();
  printf("done\n"); fflush(stdout);
	
  printf("Initializing environment..."); fflush(stdout);
  environment_navxythetatimelatad->Computedxy();

  if(argc == 4 || argc == 8)
    {
      printf("using mprims from [%s]...\n", argv[2]); fflush(stdout);
      if(!environment_navxythetatimelatad->InitializeEnv(argv[1], perimeterptsV, argv[2]))
	{
	  printf("ERROR: InitializeEnv failed\n");
	  throw new SBPL_Exception();
	}
    }
  else
    {
      printf("using default mprims...\n"); fflush(stdout);
      if(!environment_navxythetatimelatad->InitializeEnv(argv[1], perimeterptsV, NULL))
	{
	  printf("ERROR: InitializeEnv failed\n");
	  throw new SBPL_Exception();
	}
    }

  if(!environment_navxythetatimelatad->setDynamicObstacles(dynObs, false)){
    printf("The start state is in collision!\nSolution does not exist\n\n");
    throw new SBPL_Exception();
  }

  if(!environment_navxythetatimelatad->InitializeMDPCfg(&MDPCfg))
    {
      printf("ERROR: InitializeMDPCfg failed\n");
      throw new SBPL_Exception();
    }
  printf("done\n"); fflush(stdout);

  printf("Getting environment params..."); fflush(stdout);
  vector<SBPL_xythetatime_mprimitive> motionprimitiveV;
  environment_navxythetatimelatad->GetEnvParms(&size_x, &size_y, &startx_d, &starty_d, &starttheta_d, &starttime_d, &goalx_d, &goaly_d, &goaltheta_d, &cellsize, &timeResolution, &nominalvel_d, &timetoturn45degsinplace_d, &obsthresh_uc, &dynobsthresh_uc, &motionprimitiveV);
  printf("done!\n"); fflush(stdout);

  int start_t = clock();
  elapsed_time = clock() - start_t;

  bool bsearchuntilfirstsolution = true;
  //plan a path

  bool bforwardsearch = true; //Max said this should be true

  printf("Creating planner...");
  fflush(stdout);
  planner = new AdaptivePlanner(environment_navxythetatimelatad, bforwardsearch);
  planner->set_initialsolution_eps(ad_plan_eps);
  planner->set_new_sphere_radius(RADIUS);
  planner->set_tunnel_width(TUNNEL_HALFWIDTH);
  printf("done!\n");
  fflush(stdout);

  printf("Setting start state...");
  if(planner->set_start(MDPCfg.startstateid) == 0)
    {
      printf("ERROR: failed to set start state\n");
      exit(1);
    }
  #ifdef VISUALIZATIONS_ENABLED
  environment_navxythetatimelatad->visualizeState(MDPCfg.startstateid, 120, "Start state");
  #endif

  printf("Setting goal state...");
  if(planner->set_goal(MDPCfg.goalstateid) == 0)
    {
      printf("ERROR: failed to set goal state\n");
      exit(1);
    }
  #ifdef VISUALIZATIONS_ENABLED
  environment_navxythetatimelatad->visualizeState(MDPCfg.goalstateid, 240, "Goal state");
  #endif

  planner->set_search_mode(bsearchuntilfirstsolution);

  std::vector<int> solution_stateIDs_V;
  double allocated_time_secs = ad_plan_time_alloc;
  int bRet;
  elapsed_time = clock() - start_t;
  //environment_navxythetatimelatad->visualizeEnvironment();

  printf("Planning with %.2f sec. per iter (target eps: %.2f)...\n", allocated_time_secs, ad_plan_eps); fflush(stdout);

  bRet = planner->replan_new(300.0, &solution_stateIDs_V);

  printf("done planning\n");
  std::cout << "size of solution=" << solution_stateIDs_V.size() << std::endl;

  // @avemula DEBUG printing the cell coordinates of the waypoints
  for (int i=0; i<solution_stateIDs_V.size(); i++) {

    EnvNAVXYTHETATIMELATHashEntry_t * s = environment_navxythetatimelatad->getEntryFromID(solution_stateIDs_V[i]);
    printf("%d, %d, %d, %d, %d\n", s->X, s->Y, s->Theta, s->T, s->TempT);
    
    }

  environment_navxythetatimelatad->PrintTimeStat(stdout);
  
  vector<SBPL_4Dpt_t> xythetatimePath;
  environment_navxythetatimelatad->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetatimePath);
  printf("solution size=%d\n", (int)xythetatimePath.size());

  int collisions = 0;
  for(unsigned int pind=0; pind<xythetatimePath.size(); pind++){
    SBPL_4Dpt_t pt = xythetatimePath[pind];

    // @avemula DEBUG printing the xythetatimepath found
    printf("%.3f, %.3f, %.3f, %.3f \n", pt.x, pt.y, pt.theta, pt.t);
    fflush(stdout);
    
    collisions += getNumDynObsCollisions(dynObs,
					 DISCXY2CONT(CONTXY2DISC(pt.x, 0.025), 0.025), 
					 DISCXY2CONT(CONTXY2DISC(pt.y, 0.025), 0.025), 
					 CONTTIME2DISC(pt.t,timeRes));
  }
  printf("Number of collisions with dynamic obstacles = %d\n", collisions);

  if(bRet)
    printf("Solution is found\n");
  else
    printf("Solution does not exist\n");

  /*bool solutionFound;
  int numExpands;
  int solutionCost;
  double searchTime;
  planner->getSearchStats(&solutionFound, &numExpands, &solutionCost, &searchTime);
  printf("Solution Cost = %d\n", solutionCost);*/

  printf("\n");
  fflush(NULL);

    
  return bRet;
  
}

int main(int argc, char *argv[]) {

  // input format : argv[1] - env file
  // argv[2] - mprim file
  // argv[3] - dynObs file

  if (argc <= 2) {
    printf ("Incorrect usage. The correct usage is ./test_ad [env_file] [motprim_file] [dynObsFile]\n");
    return 0;
  }

  char *envFile = argv[1];
  char *mprimFile = argv[2];
  char *dynObsFile = argv[3];

  vector<SBPL_DynamicObstacle_t> dynObs = ReadDynamicObstacles(dynObsFile);

  int i = 0;
  char* c = NULL;
  ros::init(i, &c, "sbpl_dynamic_adaptive_planner");
  planxythetatimelat_ad(argc, argv, dynObs);

  return 0;

}
