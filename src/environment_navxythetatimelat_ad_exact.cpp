
/**

   Author : Anirudh Vemula
   Date : 17th March 2016

   Implements the functionality of the (x,y,theta,t) adaptive space.
   PART 2

*/

#include <vector>
#include <sbpl_dynamic_adaptive_planner/environment_navxythetatimelat.h>
#include <sbpl_dynamic_adaptive_planner/environment_navxythetatimelat_ad_exact.h>


// @avemula ADDED a check to do the backup (x,y,th)-(x,y) space or our setting (x,y,th,t)-(x,y,th) space
#define BACKUP_SPACE 0

using namespace std;

void EnvironmentNAVXYTHETATIMELATADEXACT::EnsureHeuristicsUpdated(bool bGoalHeuristics)
{
  if(bNeedtoRecomputeStartHeuristics && !bGoalHeuristics)
    {
      grid2Dsearchfromstart->search(inflatedGrid, EnvNAVXYTHETATIMELATCfg.obsthresh, 
				    EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c, EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c, 
				    SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS); 
      bNeedtoRecomputeStartHeuristics = false;
      SBPL_INFO("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c)/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs));

    }


  if(bNeedtoRecomputeGoalHeuristics && bGoalHeuristics)
    {
      grid2Dsearchfromgoal->search(inflatedGrid, EnvNAVXYTHETATIMELATCfg.obsthresh, 
				   EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c, EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c, SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS);
      bNeedtoRecomputeGoalHeuristics = false;
      SBPL_INFO("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c)/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs));

    }

}

void EnvironmentNAVXYTHETATIMELATADEXACT::ComputeHeuristicValues()
{
  //whatever necessary pre-computation of heuristic values is done here 
  SBPL_WARN("Precomputing heuristics...\n");
	
  //allocated 2D grid searches
  grid2Dsearchfromstart = new SBPL2DGridSearch(EnvNAVXYTHETATIMELATCfg.EnvWidth_c, EnvNAVXYTHETATIMELATCfg.EnvHeight_c, (float)EnvNAVXYTHETATIMELATCfg.cellsize_m);
  grid2Dsearchfromgoal = new SBPL2DGridSearch(EnvNAVXYTHETATIMELATCfg.EnvWidth_c, EnvNAVXYTHETATIMELATCfg.EnvHeight_c, (float)EnvNAVXYTHETATIMELATCfg.cellsize_m); 

  //set OPEN type to sliding buckets
  grid2Dsearchfromstart->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS); 
  grid2Dsearchfromgoal->setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS);
	
  bNeedtoRecomputeStartHeuristics = true;
  bNeedtoRecomputeGoalHeuristics = true;
	
  EnsureHeuristicsUpdated(true);
  EnsureHeuristicsUpdated(false);

  SBPL_PRINTF("done\n");
}

void EnvironmentNAVXYTHETATIMELATADEXACT::initStorageSpace(){
  overcons_vizcount = undercons_vizcount = 0;
  int maxsize = EnvNAVXYTHETATIMELATCfg.EnvWidth_c*EnvNAVXYTHETATIMELATCfg.EnvHeight_c*(NAVXYTHETATIMELAT_THETADIRS+1);
	
  //clear old space
  if(Coord2StateIDHashTable != NULL){
    printf("Why is it going in here %p\n", (void*)Coord2StateIDHashTable);
    for(int i = 0; i < HashTableSize; i++){
      for(unsigned int j = 0; j < Coord2StateIDHashTable[i].size(); j++)
	{
	  delete Coord2StateIDHashTable[i].at(j);
	}
    }
    delete[] Coord2StateIDHashTable;
    Coord2StateIDHashTable = NULL;
    }
  
  if(Coord2StateIDHashTable_lookup != NULL){
    for(int i = 0; i < maxsize; i++){
      delete Coord2StateIDHashTable_lookup[i];
    }
    delete[] Coord2StateIDHashTable_lookup;
    Coord2StateIDHashTable_lookup = NULL;
  }

  // @avemula NOTE Making sure that hashtable is always used and not lookup since time dimension is unbounded
  //init new space
  if(maxsize <= SBPL_XYTHETALAT_MAXSTATESFORLOOKUP && false)
    {
      printf("environment stores states in lookup table\n");

      Coord2StateIDHashTable_lookup = new EnvNAVXYTHETATIMELATHashEntry_t*[maxsize]; 
      for(int i = 0; i < maxsize; i++)
	Coord2StateIDHashTable_lookup[i] = NULL;
      GetHashEntry = &EnvironmentNAVXYTHETATIMELATADEXACT::GetHashEntry_lookup;
      CreateNewHashEntry = &EnvironmentNAVXYTHETATIMELATADEXACT::CreateNewHashEntry_lookup;
		
      //not using hash table
      HashTableSize = 0;
      Coord2StateIDHashTable = NULL;
    }
  else
    {		
      printf("environment stores states in hashtable\n");
      fflush(stdout);
      //initialize the map from Coord to StateID
      HashTableSize = 4*1024*1024; //should be power of two 
      Coord2StateIDHashTable = new std::vector<EnvNAVXYTHETATIMELATHashEntry_t*>[HashTableSize]; 
      GetHashEntry = &EnvironmentNAVXYTHETATIMELATADEXACT::GetHashEntry_hash;
      CreateNewHashEntry = &EnvironmentNAVXYTHETATIMELATADEXACT::CreateNewHashEntry_hash;

      //not using hash
      Coord2StateIDHashTable_lookup = NULL;
    }

  //initialize the map from StateID to Coord
  for(int i = 0; i < (int)StateID2CoordTable.size(); i++){
    delete StateID2CoordTable.at(i);
  }
  StateID2CoordTable.clear();
  for(unsigned int i = 0; i < StateID2IndexMapping.size(); i++){
    delete[] StateID2IndexMapping[i];
  }
  StateID2IndexMapping.clear();
  createdHashEntries = 0;
  ROS_INFO("done with initStorageSpace");
}

void EnvironmentNAVXYTHETATIMELATADEXACT::InitializeEnvironment()
{

  ROS_INFO("Initializing environment using the InitializeEnvironment function");

  printf("The pointer is currently %p\n", (void*)Coord2StateIDHashTable);

  
  TRACKING_TIMEOUT = 100000.0;
  getPPredTime = 0;
  FORCE3DPLANNING = false;
  double start_t = ros::Time::now().toSec();
	
  statsFile = fopen("last_run.env.stats", "w");
  fprintf(statsFile, "Map: ??? (%dx%d)\n", EnvNAVXYTHETATIMELATCfg.EnvWidth_c, EnvNAVXYTHETATIMELATCfg.EnvHeight_c);
  fprintf(statsFile, "Primitives: ??? (%d)\n", EnvNAVXYTHETATIMELATCfg.actionwidth);
  fprintf(statsFile, "Robot size: (%.3fx%.3f)\n", fabs(EnvNAVXYTHETATIMELATCfg.FootprintPolygon[0].x - EnvNAVXYTHETATIMELATCfg.FootprintPolygon[2].x), fabs(EnvNAVXYTHETATIMELATCfg.FootprintPolygon[0].y - EnvNAVXYTHETATIMELATCfg.FootprintPolygon[2].y));
  fprintf(statsFile, "=========================\n");
  fprintf(statsFile, "Start: (%d,%d,%d)\n", EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c, EnvNAVXYTHETATIMELATCfg.StartTheta);
  fprintf(statsFile, "Goal : (%d,%d,%d)\n", EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c, EnvNAVXYTHETATIMELATCfg.EndTheta);
  fprintf(statsFile, "=========================\n");
	
  trackMode = false;
  expands2D = 0;
  expands3D = 0;
  expands4D = 0;
  expandsN3D = 0;
  expandsN4D = 0;
  expands2D_tot = 0;
  expands3D_tot = 0;
  expands4D_tot = 0;
  expandsN3D_tot = 0;
  expandsN4D_tot = 0;
  expandsTrack = 0;
  expandsTrack_tot = 0;
  vizTime = 0;
  getSuccTime = 0;
  planningTime = 0;
  trackingTime = 0;
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry;

  SBPL_INFO("Allocating space for env: %dx%d", EnvNAVXYTHETATIMELATCfg.EnvWidth_c, EnvNAVXYTHETATIMELATCfg.EnvHeight_c);
  SBPL_INFO("Primitives: ??? (%d)\n", EnvNAVXYTHETATIMELATCfg.actionwidth);
  SBPL_INFO("Robot size: (%.3fx%.3f)\n", fabs(EnvNAVXYTHETATIMELATCfg.FootprintPolygon[0].x - EnvNAVXYTHETATIMELATCfg.FootprintPolygon[2].x), fabs(EnvNAVXYTHETATIMELATCfg.FootprintPolygon[0].y - EnvNAVXYTHETATIMELATCfg.FootprintPolygon[2].y));
  SBPL_INFO("=========================\n");
  SBPL_INFO("Start: (%d,%d,%d)\n", EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c, EnvNAVXYTHETATIMELATCfg.StartTheta);
  SBPL_INFO("Goal : (%d,%d,%d)\n", EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c, EnvNAVXYTHETATIMELATCfg.EndTheta);
  SBPL_INFO("=========================\n");

  //allocate sphereStateGrid
  if(sphereStateGrid == NULL || expandsGrid == NULL){
    sphereStateGrid = (unsigned char**) malloc (EnvNAVXYTHETATIMELATCfg.EnvWidth_c * sizeof(unsigned char*));
    expandsGrid = (unsigned char**) malloc (EnvNAVXYTHETATIMELATCfg.EnvWidth_c * sizeof(unsigned char*));
    for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
      sphereStateGrid[i] = (unsigned char*) malloc (EnvNAVXYTHETATIMELATCfg.EnvHeight_c * sizeof(unsigned char));
      expandsGrid[i] = (unsigned char*) malloc (EnvNAVXYTHETATIMELATCfg.EnvHeight_c * sizeof(unsigned char));
      for(int j = 0; j < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; j++){
	//init all states as out of sphere as there are no spheres yet
	sphereStateGrid[i][j] = 10;
	expandsGrid[i][j] = 0;
      }
    }
  }

  if(inflatedGrid == NULL){
    inflatedGrid = (unsigned char**) malloc (EnvNAVXYTHETATIMELATCfg.EnvWidth_c * sizeof(unsigned char*));
    for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
      inflatedGrid[i] = (unsigned char*) malloc (EnvNAVXYTHETATIMELATCfg.EnvHeight_c * sizeof(unsigned char));
    }
  }

  if(sphereDistGrid == NULL){
    sphereDistGrid = (float**) malloc (EnvNAVXYTHETATIMELATCfg.EnvWidth_c * sizeof(float*));
    for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
      sphereDistGrid[i] = (float*) malloc (EnvNAVXYTHETATIMELATCfg.EnvHeight_c * sizeof(float));
    }
  }
	
  if(stateProbGrid == NULL){
    stateProbGrid = (char**) malloc (EnvNAVXYTHETATIMELATCfg.EnvWidth_c * sizeof(char*));
    for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
      stateProbGrid[i] = (char*) malloc (EnvNAVXYTHETATIMELATCfg.EnvHeight_c * sizeof(char));
      for(int j = 0; j < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; j++){
	//init all states as out of sphere as there are no spheres yet
	stateProbGrid[i][j] = -1;
      }
    }
  }

  if(sphereIndexGrid == NULL){
    sphereIndexGrid = (int**) malloc (EnvNAVXYTHETATIMELATCfg.EnvWidth_c * sizeof(int*));
    for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
      sphereIndexGrid[i] = (int*) malloc (EnvNAVXYTHETATIMELATCfg.EnvHeight_c * sizeof(int));
    }
  }

  if(sphereIndexGrid_plan == NULL){
    sphereIndexGrid_plan = (int**) malloc (EnvNAVXYTHETATIMELATCfg.EnvWidth_c * sizeof(int*));
    for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
      sphereIndexGrid_plan[i] = (int*) malloc (EnvNAVXYTHETATIMELATCfg.EnvHeight_c * sizeof(int));
      for(int j=0; j < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; j++) {
	sphereIndexGrid_plan[i][j] = -1;
      }
    }
  }

  if(disttoObs_incells == NULL){
    disttoObs_incells = (float**) malloc (EnvNAVXYTHETATIMELATCfg.EnvWidth_c * sizeof(float*));
    for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
      disttoObs_incells[i] = (float*) malloc (EnvNAVXYTHETATIMELATCfg.EnvHeight_c * sizeof(float));
      for(int j = 0; j < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; j++){
	disttoObs_incells[i][j] = 100.0;
      }
    }
  }

  if(disttoNonfree_incells == NULL){
    disttoNonfree_incells = (float**) malloc (EnvNAVXYTHETATIMELATCfg.EnvWidth_c * sizeof(float*));
    for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
      disttoNonfree_incells[i] = (float*) malloc (EnvNAVXYTHETATIMELATCfg.EnvHeight_c * sizeof(float));
      for(int j = 0; j < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; j++){
	disttoNonfree_incells[i][j] = 100.0;
      }
    }
  }



  ComputeRevActions();

  initStorageSpace();
	

  //initialize the map from StateID to Coord

  StateID2CoordTable.clear();
	
  float half_width = 0;
  float half_length = 0;
  if(EnvNAVXYTHETATIMELATCfg.FootprintPolygon.size() > 2){
    half_width = (fabs(EnvNAVXYTHETATIMELATCfg.FootprintPolygon[0].y - EnvNAVXYTHETATIMELATCfg.FootprintPolygon[2].y) / 2.0);
    half_length = (fabs(EnvNAVXYTHETATIMELATCfg.FootprintPolygon[0].x - EnvNAVXYTHETATIMELATCfg.FootprintPolygon[2].x) / 2.0);
  } else {
    SBPL_WARN("Robot footprint empty!");
  }

  navviz = new NavVisualizer("myRobot", half_width, half_length, -EnvNAVXYTHETATIMELATCfg.EnvWidth_c * EnvNAVXYTHETATIMELATCfg.cellsize_m / 2.0f, -EnvNAVXYTHETATIMELATCfg.EnvHeight_c * EnvNAVXYTHETATIMELATCfg.cellsize_m / 2.0f, -EnvNAVXYTHETATIMELATCfg.cellsize_m / 2.0f);

  //create start state
  // @avemula DOUBT should both T and TempT be startTime for the start state?
  // @avemula LOCATION start initialised and added to the hashtable
  HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c, EnvNAVXYTHETATIMELATCfg.StartTheta, EnvNAVXYTHETATIMELATCfg.StartTime, EnvNAVXYTHETATIMELATCfg.StartTime);
  //HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c, 16);
  EnvNAVXYTHETATIMELAT.startstateid = HashEntry->stateID;

  //create goal state

  // @avemula DOUBT define what value we put in the time dimension for low-dimension states and for goal (endtime is not known)
  // @avemula DOUBT should both T and TempT be INVALID_TIME for the goal state?
  HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c, EnvNAVXYTHETATIMELATCfg.EndTheta, INVALID_TIME, INVALID_TIME);
  //HashEntry = (this->*CreateNewHashEntry)(EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c, 16);
  EnvNAVXYTHETATIMELAT.goalstateid = HashEntry->stateID;

  SBPL_INFO("Start and goal created!");

  SBPL_INFO("Inflating obstacles...");

  //inflate obstacles for 2D planning
  //printf("Pointer value is %p\n", (void*)disttoNonfree_incells);
  computeDistancestoNonfreeAreas(EnvNAVXYTHETATIMELATCfg.Grid2D, EnvNAVXYTHETATIMELATCfg.EnvWidth_c, EnvNAVXYTHETATIMELATCfg.EnvHeight_c, EnvNAVXYTHETATIMELATCfg.obsthresh, disttoObs_incells, disttoNonfree_incells);

  SBPL_INFO("L1");

  float rad = min( fabs(EnvNAVXYTHETATIMELATCfg.FootprintPolygon[0].x - EnvNAVXYTHETATIMELATCfg.FootprintPolygon[2].x) / 2.0, 
		   fabs(EnvNAVXYTHETATIMELATCfg.FootprintPolygon[0].y - EnvNAVXYTHETATIMELATCfg.FootprintPolygon[2].y) / 2.0) + EnvNAVXYTHETATIMELATCfg.cellsize_m / 2.0;
  rad = rad / EnvNAVXYTHETATIMELATCfg.cellsize_m + ADD_INFLATION;
  ROS_INFO("Inflating with rad: %.4f, obst: %d", rad, (int)EnvNAVXYTHETATIMELATCfg.obsthresh);
  for(int w = 0; w < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; w++){
    for(int h = 0; h < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; h++){
      if(disttoObs_incells[w][h] >= rad){
	inflatedGrid[w][h] = 0;
      } else {
	inflatedGrid[w][h] = EnvNAVXYTHETATIMELATCfg.obsthresh;
      }
    }
  }

  SBPL_INFO("L2\n");

  
  for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
    //delete[] disttoObs_incells[i];
    free(disttoObs_incells[i]);
  }
  // delete[] disttoObs_incells;
  free(disttoObs_incells);

	
  for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
    //delete[] disttoNonfree_incells[i];
    free(disttoNonfree_incells[i]);
  }
  //delete[] disttoNonfree_incells;
  free(disttoNonfree_incells);

  SBPL_INFO("done!");

	
  resetTrackingGrid();
  fprintf(statsFile, "Initialization: %.3f (s)\n", (ros::Time::now().toSec() - start_t));
  EnvNAVXYTHETATIMELAT.bInitialized = true;
  SBPL_INFO("Initialization: %.3f (s)\n", (ros::Time::now().toSec() - start_t));
}

// This is taken from EnvironmentNAV2D
void EnvironmentNAVXYTHETATIMELATADEXACT::Computedxy()
{

  //initialize some constants for 2D search
  dx_[0] =  1; dy_[0] =  1; dth_[0] = 2;		dxintersects_[0][0] = 0; dyintersects_[0][0] = 1; dxintersects_[0][1] = 1; dyintersects_[0][1] = 0; 
  dx_[1] =  1; dy_[1] =  0; dth_[1] = 0;		dxintersects_[1][0] = 0; dyintersects_[1][0] = 0; dxintersects_[1][1] = 0; dyintersects_[1][1] = 0;
  dx_[2] =  1; dy_[2] = -1; dth_[2] = 14;		dxintersects_[2][0] = 0; dyintersects_[2][0] = -1; dxintersects_[2][1] = 1; dyintersects_[2][1] = 0;
  dx_[3] =  0; dy_[3] =  1; dth_[3] = 4;		dxintersects_[3][0] = 0; dyintersects_[3][0] = 0; dxintersects_[3][1] = 0; dyintersects_[3][1] = 0;
  dx_[4] =  0; dy_[4] = -1; dth_[4] = 12;		dxintersects_[4][0] = 0; dyintersects_[4][0] = 0; dxintersects_[4][1] = 0; dyintersects_[4][1] = 0;
  dx_[5] = -1; dy_[5] =  1; dth_[5] = 6;		dxintersects_[5][0] = 0; dyintersects_[5][0] = 1; dxintersects_[5][1] = -1; dyintersects_[5][1] = 0;
  dx_[6] = -1; dy_[6] =  0; dth_[6] = 8;		dxintersects_[6][0] = 0; dyintersects_[6][0] = 0; dxintersects_[6][1] = 0; dyintersects_[6][1] = 0;
  dx_[7] = -1; dy_[7] = -1; dth_[7] = 10;		dxintersects_[7][0] = 0; dyintersects_[7][0] = -1; dxintersects_[7][1] = -1; dyintersects_[7][1] = 0;

  //Note: these actions have to be starting at 8 and through 15, since they 
  //get multiplied correspondingly in Dijkstra's search based on index
  dx_[8] = 2; dy_[8] = 1;	dxintersects_[8][0] = 1; dyintersects_[8][0] = 0; dxintersects_[8][1] = 1; dyintersects_[8][1] = 1;
  dx_[9] = 1; dy_[9] = 2;	dxintersects_[9][0] = 0; dyintersects_[9][0] = 1; dxintersects_[9][1] = 1; dyintersects_[9][1] = 1;
  dx_[10] = -1; dy_[10] = 2;	dxintersects_[10][0] = 0; dyintersects_[10][0] = 1; dxintersects_[10][1] = -1; dyintersects_[10][1] = 1;
  dx_[11] = -2; dy_[11] = 1;	dxintersects_[11][0] = -1; dyintersects_[11][0] = 0; dxintersects_[11][1] = -1; dyintersects_[11][1] = 1;
  dx_[12] = -2; dy_[12] = -1;	dxintersects_[12][0] = -1; dyintersects_[12][0] = 0; dxintersects_[12][1] = -1; dyintersects_[12][1] = -1;
  dx_[13] = -1; dy_[13] = -2;	dxintersects_[13][0] = 0; dyintersects_[13][0] = -1; dxintersects_[13][1] = -1; dyintersects_[13][1] = -1;
  dx_[14] = 1; dy_[14] = -2;	dxintersects_[14][0] = 0; dyintersects_[14][0] = -1; dxintersects_[14][1] = 1; dyintersects_[14][1] = -1;
  dx_[15] = 2; dy_[15] = -1; dxintersects_[15][0] = 1; dyintersects_[15][0] = 0; dxintersects_[15][1] = 1; dyintersects_[15][1] = -1;

  //compute distances
  for(int dind = 0; dind  < ENVNAV2D_MAXDIRS; dind++)
    {

      if(dx_[dind] != 0 && dy_[dind] != 0){
	if(dind <= 7)
	  dxy_distance_mm_[dind] = (int)(ENVNAV2D_COSTMULT*1.414);	//the cost of a diagonal move in millimeters
	else
	  dxy_distance_mm_[dind] = (int)(ENVNAV2D_COSTMULT*2.236);	//the cost of a move to 1,2 or 2,1 or so on in millimeters
      }
      else
	dxy_distance_mm_[dind] = ENVNAV2D_COSTMULT;	//the cost of a horizontal move in millimeters
    }
}

EnvironmentNAVXYTHETATIMELATADEXACT::EnvironmentNAVXYTHETATIMELATADEXACT()
{
  sphereStateGrid = NULL;
  sphereDistGrid = NULL;
  stateProbGrid = NULL;
  sphereIndexGrid = NULL;
  sphereIndexGrid_plan = NULL;
  inflatedGrid = NULL;
  expandsGrid = NULL;
  disttoObs_incells = NULL;
  disttoNonfree_incells = NULL;
  INVALID_TIME = INFINITECOST;
  Coord2StateIDHashTable = NULL;
  Coord2StateIDHashTable_lookup = NULL;
  MAX_TIME = 10000;
}

EnvironmentNAVXYTHETATIMELATADEXACT::~EnvironmentNAVXYTHETATIMELATADEXACT()
{
  fclose(statsFile);
  printf("destroying XYTHETATIMELAT\n");
	
  //deallocate sphereStateGrid
  for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
    free( sphereStateGrid[i] );
    free( expandsGrid[i] );
  }
  free( sphereStateGrid );
  free( expandsGrid );


  for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
    free(inflatedGrid[i]);
  }
  free(inflatedGrid);

  for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
    free(sphereDistGrid[i]);
    free(stateProbGrid[i]);
  }
  free(sphereDistGrid);
  free(stateProbGrid);

  for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
    free(sphereIndexGrid[i]);
  }
  free(sphereIndexGrid);

  for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
    free(sphereIndexGrid_plan[i]);
  }
  free(sphereIndexGrid_plan);

  /*for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
    free(disttoObs_incells[i]);
  }
  free(disttoObs_incells);

  for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
    free(disttoNonfree_incells[i]);
  }
  free(disttoNonfree_incells);*/

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

  // delete grid search
  if(grid2Dsearchfromstart != NULL)
    delete grid2Dsearchfromstart;
  grid2Dsearchfromstart = NULL;

  if(grid2Dsearchfromgoal != NULL)
    delete grid2Dsearchfromgoal;
  grid2Dsearchfromgoal = NULL;

}

void EnvironmentNAVXYTHETATIMELATADEXACT::resetStateTable(int* new_start, int* goal){
  int s_x, s_y, s_th, s_t;
  int g_x, g_y, g_th, g_t;
	
  EnvNAVXYTHETATIMELATHashEntry_t* SHashEntry = StateID2CoordTable[*new_start];
  EnvNAVXYTHETATIMELATHashEntry_t* GHashEntry = StateID2CoordTable[*goal];
	
  s_x = SHashEntry->X; s_y = SHashEntry->Y; s_th = SHashEntry->Theta; s_t = SHashEntry->T;
  g_x = GHashEntry->X; g_y = GHashEntry->Y; g_th = GHashEntry->Theta; g_t = GHashEntry->T;
	
  initStorageSpace();
	
  //add the start and goal
  EnvNAVXYTHETATIMELATHashEntry_t* StartHashEntry;
  EnvNAVXYTHETATIMELATHashEntry_t* GoalHashEntry;
  if((StartHashEntry = (this->*GetHashEntry)(s_x, s_y, s_th, s_t)) == NULL){
    //have to create a new entry
    // @avemula DOUBT start T and TempT
    StartHashEntry = (this->*CreateNewHashEntry)(s_x, s_y, s_th, s_t, s_t);
  }
  if((GoalHashEntry = (this->*GetHashEntry)(g_x, g_y, g_th, g_t)) == NULL){
    //have to create a new entry
    // @avemula DOUBT goal T and TempT
    GoalHashEntry = (this->*CreateNewHashEntry)(g_x, g_y, g_th, g_t, g_t);
  }
  *new_start = StartHashEntry->stateID;
  *goal = GoalHashEntry->stateID;
    	
  EnvNAVXYTHETATIMELAT.startstateid = StartHashEntry->stateID;
  EnvNAVXYTHETATIMELAT.goalstateid = GoalHashEntry->stateID;
}

// @avemula NOTUSED 
void EnvironmentNAVXYTHETATIMELATADEXACT::GetPreds(int TargetStateID, std::vector<int> *PredIDV, std::vector<int> *CostV)
{
  if(trackMode){
    SBPL_ERROR("Backward search disabled!");
    throw new SBPL_Exception();
    return;
  } 
  double startT = ros::Time::now().toSec();
  //start state is absorbing for predecessors
  if(TargetStateID == EnvNAVXYTHETATIMELAT.startstateid){
    return;
  }
	
  //clear output vectors
  PredIDV->clear();
  CostV->clear();
	
  GetPreds_Plan(TargetStateID, PredIDV, CostV);
  getPPredTime += (ros::Time::now().toSec() - startT);
	
  return;
}

/**
   NOTES
   =====

   1. What should the LD be?
   a. It can be (x,y,theta) space in which case we will have to significantly modify the 3D succs function and the 4D succs function (2D succs is not used)
   b. It can be (x,y) space in which case we will have to use the 2D succs (with minor modifications) as is, but change the 4D succs (3D succs is not used)

   2. What should the HD be? Simple, it is the (x,y,theta,t) space

   3. For LD states,
   a. time = INVALID_TIME (also for the goal state)
   b. theta = INVALID_THETA, time = INVALID_TIME (also for the goal state)
   
   4. Do we need to implement the (x,y,theta) - (x,y) adaptive setting? I think not. Time in (x,y) space is a big issue
 */


// @avemula NOTE use the following function only in the (a) case (as detailed above). Rename this function, maybe?
// Backup HD expansion. Same as Gochev's code
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::Get3DSuccs_HD(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV, std::vector<EnvNAVXYTHETATIMELATAction_t*> *actionV)
{
  //printf("Getting 3D succs! (%d)\n", SourceStateID); fflush(stdout);

  double startT = ros::Time::now().toSec();
  int aind;
	
  //get X, Y for the state
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
  if(HashEntry->Theta >= INVALID_THETA){
    SBPL_ERROR("Trying to generate 3D successors of a 2D state!");
    throw new SBPL_Exception();
  }
  // @avemula NEEDED? check for generating 3D successors of a 4D state

  double cost_mult = 1.0f;

  // Iterate through all possible functions possible
  for (aind = 0; aind < EnvNAVXYTHETATIMELATCfg.actionwidth; aind++)
    {
      //printf("aind: %d\n", aind); fflush(stdout);
      bool valid = true;
      EnvNAVXYTHETATIMELATAction_t* nav3daction = &EnvNAVXYTHETATIMELATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
      int newX = HashEntry->X + nav3daction->dX;
      int newY = HashEntry->Y + nav3daction->dY;
      int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, NAVXYTHETATIMELAT_THETADIRS);
      // @avemula ADDED time dimension for LD with INVALID_TIME
      int newT = INVALID_TIME;
      float newTempT = HashEntry->TempT + nav3daction->dT;

      //skip the invalid cells
      if(!IsValidCell(newX, newY)){ 
	//valid = false;
	continue;
      }
      
      //printf("Cell: (%d, %d, %d)\n", newX, newY, newTheta);

      if(lookupCoords(newX, newY) != 0){
	//printf("outside spheres!"); fflush(stdout);
	//successor falls outside all spheres
	if(lookupCoords(HashEntry->X, HashEntry->Y)!=0) {
	  //SBPL_WARN("Both source and successor fall outside sphere!");
	  valid = false;
	  //continue;
	}
			
	if(trackMode) {
	  //discard in tracking mode
	  valid = false;
	  //continue;
	} else {
	  //make 2D in planning mode
	  //get cost
	  unsigned int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav3daction, false);
	  if(cost >= INFINITECOST){
	    //discard infinite cost transitions
	    valid = false;
	    //continue;
	  }
	  newTheta = INVALID_THETA;
	  EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
	  if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	    {
	      //have to create a new entry
	      OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	    }

	  SuccIDV->push_back(OutHashEntry->stateID);
	  if(valid){
	    CostV->push_back((int) (cost_mult * cost));
	  } else {
	    CostV->push_back(INFINITECOST);
	  }
	  if(actionV != NULL)
	    actionV->push_back(nav3daction);
	}
      } else {
	//printf("inside a sphere!"); fflush(stdout);
	//successor falls in sphere
			
	double cc_startT = ros::Time::now().toSec();
	if(!IsValidConfiguration(newX, newY, newTheta)){
	  if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  valid = false;
	  //continue;
	}
			
	unsigned int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav3daction, true);
	if(cost >= INFINITECOST){
	  if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  //discard infinite cost transitions
	  //continue;
	  valid = false;
	}
	if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }

	EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
	if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	  {
	    //have to create a new entry
	    OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	  }
	SuccIDV->push_back(OutHashEntry->stateID);
	if(valid){
	  CostV->push_back((int) (cost_mult * cost));
	} else {
	  CostV->push_back(INFINITECOST);
	}
	if(actionV != NULL)
	  actionV->push_back(nav3daction);
      }
    }
  if(!trackMode) get3DSuccTime_HD += (ros::Time::now().toSec() - startT);
  //printf("Done 3D succs!\n"); fflush(stdout);
}

// @avemula OWN function
// This is a LD expansion. Must be similar to Get2DSuccs in Gochev's code
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::Get3DSuccs_LD(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV, std::vector<EnvNAVXYTHETATIMELATAction_t*> *actionV)
{
  //printf("Getting 3D succs! (%d)\n", SourceStateID); fflush(stdout);

  double startT = ros::Time::now().toSec();
  int aind;
	
  //get X, Y for the state
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
  EnvNAVXYTHETATIMELATHashEntry_t* GoalHashEntry = StateID2CoordTable[EnvNAVXYTHETATIMELAT.goalstateid];

  if(SourceStateID == EnvNAVXYTHETATIMELAT.goalstateid) {
    return;
  }
  
  if(HashEntry->Theta >= INVALID_THETA){
    SBPL_ERROR("Trying to generate 3D successors of a 2D state!");
    throw new SBPL_Exception();
  }
  // @avemula NEEDED? check for generating 3D successors of a 4D state
  if(HashEntry->T < INVALID_TIME && SourceStateID!=EnvNAVXYTHETATIMELAT.startstateid) {
    SBPL_ERROR("Trying to generate 3D successors of a 4D state!");
    throw new SBPL_Exception();
  }

  double cost_mult = 1.0f;

  // Iterate through all possible functions possible
  for (aind = 0; aind < EnvNAVXYTHETATIMELATCfg.actionwidth; aind++)
    {
      //printf("aind: %d\n", aind); fflush(stdout);
      bool valid = true;
      EnvNAVXYTHETATIMELATAction_t* nav3daction = &EnvNAVXYTHETATIMELATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
      int newX = HashEntry->X + nav3daction->dX;
      int newY = HashEntry->Y + nav3daction->dY;
      int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, NAVXYTHETATIMELAT_THETADIRS);
      // @avemula ADDED time dimension for LD with INVALID_TIME
      int newT = INVALID_TIME;
      float newTempT = HashEntry->TempT + nav3daction->dT;

      //skip the invalid cells
      if(!IsValidCell(newX, newY)){ 
	//valid = false;
	continue;
      }
      
      //printf("Cell: (%d, %d, %d)\n", newX, newY, newTheta);

      double cc_startT = ros::Time::now().toSec();
      if(!IsValidConfiguration(newX, newY, newTheta)){
	if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	valid = false;
	//continue;
      }

      // @avemula NOTE not checking collision with dynamic obstacles
      unsigned int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav3daction, false);
      
      if(cost >= INFINITECOST){
	if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	//discard infinite cost transitions
	//continue;
	valid = false;
      }
      if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
      else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }

      EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;

      // checking with goal 
      if(newX == GoalHashEntry->X && newY== GoalHashEntry->Y && newTheta == GoalHashEntry->Theta) {
	OutHashEntry = GoalHashEntry;
      }      
      else if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	{
	  //have to create a new entry
	  OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	}


      // @avemula IMPORTANT_CHANGE update tempT if we have found a better path
      if(OutHashEntry->TempT > newTempT)
	OutHashEntry->TempT = newTempT;
      
      SuccIDV->push_back(OutHashEntry->stateID);

      // @avemula ADDED checks relevant to LD succs
      // Check if the successor is in a sphere or not
      // If inside a sphere - ditch, else - keep
      if(valid && lookupCoords(HashEntry->X, HashEntry->Y)!=0 && lookupCoords(newX, newY)!=0){
	CostV->push_back((int) (cost_mult * cost));
      } else {
	CostV->push_back(INFINITECOST);
      }
      if(actionV != NULL)
	actionV->push_back(nav3daction);
      
    }
  
  // @avemula MODIFIED
  get3DSuccTime_LD += (ros::Time::now().toSec() - startT);
  //printf("Done 3D succs!\n"); fflush(stdout);
}

// @avemula ADDED Own function to get all the 4D successors of a N4D state
// @avemula ASK
void EnvironmentNAVXYTHETATIMELATADEXACT::Get3DSuccs_allT(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV, std::vector<EnvNAVXYTHETATIMELATAction_t*> *actionV)
{
  double startT = ros::Time::now().toSec();
  int aind;
	
  //get X, Y for the state
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
  EnvNAVXYTHETATIMELATHashEntry_t* GoalHashEntry = StateID2CoordTable[EnvNAVXYTHETATIMELAT.goalstateid];

  if(SourceStateID == EnvNAVXYTHETATIMELAT.goalstateid) {
    return;
  }
  
  if(HashEntry->Theta >= INVALID_THETA){
    SBPL_ERROR("Trying to generate 3D successors of a 2D state!");
    throw new SBPL_Exception();
  }
  
  // @avemula NEEDED? check for generating 3D successors of a 4D state
  if(HashEntry->T < INVALID_TIME && SourceStateID!=EnvNAVXYTHETATIMELAT.startstateid) {
    SBPL_ERROR("Trying to generate 3D successors of a 4D state!");
    throw new SBPL_Exception();
  }

  double cost_mult = 1.0f;

  // Iterate through all possible functions possible
  for (aind = 0; aind < EnvNAVXYTHETATIMELATCfg.actionwidth; aind++)
    {
      //printf("aind: %d\n", aind); fflush(stdout);
      bool valid = true;
      EnvNAVXYTHETATIMELATAction_t* nav3daction = &EnvNAVXYTHETATIMELATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
      int newX = HashEntry->X + nav3daction->dX;
      int newY = HashEntry->Y + nav3daction->dY;
      int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, NAVXYTHETATIMELAT_THETADIRS);
      // @avemula ADDED time dimension for LD with INVALID_TIME
      int newT = INVALID_TIME;
      float newTempT = HashEntry->TempT + nav3daction->dT;
      float newTempT_backup = newTempT;

      //skip the invalid cells
      if(!IsValidCell(newX, newY)){ 
	//valid = false;
	continue;
      }
      
      
      //printf("Cell: (%d, %d, %d)\n", newX, newY, newTheta);

      double cc_startT = ros::Time::now().toSec();
      if(!IsValidConfiguration(newX, newY, newTheta)){
	if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	valid = false;
	//continue;
      }

      if(lookupCoords(newX, newY)!=0) {
	// The successor not in sphere. Ditch!
	continue;
      }

      
      // @avemula NOTE not checking collision with dynamic obstacles
      unsigned int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav3daction, false);
      
      if(cost >= INFINITECOST){
	if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	//discard infinite cost transitions
	//continue;
	valid = false;
      }
      if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
      else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }

      // @avemula CHANGED the lower bound of the loop from 0 to newTempT
      int sIndex = sphereIndexGrid_plan[newX][newY];
      //printf("sphere index is %d, time is %d\n", sIndex, spheres[sIndex].t_d - (int) ((spheres[sIndex].r/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs)/EnvNAVXYTHETATIMELATCfg.timeResolution));      
      int startTime = max(spheres[sIndex].t_d - (int) ((spheres[sIndex].r/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs)/EnvNAVXYTHETATIMELATCfg.timeResolution), 0);

      //int endTime = max(spheres[sIndex].t_d + (int) ((2.1415)*((spheres[sIndex].r/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs)/EnvNAVXYTHETATIMELATCfg.timeResolution)), 0);

      int endTime = min(spheres[sIndex].t_d + (int) (((spheres[sIndex].r/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs)/EnvNAVXYTHETATIMELATCfg.timeResolution)), MAX_TIME);

      //printf("Start time for %d,%d is : %d\n", newX, newY, startTime);
      //printf("End time for %d,%d is : %d\n", newX, newY, endTime);
      
      // @avemula NOTE the main loop for guarantees
      //for(int tim=startTime; tim < MAX_TIME; tim++) {
      //for(int tim=startTime; tim < endTime+1; tim++) {
      //for(int tim = startTime; tim < startTime + 1; tim++) {
      for(int tim = (int)newTempT_backup; tim < endTime + 1; tim+=3) {
	newT = tim;
	newTempT = tim;
	EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;

	// checking with goal
	if(newX == GoalHashEntry->X && newY== GoalHashEntry->Y && newTheta == GoalHashEntry->Theta) {
	  OutHashEntry = GoalHashEntry;
	}      
	else if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	  {
	    //have to create a new entry
	    OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	  }


	// @avemula IMPORTANT_CHANGE update tempT if we have found a better path
	if(OutHashEntry->TempT > newTempT)
	  OutHashEntry->TempT = newTempT;
      
	SuccIDV->push_back(OutHashEntry->stateID);


	if(valid && lookupCoords(HashEntry->X, HashEntry->Y)!=0){
	  // @avemula REMOVED old cost value
	  //CostV->push_back((int) (cost_mult * cost));
	  //printf("Cost addition is %d + %d\n", cost, (int)(max(newT - newTempT_backup,0)*NAVXYTHETATIMELAT_COSTMULT_MTOMM*EnvNAVXYTHETATIMELATCfg.timeResolution));
	  //CostV->push_back((int) (cost_mult*cost) + max(newT - newTempT_backup,0));
	  // @avemula IMPORTANT_CHANGE using tempT to augment cost
	  //CostV->push_back((int) (cost_mult*cost) + (int)(max(newT - startTime,0)*NAVXYTHETATIMELAT_COSTMULT_MTOMM*EnvNAVXYTHETATIMELATCfg.timeResolution));
	  CostV->push_back((int) (cost_mult*cost) + (int)(max(newT - (int)newTempT_backup,0)*NAVXYTHETATIMELAT_COSTMULT_MTOMM*EnvNAVXYTHETATIMELATCfg.timeResolution));
	  //CostV->push_back((int) (cost_mult*cost));
	}
	else {
	  CostV->push_back(INFINITECOST);
	}
	if(actionV != NULL)
	  actionV->push_back(nav3daction);
      }
      
    }
  
  // @avemula MODIFIED
  //get3DSuccTime_LD += (ros::Time::now().toSec() - startT);
  //printf("Done 3D succs!\n"); fflush(stdout);
}


// @avemula NOTE use the following function only in the (a) case (as detailed above). Rename this function, maybe?
// Backup HD expansion. Same as Gochev's code
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::Get3DPreds_HD(int SourceStateID, std::vector<int> *PredIDV, std::vector<int> *CostV)
{
  double startT = ros::Time::now().toSec();
  int aind;

  //start state should be absorbing
  /*if(SourceStateID == EnvNAVXYTHETATIMELAT.startstateid){
    get3DPredTime += (ros::Time::now().toSec() - startT);
    return;
    }*/
	
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
  if(HashEntry->Theta >= INVALID_THETA){
    SBPL_ERROR("Trying to generate 3D predecessors of a 2D state!");
    throw new SBPL_Exception();
  }

  double cost_mult = 1.0f;
	
	
	
  for (aind = 0; aind < revActionsV[(unsigned int)HashEntry->Theta].size(); aind++)
    {
      bool valid = true;
      //EnvNAVXYTHETATIMELATAction_t* nav3daction = &EnvNAVXYTHETATIMELATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
      EnvNAVXYTHETATIMELATAction_t* nav3daction = revActionsV[(unsigned int)HashEntry->Theta][aind];
      int newX = HashEntry->X - nav3daction->dX;
      int newY = HashEntry->Y - nav3daction->dY;
      int newTheta = NORMALIZEDISCTHETA(nav3daction->starttheta, NAVXYTHETATIMELAT_THETADIRS);
      // @avemula ADDED time dimensions
      int newT = INVALID_TIME;
      // @avemula DOUBT Is preds function used for backward search? In backward search, goal tempT is invalid and the following assignment
      // would result in incorrect tempT      
      float newTempT = INVALID_TIME;
      if(HashEntry->TempT < INVALID_TIME)
	newTempT = HashEntry->TempT - nav3daction->dT;	

      //skip the invalid cells
      if(!IsValidCell(newX, newY)){ 
	continue;
      }

      if(lookupCoords(newX, newY) != 0){
	//predecessor falls outside all spheres
	if(lookupCoords(HashEntry->X, HashEntry->Y)!=0) {
	  continue;
	}
			
	if(trackMode) {
	  //discard in tracking mode
	  continue;
	} else {
	  //make 2D in planning mode
	  //get cost
	  unsigned int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav3daction, false);
	  if(cost >= INFINITECOST){
	    //discard infinite cost transitions
	    continue;
	  }
	  newTheta = INVALID_THETA;
	  EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
	  if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	    {
	      //have to create a new entry
	      OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	    }

	  PredIDV->push_back(OutHashEntry->stateID);
	  if(valid){
	    CostV->push_back((int) (cost_mult * cost));
	  } else {
	    CostV->push_back(INFINITECOST);
	  }
	}
      } else {
	//predecessor falls in sphere
			
	double cc_startT = ros::Time::now().toSec();
	if(!IsValidConfiguration(newX, newY, newTheta)){
	  if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  continue;
	}
	unsigned int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav3daction, true);
	if(cost >= INFINITECOST){
	  if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  //discard infinite cost transitions
	  continue;
	}
	if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }

	EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
	if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	  {
	    //have to create a new entry
	    OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	  }
	PredIDV->push_back(OutHashEntry->stateID);
	if(valid){
	  CostV->push_back((int) (cost_mult * cost));
	} else {
	  CostV->push_back(INFINITECOST);
	}
      }
    }
  if(!trackMode) get3DPredTime_HD += (ros::Time::now().toSec() - startT);
}

// @avemula OWN function
// This is a LD expansion. Must be similar to Get2DPreds in Gochev's code
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::Get3DPreds_LD(int SourceStateID, std::vector<int> *PredIDV, std::vector<int> *CostV)
{
  double startT = ros::Time::now().toSec();
  int aind;

  //start state should be absorbing
  /*if(SourceStateID == EnvNAVXYTHETATIMELAT.startstateid){
    get3DPredTime += (ros::Time::now().toSec() - startT);
    return;
    }*/
	
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
  if(HashEntry->Theta >= INVALID_THETA){
    SBPL_ERROR("Trying to generate 3D predecessors of a 2D state!");
    throw new SBPL_Exception();
  }

  double cost_mult = 1.0f;
	
  for (aind = 0; aind < revActionsV[(unsigned int)HashEntry->Theta].size(); aind++)
    {
      bool valid = true;
      //EnvNAVXYTHETATIMELATAction_t* nav3daction = &EnvNAVXYTHETATIMELATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
      EnvNAVXYTHETATIMELATAction_t* nav3daction = revActionsV[(unsigned int)HashEntry->Theta][aind];
      int newX = HashEntry->X - nav3daction->dX;
      int newY = HashEntry->Y - nav3daction->dY;
      int newTheta = NORMALIZEDISCTHETA(nav3daction->starttheta, NAVXYTHETATIMELAT_THETADIRS);
      // @avemula ADDED time dimensions
      int newT = INVALID_TIME;
      //@avemula DOUBT backward search preds
      float newTempT = INVALID_TIME;
      if(HashEntry->TempT<INVALID_TIME)
	newTempT = HashEntry->TempT - nav3daction->dT;

      //skip the invalid cells
      if(!IsValidCell(newX, newY)){ 
	continue;
      }
      			
      double cc_startT = ros::Time::now().toSec();
      if(!IsValidConfiguration(newX, newY, newTheta)){
	if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	continue;
      }
      unsigned int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav3daction, false);
      if(cost >= INFINITECOST){
	if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	//discard infinite cost transitions
	continue;
      }
      if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
      else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }

      EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
      if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	{
	  //have to create a new entry
	  OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	}
      PredIDV->push_back(OutHashEntry->stateID);
      // @avemula ADDED new checks relevant for LD preds
      // If pred in sphere - ditch it. Else, keep
      if(valid && lookupCoords(HashEntry->X, HashEntry->Y)!=0 && lookupCoords(newX, newY)!=0){
	CostV->push_back((int) (cost_mult * cost));
      } else {
	CostV->push_back(INFINITECOST);
      }
    }
  if(!trackMode) get3DPredTime_LD += (ros::Time::now().toSec() - startT);
}


// @avemula USED in this setting. MODIFIED to work in our scenario
// @avemula ASK
void EnvironmentNAVXYTHETATIMELATADEXACT::Get2DSuccs(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV, std::vector<EnvNAVXYTHETATIMELATAction_t*> *actionV)
{
  //printf("Getting 2D succs (%d)\n", SourceStateID); fflush(stdout);
  double startT = ros::Time::now().toSec();
  if(trackMode) { 
    ROS_ERROR("Getting 2D successors in tracking mode! Shouldn't happen!");
    throw new SBPL_Exception();
  }
	
  int aind;
  // @avemula IMPORTANT CHANGE 
  double cost_mult = 1.0f;
  //double cost_mult = 0.75f;
	
  //goal state should be absorbing
  if(SourceStateID == EnvNAVXYTHETATIMELAT.goalstateid){
    get2DSuccTime += (ros::Time::now().toSec() - startT);
    return;
  }
  
  //get X, Y for the state
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
	
  //iterate through actions
  bool bTestBounds = false;
  if(HashEntry->X <= 1 || HashEntry->X >= EnvNAVXYTHETATIMELATCfg.EnvWidth_c-2 || 
     HashEntry->Y <= 1 || HashEntry->Y >= EnvNAVXYTHETATIMELATCfg.EnvHeight_c-2)
    bTestBounds = true;

  for (aind = 0; aind < 8; aind++)
    {
      bool valid = true;
      int newX = HashEntry->X + dx_[aind];
      int newY = HashEntry->Y + dy_[aind];
      int newTheta = INVALID_THETA;      
      // @avemula ADDED time dimensions
      int newT = INVALID_TIME;
      float newTempT = HashEntry->TempT + (((sqrt(pow(dx_[aind],2)+pow(dy_[aind],2))*EnvNAVXYTHETATIMELATCfg.cellsize_m) / EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs)/EnvNAVXYTHETATIMELATCfg.timeResolution); // Can't get time for 2D actions!

     
      if(!IsValidCell2D(newX, newY)){
	valid = false;
	continue;
      }

      int costmult = EnvNAVXYTHETATIMELATCfg.Grid2D[newX][newY];

      //for diagonal move, take max over adjacent cells
      if(newX != HashEntry->X && newY != HashEntry->Y && aind <= 7)
	{
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[HashEntry->X][newY]);
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[newX][HashEntry->Y]);
	}
      else if(aind > 7)
	{
	  //check two more cells through which the action goes
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[HashEntry->X + dxintersects_[aind][0]][HashEntry->Y + dyintersects_[aind][0]]);
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[HashEntry->X + dxintersects_[aind][1]][HashEntry->Y + dyintersects_[aind][1]]);
	}

      //check that it is valid
      if(costmult >= EnvNAVXYTHETATIMELATCfg.obsthresh){
	//collision
	valid = false;
	continue;
      }

      int penalty = 1;
      //otherwise compute the actual cost
      int cost = (int) (cost_mult * (costmult+1)*dxy_distance_mm_[aind]*penalty);  // add penalty for offcenter	
		 
      //succ not in sphere - keep
      EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
      if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	{
	  //have to create a new entry
	  OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	}
      
      
      SuccIDV->push_back(OutHashEntry->stateID);
      int c = (int)(cost * EnvNAVXYTHETATIMELATCfg.cellsize_m / EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs);
		
      if(lookupCoords(HashEntry->X, HashEntry->Y)==0 || lookupCoords(newX, newY) == 0 || !valid){
	//source state is invalidated - discard
	CostV->push_back(INFINITECOST);
      } else {
	CostV->push_back(c+1);
      }
		
      if(actionV != NULL){
	actionV->push_back(NULL);
      }
    }
  get2DSuccTime += (ros::Time::now().toSec() - startT);
  //printf("Done 2D succs!\n"); fflush(stdout);
}

// @avemula  USED in this setting. MODIFIED to suit our scenario
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::Get2DSuccs(int x, int y, std::vector<std::vector<int> > *SuccIDV, std::vector<int> *CostV)
{
  //iterate through actions
  bool bTestBounds = false;
  if(x <= 1 || x >= EnvNAVXYTHETATIMELATCfg.EnvWidth_c-2 || 
     y <= 1 || y >= EnvNAVXYTHETATIMELATCfg.EnvHeight_c-2)
    bTestBounds = true;

  for (int aind = 0; aind < 8; aind++)
    {
      int newX = x + dx_[aind];
      int newY = y + dy_[aind];
      //int newTheta = INVALID_THETA;

      if(!IsValidCell2D(newX, newY)){
	continue;
      }
		
      int costmult = EnvNAVXYTHETATIMELATCfg.Grid2D[newX][newY];

      //for diagonal move, take max over adjacent cells
      if(newX != x && newY != y && aind <= 7)
	{
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[x][newY]);
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[newX][y]);
	}
      else if(aind > 7)
	{
	  //check two more cells through which the action goes
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[x + dxintersects_[aind][0]][y + dyintersects_[aind][0]]);
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[x + dxintersects_[aind][1]][y + dyintersects_[aind][1]]);
	}

      //check that it is valid
      if(costmult >= EnvNAVXYTHETATIMELATCfg.obsthresh){
	//collision
	continue;
      }

      int penalty = 1;
      //otherwise compute the actual cost
      int cost = (int) ((costmult+1)*dxy_distance_mm_[aind]*penalty);  // add penalty for offcenter	
		
      std::vector<int> succ (2, 0);
      succ[0] = newX;
      succ[1] = newY;
      SuccIDV->push_back(succ);
      int c = (int)(cost * EnvNAVXYTHETATIMELATCfg.cellsize_m / EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs);
      CostV->push_back(c);
    }
}


// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::Get2DPreds(int SourceStateID, std::vector<int> *PredIDV, std::vector<int> *CostV)
{
  double startT = ros::Time::now().toSec();
	
  int aind;
  double cost_mult = 1.0f;
	
  //start state should be absorbing
  /*if(SourceStateID == EnvNAVXYTHETATIMELAT.startstateid){
    get2DPredTime += (ros::Time::now().toSec() - startT);
    return;
    }*/
  //get X, Y for the state
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
	
  //iterate through actions
  bool bTestBounds = false;
  if(HashEntry->X <= 1 || HashEntry->X >= EnvNAVXYTHETATIMELATCfg.EnvWidth_c-2 || 
     HashEntry->Y <= 1 || HashEntry->Y >= EnvNAVXYTHETATIMELATCfg.EnvHeight_c-2)
    bTestBounds = true;

  for (aind = 0; aind < 8; aind++)
    {
      bool valid = true;
      int newX = HashEntry->X - dx_[aind];
      int newY = HashEntry->Y - dy_[aind];
      int newTheta = INVALID_THETA;
      // @avemula ADDED time dimension
      int newT = INVALID_TIME;
      float newTempT = INVALID_TIME; // Can't get time for 2D actions

      if(!IsValidCell2D(newX, newY)){
	continue;
      }

      int costmult = EnvNAVXYTHETATIMELATCfg.Grid2D[newX][newY];

      //for diagonal move, take max over adjacent cells
      if(newX != HashEntry->X && newY != HashEntry->Y && aind <= 7)
	{
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[HashEntry->X][newY]);
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[newX][HashEntry->Y]);
	}
      else if(aind > 7)
	{
	  //check two more cells through which the action goes
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[HashEntry->X + dxintersects_[aind][0]][HashEntry->Y + dyintersects_[aind][0]]);
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[HashEntry->X + dxintersects_[aind][1]][HashEntry->Y + dyintersects_[aind][1]]);
	}

      //check that it is valid
      if(costmult >= EnvNAVXYTHETATIMELATCfg.obsthresh){
	//collision
	//valid = false;
	continue;
      }

      int penalty = 1;
      //otherwise compute the actual cost
      int cost = (int) (cost_mult * (costmult+1)*dxy_distance_mm_[aind]*penalty);  // add penalty for offcenter	
		 
      //succ not in sphere - keep
       EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
       if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	{
	  //have to create a new entry
	  OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	}
		
      PredIDV->push_back(OutHashEntry->stateID);
      int c = (int)(cost * EnvNAVXYTHETATIMELATCfg.cellsize_m / EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs);
		
      if(lookupCoords(HashEntry->X, HashEntry->Y)==0 || lookupCoords(newX, newY) == 0 || !valid){
	//source state is invalidated - discard
	CostV->push_back(INFINITECOST);
      } else {
	CostV->push_back(c+1);
      }
    }
  get2DPredTime += (ros::Time::now().toSec() - startT);
}

// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::Get2DPreds(int x, int y, std::vector<std::vector<int> > *PredIDV, std::vector<int> *CostV)
{
	
  int aind;
  double cost_mult = 1.0f;
	
  //iterate through actions
  bool bTestBounds = false;
  if(x <= 1 || x >= EnvNAVXYTHETATIMELATCfg.EnvWidth_c-2 || 
     y <= 1 || y >= EnvNAVXYTHETATIMELATCfg.EnvHeight_c-2)
    bTestBounds = true;

  for (aind = 0; aind < 8; aind++)
    {
      int newX = x - dx_[aind];
      int newY = y - dy_[aind];
      //int newTheta = INVALID_THETA;

      if(!IsValidCell2D(newX, newY)){
	continue;
      }	       

      int costmult = EnvNAVXYTHETATIMELATCfg.Grid2D[newX][newY];

      //for diagonal move, take max over adjacent cells
      if(newX != x && newY != y && aind <= 7)
	{
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[x][newY]);
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[newX][y]);
	}
      else if(aind > 7)
	{
	  //check two more cells through which the action goes
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[x + dxintersects_[aind][0]][y + dyintersects_[aind][0]]);
	  costmult = __max(costmult, EnvNAVXYTHETATIMELATCfg.Grid2D[x + dxintersects_[aind][1]][y + dyintersects_[aind][1]]);
	}

      //check that it is valid
      if(costmult >= EnvNAVXYTHETATIMELATCfg.obsthresh){
	//collision
	continue;
      }

      int penalty = 1;
      //otherwise compute the actual cost
      int cost = (int) (cost_mult * (costmult+1)*dxy_distance_mm_[aind]*penalty);  // add penalty for offcenter	
		
		
      std::vector<int> pred (2, 0);
      pred[0] = newX;
      pred[1] = newY;
      PredIDV->push_back(pred);
      int c = (int)(cost * EnvNAVXYTHETATIMELATCfg.cellsize_m / EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs);
      //SBPL_ERROR("2D action: dX: %d  dY: %d  cost: %d", dx_[aind], dy_[aind], c);
      CostV->push_back(c);
    }
  //printf("Generated %d 2D successors!\n", SuccIDV->size());
}


// @avemula ADDED the super important get4dsuccsfunction
// @avemula MODIFIED v2.0 to get all possible successors (like the exact case)
// @avemula ASK
void EnvironmentNAVXYTHETATIMELATADEXACT::Get4DSuccs(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV, std::vector<EnvNAVXYTHETATIMELATAction_t*> *actionV)
{

  double startT = ros::Time::now().toSec();
  int aind;
	
  //get X, Y for the state
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
  if(HashEntry->Theta >= INVALID_THETA || HashEntry->T >= INVALID_TIME){
    SBPL_ERROR("Trying to generate 4D successors of a 2D state!");
    throw new SBPL_Exception();
  }
  
  if(HashEntry->T >= INVALID_TIME) {
    SBPL_ERROR("Trying to generate 4D successors of a 3D state!");
    fflush(stdout);
    throw new SBPL_Exception();
  }
  
  //printf("Expanding ID : %d state %d,%d,%d,%d\n", SourceStateID, HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T);

  double cost_mult = 1.0f;

  // Iterate through all possible functions possible
  for (aind = 0; aind < EnvNAVXYTHETATIMELATCfg.actionwidth; aind++)
    {

      bool valid = true;
      EnvNAVXYTHETATIMELATAction_t* nav3daction = &EnvNAVXYTHETATIMELATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
      int newX = HashEntry->X + nav3daction->dX;
      int newY = HashEntry->Y + nav3daction->dY;
      int newTheta = NORMALIZEDISCTHETA(nav3daction->endtheta, NAVXYTHETATIMELAT_THETADIRS);
      // @avemula ADDED time dimension
      int newT = HashEntry->T + nav3daction->dT;
      float newTempT = HashEntry->TempT + nav3daction->dT;

      
      //skip the invalid cells
      if(!IsValidCell(newX, newY)){ 
	//valid = false;
	continue;
      }
      
      if(lookupCoords(newX, newY) != 0){
	//printf("outside spheres!"); fflush(stdout);
	//successor falls outside all spheres
	if(lookupCoords(HashEntry->X, HashEntry->Y)!=0) {
	  //SBPL_WARN("Both source and successor fall outside sphere!");
	  valid = false;
	  //continue;
	}
			
	if(trackMode) {
	  //discard in tracking mode
	  valid = false;
	  //continue;
	}
	else {
	  //make 2D in planning mode
	  //get cost
	  
	  // @avemula CHANGED checking collision with a dynamic obstacle	 
	  unsigned int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav3daction, true);
	  
	  if(cost >= INFINITECOST){
	    //discard infinite cost transitions
	    valid = false;
	    //continue;
	  }
	  
	  // @avemula ADDED both theta and time are invalid in 2D
	  newTheta = INVALID_THETA;
	  newT = INVALID_TIME;
	  EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
	  if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	    {
	      //have to create a new entry
	      OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	    }
	  
	  SuccIDV->push_back(OutHashEntry->stateID);
	  if(valid){
	    CostV->push_back((int) (cost_mult * cost));
	  } else {
	    CostV->push_back(INFINITECOST);
	  }
	  if(actionV != NULL)
	    actionV->push_back(nav3daction);
	}
      }

      else {
	//printf("inside a sphere!"); fflush(stdout);
	//successor falls in sphere
	
	double cc_startT = ros::Time::now().toSec();

	// Check if valid configuration
	if(!IsValidConfiguration(newX, newY, newTheta)){
	  if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  valid = false;
	  //continue;
	}

	// @avemula ADDED a check on time dimensions
	// @avemula ASK bounding time dimension for normal 4D planning
	if (newT >= MAX_TIME) {
	  valid = false;
	}

	unsigned int cost;
	// @avemula IMPORTANT CHANGE
	cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav3daction, true);

	
	if(cost >= INFINITECOST){
	  if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  //discard infinite cost transitions
	  //continue;
	  valid = false;
	}
	if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	
	EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
	if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	  {
	    //have to create a new entry
	    OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	  }
	
	SuccIDV->push_back(OutHashEntry->stateID);
	if(valid){
	  CostV->push_back((int) (cost_mult * cost));
	} else {
	  CostV->push_back(INFINITECOST);
	}
	if(actionV != NULL)
	  actionV->push_back(nav3daction);
      }
    }
  if(!trackMode) get4DSuccTime += (ros::Time::now().toSec() - startT);
  //printf("Done 4D succs!\n"); fflush(stdout);
}

// @avemula ADDED the function for generating 4D preds
// @avemula MODIFIED v2.0 to match the exact case
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::Get4DPreds(int SourceStateID, std::vector<int> *PredIDV, std::vector<int> *CostV)
{
  double startT = ros::Time::now().toSec();
  int aind;

  //start state should be absorbing
  /*if(SourceStateID == EnvNAVXYTHETATIMELAT.startstateid){
    get3DPredTime += (ros::Time::now().toSec() - startT);
    return;
    }*/
	
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
  if(HashEntry->Theta >= INVALID_THETA || HashEntry->T >= INVALID_TIME){
    SBPL_ERROR("Trying to generate 4D predecessors of a 2D state!");
    throw new SBPL_Exception();
  }

  /*if(HashEntry->T == INVALID_TIME){
    SBPL_ERROR("Trying to generate 4D predecessors of a 3D state!");
    throw new SBPL_Exception();
    }*/

  double cost_mult = 1.0f;
	
	
	
  for (aind = 0; aind < revActionsV[(unsigned int)HashEntry->Theta].size(); aind++)
    {
      bool valid = true;
      //EnvNAVXYTHETATIMELATAction_t* nav3daction = &EnvNAVXYTHETATIMELATCfg.ActionsV[(unsigned int)HashEntry->Theta][aind];
      EnvNAVXYTHETATIMELATAction_t* nav3daction = revActionsV[(unsigned int)HashEntry->Theta][aind];
      int newX = HashEntry->X - nav3daction->dX;
      int newY = HashEntry->Y - nav3daction->dY;
      int newTheta = NORMALIZEDISCTHETA(nav3daction->starttheta, NAVXYTHETATIMELAT_THETADIRS);
      // @avemula ADDED time dimensions
      int newT = HashEntry->T - nav3daction->dT;
      float newTempT = HashEntry->TempT - nav3daction->dT;	

      //skip the invalid cells
      if(!IsValidCell(newX, newY)){ 
	continue;
      }

      if(lookupCoords(newX, newY) != 0){
	//predecessor falls outside all spheres
	if(lookupCoords(HashEntry->X, HashEntry->Y)!=0) {
	  continue;
	}
			
	if(trackMode) {
	  //discard in tracking mode
	  continue;
	} else {
	  //make 2D in planning mode
	  //get cost
	  unsigned int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav3daction, false);
	  if(cost >= INFINITECOST){
	    //discard infinite cost transitions
	    continue;
	  }
	  // @avemula ADDED both theta and time must be invalid in 2D case
	  newTheta = INVALID_THETA;
	  newT = INVALID_TIME;
	  EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
	  if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	    {
	      //have to create a new entry
	      OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	    }

	  PredIDV->push_back(OutHashEntry->stateID);
	  if(valid){
	    CostV->push_back((int) (cost_mult * cost));
	  } else {
	    CostV->push_back(INFINITECOST);
	  }
	}
      } else {
	//predecessor falls in sphere
			
	double cc_startT = ros::Time::now().toSec();
	if(!IsValidConfiguration(newX, newY, newTheta)){
	  if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  continue;
	}
	unsigned int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, HashEntry->T, nav3daction, true);
	if(cost >= INFINITECOST){
	  if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	  //discard infinite cost transitions
	  continue;
	}
	if(trackMode){ tCollCheckTime += (ros::Time::now().toSec() - cc_startT); }
	else { pCollCheckTime += (ros::Time::now().toSec() - cc_startT); }

	EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
	if((OutHashEntry = (this->*GetHashEntry)(newX, newY, newTheta, newT)) == NULL)
	  {
	    //have to create a new entry
	    OutHashEntry = (this->*CreateNewHashEntry)(newX, newY, newTheta, newT, newTempT);
	  }
	PredIDV->push_back(OutHashEntry->stateID);
	if(valid){
	  CostV->push_back((int) (cost_mult * cost));
	} else {
	  CostV->push_back(INFINITECOST);
	}
      }
    }
  if(!trackMode) get4DPredTime += (ros::Time::now().toSec() - startT);
}


unsigned int EnvironmentNAVXYTHETATIMELATADEXACT::lookupCoords(unsigned int x, unsigned int y){
  if(!sphereStateGrid){
    printf("sphereStateGrid not initialized!");
    throw SBPL_Exception();
  }

  if(FORCE3DPLANNING){
    return 0;
  }

  char tmp = sphereStateGrid[x][y];
  char p = tmp % 4; //planning mode value
  char t = (tmp>>2) % 4; //tracking mode value       
	
  if (trackMode) {
    //tracking
    return t;
  } else {
    return p;
  }
}


bool EnvironmentNAVXYTHETATIMELATADEXACT::stopTracking(int StateID){
  if((ros::Time::now().toSec() - LastBestTrackedUpdate) > TRACKING_TIMEOUT){
    EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[StateID];
    EnvNAVXYTHETATIMELATHashEntry_t* GoalEntry = StateID2CoordTable[EnvNAVXYTHETATIMELAT.goalstateid];
    int distsq = (HashEntry->X - GoalEntry->X)*(HashEntry->X - GoalEntry->X) + (HashEntry->Y - GoalEntry->Y) * (HashEntry->Y - GoalEntry->Y);
    if(distsq > 20*20) { 
      //SBPL_ERROR("Far from goal and no advance!--stopping");
      //no tracking advance in a long time and far from goal
      //stop generating tracking successors to terminate the search
      return true;
    }
  }
  return false;
}

// @avemula MODIFIED two versions of getsuccs track one with 3D expansions and the other with 4D expansions
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::GetSuccs_Track_3D(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV, std::vector<EnvNAVXYTHETATIMELATAction_t*> *actionV)
{
  if(!trackMode){
    ROS_WARN("GetSuccs_Track called in planning mode! Shouldn't happen!");
    throw new SBPL_Exception();
    return;
  }
	
  expandsTrack++;
  expandsTrack_tot++;
  Get3DSuccs_HD(SourceStateID, SuccIDV, CostV, actionV);	
}


// @avemula ASK
void EnvironmentNAVXYTHETATIMELATADEXACT::GetSuccs_Track_4D(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV, std::vector<EnvNAVXYTHETATIMELATAction_t*> *actionV)
{
  if(!trackMode){
    ROS_WARN("GetSuccs_Track called in planning mode! Shouldn't happen!");
    throw new SBPL_Exception();
    return;
  }

  expandsTrack++;
  expandsTrack_tot++;
  stat_->recordTrackExpansion();
  // @avemula DOUBT do we need to count this as a 4D expansion too?
  Get4DSuccs(SourceStateID, SuccIDV, CostV, actionV);	
}


// @avemula MODIFIED two versions of getsuccsplan
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::GetSuccs_Plan_3D(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV, std::vector<EnvNAVXYTHETATIMELATAction_t*> *actionV)
{
  //printf("GetSuccs_Plan(%d)\n", SourceStateID); fflush(stdout);
  if(trackMode){
    ROS_WARN("GetSuccs_Plan called in tracking mode! Shouldn't happen!");
    throw new SBPL_Exception();
    return;
  }
	
  int iState_x, iState_y, iState_theta, iState_t;

  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
  iState_x = HashEntry->X;
  iState_y = HashEntry->Y;
  iState_theta = HashEntry->Theta;
  iState_t = HashEntry->T;
  //printf("looking up coords %d,%d\n", iState_x, iState_y);
  char t = lookupCoords(iState_x, iState_y);
  if(t == 0){
    //in sphere
    //printf("in sphere\n"); fflush(stdout);
    if(iState_theta < INVALID_THETA){
      //valid 3D state
      expandsGrid[iState_x][iState_y] = 3;
      expands3D++;
      expands3D_tot++;
      Get3DSuccs_HD(SourceStateID, SuccIDV, CostV, actionV);
    } /*else {
      //invalidated 2D state
      expands2D++;
      Get2DSuccs(SourceStateID, SuccIDV, CostV);
      }*/
  }
  else if (t==1){
    //printf("near sphere\n"); fflush(stdout);
    //near sphere
    if(iState_theta < INVALID_THETA){
      //must be start or goal
      if(SourceStateID != EnvNAVXYTHETATIMELAT.goalstateid && SourceStateID != EnvNAVXYTHETATIMELAT.startstateid){
	SBPL_ERROR("Currently not in sphere, but valid angle!!! Should not happen except for start and goal!");
	throw new SBPL_Exception();
      }
    }
    //valid 2D state - get 2D succs and 3D succs
    expandsGrid[iState_x][iState_y] = 2;
    expandsN3D++;
    expandsN3D_tot++;
    Get2DSuccs(SourceStateID, SuccIDV, CostV, actionV);
    for(unsigned int th = 0; th < INVALID_THETA; th++){
      Get3DSuccs_HD(GetStateFromCoord(iState_x, iState_y, th, iState_t), SuccIDV, CostV, actionV);
    }
  } else {
    //printf("far from sphere\n"); fflush(stdout);
    //far from sphere
    if(iState_theta < INVALID_THETA){
      //must be start or goal
      if(SourceStateID != EnvNAVXYTHETATIMELAT.goalstateid && SourceStateID != EnvNAVXYTHETATIMELAT.startstateid){
	SBPL_ERROR("Currently not in sphere, but valid angle!!! Should not happen except for start and goal!");
	throw new SBPL_Exception();
      }
    }
    expandsGrid[iState_x][iState_y] = 1;
    expands2D++;
    expands2D_tot++;
    //valid 2D state - get 2D succs only
    Get2DSuccs(SourceStateID, SuccIDV, CostV, actionV);
  }
  //printf("Done!\n");
}

// @avemula MODIFIED v2.0 to match the exact case
// @avemula ASK
void EnvironmentNAVXYTHETATIMELATADEXACT::GetSuccs_Plan_4D(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV, std::vector<EnvNAVXYTHETATIMELATAction_t*> *actionV)
{
  
  if(trackMode){
    ROS_WARN("GetSuccs_Plan called in tracking mode! Shouldn't happen!");
    throw new SBPL_Exception();
    return;
  }
	
  int iState_x, iState_y, iState_theta, iState_t;
  float iState_tempT;

  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];
  iState_x = HashEntry->X;
  iState_y = HashEntry->Y;
  iState_theta = HashEntry->Theta;
  iState_t = HashEntry->T;
  iState_tempT = HashEntry->TempT;


  // @avemula DEBUG
  /*printf("Plan expanding %d %d %d %d with heuristic %d\n", iState_x, iState_y, iState_theta, iState_t, GetGoalHeuristic(SourceStateID));
    fflush(stdout);*/
  
  char t = lookupCoords(iState_x, iState_y);
  if(t == 0){
    //in sphere
    if(iState_theta < INVALID_THETA && iState_t < INVALID_TIME){
      //valid 4D state      
      expandsGrid[iState_x][iState_y] = 3;
      expands4D++;
      expands4D_tot++;
      Get4DSuccs(SourceStateID, SuccIDV, CostV, actionV);
      stat_->recordHDExpansion();
    }
    else {
      //invalidated 2D state
      printf("2D state inside a sphere. SHould not happen!\n");
      throw new SBPL_Exception();
      //expands2D++;
      //Get2DSuccs(SourceStateID, SuccIDV, CostV);
    }
  }
  else if (t==1){
    //near sphere
    // @avemula ASK important case  (what to do for near 4D state?)
    
    if(iState_theta < INVALID_THETA && iState_t < INVALID_TIME){
      //must be start or goal
      if(SourceStateID != EnvNAVXYTHETATIMELAT.goalstateid && SourceStateID != EnvNAVXYTHETATIMELAT.startstateid){
	SBPL_ERROR("Currently not in sphere, but valid time!!! Should not happen except for start and goal!");
	throw new SBPL_Exception();
      }
    }

    
    //valid 2D state - get 2D succs and 4D succs
    expandsGrid[iState_x][iState_y] = 2;
    expandsN4D++;
    expandsN4D_tot++;

    stat_->recordNearHDExpansion();
    
    Get2DSuccs(SourceStateID, SuccIDV, CostV, actionV);


    // @avemula NOTES
    /**
       There are multiple ways we could deal with this
       1. Since, we dont know theta and t of the 2D state, we can project it into all possible 4D states and get successors of those 4D states. Theta always takes all the 16 values. But t,
          a. t takes values from 0 to MAX_TIME
	  b. t takes values from tempT to MAX_TIME
	  c. t takes values from startTime (t-r/v) to endTime (t+r/v)
	  d. t takes only the value startTime
	  e. Use Get3DSuccs_allT with cost added for time jumps
       2. How do you deal with the cost associated with the time jump? It should be added in somehow
     **/

    // Version a
    /*for(int th=0; th < INVALID_THETA; th++) {
      if(!IsValidConfiguration(iState_x, iState_y, th))
	continue;

      for(int tim=0; tim < MAX_TIME; tim++) {
	Get4DSuccs(GetStateFromCoord(iState_x, iState_y, th, tim), SuccIDV, CostV, actionV);
      }
      }*/

    // Version b
    /*for(int th=0; th < INVALID_THETA; th++) {
      if(!IsValidConfiguration(iState_x, iState_y, th))
	continue;

      for(int tim=(int)iState_tempT; tim < MAX_TIME; tim++) {
	Get4DSuccs(GetStateFromCoord(iState_x, iState_y, th, tim), SuccIDV, CostV, actionV);
      }
      }*/


    // Version c
    /*int sIndex = sphereIndexGrid_plan[iState_x][iState_y];
    int startTime = max(spheres[sIndex].t_d - (int) ((spheres[sIndex].r/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs)/EnvNAVXYTHETATIMELATCfg.timeResolution), 0);
    int endTime = min(spheres[sIndex].t_d + (int) ((spheres[sIndex].r/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs)/EnvNAVXYTHETATIMELATCfg.timeResolution), MAX_TIME);
    for(int th=0; th < INVALID_THETA; th++) {
      if(!IsValidConfiguration(iState_x, iState_y, th))
	continue;

      for(int tim=startTime; tim < endTime; tim++) {
	Get4DSuccs(GetStateFromCoord(iState_x, iState_y, th, tim), SuccIDV, CostV, actionV);
      }
      }*/


    // Version d
    /*int sIndex = sphereIndexGrid_plan[iState_x][iState_y];
    int startTime = max(spheres[sIndex].t_d - (int) ((spheres[sIndex].r/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs)/EnvNAVXYTHETATIMELATCfg.timeResolution), 0);

    for(int th=0; th < INVALID_THETA; th++) {
      if(!IsValidConfiguration(iState_x, iState_y, th))
	continue;

      Get4DSuccs(GetStateFromCoord(iState_x, iState_y, th, startTime), SuccIDV, CostV, actionV);
      }*/

    // Version e
    for(int th=0; th < INVALID_THETA; th++) {
      if(!IsValidConfiguration(iState_x, iState_y, th))
	continue;

      Get3DSuccs_allT(GetStateFromCoord(iState_x, iState_y, th, iState_t), SuccIDV, CostV, actionV);
    }
    
    
    // @avemula ADDED cannot do the same as before since theta is defined now. We need to generate successors according to tempT
    // @avemula REMOVED v2.0 this doesn't make sense anymore as we have 2d - 4d and following exact method
    //Get4DSuccs(GetStateFromCoord(iState_x, iState_y, iState_theta, iState_tempT), SuccIDV, CostV, actionV);
    // @avemula ADDED all possible successor states
    /*int sIndex = sphereIndexGrid_plan[iState_x][iState_y];
    int startTime = max(spheres[sIndex].t_d - (int) ((spheres[sIndex].r/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs)/EnvNAVXYTHETATIMELATCfg.timeResolution), 0);
    for(unsigned int th=0; th < INVALID_THETA; th++) {      
      // @avemula ADDED a new function to generate all HD successors from a N4D state
      if(!IsValidConfiguration(iState_x, iState_y, th))
	continue;
      
      //Get3DSuccs_allT(GetStateFromCoord(iState_x, iState_y, th, iState_t, iState_tempT), SuccIDV, CostV, actionV);

      // @avemula ADDED new version of n4d expansion      
      for(int tim=startTime; tim < startTime + 1; tim++) {
	Get4DSuccs(GetStateFromCoord(iState_x, iState_y, th, tim, tim), SuccIDV, CostV, actionV);
	}
      
	}    */
  }
  else {
    //far from sphere
    if(iState_theta < INVALID_THETA && iState_t < INVALID_TIME){
      //must be start or goal
      if(SourceStateID != EnvNAVXYTHETATIMELAT.goalstateid && SourceStateID != EnvNAVXYTHETATIMELAT.startstateid){
	SBPL_ERROR("Currently not in sphere, but valid angle!!! Should not happen except for start and goal!");
	throw new SBPL_Exception();
      }
    }
    expandsGrid[iState_x][iState_y] = 1;
    expands2D++;
    expands2D_tot++;
    stat_->recordLDExpansion();
    Get2DSuccs(SourceStateID, SuccIDV, CostV, actionV);
  }
}

// @avemula MODIFIED so that it accounts for both the spaces
// @avemula ASK
void EnvironmentNAVXYTHETATIMELATADEXACT::GetSuccs(int SourceStateID, std::vector<int> *SuccIDV, std::vector<int> *CostV, std::vector<EnvNAVXYTHETATIMELATAction_t*> *actionV)
{
  //clear output vectors
  double startT = ros::Time::now().toSec();
  
  SuccIDV->clear();
  CostV->clear();
  if(actionV != NULL) actionV->clear();

  // @avemula DOUBT This *cannot* be the check for whether goal has been reached since goalstateid entry has INVALID_TIME
  //goal state is absorbing for successors
  if(SourceStateID == EnvNAVXYTHETATIMELAT.goalstateid){
    return;
  }

  // @avemula ADDED a new function to check if it is goal
  if(isGoal(SourceStateID)) {
    return;
  }
	
  if(FORCE3DPLANNING){
    
    GetSuccs_Plan_4D(SourceStateID, SuccIDV, CostV, actionV);
    getPSuccTime += (ros::Time::now().toSec() - startT);
    return;
  }
	
  if(trackMode){
    GetSuccs_Track_4D(SourceStateID, SuccIDV, CostV, actionV);
    getTSuccTime += (ros::Time::now().toSec() - startT);
  }

  else {
    GetSuccs_Plan_4D(SourceStateID, SuccIDV, CostV, actionV);
    getPSuccTime += (ros::Time::now().toSec() - startT);
  }
}

// @avemula Two versions of the gettrackingmodstates function
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::getTrackingModStates_3D(std::vector<int>* planModStates, std::vector<int>* trkModStates){
  trkModStates->clear();
  for(uint i = 0; i < planModStates->size(); i++){
    int x,y,th,t;
    GetCoordFromState(planModStates->at(i), x, y, th, t);
    if(th == INVALID_THETA){
      //low-d state -- check if its high-d projections exist? if they do then they are modified
      for(int th = 0; th < INVALID_THETA; th++){
	if(ExistsStateFromCoord(x,y,th,t)){
	  trkModStates->push_back(GetStateFromCoord(x,y,th,t));
	}
      }
    } else {
      //high-d state
      trkModStates->push_back(planModStates->at(i));
    }
  }
  ROS_ERROR("Got %d tracking modified states!", (int)trkModStates->size());
}

// @avemula USED and MODIFIED v2.0 accordingly
void EnvironmentNAVXYTHETATIMELATADEXACT::getTrackingModStates_4D(std::vector<int>* planModStates, std::vector<int>* trkModStates){
  trkModStates->clear();
  for(uint i = 0; i < planModStates->size(); i++){
    int x,y,th,t;
    float tempT;
    GetCoordFromState(planModStates->at(i), x, y, th, t, tempT);
    if(th>=INVALID_THETA || t >= INVALID_TIME){
      //low-d state -- check if its high-d projections exist? if they do then they are modified
      for(int th = 0; th < INVALID_THETA; th++){
	for(int tim=0; tim < MAX_TIME; tim++) {
	  if(ExistsStateFromCoord(x,y,th,tim)){
	    trkModStates->push_back(GetStateFromCoord(x,y,th,tim));
	  }
	}
      }
      // @avemula REMOVED v2.0
      /*if (ExistsStateFromCoord(x,y,th,tempT))
	trkModStates->push_back(GetStateFromCoord(x,y,th,tempT));*/
    } else {
      //high-d state
      trkModStates->push_back(planModStates->at(i));
    }
  }
  ROS_ERROR("Got %d tracking modified states!", (int)trkModStates->size());
}

// @avemula ADDED new function growAllSpheres
// @avemula NOTUSED v2.0
void EnvironmentNAVXYTHETATIMELATADEXACT::growAllSpheres() {
  for(int i=spheres.size()-1; i>=1; i--) {
    double x,y,r;    
    int x_d,y_d;
    r = spheres[i].r*1.5;
    spheres[i].r *= 1.5;
    x = spheres[i].x;
    y = spheres[i].y;
    x_d = (int) (x / EnvNAVXYTHETATIMELATCfg.cellsize_m);
    y_d = (int) (y / EnvNAVXYTHETATIMELATCfg.cellsize_m);

    float tresh = NEARTRESH3D;
    int min_x = max(0, CONTXY2DISC(x - r - tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) - 1);
    int max_x = min(CONTXY2DISC(x + r + tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) + 1, EnvNAVXYTHETATIMELATCfg.EnvWidth_c-1);
    int min_y = max(0, CONTXY2DISC(y - r - tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) - 1);
    int max_y = min(CONTXY2DISC(y + r + tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) + 1, EnvNAVXYTHETATIMELATCfg.EnvHeight_c-1);

    for(int i = min_x; i <= max_x; i++){
      for(int j = min_y; j <= max_y; j++){
	float dist = sqrt( (x - DISCXY2CONT(i, EnvNAVXYTHETATIMELATCfg.cellsize_m))*(x - DISCXY2CONT(i, EnvNAVXYTHETATIMELATCfg.cellsize_m)) + (y - DISCXY2CONT(j, EnvNAVXYTHETATIMELATCfg.cellsize_m))*(y - DISCXY2CONT(j, EnvNAVXYTHETATIMELATCfg.cellsize_m)) );
	char tmp = lookupCoords(i, j);
	char t = tmp; //near or in a shpere : no change

	// @avemula DOUBT COMMENTED the entire modstates part due to the unboundedness of the time dimension
	if (dist <= r){
	  t = 0;
	}
      
	if(dist <= r + tresh && t != 0){
	  t = 1;       
	}
      
	char tmpv = sphereStateGrid[i][j];
	char pv = tmpv % 4; //planning mode value
	char tv = (tmpv>>2) % 4; //tracking mode value
	if(!trackMode) {
	  //only affect planning and tracking grid
	  sphereStateGrid[i][j] = 4*t + t;
	} else {
	  //only affect tracking grid      
	  sphereStateGrid[i][j] = 4*t + pv;
	}
	
      }
    }    
  }
}

// @avemula MODIFED v2.0 to suit our scenario
// @avemula ASK
void EnvironmentNAVXYTHETATIMELATADEXACT::addSphere(int StateID, double r_, std::vector<int> *modStates)
{
	
  if(FORCE3DPLANNING) return;
  //if in tracking mode put the spheres in both grids
  //if in planning mode put the spheres in the tracking grid
  // in shpereStateGrid each char 00      :00      :00      :00
  //                              not used not used tracking planning
  
  int x_d, y_d, theta_d, t_d;
  float tempT_d;
  double x, y;
  double r = r_;
  GetCoordFromState(StateID, x_d, y_d, theta_d, t_d, tempT_d);
  x = x_d * EnvNAVXYTHETATIMELATCfg.cellsize_m;
  y = y_d * EnvNAVXYTHETATIMELATCfg.cellsize_m;
  // @avemula DEBUG
  
  
  bool growing = false;
  int sphere_detected = spheres.size();
	
  if(lookupCoords(x_d, y_d) == 0){
    //sphere is here already
    //find the sphere and grow it
    if(!trackMode) {
      for(int i = spheres.size() - 1; i >= 1; i--){
				
	if(((x-spheres[i].x)*(x-spheres[i].x) + (y-spheres[i].y)*(y-spheres[i].y)) < fabs(max(spheres[i].r - r_, 0.10))){
	  printf("Growing sphere!\n");
	  growing = true;
	  EnvNAVXYTHETATIMELATHashEntry_t* StartEntry = StateID2CoordTable[StartStateID];
	  r = 1.5*spheres[i].r;// + NEARTRESH3D + EnvNAVXYTHETATIMELATCfg.cellsize_m;
	  
	  spheres[i].r *= 1.5;
	  spheres[i].t_d = t_d;
	  x = spheres[i].x;
	  y = spheres[i].y;
	  
	  x_d = (int) (x / EnvNAVXYTHETATIMELATCfg.cellsize_m);
	  y_d = (int) (y / EnvNAVXYTHETATIMELATCfg.cellsize_m);
	  #ifdef VISUALIZATIONS_ENABLED
	  visualizeSphere(&(spheres[i]), "grown-sphere");
	  #endif
	  sphere_detected = i;
	  break;
	}
      }
    }
  }

  if (!trackMode) {
    int startTime = max(t_d - (int) ((r/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs)/EnvNAVXYTHETATIMELATCfg.timeResolution), 0);
    int endTime = min(t_d + (int) ((r/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs)/EnvNAVXYTHETATIMELATCfg.timeResolution), MAX_TIME);
    printf("Adding sphere at %f, %f with time %d and ranging from %d to %d\n", x, y, t_d, startTime, endTime);
    fflush(stdout);
  }

  //TODO: in track mode for each cell store the shortest distance and the index of the point along the path at that distance
  //so that given a cell, we can quickly look up the the nearest point along the path corresponding to that cell
  if(!trackMode && !growing){
    tSphere_exact* s = new tSphere_exact();
    spheres.push_back(*s);
    spheres[spheres.size()-1].x = x;
    spheres[spheres.size()-1].y = y;
    spheres[spheres.size()-1].r = r;
    spheres[spheres.size()-1].t_d = t_d;
    // @avemula COMMENTED visualization stuff
    #ifdef VISUALIZATIONS_ENABLED
    visualizeSphere(&(spheres[spheres.size()-1]), "new-sphere");
    #endif
  }

  float tresh = NEARTRESH3D;
  int min_x = max(0, CONTXY2DISC(x - r - tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) - 1);
  int max_x = min(CONTXY2DISC(x + r + tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) + 1, EnvNAVXYTHETATIMELATCfg.EnvWidth_c-1);
  int min_y = max(0, CONTXY2DISC(y - r - tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) - 1);
  int max_y = min(CONTXY2DISC(y + r + tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) + 1, EnvNAVXYTHETATIMELATCfg.EnvHeight_c-1);
  //int minStateID = -1;

  // @avemula DEBUG
  //printf("xdim : %d %d, ydim : %d %d\n", min_x, max_x, min_y, max_y);
  //fflush(stdout);
  
  for(int i = min_x; i <= max_x; i++){
    for(int j = min_y; j <= max_y; j++){
      //for (int th = 0; th < INVALID_THETA; th++) {
	
      float dist = sqrt( (x - DISCXY2CONT(i, EnvNAVXYTHETATIMELATCfg.cellsize_m))*(x - DISCXY2CONT(i, EnvNAVXYTHETATIMELATCfg.cellsize_m)) + (y - DISCXY2CONT(j, EnvNAVXYTHETATIMELATCfg.cellsize_m))*(y - DISCXY2CONT(j, EnvNAVXYTHETATIMELATCfg.cellsize_m)) );
      char tmp = lookupCoords(i, j);
      char t = tmp; //near or in a shpere : no change

      // @avemula DOUBT COMMENTED the entire modstates part due to the unboundedness of the time dimension
      // @avemula ADDED back v2.0
      if (dist <= r){
	t = 0;
	if(!trackMode) {
	  // @avemula ASK
	  sphereIndexGrid_plan[i][j] = sphere_detected;
	}
	//in sphere -- modified states in both tracking and planning
	/*if(modStates!=NULL && tmp != 0){ //not is sphere already
	    
	  if(!trackMode){
	      
	    if(ExistsStateFromCoord(i,j, INVALID_THETA, INVALID_TIME)){
	      int stateID = GetStateFromCoord(i,j,INVALID_THETA,INVALID_TIME);
	      if(stateID != StartStateID){
		modStates->push_back(stateID);
	      }
	    }
	  }
	    
	  else {
	    // @ avemula NEEDED? Do we need to do this because, this is expensive
	    for(int th = 0; th < INVALID_THETA; th++){
	      for(int tim=0; tim<INVALID_TIME; tim++) {
		if(ExistsStateFromCoord(i,j,th, tim)){
		  int stateID = GetStateFromCoord(i,j,th, tim);
		  if(stateID != StartStateID){
		    modStates->push_back(stateID);
		  }
		}
	      }
	    }
	    
	    }	    
	    }*/
      }
      
      if(dist <= r + tresh && t != 0){
	t = 1;

	if(!trackMode) {
	  //printf("Assigning %d,%d to %d\n", i, j, sphere_detected);
	  sphereIndexGrid_plan[i][j] = sphere_detected;
	}
	//near sphere -- modified states only in planning
	/*if(!trackMode && modStates!=NULL && tmp != 0 && tmp != 1 && ExistsStateFromCoord(i,j,INVALID_THETA, INVALID_TIME)){
	  //not near or in sphere already
	  int stateID = GetStateFromCoord(i,j,INVALID_THETA, INVALID_TIME);
	  if(stateID != StartStateID){
	    modStates->push_back(stateID);
	  }
	  }*/
      }

      char tmpv = sphereStateGrid[i][j];
      char pv = tmpv % 4; //planning mode value
      char tv = (tmpv>>2) % 4; //tracking mode value
      if(!trackMode) {
	//only affect planning and tracking grid
	sphereStateGrid[i][j] = 4*t + t;
      } else {
	//only affect tracking grid      
	sphereStateGrid[i][j] = 4*t + pv;
      }
    }
  }
}

// @avemula NOTADDED several visualization functions (can always be done later)
void EnvironmentNAVXYTHETATIMELATADEXACT::visualizeEnvironment()
{
	double start_t = ros::Time::now().toSec();
#ifdef VISUALIZATIONS_ENABLED
	//grid_->visualize();
	SBPL_INFO("Visualizing environment!");
	
	//draw obstacles
	std::vector<std::vector<double> > hard_obstacles_;
	std::vector<std::vector<double> > inflated_obstacles_inner_;
	std::vector<std::vector<double> > inflated_obstacles_outer_;
	std::vector<std::vector<double> > stateprob_pts_;
	std::vector<double> stateprob_col_;
	std::vector<std::vector<double> > region3D_;
	std::vector<std::vector<double> > regionN3D_;
	std::vector<std::vector<double> > expandsTrack_;
	std::vector<std::vector<double> > expands3D_;
	std::vector<std::vector<double> > expandsN3D_;
	std::vector<std::vector<double> > expands2D_;
	std::vector<std::vector<double> > heuristic_pts_;
	std::vector<double> heuristic_col_;
	for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){

		for(int j = 0; j < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; j++){
			std::vector<double> obs_(6, EnvNAVXYTHETATIMELATCfg.cellsize_m);
			obs_[0] = EnvNAVXYTHETATIMELATCfg.cellsize_m * i;
			obs_[1] = EnvNAVXYTHETATIMELATCfg.cellsize_m * j;
			obs_[2] = -0.05;
			
			
			if(inflatedGrid[i][j] >= EnvNAVXYTHETATIMELATCfg.obsthresh){
				//inflated obstacle
				inflated_obstacles_inner_.push_back(obs_);
			}
			if(EnvNAVXYTHETATIMELATCfg.Grid2D[i][j] >= EnvNAVXYTHETATIMELATCfg.obsthresh){
				//hard obstacle
				hard_obstacles_.push_back(obs_);
			}
			if(lookupCoords(i, j) == 0){
				region3D_.push_back(obs_);
			} else if(lookupCoords(i, j) == 1){
				regionN3D_.push_back(obs_);
			}
			switch(expandsGrid[i][j]){
				case 4: 
					expandsTrack_.push_back(obs_);
					break;
				case 3: 
					expands3D_.push_back(obs_);
					break;
				case 2: 
					expandsN3D_.push_back(obs_);
					break;
				case 1: 
					expands2D_.push_back(obs_);
					break;
				default:
					break;
			}
			#ifdef VISUALIZE_STATEPROB
			if(i % 2 == 0 && j % 2 == 0){
				double pVal = GetStateProb(i,j);
				if(pVal > 0 && cmap_updated){
					stateprob_pts_.push_back(obs_);
					stateprob_col_.push_back(120.0*pVal);
				}
			}
			#endif
			
			#ifdef VISUALIZE_HEURISTIC
			int gx, gy, gth,gt;
			double hVal;
			
			if(trackMode){
			  GetCoordFromState(EnvNAVXYTHETATIMELAT.goalstateid, gx, gy, gth,gt);
			  hVal = GetGoalHeuristic(GetStateFromCoord(i, j, gth,gt))/60.0f;
			} else {
			  GetCoordFromState(EnvNAVXYTHETATIMELAT.startstateid, gx, gy, gth,gt);
			  hVal = GetGoalHeuristic(GetStateFromCoord(i, j, gth,gt))/60.0f;
			}
			if(i % 2 == 0 && j % 2 == 0){
				heuristic_pts_.push_back(obs_);
				heuristic_col_.push_back((double) __max(240.0f - hVal, 0.0f));
			}
			/*if(fvalField[i][j] > 0){
				heuristic_pts_.push_back(obs_);
				double hVal = fvalField[i][j] * 0.0002f;
				heuristic_col_.push_back((double) __max(240.0f - hVal, 0.0f));
			}*/
			#endif
			expandsGrid[i][j] = 0;
		}
	}
	printf("Got %d 2D states\n", (int)expands2D_.size());
	printf("Got %d 4D states\n", (int)expands3D_.size());
	printf("Got %d N4D states\n", (int)expandsN3D_.size());
	printf("Got %d Track states\n", (int)expandsTrack_.size());
	#ifdef VISUALIZE_HEURISTIC
		//printf("Visualizing heuristic\n");
		//fflush(stdout);
		if(trackMode){
			navviz->visualizePoints(heuristic_pts_, heuristic_col_, std::string("heuristic_trk"));
		} else {
			navviz->visualizePoints(heuristic_pts_, heuristic_col_, std::string("heuristic_pln"));
		}
	#endif
	#ifdef VISUALIZE_STATEPROB
		navviz->visualizePoints(stateprob_pts_, stateprob_col_, std::string("stateprob"));
	#endif
	#ifdef VISUALIZE_OBSTACLES
		//name = new std::string("hard_obstacles");
		//navviz->visualizeSimpleObstacles(hard_obstacles_, name, 1, 1.0f);
		//delete name;
		printf("Number of hard obstacles is %d\n", (int)hard_obstacles_.size());
		navviz->visualizePoints(hard_obstacles_, 40, std::string("hard_obstacles"));
		#ifdef VISUALIZE_INFLATED_OBSTACLES
		//name = new std::string("inflated_obstacles");
		//navviz->visualizeSimpleObstacles(inflated_obstacles_, name, 60, 0.2f);
		//delete name;
		navviz->visualizePoints(inflated_obstacles_inner_, 30, std::string("inflated_obstacles_inner"));
		navviz->visualizePoints(inflated_obstacles_outer_, 120, std::string("inflated_obstacles_outer"));
		#endif
	#endif

		// @avemula TODO add visualization for dynamic obstacles

#ifdef VISUALIZE_DYNAMIC_OBSTACLES
		printf("Number of dynamic obstaces is %d\n", (int)dynamicObstacles.size());
		//navviz->visualizeDynamicObstacles(dynamicObstacles,
		for(int ind=0; ind<dynamicObstacles.size(); ind++) {

		  SBPL_DynamicObstacle_t dynObs = dynamicObstacles[ind];
		  // @avemula NOTE assuming that the dynamic obstacle will have only one trajectory
		  vector<SBPL_Traj_Pt_t> dynTraj = dynObs.trajectories[0].points;
		  std::vector<std::vector<double> > path;
		  
		  for(unsigned int di = 0; di < dynTraj.size(); di++){
		    SBPL_Traj_Pt_t pt = dynTraj[di];
		    std::vector<double> waypoint (3,0.0f);
		    waypoint[0] = pt.x;
		    waypoint[1] = pt.y;
		    //waypoint[2] = 0.0;
		    double radius = (dynObs.radius + pt.std_dev*3);
		    //navviz->visualizeCircle(waypoint, 30, std::string("dynObs")+ boost::lexical_cast<std::string>(di)+ boost::lexical_cast<std::string>(ind), radius);
		    path.push_back(waypoint);
		  }
		  navviz->visualizePath(path, 10, 10, std::string("dynamic_obstacle")+ boost::lexical_cast<std::string>(ind));
		  path.clear();		 
		}
		
#endif
		
	#ifdef VISUALIZE_REGIONS
		navviz->visualizePoints(region3D_, 120, std::string("3D_regions"));
		navviz->visualizePoints(regionN3D_, 60, std::string("near-3D_regions"));
	#endif
	#ifdef VISUALIZE_EXPANDED_STATES
		navviz->visualizePoints(expands2D_, 120, std::string("2D expansions"));
		navviz->visualizePoints(expands2D_, 120, std::string("2D expansions"));
		navviz->visualizePoints(expandsN3D_, 120, std::string("near-3D expansions"));
		navviz->visualizePoints(expandsN3D_, 120, std::string("near-3D expansions"));
		navviz->visualizePoints(expands3D_,  1, std::string("3D expansions"));
		navviz->visualizePoints(expands3D_,  1, std::string("3D expansions"));
		navviz->visualizePoints(expandsTrack_, 240, std::string("Track expansions"));
		navviz->visualizePoints(expandsTrack_, 240, std::string("Track expansions"));
	#endif
	#ifdef VISUALIZE_SPHERES
		for(unsigned int i = 0; i < spheres.size(); i++){
			visualizeSphere(&(spheres[i]), std::string("sphere") + boost::lexical_cast<std::string>(i));
		}
	#endif
	
	for(unsigned int i = 0; i < heuristic_pts_.size(); i++){
		heuristic_pts_[i].clear();
	}
	heuristic_pts_.clear();
	heuristic_col_.clear();
	for(unsigned int i = 0; i < hard_obstacles_.size(); i++){
		hard_obstacles_[i].clear();
	}
	hard_obstacles_.clear();
	for(unsigned int i = 0; i < inflated_obstacles_inner_.size(); i++){
		inflated_obstacles_inner_[i].clear();
	}
	inflated_obstacles_inner_.clear();
	for(unsigned int i = 0; i < inflated_obstacles_outer_.size(); i++){
		inflated_obstacles_outer_[i].clear();
	}
	inflated_obstacles_outer_.clear();
	for(unsigned int i = 0; i < region3D_.size(); i++){
		region3D_[i].clear();
	}
	region3D_.clear();
	for(unsigned int i = 0; i < regionN3D_.size(); i++){
		regionN3D_[i].clear();
	}
	regionN3D_.clear();
	for(unsigned int i = 0; i < expandsTrack_.size(); i++){
		expandsTrack_[i].clear();
	}
	expandsTrack_.clear();
	for(unsigned int i = 0; i < expands3D_.size(); i++){
		expands3D_[i].clear();
	}
	expands3D_.clear();
	for(unsigned int i = 0; i < expandsN3D_.size(); i++){
		expandsN3D_[i].clear();
	}
	expandsN3D_.clear();
	for(unsigned int i = 0; i < expands2D_.size(); i++){
		expands2D_[i].clear();
	}
	expands2D_.clear();
#else
	ROS_WARN("VISUALIZATIONS DISABLED! Recompile with #define VISUALIZATIONS_ENABLED set");
#endif
	printf("Done visualizing!\n"); fflush(stdout);
	vizTime += (ros::Time::now().toSec() - start_t);
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualize3DState(int x, int y, int theta, int color, std::string name){
#ifdef USEPR2
  BodyPose bp;
  bp.x = DISCXY2CONT(x, EnvNAVXYTHETATIMELATCfg.cellsize_m) - 0.5 * EnvNAVXYTHETATIMELATCfg.EnvWidth_c * EnvNAVXYTHETATIMELATCfg.cellsize_m;;
  bp.y = DISCXY2CONT(y, EnvNAVXYTHETATIMELATCfg.cellsize_m) - 0.5 * EnvNAVXYTHETATIMELATCfg.EnvHeight_c * EnvNAVXYTHETATIMELATCfg.cellsize_m;;
  bp.theta = DiscTheta2Cont(theta, INVALID_THETA);
  bp.z = bpose_.z;
  pviz_.visualizeRobot(rangles_, langles_, bp, color, name, 0);
#else 
  std::vector<double> coords(3,0.0);
  coords[0] = DISCXY2CONT(x, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  coords[1] = DISCXY2CONT(y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  coords[2] = DiscTheta2Cont(theta, INVALID_THETA);
  navviz->visualizeConfiguration(color, coords, name);
#endif
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualize3DState(std::string name, double x, double y, double theta, int color){
#ifdef USEPR2
  BodyPose bp;
  bp.x = x - 0.5 * EnvNAVXYTHETATIMELATCfg.EnvHeight_c * EnvNAVXYTHETATIMELATCfg.cellsize_m;
  bp.y = y - 0.5 * EnvNAVXYTHETATIMELATCfg.EnvHeight_c * EnvNAVXYTHETATIMELATCfg.cellsize_m;
  bp.theta = theta;
  bp.z = bpose_.z;
  pviz_.visualizeRobot(rangles_, langles_, bp, color, name, 0);
#else 
  std::vector<double> coords(3,0.0);
  coords[0] = x;
  coords[1] = y;
  coords[2] = theta;
  navviz->visualizeConfiguration(color, coords, name);
#endif
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualizeState(std::string name, std::vector<double> &coords, int col){
  visualize3DState(name, coords[0]+ 0.5 * EnvNAVXYTHETATIMELATCfg.EnvWidth_c * EnvNAVXYTHETATIMELATCfg.cellsize_m, coords[1] + 0.5 * EnvNAVXYTHETATIMELATCfg.EnvHeight_c * EnvNAVXYTHETATIMELATCfg.cellsize_m, coords[2], col);
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualize2DState(int x, int y, int color, std::string name){
  double x_d = DISCXY2CONT(x, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  double y_d = DISCXY2CONT(y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  std::vector<double> coords;
  coords.push_back(x_d);
  coords.push_back(y_d);
  coords.push_back(0.0f);
  navviz->visualizeSphere(coords, color, name, 0.10);
  coords.clear();
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualizeState(int stateID, int color, std::string name){
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  if(HashEntry->Theta >= ENVNAV2D_MAXDIRS) {
    visualize2DState(HashEntry->X, HashEntry->Y, color, name);
  } else {
    visualize3DState(HashEntry->X, HashEntry->Y, HashEntry->Theta, color, name);
  }
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualizeTransition(int sStateID, int tStateID, int color, std::string name){
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry1 = StateID2CoordTable[sStateID];
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry2 = StateID2CoordTable[tStateID];
  std::vector<double> c1 (6,0.0f);
  std::vector<double> c2 (6,0.0f);
  c1[0] = DISCXY2CONT(HashEntry1->X, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  c1[1] = DISCXY2CONT(HashEntry1->Y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  c2[0] = DISCXY2CONT(HashEntry2->X, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  c2[1] = DISCXY2CONT(HashEntry2->Y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  navviz->visualizeLine(c1, c2, color, name);
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualizeStates(std::vector<int> *stateIDs, int lcolor, int hcolor, std::string ns){
  if(stateIDs->size() == 0) return;
  std::vector<std::vector<double> > lStates;
  std::vector<std::vector<double> > hStates;
  for(unsigned int i = 0; i < stateIDs->size(); i++){
    int x, y, th,t;
    //SBPL_WARN("Getting state %d...", stateIDs->at(i));
    GetCoordFromState(stateIDs->at(i), x, y, th, t);
    if(th >= INVALID_THETA) {
      std::vector<double> t (6,0.02f);
      t[0] = DISCXY2CONT(x, EnvNAVXYTHETATIMELATCfg.cellsize_m); t[1] = DISCXY2CONT(y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
      lStates.push_back(t);
    } else {
      std::vector<double> tt (6,0.02f);
      tt[0] = DISCXY2CONT(x, EnvNAVXYTHETATIMELATCfg.cellsize_m); tt[1] = DISCXY2CONT(y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
      hStates.push_back(tt);
    }
  }
  //SBPL_WARN("Visualizing %d states...", stateIDs->size());
  if(lStates.size() > 0) {
    navviz->visualizePoints(lStates, lcolor, ns + "_lowD");
  }
  if(hStates.size() > 0) {
    navviz->visualizePoints(hStates, hcolor, ns + "_highD");
  }
  lStates.clear();
  hStates.clear();
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualizeStates(std::vector<int> *stateIDs, std::vector<int> *colorsV, std::string ns){
  if(stateIDs->size() == 0) return;
  std::vector<std::vector<double> > PtsV;
  std::vector<double> ColV;
  for(unsigned int i = 0; i < stateIDs->size(); i++){
    int x, y, th,tim;
    GetCoordFromState(stateIDs->at(i), x, y, th,tim);
    std::vector<double> t (6,0.02f);
    t[0] = DISCXY2CONT(x, EnvNAVXYTHETATIMELATCfg.cellsize_m); t[1] = DISCXY2CONT(y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
    PtsV.push_back(t);
    ColV.push_back((double)min(max(colorsV->at(i) / 200, 0), 240));
  }
  navviz->visualizePoints(PtsV, ColV, ns);
  PtsV.clear();
  ColV.clear();
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualizeStatePath(std::vector<int> *stateID_v, int scolor, int ecolor, std::string name){
  //float color_step = (ecolor - scolor) / ((float) stateID_v->size());
  std::vector<std::vector<double> > path;
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry;
  for(unsigned int i = 0; i < stateID_v->size(); i++){
    HashEntry = StateID2CoordTable[stateID_v->at(i)];
    std::vector<double> waypoint (2,0.0f);
    waypoint[0] = HashEntry->X * EnvNAVXYTHETATIMELATCfg.cellsize_m;
    waypoint[1] = HashEntry->Y * EnvNAVXYTHETATIMELATCfg.cellsize_m;
    if(HashEntry->Theta < INVALID_THETA && scolor != 1){
      //visualize3DState(HashEntry->X, HashEntry->Y, HashEntry->Theta, scolor + i*color_step, "waypoint" + boost::lexical_cast<string>(i));
    }
    path.push_back(waypoint);
  }
  navviz->visualizePath(path, scolor, ecolor, name);
  path.clear();
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualizePath(std::vector<int> *path, std::string name){
  //std::vector<EnvNAVXYTHETATIMELAT3Dpt_t> xythetaPath;
  std::vector<int> costs;
  //int cost;
  #ifdef VISUALIZATIONS_ENABLED
  visualizeStatePath(path, 0, 240, name);
  #endif
  //ConvertStateIDPathintoXYThetaPath(path, &xythetaPath, &cost, &costs);
  //visualizeTrajectory(&xythetaPath, 0, 240);
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualizeTrajectory(std::vector<SBPL_4Dpt_t> *traj, int scolor, int ecolor, std::string name){
  int throttle = max((int)traj->size() / 400, 2);
  std::vector<std::vector<double> > path;
  for(unsigned int i = 0; i < traj->size(); i+=throttle){
    std::vector<double> waypoint (2,0.0f);
    waypoint[0] = traj->at(i).x;
    waypoint[1] = traj->at(i).y;
    path.push_back(waypoint);
  }
  navviz->visualizePath(path, scolor, ecolor, name);
  path.clear();
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualizeTrajectory(std::vector<std::vector<double> > *traj, int scolor, int ecolor, std::string name){
  int throttle = max((int)traj->size() / 400, 2);
  std::vector<std::vector<double> > path;
  for(unsigned int i = 0; i < traj->size(); i+=throttle){
    std::vector<double> waypoint (2,0.0f);
    waypoint[0] = traj->at(i)[0];
    waypoint[1] = traj->at(i)[1];
    path.push_back(waypoint);
  }
  navviz->visualizePath(path, scolor, ecolor, name);
  path.clear();
}

void EnvironmentNAVXYTHETATIMELATADEXACT::visualizeSphere(tSphere_exact* sphere, std::string name)
{
	double start_t = ros::Time::now().toSec();
	std::vector<double> pos;
	pos.push_back(sphere->x);
	pos.push_back(sphere->y);
	pos.push_back(0.0);
	navviz->visualizeCircle(pos, 200, name, sphere->r);
	//navviz->visualizeCircle(pos, 30, name + "_outer", sphere->r + 0.4);
	vizTime += (ros::Time::now().toSec() - start_t);

}


// @avemula MODIFIED the following function to suit the 4D case. Some bugs might result
// @avemula MODIFIED again v2.0 to suit the 2D-4D case
void EnvironmentNAVXYTHETATIMELATADEXACT::ConvertStateIDPathintoXYThetaPath(std::vector<int>* stateIDPath, std::vector<SBPL_4Dpt_t>* xythetaPath, int *cost, std::vector<int> *costs)
{
  if(cost != NULL) *cost = 0;
  if(costs != NULL){
    costs->clear();
    costs->push_back(0);
  }
  
  std::vector<EnvNAVXYTHETATIMELATAction_t*> actionV;
  std::vector<int> CostV;
  std::vector<int> SuccIDV;
  int targetx_c, targety_c, targettheta_c, targett_c;
  float targettempT_c;
  int sourcex_c, sourcey_c, sourcetheta_c, sourcet_c;
  float sourcetempT_c;

  
  
  xythetaPath->clear();
	
  try {

    for(int pind = 0; pind < (int)(stateIDPath->size())-1; pind++)
      {
	int sourceID = stateIDPath->at(pind);
	int targetID = stateIDPath->at(pind+1);


#if DEBUG
	//GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
#endif
	
	GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targett_c, targettempT_c);
	if(targettheta_c == INVALID_THETA){
	  //SBPL_ERROR("Got to the end of the 3D chunk");
	  break;
	}
	/*if (targett_c == INVALID_TIME) {
	  //SBPL_ERROR("Got to the end of the 4D chunk");
	  // @avemula CHANGED commented the break
	  //break;
	  }*/


	//get successors and pick the target via the cheapest action
	SuccIDV.clear();
	CostV.clear();
	actionV.clear();
	GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);
		
	int bestcost = INFINITECOST;
	int bestsind = -1;

#if DEBUG
	//GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c);
	//GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c);
	//fprintf(fDeb, "looking for %d %d %d -> %d %d %d (numofsuccs=%d)\n", sourcex_c, sourcey_c, sourcetheta_c,
	//			targetx_c, targety_c, targettheta_c, SuccIDV.size()); 
#endif

	for(int sind = 0; sind < (int)SuccIDV.size(); sind++)
	  {
#if DEBUG
	    //int x_c, y_c, theta_c;
	    //GetCoordFromState(SuccIDV[sind], x_c, y_c, theta_c);
	    //fprintf(fDeb, "succ: %d %d %d\n", x_c, y_c, theta_c); 
#endif

	    if(SuccIDV[sind] == targetID && CostV[sind] <= bestcost)
	      {
		//printf("Successor found!\n"); fflush(stdout);
		bestcost = CostV[sind];
		bestsind = sind;
	      }
	  }

		
	if(bestsind == -1)
	  {
	    SBPL_ERROR("ERROR: successor not found for transition:\n");
	    GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c, sourcetempT_c);
	    GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targett_c, targettempT_c);
	    // @avemula POSSIBILITY can be changed for debugging 
	    SBPL_ERROR("%d %d %d (%d) -> %d %d %d (%d)\n", sourcex_c, sourcey_c, sourcetheta_c, 
		       lookupCoords(sourcex_c, sourcey_c),
		       targetx_c, targety_c, targettheta_c,
		       lookupCoords(targetx_c, targety_c));
	    //GetAllSuccs(sourceID, &SuccIDV, &CostV, &actionV);
	    SBPL_ERROR("Possible successors of source state:\n");
	    for(unsigned int iter = 0; iter < SuccIDV.size(); iter++){
	      int x, y, th,t;
	      float tempT;
	      GetCoordFromState(SuccIDV[iter], x, y, th,t,tempT);
	      printf("ID: %d \tX: %d\t Y: %d\t Th: %d\n", SuccIDV[iter], x, y, th);
	    }
	    return;
	    /*SuccIDV.clear();
	      CostV.clear();
	      GetPreds(targetID, &SuccIDV, &CostV);
	      SBPL_ERROR("Possible predecessors of target state:\n");
	      for(unsigned int iter = 0; iter < SuccIDV.size(); iter++){
	      int x, y, th;
	      GetCoordFromState(SuccIDV[iter], x, y, th);
	      printf("ID: %d \tX: %d\t Y: %d\t Th: %d\n", SuccIDV[iter], x, y, th);
	      }*/
	    throw new SBPL_Exception();
	  }

	if(cost != NULL){
	  *cost += bestcost;
	}
	if(costs != NULL){
	  costs->push_back(*cost);
	}

	//now push in the actual path

	int sourcex_c, sourcey_c, sourcetheta_c, sourcet_c;
	float sourcetempT_c;
	GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcetheta_c, sourcet_c, sourcetempT_c);
	GetCoordFromState(targetID, targetx_c, targety_c, targettheta_c, targett_c, targettempT_c);
	double sourcex, sourcey, sourcet, targetx, targety, targett;
	float sourcetempT;
	sourcex = DISCXY2CONT(sourcex_c, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	sourcey = DISCXY2CONT(sourcey_c, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	targetx = DISCXY2CONT(targetx_c, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	targety = DISCXY2CONT(targety_c, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	targett = DISCXY2CONT(targett_c, EnvNAVXYTHETATIMELATCfg.timeResolution);
	sourcet = DISCTIME2CONT(sourcet_c, EnvNAVXYTHETATIMELATCfg.timeResolution);
	sourcetempT = DISCTIME2CONT(sourcetempT_c, EnvNAVXYTHETATIMELATCfg.timeResolution);


	// @avemula DEBUG printing the coordinates of the waypoints in the solution path
	//printf("Source : %d, %d, %d, %d, %d\n", sourcex_c, sourcey_c, sourcetheta_c, sourcet_c, sourcetempT_c);
	//printf("Target : %d, %d, %d, %d, %d\n", targetx_c, targety_c, targettheta_c, targett_c, targettempT_c);
	//fflush(stdout);
	
	//Check if actionV[bestsind] == NULL
	if(actionV[bestsind] == NULL) {
	  //pushback the start and end point
	  /*EnvNAVXYTHETATIMELAT3Dpt_t intermpt;
	    for(float t = 0; t <= 1; t+=1){
	    intermpt.x = (1-t) * sourcex + t * targetx;
	    intermpt.y = (1-t) * sourcey + t * targety;
	    if(sourcex == targetx && sourcey > targety) {
	    intermpt.theta = DiscTheta2Cont(12, ENVNAV2D_MAXDIRS); //going straight up
	    } else if(sourcex == targetx && sourcey < targety) {
	    intermpt.theta = DiscTheta2Cont(4, ENVNAV2D_MAXDIRS); //going straight down
	    } else if(sourcex > targetx && sourcey > targety) {
	    intermpt.theta = DiscTheta2Cont(10, ENVNAV2D_MAXDIRS); //going up and left
	    } else if(sourcex > targetx && sourcey < targety) {
	    intermpt.theta = DiscTheta2Cont(6, ENVNAV2D_MAXDIRS); //going down and left
	    } else if(sourcex < targetx && sourcey > targety) {
	    intermpt.theta = DiscTheta2Cont(14, ENVNAV2D_MAXDIRS); //going up and right
	    } else if(sourcex < targetx && sourcey < targety) {
	    intermpt.theta = DiscTheta2Cont(2, ENVNAV2D_MAXDIRS); //going down and right
	    } else if(sourcex > targetx && sourcey == targety) {
	    intermpt.theta = DiscTheta2Cont(8, ENVNAV2D_MAXDIRS); //going left
	    } else if(sourcex < targetx && sourcey == targety) {
	    intermpt.theta = DiscTheta2Cont(0, ENVNAV2D_MAXDIRS); //going right
	    }
	    xythetaPath->push_back(intermpt);
	    }*/
	} else {
	  //TODO - when there are no motion primitives we should still print source state
	  int lastTh = 0;
	  int lastT = 0;
	  // @avemula DEBUG
	  //printf("Start\n");
	  for(int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size())-1; ipind++) 
	    {
	      //translate appropriately
	      //EnvNAVXYTHETATIMELAT3Dpt_t intermpt = actionV[bestsind]->intermptV[ipind];
	      SBPL_4Dpt_t intermpt = actionV[bestsind]->intermptV[ipind];
	      intermpt.x += sourcex;
	      intermpt.y += sourcey;
	      // @avemula ADDED v2.0 t variable can be used since tracking is done
	      intermpt.t += sourcet;
	      // @avemula CHANGE the above line is commented
	      // @avemula REMOVED v2.0 tempT variable not used
	      //intermpt.t += sourcetempT;
	      
	      // @avemula DEBUG
	      //printf("%.3f %.3f %.3f %.3f\n", intermpt.x, intermpt.y, intermpt.theta, intermpt.t);
	      //fflush(stdout);
	      
#if DEBUG
	      //int nx = CONTXY2DISC(intermpt.x, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	      //int ny = CONTXY2DISC(intermpt.y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	      //fprintf(fDeb, "%.3f %.3f %.3f (%d %d %d cost=%d) ", 
	      //	intermpt.x, intermpt.y, intermpt.theta, 
	      //	nx, ny, 
	      //	ContTheta2Disc(intermpt.theta, NAVXYTHETATIMELAT_THETADIRS), EnvNAVXYTHETATIMELATCfg.Grid2D[nx][ny]);
	      //if(ipind == 0) fprintf(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
	      //else fprintf(fDeb, "\n");
#endif

	      //store
	      xythetaPath->push_back(intermpt);
	      lastTh = ContTheta2Disc(intermpt.theta,ENVNAV2D_MAXDIRS);
	      lastT = CONTTIME2DISC(intermpt.t, EnvNAVXYTHETATIMELATCfg.timeResolution);
	    }
	  SBPL_4Dpt_t interm;
	  interm.x = targetx;
	  interm.y = targety;
	  // @avemula ADDED v2.0 theta could be unknown
	  if(targettheta_c >= INVALID_THETA){
	    interm.theta = DiscTheta2Cont(lastTh, ENVNAV2D_MAXDIRS);
	  } else {
	    interm.theta = DiscTheta2Cont(targettheta_c, ENVNAV2D_MAXDIRS);
	  }
	  // @avemula REMOVED v2.0 theta could be unknown
	  //interm.theta = DiscTheta2Cont(targettheta_c, ENVNAV2D_MAXDIRS);

	  
	  if(targettempT_c < INVALID_TIME)
	    interm.t = DISCTIME2CONT(targettempT_c, EnvNAVXYTHETATIMELATCfg.timeResolution);
	  else
	    interm.t = DISCTIME2CONT(lastT, EnvNAVXYTHETATIMELATCfg.timeResolution);

	  // @avemula CHANGED commented the following line
	  // @avemula NEEDED? should the following line be there
	  xythetaPath->push_back(interm);
	  // @avemula DEBUG
	  //printf("%.3f %.3f %.3f %.3f\n", interm.x, interm.y, interm.theta, interm.t);
	  //printf("End\n");
	  //fflush(stdout);
	}
      }
	
  } catch (exception& e){
    ROS_ERROR("Exception: %s", e.what());
    return;
  } catch (int err){
    ROS_ERROR("Exception: %d", err);
  }

}

// @avemula NOTUSED
EnvNAVXYTHETATIMELATHashEntry_t* EnvironmentNAVXYTHETATIMELATADEXACT::CreateNewHashEntry_lookup(int X, int Y, int Theta, int T, float tempT) 
{
  int i;
	

#if TIME_DEBUG	
  double currenttime = ros::Time::now().toSec();
#endif

  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = new EnvNAVXYTHETATIMELATHashEntry_t;

  HashEntry->X = X;
  HashEntry->Y = Y;
  HashEntry->Theta = Theta;
  HashEntry->T = T;
  HashEntry->TempT = tempT;
  HashEntry->iteration = 0;

  HashEntry->stateID = StateID2CoordTable.size();

  //insert into the tables
  StateID2CoordTable.push_back(HashEntry);

  int index = XYTHETATIME2INDEX_AD(X,Y,Theta,T);

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
      printf("%d vs. %d", HashEntry->stateID, (int)StateID2IndexMapping.size()-1);
      throw new SBPL_Exception();
    }

#if TIME_DEBUG
  time_createhash += ros::Time::now().toSec()-currenttime;
#endif
  createdHashEntries++;
  return HashEntry;
}



EnvNAVXYTHETATIMELATHashEntry_t* EnvironmentNAVXYTHETATIMELATADEXACT::CreateNewHashEntry_hash(int X, int Y, int Theta, int T, float TempT) 
{
  int i;

#if TIME_DEBUG	
  double currenttime = ros::Time::now().toSec();
#endif

  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = new EnvNAVXYTHETATIMELATHashEntry_t;

  HashEntry->X = X;
  HashEntry->Y = Y;
  HashEntry->Theta = Theta;
  HashEntry->T = T;
  HashEntry->TempT = TempT;
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
      printf("%d vs. %d", HashEntry->stateID, (int)StateID2IndexMapping.size()-1);
      throw new SBPL_Exception();	
    }

#if TIME_DEBUG
  time_createhash += ros::Time::now().toSec()-currenttime;
#endif
  createdHashEntries++;
  return HashEntry;
}

// @avemula COMMENTED out the whole function. Not essential and also puts a sphere at the start state
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::moveStartTo(int StateID, std::vector<int> *modStates){

  /*int mS_initSize = 0;
  if(modStates != NULL){
    mS_initSize = (int)modStates->size();
  }
  
  int x,y,theta,t,tempT;
  GetCoordFromState(StateID, x, y, theta, t, tempT);
  if(!IsValidConfiguration(x,y,theta))
    {
      SBPL_ERROR("Start configuration %d %d %d is invalid\n", x,y,theta);
      throw new SBPL_Exception();
    }
	
  printf("Moving start to (%d, %d, %d)!\n", x, y, theta); fflush(stdout);

  //need to recompute start heuristics?
  if(EnvNAVXYTHETATIMELAT.startstateid != StateID)
    {
      bNeedtoRecomputeStartHeuristics = true;
      bNeedtoRecomputeGoalHeuristics = true; //because termination condition can be not all states TODO - make it dependent on term. condition
    }
	
  float r = START_RADIUS;
  float tresh = NEARTRESH3D;
  float cx = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  float cy = DISCXY2CONT(EnvNAVXYTHETATIMELATCfg.StartY_c, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  float nx = DISCXY2CONT(x, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  float ny = DISCXY2CONT(y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
	
  int cmin_x = max(0, CONTXY2DISC(cx - r - tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) - 1);
  int cmax_x = min(CONTXY2DISC(cx + r + tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) + 1, EnvNAVXYTHETATIMELATCfg.EnvWidth_c-1);
  int cmin_y = max(0, CONTXY2DISC(cy - r - tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) - 1);
  int cmax_y = min(CONTXY2DISC(cy + r + tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) + 1, EnvNAVXYTHETATIMELATCfg.EnvHeight_c-1);
	
  int nmin_x = max(0, CONTXY2DISC(nx - r - tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) - 1);
  int nmax_x = min(CONTXY2DISC(nx + r + tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) + 1, EnvNAVXYTHETATIMELATCfg.EnvWidth_c-1);
  int nmin_y = max(0, CONTXY2DISC(ny - r - tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) - 1);
  int nmax_y = min(CONTXY2DISC(ny + r + tresh, EnvNAVXYTHETATIMELATCfg.cellsize_m) + 1, EnvNAVXYTHETATIMELATCfg.EnvHeight_c-1);
	
  int min_x = min(cmin_x, nmin_x);
  int max_x = max(cmax_x, nmax_x);
  int min_y = min(cmin_y, nmin_y);
  int max_y = max(cmax_y, nmax_y);
	
  char name[200];
  //int minStateID = -1;
  for(int i = min_x; i <= max_x; i++){
    for(int j = min_y; j <= max_y; j++){
      for(int th = 0; th <= INVALID_THETA; th++){
	if(ExistsStateFromCoord(i,j,th)){
	  int stateID = GetStateFromCoord(i,j,th);
	  if(stateID != StartStateID){
	    modStates->push_back(stateID);
	    sprintf(name, "startsphere%d%d\0", i, j);
	    //visualize2DState(i, j, 0, std::string(name));
	  }
	}
      } 
    }
  }
	
  //set start
  StartStateID = StateID;
  EnvNAVXYTHETATIMELAT.startstateid = StateID;
  EnvNAVXYTHETATIMELATCfg.StartX_c = x;
  EnvNAVXYTHETATIMELATCfg.StartY_c = y;
  EnvNAVXYTHETATIMELATCfg.StartTheta = theta;
	
  if(modStates != NULL){
    printf("Got %d modified states after moving the start!\n", (int)modStates->size());
    fflush(stdout);
    }*/
	
  //visualizeState(EnvNAVXYTHETATIMELAT.startstateid, 120, "Start state");
}


// @avemula NOTUSED
EnvNAVXYTHETATIMELATHashEntry_t* EnvironmentNAVXYTHETATIMELATADEXACT::GetHashEntry_lookup(int X, int Y, int Theta, int T)
{
  /*printf("EnvironmentNAVXYTHETATIMELATADEXACT::GetHashEntry_lookup(%d, %d, %d)\n", X, Y, Theta);
    fflush(stdout);*/
  if(!trackMode) {
    if(EnvNAVXYTHETATIMELAT.goalstateid < (int)StateID2CoordTable.size() && EnvNAVXYTHETATIMELAT.startstateid < (int)StateID2CoordTable.size()){
      
      EnvNAVXYTHETATIMELATHashEntry_t* Goal = StateID2CoordTable[EnvNAVXYTHETATIMELAT.goalstateid];
      EnvNAVXYTHETATIMELATHashEntry_t* Start = StateID2CoordTable[EnvNAVXYTHETATIMELAT.startstateid];
      
      /*if(X == Goal->X && Y == Goal->Y && Theta == INVALID_THETA){
	return Goal;
	}*/
      // @avemula IMPORTANT goal checking can be taken care here
      if (X==Goal->X && Y == Goal->Y && Theta == Goal->Theta && T >= INVALID_TIME)
	return Goal;
      
      /*if(X == Start->X && Y == Start->Y && Theta == INVALID_THETA){
	return Start;
	}*/

      if(X == Start->X && Y == Start->Y && Theta == Start->Theta && T >= INVALID_TIME)
	return Start;
      
    }
  }
  
  int index = XYTHETATIME2INDEX_AD(X,Y,Theta,T);
  /*printf("Idx: %d of %d\n", index, (int)EnvNAVXYTHETATIMELATCfg.EnvWidth_c*EnvNAVXYTHETATIMELATCfg.EnvHeight_c*(NAVXYTHETATIMELAT_THETADIRS+1));
    fflush(stdout);*/
  return Coord2StateIDHashTable_lookup[index];
}

// @avemula ASK
EnvNAVXYTHETATIMELATHashEntry_t* EnvironmentNAVXYTHETATIMELATADEXACT::GetHashEntry_hash(int X, int Y, int Theta, int T)
{

  if(!trackMode) {
    if(EnvNAVXYTHETATIMELAT.goalstateid < (int) StateID2CoordTable.size() && EnvNAVXYTHETATIMELAT.startstateid < (int) StateID2CoordTable.size()){
      
      EnvNAVXYTHETATIMELATHashEntry_t* Goal = StateID2CoordTable[EnvNAVXYTHETATIMELAT.goalstateid];
      EnvNAVXYTHETATIMELATHashEntry_t* Start = StateID2CoordTable[EnvNAVXYTHETATIMELAT.startstateid];

      // @avemula ADDED v2.0 Goal checking
      if(X == Goal->X && Y == Goal->Y){
	return Goal;
	}
      // @avemula CHANGED but DOUBT
      if (X==Goal->X && Y == Goal->Y && Theta == Goal->Theta)
	return Goal;
      
      /*if(X == Start->X && Y == Start->Y && Theta == INVALID_THETA){
	return Start;
	}*/
      if(X == Start->X && Y == Start->Y && Theta == Start->Theta && T >= INVALID_TIME)
	return Start;
    }
  }
  // @avemula ADDED goal check even in trackmode
  else {
    EnvNAVXYTHETATIMELATHashEntry_t* Goal = StateID2CoordTable[EnvNAVXYTHETATIMELAT.goalstateid];


    // @avemula ADDED bullshit
    if(X==Goal->X && Y == Goal->Y)
      return Goal;
    
    if (X==Goal->X && Y == Goal->Y && Theta == Goal->Theta)
      return Goal;
  }

#if TIME_DEBUG
  double currenttime = ros::Time::now().toSec();
#endif



  
  int binid = GETHASHBIN(X, Y, Theta, T);	

#if DEBUG
  if ((int)Coord2StateIDHashTable[binid].size() > 5)
    {
      SBPL_FPRINTF(fDeb, "WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n", 
		   binid, X, Y, (int)Coord2StateIDHashTable[binid].size());
		
      PrintHashTableHist(fDeb);		
    }
#endif

  //iterate over the states in the bin and select the perfect match
  vector<EnvNAVXYTHETATIMELATHashEntry_t*>* binV = &Coord2StateIDHashTable[binid];
  for(int ind = 0; ind < (int)binV->size(); ind++)
    {
      EnvNAVXYTHETATIMELATHashEntry_t* hashentry = binV->at(ind);
		
      if( hashentry->X == X  && hashentry->Y == Y && hashentry->Theta == Theta && hashentry->T == T)
	{
#if TIME_DEBUG
	  time_gethash += ros::Time::now().toSec()-currenttime;
#endif
	  return hashentry;
	}
    }

#if TIME_DEBUG	
  time_gethash += ros::Time::now().toSec()-currenttime;
#endif

  return NULL;	  
}



int EnvironmentNAVXYTHETATIMELATADEXACT::GetStateFromCoord(int x, int y, int theta, int t) {

  EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
  if((OutHashEntry = (this->*GetHashEntry)(x, y, theta, t)) == NULL){
    //have to create a new entry
    // @avemula MODIFIED used the same t as tempT for createnewhashentry. Possible bug
    OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta, t, t);
  }
  return OutHashEntry->stateID;
}


// @avemula ADDED a new getstatefromcoord function which also takes tempT
int EnvironmentNAVXYTHETATIMELATADEXACT::GetStateFromCoord(int x, int y, int theta, int t, float tempT) {

  EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
  if((OutHashEntry = (this->*GetHashEntry)(x, y, theta, t)) == NULL){
    //have to create a new entry
    // @avemula MODIFIED used the same t as tempT for createnewhashentry. Possible bug
    OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta, t, tempT);
  }
  return OutHashEntry->stateID;
}


bool EnvironmentNAVXYTHETATIMELATADEXACT::ExistsStateFromCoord(int x, int y, int theta, int t) {
    EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
    if((OutHashEntry = (this->*GetHashEntry)(x, y, theta, t)) == NULL){
        //have to create a new entry
	return false;
    }
    return true;
}


int EnvironmentNAVXYTHETATIMELATADEXACT::SetGoal(double x_m, double y_m, double theta_rad){
  
  int x = CONTXY2DISC(x_m + 0.5 * EnvNAVXYTHETATIMELATCfg.EnvWidth_c * EnvNAVXYTHETATIMELATCfg.cellsize_m, EnvNAVXYTHETATIMELATCfg.cellsize_m); 
  int y = CONTXY2DISC(y_m + 0.5 * EnvNAVXYTHETATIMELATCfg.EnvHeight_c * EnvNAVXYTHETATIMELATCfg.cellsize_m, EnvNAVXYTHETATIMELATCfg.cellsize_m);
  int theta = ContTheta2Disc(theta_rad, NAVXYTHETATIMELAT_THETADIRS);

  printf("env: setting goal to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m, theta_rad, x, y, theta);

  if(!IsWithinMapCell(x,y))
    {
      printf("ERROR: trying to set a goal cell %d %d that is outside of map\n", x,y);
      return -1;
    }

  if(!IsValidConfiguration(x,y,theta))
    {
      printf("WARNING: goal configuration is invalid\n");
    }

  EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
  // @avemula MODIFIED invalid_time as the t value for goal hashentry
  if((OutHashEntry = (this->*GetHashEntry)(x, y, theta, INVALID_TIME)) == NULL){
    //have to create a new entry
    OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta, INVALID_TIME, INVALID_TIME);
  }

  //need to recompute start heuristics?
  if(EnvNAVXYTHETATIMELAT.goalstateid != OutHashEntry->stateID)
    {
      bNeedtoRecomputeStartHeuristics = true; //because termination condition may not plan all the way to the new goal
      bNeedtoRecomputeGoalHeuristics = true; //because goal heuristics change
    }

  EnvNAVXYTHETATIMELAT.goalstateid = OutHashEntry->stateID;

  EnvNAVXYTHETATIMELATCfg.EndX_c = x;
  EnvNAVXYTHETATIMELATCfg.EndY_c = y;
  EnvNAVXYTHETATIMELATCfg.EndTheta = theta;
  
  #ifdef VISUALIZATIONS_ENABLED
  visualizeState(EnvNAVXYTHETATIMELAT.goalstateid, 240, "Goal State");
  #endif
  //precomputeHeuristicValues(x, y);
  ComputeHeuristicValues();
  return EnvNAVXYTHETATIMELAT.goalstateid;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAVXYTHETATIMELATADEXACT::SetStart(double x_m, double y_m, double theta_rad, double t_m){

  reset();

  SBPL_INFO("Start: (%.3f, %.3f, %.3f, %.3f)", x_m, y_m, theta_rad, t_m);
  int x = CONTXY2DISC(x_m + 0.5 * EnvNAVXYTHETATIMELATCfg.EnvWidth_c * EnvNAVXYTHETATIMELATCfg.cellsize_m, EnvNAVXYTHETATIMELATCfg.cellsize_m); 
  int y = CONTXY2DISC(y_m + 0.5 * EnvNAVXYTHETATIMELATCfg.EnvHeight_c * EnvNAVXYTHETATIMELATCfg.cellsize_m, EnvNAVXYTHETATIMELATCfg.cellsize_m); 
  int theta = ContTheta2Disc(theta_rad, NAVXYTHETATIMELAT_THETADIRS);
  int t = CONTTIME2DISC(t_m, EnvNAVXYTHETATIMELATCfg.timeResolution);
  SBPL_INFO("Start: (%d, %d, %d, %d)", x, y, theta, t);

  if(!IsWithinMapCell(x,y))
    {
      printf("ERROR: trying to set a start cell %d %d that is outside of map\n", x,y);
      return -1;
    }

  printf("env: setting start to %.3f %.3f %.3f %.3f (%d %d %d %d)\n", x_m, y_m, theta_rad, t_m, x, y, theta, t);

  if(!IsValidConfiguration(x,y,theta))
    {
      printf("WARNING: start configuration %d %d %d is invalid\n", x,y,theta);
    }

  EnvNAVXYTHETATIMELATHashEntry_t* OutHashEntry;
  printf("Getting hash entry!\n");
  fflush(stdout);
  if((OutHashEntry = (this->*GetHashEntry)(x, y, theta, t)) == NULL){
    //have to create a new entry
    printf("Creating new hash entry!\n");
    fflush(stdout);
    // @avemula MODIFIED tempT = t for start
    OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta, t, t);
  }

  //need to recompute start heuristics?
  if(EnvNAVXYTHETATIMELAT.startstateid != OutHashEntry->stateID)
    {
      bNeedtoRecomputeStartHeuristics = true;
      bNeedtoRecomputeGoalHeuristics = true; //because termination condition can be not all states TODO - make it dependent on term. condition
    }

  //set start
  StartStateID = OutHashEntry->stateID;
  EnvNAVXYTHETATIMELAT.startstateid = OutHashEntry->stateID;
  EnvNAVXYTHETATIMELATCfg.StartX_c = x;
  EnvNAVXYTHETATIMELATCfg.StartY_c = y;
  EnvNAVXYTHETATIMELATCfg.StartTheta = theta;
  EnvNAVXYTHETATIMELATCfg.StartTime = t;
  #ifdef VISUALIZATIONS_ENABLED
  visualizeState(EnvNAVXYTHETATIMELAT.startstateid, 120, "Start state");
  #endif
  //pause();
  return EnvNAVXYTHETATIMELAT.startstateid;    

}

void EnvironmentNAVXYTHETATIMELATADEXACT::clearHashTable(){
  int maxsize = EnvNAVXYTHETATIMELATCfg.EnvWidth_c*EnvNAVXYTHETATIMELATCfg.EnvHeight_c*(NAVXYTHETATIMELAT_THETADIRS+1);
  if(maxsize <= 200)
    {
      printf("environment stores states in lookup table\n");
      printf("nothing to clear\n");
    }
  else
    {	
      printf("environment stores states in hashtable\n");
      printf("clearing hash table...");

      //initialize the map from Coord to StateID
      for (int i = 0; i < (int)StateID2CoordTable.size(); i++)
	{
	  delete StateID2CoordTable.at(i);
	  StateID2CoordTable.at(i) = NULL;
	}
      StateID2CoordTable.clear();

      //not using hash
      Coord2StateIDHashTable_lookup = NULL;
    }
}

void EnvironmentNAVXYTHETATIMELATADEXACT::resetTrackingGrid(){
  for(int w = 0; w < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; w++){
    for(int h = 0; h < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; h++){
      sphereIndexGrid[w][h] = -1;
      sphereDistGrid[w][h] = 0;
      //00 00 xx xx
      char t = (sphereStateGrid[w][h] % 4);
      sphereStateGrid[w][h] = t * 4 + t;
    }
  }
}

void EnvironmentNAVXYTHETATIMELATADEXACT::updateTrackingGrid(const std::vector<int> *adplanIDV){
  for(int w = 0; w < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; w++){
    for(int h = 0; h < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; h++){
      sphereIndexGrid[w][h] = -1;
      sphereDistGrid[w][h] = 0;
      if(lookupCoords(w,h) == 0){
	for(unsigned int p = 0; p < adplanIDV->size(); p++){
	  int x, y, th, t;
	  GetCoordFromState(adplanIDV->at(p), x, y, th, t);
	  float dist = EuclideanDistance_m(x, y, w, h);
	  if(sphereIndexGrid[w][h] < 0){
	    sphereIndexGrid[w][h] = p;
	    sphereDistGrid[w][h] = dist;
	  }
	  if(sphereDistGrid[w][h] > dist){
	    sphereIndexGrid[w][h] = p;
	    sphereDistGrid[w][h] = dist;
	  }
	}
      }
    }
  }
}

void EnvironmentNAVXYTHETATIMELATADEXACT::resetExpandedStates(){
  LastBestTrackedUpdate = ros::Time::now().toSec();
}

void EnvironmentNAVXYTHETATIMELATADEXACT::expandingState(int StateID){
  if(!trackMode) {
    return;
  } else {
    EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[StateID];
    int x = HashEntry->X;
    int y = HashEntry->Y;
    expandsGrid[x][y] = 4;
    if(sphereIndexGrid[x][y] >= BestTrackedIndex || BestTrackedIndex == -1){
      LastBestTrackedUpdate = ros::Time::now().toSec();
    }
    if(sphereIndexGrid[x][y] > BestTrackedIndex || BestTrackedIndex == -1){
      BestTrackedIndex = sphereIndexGrid[x][y];
      BestTrackedStateID = StateID;
      //SBPL_ERROR("Tracking progress!!!");
      //SBPL_ERROR("Updating BestTrackedIndex: %d / BestTrackedStateID: %d", BestTrackedIndex, BestTrackedStateID);
    }
  }
}

void EnvironmentNAVXYTHETATIMELATADEXACT::sendStateData(int StateID, int fVal, int gVal, int hVal){
  //NOTHING TO DO
}

int  EnvironmentNAVXYTHETATIMELATADEXACT::getBestSeenState(){
  //SBPL_ERROR("BestTrackedStateID: %d", BestTrackedStateID);
  return BestTrackedStateID;
}

void EnvironmentNAVXYTHETATIMELATADEXACT::reset(){
  //resetting sphere information
  SBPL_ERROR("resetting environment!");
  spheres.clear();
  for(int i = 0; i < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; i++){
    for(int j = 0; j < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; j++){
      //init all states as out of sphere as there are no spheres yet
      sphereStateGrid[i][j] = 10;
      expandsGrid[i][j] = 0;
    }
  }
  //delete created states

  // @avemula INCORRECT/BUG not the right maxsize since there can be multiple states with same (x,y,theta) with different t
  int maxsize = EnvNAVXYTHETATIMELATCfg.EnvWidth_c*EnvNAVXYTHETATIMELATCfg.EnvHeight_c*(NAVXYTHETATIMELAT_THETADIRS+1);
	
  //init new space
  if(Coord2StateIDHashTable_lookup!=NULL)
    {
      printf("environment stores states in lookup table\n");

      for(int i = 0; i < maxsize; i++)
	Coord2StateIDHashTable_lookup[i] = NULL;
      GetHashEntry = &EnvironmentNAVXYTHETATIMELATADEXACT::GetHashEntry_lookup;
      CreateNewHashEntry = &EnvironmentNAVXYTHETATIMELATADEXACT::CreateNewHashEntry_lookup;
		
      //not using hash table
      HashTableSize = 0;
      Coord2StateIDHashTable = NULL;
    }
  else
    {		
      printf("environment stores states in hashtable\n");

      //initialize the map from Coord to StateID
      //HashTableSize = 4*1024*1024; //should be power of two
      // @avemula MODIFIED to reflect the true size
      HashTableSize = (int) Coord2StateIDHashTable->size();
      for(int i = 0; i < HashTableSize; i++){
	Coord2StateIDHashTable[i].clear();
      }
      GetHashEntry = &EnvironmentNAVXYTHETATIMELATADEXACT::GetHashEntry_hash;
      CreateNewHashEntry = &EnvironmentNAVXYTHETATIMELATADEXACT::CreateNewHashEntry_hash;

      //not using hash
      Coord2StateIDHashTable_lookup = NULL;
    }

  //initialize the map from StateID to Coord
  for(int i = 0; i < (int)StateID2CoordTable.size(); i++){
    delete StateID2CoordTable.at(i);
  }
  StateID2CoordTable.clear();
  for(unsigned int i = 0; i < StateID2IndexMapping.size(); i++){
    delete[] StateID2IndexMapping[i];
  }
  StateID2IndexMapping.clear();
  createdHashEntries = 0;
	
  expands2D = 0;
  expands3D = 0;
  expands4D = 0;
  expandsN3D = 0;
  expandsN4D = 0;
  expands2D_tot = 0;
  expands3D_tot = 0;
  expands4D_tot = 0;
  expandsN3D_tot = 0;
  expandsN4D_tot = 0;
  expandsTrack = 0;
  expandsTrack_tot = 0;
  vizTime = 0;
  getSuccTime = 0;
  planningTime = 0;
  trackingTime = 0;
}


void EnvironmentNAVXYTHETATIMELATADEXACT::setTrackMode(const std::vector<int> &adplanIDV, int cost, double tunnel_half_width, std::vector<int> *ModStates){
  
  double start_t = ros::Time::now().toSec();
  if(ModStates!=NULL) ModStates->clear();
  int same_until = compareADPlanVs(&adplanIDV, &prev_ADPlanIDV);
	
  //SBPL_WARN("Same until: %d", same_until);
  same_until = max(0, same_until - 1);
  
  iterPlanningTime = (ros::Time::now().toSec() - iterPStartTime); //update the planning timer
  iterTStartTime = ros::Time::now().toSec();
  BestTrackedIndex = -1;
  MaxTrackIndex = adplanIDV.size()-1;
  BestTrackedStateID = 0;
  resetTrackingGrid();
  getTSuccTime = 0;
  tCollCheckTime = 0;
  expandsTrack = 0;
  lastSearchTimedOut = false;
	
  //updateTrackingGrid(&adplanIDV);
  
  trackMode = true;
  plan_solutionIDs_V = &adplanIDV;
  
  int THW = (int) (tunnel_half_width / EnvNAVXYTHETATIMELATCfg.cellsize_m);
  //construct tunnel of spheres around the path
  prev_ADPlanIDV.clear();
  
  for(unsigned int i = 0; i < adplanIDV.size(); i++){
    prev_ADPlanIDV.push_back(adplanIDV[i]);
    int x, y, th, t;
    float tempT;
    GetCoordFromState(adplanIDV.at(i), x, y, th, t, tempT);
    
    if(i >= same_until)
      {
	addSphere(adplanIDV.at(i), tunnel_half_width, ModStates);
      }

    else
      {
	addSphere(adplanIDV.at(i), tunnel_half_width, NULL);
      }    
    
    int min_x = max(0, x - THW - 1);
    int max_x = min(x + THW + 1, EnvNAVXYTHETATIMELATCfg.EnvWidth_c-1);
    int min_y = max(0, y - THW - 1);
    int max_y = min(y + THW + 1, EnvNAVXYTHETATIMELATCfg.EnvHeight_c-1);
    
    for(int j = min_x; j <= max_x; j++){
      for(int k = min_y; k <= max_y; k++){
	//compute distance from cell to x, y
	float dist = EuclideanDistance_m(x, y, j, k);
	if(sphereIndexGrid[j][k] < 0){
	  sphereIndexGrid[j][k] = i;
	  sphereDistGrid[j][k] = dist;
	}
	if(sphereDistGrid[j][k] > dist){
	  sphereIndexGrid[j][k] = i;
	  sphereDistGrid[j][k] = dist;
	}
      }
    }
  }

  /*FILE * sphereindex;
  sphereindex = fopen("sphereindex", "w");
  for(int w = 0; w < EnvNAVXYTHETATIMELATCfg.EnvWidth_c; w++){
    for(int h = 0; h < EnvNAVXYTHETATIMELATCfg.EnvHeight_c; h++){
      fprintf(sphereindex,"%d,", sphereIndexGrid[w][h]);
    }
    fprintf(sphereindex,"\n");
  }
  fclose(sphereindex);*/
  
  maxSphereIndex = adplanIDV.size();
  setTrackTime = ros::Time::now().toSec() - start_t;
  LastBestTrackedUpdate = ros::Time::now().toSec();

  printf("Maxsphereindex : %d\n", maxSphereIndex);
  fflush(stdout);

  
#ifdef VISUALIZATIONS_ENABLED
  visualizeEnvironment();
#endif
  
  if(ModStates != NULL){
    SBPL_ERROR("set tracking mode in %.3f sec. and got %d modified states!", setTrackTime, (int)ModStates->size());
  } else {
    SBPL_ERROR("set tracking mode in %.3f sec.!", setTrackTime);
  }
  //pause();
}

void EnvironmentNAVXYTHETATIMELATADEXACT::setPlanMode() {
  iterPStartTime = ros::Time::now().toSec();
  expands2D = 0;
  expands3D = 0;
  expands4D = 0;
  expandsN3D = 0;
  expandsN4D = 0;
  vizTime = 0;
  get3DSuccTime_LD = 0;
  get3DSuccTime_HD = 0;
  get2DSuccTime = 0;
  get4DSuccTime = 0;
  getPSuccTime = 0;
  pCollCheckTime = 0;
  trackMode = false;
  lastSearchTimedOut = false;
  #ifdef VISUALIZATIONS_ENABLED
  visualizeEnvironment();
  #endif
}

bool EnvironmentNAVXYTHETATIMELATADEXACT::IsValidCell2D(int X, int Y)
{
  return (X >= 0 && X < EnvNAVXYTHETATIMELATCfg.EnvWidth_c && 
	  Y >= 0 && Y < EnvNAVXYTHETATIMELATCfg.EnvHeight_c && 
	  inflatedGrid[X][Y] < EnvNAVXYTHETATIMELATCfg.obsthresh);
} 

int EnvironmentNAVXYTHETATIMELATADEXACT::GetPathIndexForState(int SID){
  if(!trackMode) return -1;
  //only useful in tracking mode
  int x, y, th, t;
  float tempT;
  GetCoordFromState(SID, x, y, th, t,tempT);
  return sphereIndexGrid[x][y];
}

// @avemula ASK ASK
int EnvironmentNAVXYTHETATIMELATADEXACT::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
  printf("Heuristic returned is 0!!\n");
  fflush(stdout);
  return 0;
#endif

#if DEBUG
  if(stateID >= (int)StateID2CoordTable.size())
    {
      SBPL_ERROR("ERROR in EnvNAVXYTHETATIMELAT... function: stateID illegal\n");

      throw new SBPL_Exception();
    }
#endif

  int track_h = 0;
  if(trackMode){
    
    EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    if(sphereIndexGrid[HashEntry->X][HashEntry->Y]==-1) {
      // Outside the tunnel
      track_h = INFINITECOST;
      return track_h;
    }
    else {
      // @avemula CHANGED to reflect the actual value of the heuristic
      // @avemula ASK What's with this weird heuristic?
      track_h = 2*4*27*(maxSphereIndex - sphereIndexGrid[HashEntry->X][HashEntry->Y]);
      return track_h;
    }
  }
  /* else {
      //printf("Getting the heuristic of a state inside the tunnel\n");
      track_h = 10*(maxSphereIndex - sphereIndexGrid[HashEntry->X][HashEntry->Y]);
      }
    //printf("Returned heuristic value is %d\n", track_h);
    fflush(stdout);
    return track_h;
    }*/

  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[stateID];

  int h2D = grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y); //computes distances from start state that is grid2D, so it is EndX_c EndY_c 
  int hEuclid = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EuclideanDistance_m(HashEntry->X, HashEntry->Y, EnvNAVXYTHETATIMELATCfg.EndX_c, EnvNAVXYTHETATIMELATCfg.EndY_c));
	
  /*#ifdef USE_EUC_DIST_HEUR
    return (int)( ((double)hEuclid) / EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs);
    #endif*/
		
  //define this function if it is used in the planner (heuristic backward search would use it)
  //printf("Returned heuristic looks like %d\n",(int)(((double)h2D)/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs));
  int retval =  (int)(((double)__max(h2D,hEuclid))/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs);

  if(trackMode) {
    //printf("retval=%d, track_h=%d\n", retval, track_h);
    //sleep(1);
    retval = (int)((retval+track_h));    
  }
  
  return retval;
}

// @avemula NOTUSED
 int EnvironmentNAVXYTHETATIMELATADEXACT::GetStartHeuristic(int stateID)
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
  
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[stateID];
  int hEuclid = (int)(NAVXYTHETATIMELAT_COSTMULT_MTOMM*EuclideanDistance_m(EnvNAVXYTHETATIMELATCfg.StartX_c, EnvNAVXYTHETATIMELATCfg.StartY_c, HashEntry->X, HashEntry->Y));
  int h2D = grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
  /*#ifdef USE_EUC_DIST_HEUR
    return (int)( ((double)hEuclid) / EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs);
    #endif*/
		
  //define this function if it is used in the planner (heuristic backward search would use it)
  return (int)(((double)__max(h2D,hEuclid))/EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs); 
}

 
 void EnvironmentNAVXYTHETATIMELATADEXACT::GetSuccs_Track(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV) {
   GetSuccs_Track_4D(SourceStateID, SuccIDV, CostV, NULL);
}
	

 void EnvironmentNAVXYTHETATIMELATADEXACT::GetSuccs_Plan(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV) {
   GetSuccs_Plan_4D(SourceStateID, SuccIDV, CostV, NULL);
}
	
 void EnvironmentNAVXYTHETATIMELATADEXACT::GetPreds_Track(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) {
  SBPL_ERROR("GetPreds_Track not implemented for this planner. Tracking should always be done forward!");
  throw new SBPL_Exception();
  return;
}

// @avemula NOTE not gonna account for a backup space. For now, implementing only for 4D space
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::GetPreds_Plan(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) {
  
  if(trackMode){
    ROS_WARN("GetPreds_Plan called in tracking mode! Shouldn't happen!");
    throw new SBPL_Exception();
    return;
  }
	
  int iState_x, iState_y, iState_theta, iState_t;
  float iState_tempT;
	
  EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[TargetStateID];
  iState_x = HashEntry->X;
  iState_y = HashEntry->Y;
  iState_theta = HashEntry->Theta;
  iState_t = HashEntry->T;
  iState_tempT = HashEntry->TempT;
  /*
    if(iState_theta >= INVALID_THETA){
    Get2DPreds(TargetStateID, PredIDV, CostV);
    for(int i = 0; i < INVALID_THETA; i++){
    if(ExistsStateFromCoord(HashEntry->X,HashEntry->Y,i)){
    Get3DPreds(GetStateFromCoord(HashEntry->X,HashEntry->Y,i), PredIDV, CostV);
    }
    }
    } else {
    Get3DPreds(TargetStateID, PredIDV, CostV);
    }*/
	
  unsigned int t = lookupCoords(iState_x, iState_y);
  
  if(t == 0) {
    //currently in sphere -- possible predecessors are N4D or 4D (always through 4D actions though)
  
    /*if(iState_theta < INVALID_THETA)
      {
      Get3DPreds(TargetStateID, PredIDV, CostV);
      }*/
    if(iState_t < INVALID_TIME && iState_theta < INVALID_THETA)
      {
	Get4DPreds(TargetStateID, PredIDV, CostV);
      }
    else
      {
	SBPL_WARN("3D state in 4D region!");
	//visualizeEnvironment();
	//visualizeState(TargetStateID, 0, "bad_state");
	//visualizeState(TargetStateID, 0, "bad_state");
	pause();
      }
  }
  else if ( t == 1 ) {
    //state near a shpere -- possible predecessors are 3D, N4D or 4D (through both 4D and 3D actions)
    
    /*if(iState_theta < INVALID_THETA){
    //valid 3D coords - invalid state unless start or goal
    if(TargetStateID != EnvNAVXYTHETATIMELAT.goalstateid && TargetStateID != EnvNAVXYTHETATIMELAT.startstateid){
    SBPL_ERROR("Currently not in sphere, but valid angle!!! Should not happen except for start and goal!");
    throw new SBPL_Exception();
    }
    //if start or goal - generate 2D preds
    }*/

    if(iState_t < INVALID_TIME && iState_theta < INVALID_THETA){
      //valid 4D coords - invalid state unless start or goal
      if(TargetStateID != EnvNAVXYTHETATIMELAT.goalstateid && TargetStateID != EnvNAVXYTHETATIMELAT.startstateid){
	SBPL_ERROR("Currently not in sphere, but valid angle!!! Should not happen except for start and goal!");
	throw new SBPL_Exception();
      }
      //if start or goal - generate 2D preds
    }


    //currently near a sphere and a valid 3D state, generate 3D preds and all 4D preds from all possible angle orientations

    expandsGrid[iState_x][iState_y] = 2;
    expandsN4D++;
    expandsN4D_tot++;
    // @avemula REMOVED and ADDED v2.0 3d -> 2d
    //Get3DPreds_LD(TargetStateID, PredIDV, CostV);
    Get2DPreds(TargetStateID, PredIDV, CostV);

    /*for(int tht = 0; tht < INVALID_THETA; tht++){
      Get3DPreds(GetStateFromCoord(iState_x, iState_y, tht), PredIDV, CostV);
      }*/
    // @avemula ADDED to get 4dpreds use tempT rather than invalid_t
    // @avemula REMOVED v2.0 should match the exact method
    //Get4DPreds(GetStateFromCoord(iState_x, iState_y, iState_theta, iState_tempT), PredIDV, CostV);
    // @avemula ADDED v2.0 must consider all pssible predicates
    for(int tht=0; tht < INVALID_THETA ; tht++) {
      for(int tim=0; tim < MAX_TIME; tim++) {
	Get4DPreds(GetStateFromCoord(iState_x, iState_y, tht, tim), PredIDV, CostV);
      }
    }

  }
  else {
    //state far from sphere

    /*if(iState_theta < INVALID_THETA){
    //valid 3D coords - invalid state unless start or goal
    if(TargetStateID != EnvNAVXYTHETATIMELAT.goalstateid && TargetStateID != EnvNAVXYTHETATIMELAT.startstateid){
    SBPL_ERROR("Currently not in sphere, but valid angle!!! Should not happen except for start and goal!");
    throw new SBPL_Exception();
    }
    //if start or goal - generate 2D preds
    }*/

    if(iState_t<INVALID_TIME && iState_theta < INVALID_THETA) {
      if(TargetStateID != EnvNAVXYTHETATIMELAT.goalstateid && TargetStateID != EnvNAVXYTHETATIMELAT.startstateid){
	SBPL_ERROR("Currently not in sphere, but valid angle!!! Should not happen except for start and goal!");
	throw new SBPL_Exception();
      }
      //if start or goal - generate 3D preds
    }

    //currently 2D far from spheres, only 2D predecessors
    expandsGrid[iState_x][iState_y] = 1;
    expands2D++;
    expands2D_tot++;
    Get2DPreds(TargetStateID, PredIDV, CostV);
  }
}

// @avemula NOTCHANGED logging functions left as it is
void EnvironmentNAVXYTHETATIMELATADEXACT::logStats(){
  iterTrackingTime = (ros::Time::now().toSec() - iterTStartTime);
  fprintf(statsFile, "[Planning] Expands: (3D: %d)(2D: %d)(N3D: %d)\n", expands3D, expands2D, expandsN3D);
  fprintf(statsFile, "[Planning] Successor generation times: (3D: %.3fs)(2D: %.3fs)\n", get3DSuccTime_LD, get2DSuccTime);
  fprintf(statsFile, "[Planning] \t as percentages of iteration planning time: (3D: %.3f%%)(2D: %.3f%%)\n", (100.0f * get3DSuccTime) / (iterPlanningTime), (100.0f * get2DSuccTime) / (iterPlanningTime));
  fprintf(statsFile, "[Planning] Predecessor generation times: (3D: %.3fs)(2D: %.3fs)\n", get3DPredTime_LD, get2DPredTime);
  fprintf(statsFile, "[Planning] \t as percentages of iteration planning time: (3D: %.3f%%)(2D: %.3f%%)\n", (100.0f * get3DPredTime) / (iterPlanningTime), (100.0f * get2DPredTime) / (iterPlanningTime));
  fprintf(statsFile, "[Planning] 3D collision checking: (%.3fs)(%.3f%%)\n", pCollCheckTime, 100.0f * pCollCheckTime / (iterPlanningTime));
  fprintf(statsFile, "[Planning] Other: (%.3fs)(%.3f%%)\n", iterPlanningTime - (getPSuccTime + getPPredTime), 100.0f * (iterPlanningTime - (getPSuccTime + getPPredTime)) / iterPlanningTime);
  fprintf(statsFile, "---------------------------------------------------\n");
  fprintf(statsFile, "[Tracking] Expands: (3D: %d)\n", expandsTrack);
  fprintf(statsFile, "[Tracking] Successor generation time: 3D: %.3f (s)\n", getTSuccTime);
  fprintf(statsFile, "[Tracking] \t as percentage of iteration tracking time: (%.3f%%)\n", 100.0f * getTSuccTime / iterTrackingTime); 
  fprintf(statsFile, "[Tracking] Set track mode time: %.3f (%.3f%%)\n", setTrackTime, setTrackTime / iterTrackingTime);
  fprintf(statsFile, "[Tracking] Collision checking: (%.3fs)(%.3f%%)\n", tCollCheckTime, 100.0f * tCollCheckTime / (iterTrackingTime));
  fprintf(statsFile, "[Tracking] Other: (%.3fs)(%.3f%%)\n", iterTrackingTime - getTSuccTime, 100.0f * (iterTrackingTime - getTSuccTime) / iterTrackingTime);
  fprintf(statsFile, "---------------------------------------------------\n");
  fprintf(statsFile, "[Other] Visualization Time: %.3f (s)\n", vizTime);
  fprintf(statsFile, "---------------------------------------------------\n");
  fprintf(statsFile, "Total Expands: (3D: %d)(2D: %d)(N3D: %d) (Track: %d)\n", expands3D_tot, expands2D_tot, expandsN3D_tot, expandsTrack_tot);
  fprintf(statsFile, "===================================================\n");
	
  //visualizeEnvironment();
}

void EnvironmentNAVXYTHETATIMELATADEXACT::logStat(char* info){
  fprintf(statsFile, "%s\n", info);
}

void EnvironmentNAVXYTHETATIMELATADEXACT::SearchTimeoutCallback() {
  SBPL_WARN("PLANNING TIMED OUT!");
  lastSearchTimedOut = true;
}

void EnvironmentNAVXYTHETATIMELATADEXACT::addSphereWhereTrackingFailed(double rad) {
  //setPlanMode();
  //addSphere(BestStateID, rad);
  /*EnvNAVXYTHETATIMELATHashEntry_t* HashEntry = StateID2CoordTable[BestTrackedStateID];
    EnvNAVXYTHETATIMELATHashEntry_t* StartEntry = StateID2CoordTable[StartStateID];
    double dx = (HashEntry->X - StartEntry->X) * EnvNAVXYTHETATIMELATCfg.cellsize_m;
    double dy = (HashEntry->Y - StartEntry->Y) * EnvNAVXYTHETATIMELATCfg.cellsize_m;
    if(sqrtf(dx*dx + dy*dy) <= 4.0){*/
  //	addSphere(BestTrackedStateID, rad);
  //}
}

int EnvironmentNAVXYTHETATIMELATADEXACT::getTrackingFailStateID(){
  return BestTrackedStateID;
}


// @avemula BUGGY implementation
// @avemula ASK
int EnvironmentNAVXYTHETATIMELATADEXACT::getTransitionCost(int sourceSID, int targetSID){
  int sourceID = sourceSID;
  int targetID = targetSID;
  std::vector<int> SuccV;
  std::vector<int> CostV;

  int x,y,th,t;
  float tempT;
  int tx, ty, tth, tt;
  float ttempT;
  GetCoordFromState(sourceID,x,y,th,t,tempT);
  GetCoordFromState(targetID,tx,ty,tth,tt,ttempT);
  int bestcost = INFINITECOST;

	
  //if(th < INVALID_THETA){
  if(t < INVALID_TIME && th<INVALID_THETA) {
    //looking through 4D transitions
    bool orig_trackMode = trackMode;
    //set the environment in planning mode temporarily
    trackMode = false;
    Get4DSuccs(sourceID, &SuccV, &CostV);
    //restore the environment mode
    trackMode = orig_trackMode;
	

    //planning 4D transitions
    for(unsigned int i = 0; i < SuccV.size(); i++){
      if(SuccV[i] == targetID){
	//if(!orig_trackMode) return CostV[i];
	return CostV[i];
      }
    }
		
    SuccV.clear();
    CostV.clear();
    //set the environment in tracking mode temporarily
    trackMode = true;
    Get4DSuccs(sourceID, &SuccV, &CostV);
    //restore the environment mode
    trackMode = orig_trackMode;
    //tracking 3D transitions
    for(unsigned int i = 0; i < SuccV.size(); i++){
      if(SuccV[i] == targetID && bestcost > CostV[i]){
	//if(!orig_trackMode) return 5 * CostV[i];
	bestcost = CostV[i];
      }
    }
    if(bestcost < INFINITECOST){
      return bestcost;
    }
  }
  
  else {	
    //looking through 2D transitions
    //set the environment in planning mode temporarily
    bool orig_trackMode = trackMode;
    trackMode = false;
    // @avemula REMOVED and ADDED v2.0 3d -> 2d
    //Get3DSuccs_LD(sourceID, &SuccV, &CostV);
    Get2DSuccs(sourceID, &SuccV, &CostV);
    //if near 4D state - then generate 4D succs as well
    if(lookupCoords(x,y) == 1){
      for(unsigned int i = 0; i < INVALID_THETA; i++){
	//also generate 3D succs
	//Get3DSuccs(GetStateFromCoord(x,y,i), &SuccV, &CostV);
	/*for(int tim=0; tim < MAX_TIME; tim++) {
	  Get4DSuccs(GetStateFromCoord(x,y,i,tim), &SuccV, &CostV);
	  }*/
	Get3DSuccs_allT(GetStateFromCoord(x,y,i,INVALID_TIME, tempT), &SuccV, &CostV);
      }
      //Get4DSuccs(GetStateFromCoord(x,y,th,tempT), &SuccV, &CostV);
    }	
    //restore the environment mode
    trackMode = orig_trackMode;
	
    for(unsigned int i = 0; i < SuccV.size(); i++){
      if(SuccV[i] == targetID && bestcost > CostV[i]){
	bestcost = CostV[i];
      }
    }
    if(bestcost < INFINITECOST){
      return bestcost;
    }
  }
  /*
    sourceID = targetSID;
    targetID = sourceSID;
    SuccV.clear();
    CostV.clear();
    GetCoordFromState(sourceID, x,y,th);
	
    if(th < INVALID_THETA){
    //looking through 3D transitions
    bool orig_trackMode = trackMode;
    //set the environment in planning mode temporarily
    trackMode = false;
    Get3DPreds(sourceID, &SuccV, &CostV);
    //restore the environment mode
    trackMode = orig_trackMode;
	
    //planning 3D transitions
    for(unsigned int i = 0; i < SuccV.size(); i++){
    if(SuccV[i] == targetID){
    //if(!orig_trackMode) return CostV[i];
    return CostV[i];
    }
    }
		
    } else {	
    //looking through 2D transitions
    //set the environment in planning mode temporarily
    bool orig_trackMode = trackMode;
    trackMode = false;
    Get2DPreds(sourceID, &SuccV, &CostV);
    //restore the environment mode
    trackMode = orig_trackMode;
	
    for(unsigned int i = 0; i < SuccV.size(); i++){
    if(SuccV[i] == targetID){
    return CostV[i];
    }
    }
    }*/
  printf("Transition not found\n");
  fflush(stdout);
  SBPL_ERROR("Transition not found!");
  SBPL_ERROR("(%d,%d,%d) -> (%d,%d,%d)", x,y,th,tx,ty,tth);
  //throw new SBPL_Exception();
  return 1000000;
}

//returns the new start state to be used
// @avemula DOUBT using something like lastvalid horizon. Do we really need this function?
// @avemula NOTUSED
int EnvironmentNAVXYTHETATIMELATADEXACT::getExecutablePiece(std::vector<int>* path, double time_limit, std::vector<std::vector<double> > *xythetaPath, double* horizon){

  SBPL_ERROR("Converting state ID path to x,y,th path...");
  //std::vector<EnvNAVXYTHETATIMELAT3Dpt_t> xytP;
  std::vector<SBPL_4Dpt_t> xytP;
  ConvertStateIDPathintoXYThetaPath(path, &xytP, NULL, NULL);
  SBPL_ERROR("Done!");
  xythetaPath->clear();
  double time_elapsed = 0.0;
  int lastValidStartID = path->at(0);
  double lastValidHorizon = 0;
	
  for(unsigned int i = 1; i < xytP.size(); i++){
    double dist = sqrtf((xytP[i].x - xytP[i-1].x)*(xytP[i].x - xytP[i-1].x) + 
			(xytP[i].y - xytP[i-1].y)*(xytP[i].y - xytP[i-1].y));
    time_elapsed += dist / EnvNAVXYTHETATIMELATCfg.nominalvel_mpersecs;
    std::vector<double> coords;
    coords.push_back(xytP[i].x - 0.06 - 0.5 * EnvNAVXYTHETATIMELATCfg.EnvWidth_c * EnvNAVXYTHETATIMELATCfg.cellsize_m);
    coords.push_back(xytP[i].y - 0.5 * EnvNAVXYTHETATIMELATCfg.EnvHeight_c * EnvNAVXYTHETATIMELATCfg.cellsize_m);
    coords.push_back(xytP[i].theta);
    coords.push_back(xytP[i].t);
    xythetaPath->push_back(coords);
		
    int newX = CONTXY2DISC(xytP[i].x, EnvNAVXYTHETATIMELATCfg.cellsize_m);
    int newY = CONTXY2DISC(xytP[i].y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
    int newTh = ContTheta2Disc(xytP[i].theta, NAVXYTHETATIMELAT_THETADIRS);
    int newT = CONTTIME2DISC(xytP[i].t, EnvNAVXYTHETATIMELATCfg.timeResolution);
		
    if(IsValidConfiguration(newX, newY, newTh) ){
      lastValidStartID = GetStateFromCoord(newX, newY, newTh, newT);
      lastValidHorizon = time_elapsed;
    }
		
    if(time_elapsed >= time_limit){
      break;
    }
  }
  *horizon = lastValidHorizon;
  //visualizeState(lastValidStartID, 120, "next-start");
  return lastValidStartID;
	
  //SBPL_WARN("Got to the end!");
  //*horizon = time_elapsed;
  //int newX = CONTXY2DISC(xytP[xytP.size()-1].x,EnvNAVXYTHETATIMELATCfg.cellsize_m);
  //int newY = CONTXY2DISC(xytP[xytP.size()-1].y,EnvNAVXYTHETATIMELATCfg.cellsize_m);
  //int newTh = ContTheta2Disc(xytP[xytP.size()-1].theta, NAVXYTHETATIMELAT_THETADIRS);
  //return GetStateFromCoord(newX, newY, newTh);
}


int EnvironmentNAVXYTHETATIMELATADEXACT::getNearestState(int state_id, int start_ind, std::vector<int>* path){
  double min_dist = 1000000.0f;
  int best = -1;
  int x1, y1, th1, t1;
  GetCoordFromState(state_id, x1, y1, th1, t1);
  for(unsigned int i = start_ind; i < path->size(); i++){
    int x2, y2, th2, t2;
    GetCoordFromState(path->at(i), x2, y2, th2, t2);
    double dist = EuclideanDistance_m(x1,y1,x2,y2);
    if(dist < min_dist){
      min_dist = dist;
      best = i;
    }
  }
  return best;
}

// @avemula NOTE Might be useless in our scenario. But let's just implement it.
// @avemula BUGGY implementation
// @avemula ASK
void EnvironmentNAVXYTHETATIMELATADEXACT::processCostlyPath(std::vector<int>* planning_path, std::vector<int>* tracking_path){
  double start_t = ros::Time::now().toSec();
  //SBPL_WARN("Getting track path costs");
  std::vector<int> tpath_cost;
  std::vector<double> epsV;
  tpath_cost.push_back(0);
  for(unsigned int i = 1; i < tracking_path->size(); i++){
    tpath_cost.push_back(tpath_cost[i-1] + getTransitionCost(tracking_path->at(i-1), tracking_path->at(i)));
  }
  //SBPL_WARN("Getting plan path costs");
  setPlanMode();
  std::vector<int> ppath_cost;
  ppath_cost.push_back(0);
  for(unsigned int i = 1; i < planning_path->size(); i++){
    ppath_cost.push_back(ppath_cost[i-1] + getTransitionCost(planning_path->at(i-1), planning_path->at(i)));
  }
	
  SBPL_WARN("Comparing...");
  //int t_progress = 0;
  int p_progress = 0;
  double max_eps = 1.0f;
  int max_state = -1;
  int max_state_t = -1;
  for(unsigned int i = 0; i < tracking_path->size(); i++){
    int match = getNearestState(tracking_path->at(i), p_progress, planning_path);
    p_progress = match;
    double eps = (tpath_cost[i] / (1.0f * ppath_cost[match]));
    epsV.push_back(eps);
    int x, y, th, t;
    GetCoordFromState(planning_path->at(match), x, y, th, t);
    if(max_eps <= eps){
      max_eps = eps;
      max_state = match;
      max_state_t = i;
    }
    //SBPL_WARN("costs: [%d, %d] vs [%d, %d] [eps: %.4f]", match, ppath_cost[match], i, tpath_cost[i], eps);
  }
  SBPL_WARN("largest eps found %.4f at planing path index %d (StateID: %d)", max_eps, max_state, planning_path->at(max_state));
  SBPL_WARN("time: %.4f", (ros::Time::now().toSec() - start_t));
  addSphere(planning_path->at(max_state), 1.0f);
  double min_eps = max_eps;
  int min_state = -1;
  for(int i = max_state_t; i > 0; i--){
    if(epsV[i] < min_eps && epsV[i] > 1.0f){
      min_eps = epsV[i];
      min_state = i;
    }
  }

  if(min_state != -1){
    addSphere(tracking_path->at(min_state), 1.0f);
  }
  //addSphere(planning_path->at(max(0,max_state-1)), 0.5f);
  //addSphere(planning_path->at(max(0,max_state-2)), 0.25f);
  //addSphere(planning_path->at(max(0,max_state-3)), 0.125f);
}

// @avemula CHANGED from 3d to 4d
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::GetAll4DSuccs(int x, int y, int th, int t, std::vector<std::vector<int> > *Succs, std::vector<int> *costs){
  //iterate through actions
  printf("[");
  fflush(stdout);
  for (int aind = 0; aind < EnvNAVXYTHETATIMELATCfg.actionwidth; aind++)
    {
      std::vector<int> succ(4, 0);
      EnvNAVXYTHETATIMELATAction_t* nav3daction = &EnvNAVXYTHETATIMELATCfg.ActionsV[(unsigned int)th][aind];
      succ[0] = x + nav3daction->dX;
      succ[1] = y + nav3daction->dY;
      succ[2] = NORMALIZEDISCTHETA(nav3daction->endtheta, NAVXYTHETATIMELAT_THETADIRS);
      succ[3] = t + nav3daction->dT;
		
      int cost = GetActionCost(x, y, th, t, nav3daction, true);

      Succs->push_back(succ);
      costs->push_back(cost);		
    }
  printf("]");
  fflush(stdout);
}

// @avemula CHANGED from 3d to 4d
// @avemula NOTUSED
void EnvironmentNAVXYTHETATIMELATADEXACT::GetAll4DPreds(int x, int y, int th, int t, std::vector<std::vector<int> > *Preds, std::vector<int> *costs){
  //iterate through actions

  int aind;
  Preds->clear();
  costs->clear();
    	
  //iterate through actions
  vector<EnvNAVXYTHETATIMELATAction_t*>* actionsV = &EnvNAVXYTHETATIMELATCfg.PredActionsV[(unsigned int)th];
  for (aind = 0; aind < (int)EnvNAVXYTHETATIMELATCfg.PredActionsV[(unsigned int)th].size(); aind++)
    {
      std::vector<int> pred(4, 0);
      EnvNAVXYTHETATIMELATAction_t* nav3daction = actionsV->at(aind);

      pred[0] = x - nav3daction->dX;
      pred[1] = y - nav3daction->dY;
      pred[2] = nav3daction->starttheta;
      pred[3] = t - nav3daction->dT;
	
      //get cost
      int cost = GetActionCost(pred[0], pred[1], pred[2], pred[3], nav3daction, true);
	    
    	
      Preds->push_back(pred);
      costs->push_back(cost);
    }
}

// @avemula USELESS function
void EnvironmentNAVXYTHETATIMELATADEXACT::PlannerExpandingState(int StateID, std::vector<int> *info){
  //visualizeState(StateID, 300, "hmmm");
  return;
  /*int x,y,th, t;
  GetCoordFromState(StateID, x, y, th, t);
	
  if(lookupCoords(x,y)==0 && th >= INVALID_THETA){
    SBPL_ERROR("Non-infinite successor transition found for %d!", StateID);
    std::vector<int> SV;
    std::vector<int> CV;
    visualizeState(StateID, 300, "hmmm");
    GetSuccs(StateID, &SV, &CV);
    for(unsigned int i = 0; i < SV.size(); i++){
      if(CV[i] < INFINITECOST){
	//visualizeState(SV[i], 300, "hmmm-to" + boost::lexical_cast<string>(i));
	visualizeTransition(StateID, SV[i], 300, "hmmm-trans-" + boost::lexical_cast<string>(i));
      }
    }
    throw new SBPL_Exception();
    }*/
}


// @avemula ADDED new function to check if the given path is collision free with respect
// to dynamic obstacles
bool EnvironmentNAVXYTHETATIMELATADEXACT::isCollisionFree(std::vector<int> *stateIDV, std::vector<int> *loc) {
  bool valid = true;
  vector<SBPL_4Dpt_t> xythetaPath;
  ConvertStateIDPathintoXYThetaPath(stateIDV, &xythetaPath);

  for(int i=0; i<xythetaPath.size(); i++) {

    SBPL_4Dpt_t s = xythetaPath[i];
    int x_d,y_d,theta_d,t_d;
    SBPL_4Dcell_t c;
    c.x = CONTXY2DISC(s.x, EnvNAVXYTHETATIMELATCfg.cellsize_m);
    c.y = CONTXY2DISC(s.y, EnvNAVXYTHETATIMELATCfg.cellsize_m);
    c.theta = ContTheta2Disc(s.theta, NAVXYTHETATIMELAT_THETADIRS);
    c.t = CONTTIME2DISC(s.t, EnvNAVXYTHETATIMELATCfg.timeResolution);

    if(getDynamicObstacleCost(c) > EnvNAVXYTHETATIMELATCfg.dynamic_obstacle_collision_cost_thresh) {
      valid =  false;
      // @avemula NOTE can be changed to put spheres at all potential collision states
      printf("Current path COLLIDING with a dynamic obstacle at %f,%f\n", s.x, s.y);
      fflush(stdout);
      loc->push_back(GetStateFromCoord(c.x,c.y,c.theta,c.t));
      return valid;
    }

  }

  return valid;
}

bool EnvironmentNAVXYTHETATIMELATADEXACT::isGoal(int stateID) {

  int x,y,th,t;
  GetCoordFromState(stateID, x, y, th, t);
  EnvNAVXYTHETATIMELATHashEntry_t* Goal = StateID2CoordTable[EnvNAVXYTHETATIMELAT.goalstateid];

  if(!trackMode){
    if(x==Goal->X && y==Goal->Y) {
      return true;
    }
  }
  else {
    if(x==Goal->X && y==Goal->Y && th==Goal->Theta) {
      return true;
    }
  }

  return false;
}


// @avemula ADDED own funtion to set the time bound of the expansions
void EnvironmentNAVXYTHETATIMELATADEXACT::setTimeBound() {

  double MAXTIME = 0.0;
  for(int i=0; i<dynamicObstacles.size(); i++) {

    vector<SBPL_Trajectory_t> traj = dynamicObstacles[i].trajectories;

    for(int j=0; j<traj.size(); j++) {

      vector<SBPL_Traj_Pt_t> pts = traj[j].points;
      SBPL_Traj_Pt_t end_pt = pts[pts.size()-1];

      if(MAXTIME < end_pt.t)
	MAXTIME = end_pt.t;      
    }

  }

  MAX_TIME = MAXTIME/EnvNAVXYTHETATIMELATCfg.timeResolution;
  //MAX_TIME = 100;
  printf("Time bound SET at %d\n", MAX_TIME);
}

// @avemula ADDED a new helper print function
void EnvironmentNAVXYTHETATIMELATADEXACT::printStatePath(vector<int> *StateIDV) {

  for (int i=0; i<StateIDV->size(); i++) {
    int x,y,th,t;
    float tempT;
    EnvNAVXYTHETATIMELATHashEntry_t* s = StateID2CoordTable[StateIDV->at(i)];
    //GetCoordFromState(StateIDV->at(i), x, y, th, t, tempT);
    x = s->X; y=s->Y; th=s->Theta; t=s->T; tempT=s->TempT;
    printf("%d %d %d %d %f\n", x, y, th, t, tempT);
    if(t<INVALID_TIME && getInterval(x,y,t)<0)
      printf("COLLIDING HERE\n");
  }
  printf("STATE ID PATH PRINTED\n");
}
