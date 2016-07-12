/**

   Author : Anirudh Vemula
   Date : 11 March 2016

   The adaptive planner functionality file

   Taken from Gochev's code on path planning with adaptive dimensionality

*/


//#include <sbpl_adaptive/sbpl_adaptive/headers.h>
#include <sbpl/headers.h>
#include <sbpl_dynamic_adaptive_planner/adaptive_planner.h>
#include <sbpl_dynamic_adaptive_planner/AdaptiveDiscreteSpaceTimeInformation.h>
#include <sbpl_dynamic_adaptive_planner/araplanner_ad.h>
#include <sbpl_dynamic_adaptive_planner/weightedAStar.h>

#define TRACK_RETRIES 0

using namespace std;

//====================================== PUBLIC ====================================================

AdaptivePlanner::AdaptivePlanner(AdaptiveDiscreteSpaceTimeInformation* environment, bool bSearchForward)
{
	logstream_ = stdout;
	SBPL_INFO("Creating adaptive planner...");
	bforwardsearch = bSearchForward;
	adaptive_environment_.reset(environment);

	StartStateID = environment->StartStateID;
	GoalStateID = environment->GoalStateID;

	final_eps_planning_time = -1.0;
	final_eps = -1.0;
	newSphereRad = 1.0;
	tunnelWidth = 1.0;

	
	timePerRetry = 5.0;

	planningEPS = -1.0;
	trackingEPS = -1.0;
	targetEPS = -1.0;

	nIterations = 0;
	plan_time_total_s = 0;
	track_time_total_s = 0;
	bsearchuntilfirstsolution = true;
	repair_time = 0;
	searchexpands = 0;

	stat_.reset(new AdaptivePlannerCSVStat_c());
	adaptive_environment_->setStat(stat_.get());

	SBPL_INFO("Initializing planners...");

	// @avemula ASK other planners that could possibly be used. TreerestoringA*?
	
	planner.reset(new ARAPlanner(adaptive_environment_.get(), bSearchForward));
	//planner.reset(new ADPlanner(adaptive_environment_.get(), bSearchForward));
	//planner.reset(new WeightedAStar(adaptive_environment_.get(), bSearchForward));
		      
	planner->set_search_mode(false);
	tracker.reset(new ARAPlanner_AD(adaptive_environment_.get(), bSearchForward));
	tracker->set_search_mode(false);
	SBPL_INFO("done!");
}


AdaptivePlanner::~AdaptivePlanner()
{

}

//----------------------------- Interface functions ----------------------------------------------

//returns 1 if found a solution, and 0 otherwise
int AdaptivePlanner::replan(double allocated_time_secs, double allocated_time_per_retry_, std::vector<int>* solution_stateIDs_V, int* psolcost)
{
	SBPL_INFO("Adaptive planning begins...");
	adaptive_environment_->setPlanMode();
	//DO THE ADAPTIVE PLANNING LOOP HERE
	std::vector<int> planning_stateV;
	std::vector<int> tracking_stateV;

	MY_TIME_TYPE start_t = MY_TIME_NOW;
	track_time_total_s = 0;
	plan_time_total_s = 0;

	int round = 0;

	double ad_plan_time_alloc = allocated_time_per_retry_;
	double ad_track_time_alloc = allocated_time_per_retry_;

	SBPL_INFO("Retry time limits: (Planning: %.4f) (Tracking: %.4f)", ad_plan_time_alloc, ad_track_time_alloc);

	int planning_bRet;
	int tracking_bRet;

	SBPL_INFO("Setting start and goal for planner and tracker...");

	if(planner->set_start(StartStateID) == 0)
	{
		SBPL_ERROR("ERROR: planner failed to set start state");
		throw SBPL_Exception();
	}

	if(tracker->set_start(StartStateID) == 0){
		SBPL_ERROR("ERROR: tracker failed to set start state");
		throw SBPL_Exception();
	}

	if(planner->set_goal(GoalStateID) == 0)
	{
		SBPL_ERROR("ERROR: planner failed to set goal state");
		throw SBPL_Exception();
	}

	if(tracker->set_goal(GoalStateID) == 0){
		SBPL_ERROR("ERROR: tracker failed to set goal state");
		throw SBPL_Exception();
	}

	SBPL_INFO("Setting environment in planning mode...");
	adaptive_environment_->setPlanMode();

	// @avemula TODO might need to remove this
	//SBPL_INFO("Adding start (%d, %.3f) and goal (%d, %.3f) spheres...", StartStateID, newSphereRad, GoalStateID, newSphereRad);
	// @avemula CHANGED commented out the following two lines.

	// @avemula ASK sphere at goal
	// No initial spheres placed at start and goal
	//adaptive_environment_->addSphere(GoalStateID,  newSphereRad);
	adaptive_environment_->addSphere(StartStateID, newSphereRad-0.4);

	
	// @avemula ADDED v2.0 need to add a sphere at the start
	//SBPL_INFO("Adding start (%d, %.3f) sphere...", StartStateID, newSphereRad);
	//adaptive_environment_->addSphere(StartStateID, newSphereRad);
	// @avemula NOTE Can't add sphere at the goal since time is unknown

	std::vector<int> ModifiedStates;
	std::vector<int> TrkModifiedStates;
	repair_time = 0.0;
	
	SBPL_INFO("Forcing planning from scratch for planner and tracker...");
	planner->force_planning_from_scratch();
	tracker->force_planning_from_scratch();
	
	std::vector<int> LastTrackPath;
	bool LastTrackSuccess = false;
	double t_elapsed_s;

	stat_->setInitialEps(targetEPS);

#if ADP_STEPBYSTEP
	SBPL_INFO("Ready to plan!");
	this->pause();
#endif


	std::vector<int> new_sphere_locations; //as stateIDs
	do {
		if(MY_TIME_DIFF_S(MY_TIME_NOW, start_t) > allocated_time_secs){
			SBPL_INFO("Search ran out of time!");
			solution_stateIDs_V->clear();
			for(unsigned int i = 0; i < LastTrackPath.size(); i++){
				solution_stateIDs_V->push_back(LastTrackPath[i]);
			}
			SBPL_INFO("Done in: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
			nIterations = round;

			stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
			stat_->setFinalEps(-1.0);
			return LastTrackSuccess;
		}

		SBPL_INFO("=======================================");
		SBPL_INFO("||          Iteration %03d            ||", round);
		SBPL_INFO("=======================================");
		MY_TIME_TYPE iter_start = MY_TIME_NOW;
		MY_TIME_TYPE plan_start = MY_TIME_NOW;
		MY_TIME_TYPE track_start;

		double plan_time_s = 0;
		double track_time_s = 0;

		int p_Cost;		  //planning solution cost
		int t_Cost;		  //tracking solution cost

		//==================================== PLANNING =================================
		SBPL_INFO("\t=======================================");
		SBPL_INFO("\t||          Planning Phase           ||");
		SBPL_INFO("\t=======================================");

		planner->force_planning_from_scratch();
		planner->set_initialsolution_eps(planningEPS);
		// @avemula ASK planning set search mode
		planner->set_search_mode(false);

		adaptive_environment_->setPlanMode();
		//add pending new spheres
		SBPL_INFO("Adding %zd pending spheres...", new_sphere_locations.size());
		for(int stateID : new_sphere_locations){
		  adaptive_environment_->addSphere(stateID, newSphereRad, &ModifiedStates);
		}
		new_sphere_locations.clear();
		planning_stateV.clear();
		planning_bRet = 0;

#if ADP_STEPBYSTEP
		adaptive_environment_->visualizeEnvironment();
		SBPL_INFO("Ready for planning phase!");
		SBPL_INFO("Retry time limits: (Planning: %.4f) (Tracking: %.4f)", ad_plan_time_alloc, ad_track_time_alloc);
		this->pause();
#endif
		MY_TIME_TYPE iter_time;
		double iter_elapsed;
		//PLANNING HERE!
		t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
		while(t_elapsed_s < allocated_time_secs){
			SBPL_INFO("Still have time (%.3fs)...planning", allocated_time_secs - t_elapsed_s);
			iter_time = MY_TIME_NOW;
			planning_bRet = planner->replan(ad_plan_time_alloc, &planning_stateV, &p_Cost);
			t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
			iter_elapsed = MY_TIME_DIFF_S(MY_TIME_NOW, iter_time);
			if(planning_bRet) break;

			// @avemula CHECK if solution doesn't exist
			// @avemula ASK
			if(!planning_bRet && iter_elapsed < (ad_plan_time_alloc-1)) break;
		}

		plan_time_s = MY_TIME_DIFF_S(MY_TIME_NOW, plan_start);
		plan_time_total_s += plan_time_s;
		stat_->addPlanningPhaseTime(plan_time_s);

		SBPL_INFO("Planner done in %.3fs...", plan_time_s);

		// @avemula DEBUG
		//adaptive_environment_->printStatePath(&planning_stateV);

		
#if ADP_STEPBYSTEP
		adaptive_environment_->visualizeEnvironment();		
		adaptive_environment_->visualizeStatePath(&planning_stateV, 120, 120, "planning_path");
		SBPL_INFO("Planning phase done!");
		this->pause();
#endif
		// @avemula NOTE Can be claimed in our scenario
		if(!planning_bRet || planning_stateV.size() == 0){
			SBPL_ERROR("Solution could not be found within the allowed time (%.3fs.) after %d iterations", ad_plan_time_alloc, round);
			#if ADP_GRAPHICAL
			adaptive_environment_->visualizeEnvironment();
			#endif
			nIterations = round;
			stat_->setFinalEps(-1.0);
			stat_->setSuccess(false);
			stat_->setNumIterations(nIterations+1);
			return planning_bRet;
		}

		#if ADP_GRAPHICAL
		adaptive_environment_->visualizeEnvironment();
		adaptive_environment_->visualizeStatePath(&planning_stateV, 120, 120, "planning_path");
		#endif
		
		if(adaptive_environment_->isHighDPath(&planning_stateV)){
			solution_stateIDs_V->clear();
			for(unsigned int i = 0; i < planning_stateV.size(); i++){
				solution_stateIDs_V->push_back(planning_stateV[i]);
			}
			SBPL_INFO("Path Cost : %d", p_Cost);
			
			SBPL_INFO("Iteration Time: %.3f sec (avg: %.3f)", MY_TIME_DIFF_S(MY_TIME_NOW, iter_start), MY_TIME_DIFF_S(MY_TIME_NOW, start_t) / (round+1.0));
			SBPL_INFO("Done in: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
			nIterations = round;
			stat_->setFinalEps(planner->get_final_epsilon());
			stat_->setFinalPlanCost(p_Cost);
			stat_->setFinalTrackCost(p_Cost);
			stat_->setNumIterations(nIterations+1);
			stat_->setSuccess(true);
			stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
			return true;
		}
		

		//==================================== TRACKING ====================================
		SBPL_INFO("\t=======================================");
		SBPL_INFO("\t||          Tracking Phase           ||");
		SBPL_INFO("\t=======================================");
		track_start = MY_TIME_NOW;
		adaptive_environment_->setTrackMode(planning_stateV, p_Cost, tunnelWidth, &TrkModifiedStates);
		tracker->force_planning_from_scratch();
		tracker->set_initialsolution_eps(targetEPS / planner->get_final_epsilon());
		printf("Tracker about to start with eps: %f\n", targetEPS / planner->get_final_epsilon());		
		fflush(stdout);
		// @avemula ASK
		//tracker->set_search_mode(false);
		tracker->set_search_mode(true);
		tracking_stateV.clear();
		tracking_bRet = 0;

#if ADP_STEPBYSTEP
		//adaptive_environment_->visualizeEnvironment();
		SBPL_INFO("Ready for tracking phase!");
		this->pause();
#endif

		//TRACKING HERE!!!
		t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
		int last_bestTrackedID = -1;
		int retries = 0;
		while(t_elapsed_s < allocated_time_secs){
			SBPL_INFO("Still have time (%.3fs)...tracking", allocated_time_secs - t_elapsed_s);
			tracking_bRet = tracker->replan(ad_track_time_alloc, &tracking_stateV, &t_Cost);
			t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
			if(tracking_bRet) {
				break;
			} else {
				int new_bestTrackedID = adaptive_environment_->getBestSeenState();
				if(new_bestTrackedID == last_bestTrackedID){
					if(retries>=TRACK_RETRIES){
						SBPL_WARN("No progress...giving up this tracking iteration");
						break;
					}
					retries++;
				} else {
					last_bestTrackedID = new_bestTrackedID;
					retries=0;
				}
			}
		}

		//adaptive_environment_->printStatePath(&tracking_stateV);

		track_time_s = MY_TIME_DIFF_S(MY_TIME_NOW, track_start);
		track_time_total_s += track_time_s;
		stat_->addTrackingPhaseTime(track_time_s);

		LastTrackPath.clear();
		for(unsigned int i = 0; i < tracking_stateV.size(); i++){
			LastTrackPath.push_back(tracking_stateV[i]);
		}
		LastTrackSuccess = tracking_bRet;

		SBPL_INFO("Tracker done in %.3fs...", track_time_s);

		ModifiedStates.clear();
		SBPL_INFO("[Planning] Time: %.3fs (%.1f%% of iter time)", plan_time_s, 100*plan_time_s / (plan_time_s + track_time_s));
		SBPL_INFO("[Tracking] Time: %.3fs (%.1f%% of iter time)", track_time_s, 100*track_time_s / (plan_time_s + track_time_s));

		#if ADP_GRAPHICAL
		adaptive_environment_->visualizeEnvironment();
		adaptive_environment_->visualizeStatePath(&tracking_stateV, 120, 120, "tracking_path");
		#endif

#if ADP_STEPBYSTEP
		//adaptive_environment_->visualizeEnvironment();
		SBPL_INFO("Tracking phase done!");
		this->pause();
#endif
		/*if(tracking_bRet && (t_Cost / (1.0f * p_Cost)) > trackingEPS * planningEPS){
		  // @avemula ASK
		  //tracking found a costly path
		  SBPL_INFO("Tracking succeeded - costly path found! tCost %d / pCost %d (eps: %.3f, target: %.3f)", t_Cost, p_Cost, (t_Cost/(double)p_Cost), trackingEPS * planningEPS);
		  adaptive_environment_->processCostlyPath(planning_stateV, tracking_stateV, &new_sphere_locations);
		  printf("Processing costly path done\n");
		  fflush(stdout);
		  if(new_sphere_locations.size() == 0){
		    SBPL_ERROR("No new spheres added during this planning episode!!!");
		    throw SBPL_Exception();
		  }
		  }*/

		//else if (tracking_bRet && (t_Cost / (1.0f * p_Cost)) <= trackingEPS * planningEPS ) {
		if(tracking_bRet) {
		  SBPL_INFO("Tracking succeeded! - good path found! tCost %d / pCost %d (eps: %.3f, target: %.3f)", t_Cost, p_Cost, (t_Cost/(double)p_Cost), trackingEPS * planningEPS);
#if ADP_GRAPHICAL
		  adaptive_environment_->visualizeEnvironment();
		  adaptive_environment_->visualizeStatePath(&tracking_stateV, 120, 120, "tracking_path");
#endif
		  solution_stateIDs_V->clear();
		  for(unsigned int i = 0; i < tracking_stateV.size(); i++){
		    solution_stateIDs_V->push_back(tracking_stateV[i]);
		  }
		  SBPL_INFO("Iteration Time: %.3f sec (avg: %.3f)", MY_TIME_DIFF_S(MY_TIME_NOW, iter_start), MY_TIME_DIFF_S(MY_TIME_NOW, start_t) / (round+1.0));
		  SBPL_INFO("Done in: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
		  nIterations = round;
		  stat_->setFinalEps(planner->get_final_epsilon() * tracker->get_final_epsilon());
		  stat_->setFinalPlanCost(p_Cost);
		  stat_->setFinalTrackCost(t_Cost);
		  stat_->setNumIterations(nIterations+1);
		  stat_->setSuccess(true);
		  stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
		  return true;
		} else {
		  SBPL_WARN("Tracking Failed!");
		  //since tracking failed -- introduce new spheres
		  if(tracking_stateV.size() > 0){
		    //get the point of failure
		    int TrackFail_StateID = tracking_stateV[tracking_stateV.size()-1];
		    new_sphere_locations.push_back(TrackFail_StateID);
		  } else {
		    SBPL_ERROR("No new spheres added during this planning episode!!!");
		    throw SBPL_Exception();
		  }
		}
		SBPL_INFO("Iteration Time: %.3f sec (avg: %.3f)", MY_TIME_DIFF_S(MY_TIME_NOW, iter_start), MY_TIME_DIFF_S(MY_TIME_NOW, start_t) / (round+1.0));
		SBPL_INFO("Total Time so far: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
		round++;
	} while (true);

	stat_->setFinalEps(-1);
	stat_->setFinalPlanCost(INFINITECOST);
	stat_->setFinalTrackCost(INFINITECOST);
	stat_->setNumIterations(round+1);
	stat_->setSuccess(false);
	stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));

	return tracking_bRet;
}

int AdaptivePlanner::replan_new(double allocated_time_secs, double allocated_time_per_retry_, std::vector<int>* solution_stateIDs_V, int* psolcost)
{
	SBPL_INFO("Adaptive planning begins...");
	adaptive_environment_->setPlanMode();
	//DO THE ADAPTIVE PLANNING LOOP HERE
	std::vector<int> planning_stateV;
	std::vector<int> tracking_stateV;

	MY_TIME_TYPE start_t = MY_TIME_NOW;
	track_time_total_s = 0;
	plan_time_total_s = 0;

	int round = 0;

	double ad_plan_time_alloc = allocated_time_per_retry_;
	double ad_track_time_alloc = allocated_time_per_retry_;

	SBPL_INFO("Retry time limits: (Planning: %.4f) (Tracking: %.4f)", ad_plan_time_alloc, ad_track_time_alloc);

	int planning_bRet;
	int tracking_bRet;

	SBPL_INFO("Setting start and goal for planner and tracker...");

	if(planner->set_start(StartStateID) == 0)
	{
		SBPL_ERROR("ERROR: planner failed to set start state");
		throw SBPL_Exception();
	}

	if(tracker->set_start(StartStateID) == 0){
		SBPL_ERROR("ERROR: tracker failed to set start state");
		throw SBPL_Exception();
	}

	if(planner->set_goal(GoalStateID) == 0)
	{
		SBPL_ERROR("ERROR: planner failed to set goal state");
		throw SBPL_Exception();
	}

	if(tracker->set_goal(GoalStateID) == 0){
		SBPL_ERROR("ERROR: tracker failed to set goal state");
		throw SBPL_Exception();
	}

	SBPL_INFO("Setting environment in planning mode...");
	adaptive_environment_->setPlanMode();

	// @avemula TODO might need to remove this
	//SBPL_INFO("Adding start (%d, %.3f) and goal (%d, %.3f) spheres...", StartStateID, newSphereRad, GoalStateID, newSphereRad);
	// @avemula CHANGED commented out the following two lines.
	// No initial spheres placed at start and goal
	//adaptive_environment_->addSphere(GoalStateID,  newSphereRad);
	adaptive_environment_->addSphere(StartStateID, newSphereRad);

	std::vector<int> ModifiedStates;
	std::vector<int> TrkModifiedStates;
	repair_time = 0.0;
	
	SBPL_INFO("Forcing planning from scratch for planner and tracker...");
	planner->force_planning_from_scratch();
	tracker->force_planning_from_scratch();
	
	std::vector<int> LastTrackPath;
	bool LastTrackSuccess = false;
	double t_elapsed_s;

	//stat_->setInitialEps(planningEPS*trackingEPS);

#if ADP_STEPBYSTEP
	SBPL_INFO("Ready to plan!");
	this->pause();
#endif


	std::vector<int> new_sphere_locations; //as stateIDs
	do {
		if(MY_TIME_DIFF_S(MY_TIME_NOW, start_t) > allocated_time_secs){
			SBPL_INFO("Search ran out of time!");
			solution_stateIDs_V->clear();
			for(unsigned int i = 0; i < LastTrackPath.size(); i++){
				solution_stateIDs_V->push_back(LastTrackPath[i]);
			}
			SBPL_INFO("Done in: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
			nIterations = round;

			//stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
			//stat_->setFinalEps(-1.0);
			return LastTrackSuccess;
		}

		SBPL_INFO("=======================================");
		SBPL_INFO("||          Iteration %03d            ||", round);
		SBPL_INFO("=======================================");
		MY_TIME_TYPE iter_start = MY_TIME_NOW;
		MY_TIME_TYPE plan_start = MY_TIME_NOW;
		MY_TIME_TYPE track_start;

		double plan_time_s = 0;
		double track_time_s = 0;

		int p_Cost;		  //planning solution cost
		int t_Cost;		  //tracking solution cost

		//==================================== PLANNING =================================
		SBPL_INFO("\t=======================================");
		SBPL_INFO("\t||          Planning Phase           ||");
		SBPL_INFO("\t=======================================");

		planner->force_planning_from_scratch();
		planner->set_initialsolution_eps(planningEPS);
		planner->set_search_mode(false);

		adaptive_environment_->setPlanMode();
		//add pending new spheres
		SBPL_INFO("Adding %zd pending spheres...", new_sphere_locations.size());
		for(int stateID : new_sphere_locations){
			adaptive_environment_->addSphere(stateID, newSphereRad, &ModifiedStates);
		}
		new_sphere_locations.clear();
		planning_stateV.clear();
		planning_bRet = 0;

#if ADP_STEPBYSTEP
		//adaptive_environment_->visualizeEnvironment();
		SBPL_INFO("Ready for planning phase!");
		this->pause();
#endif

		//PLANNING HERE!
		t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);

		if(t_elapsed_s > allocated_time_secs){
		  SBPL_INFO("No solution found within the allocated time");		  
		  return false;
		}

		// @avemula CHANGED removed the looped execution of the planner's replan function
		/*while(t_elapsed_s < allocated_time_secs){
			SBPL_INFO("Still have time (%.3fs)...planning", allocated_time_secs - t_elapsed_s);
			planning_bRet = planner->replan(ad_plan_time_alloc, &planning_stateV, &p_Cost);
			t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
			if(planning_bRet) break;
			}*/

		// @avemula ADDED one single replan call with all the time given
		planning_bRet = planner->replan(allocated_time_secs - t_elapsed_s, &planning_stateV, &p_Cost);

		plan_time_s = MY_TIME_DIFF_S(MY_TIME_NOW, plan_start);
		plan_time_total_s += plan_time_s;
		//stat_->addPlanningPhaseTime(plan_time_s);

		SBPL_INFO("Planner done in %.3fs...", plan_time_s);

#if ADP_STEPBYSTEP
		//adaptive_environment_->visualizeEnvironment();
		SBPL_INFO("Planning phase done!");
		this->pause();
#endif

		// @avemula REMOVED no path in AD doesn't imply no path at all. We need to grow all the spheres
		/*if(!planning_bRet || planning_stateV.size() == 0){
			SBPL_ERROR("Solution could not be found within the allowed time (%.3fs.) after %d iterations", ad_plan_time_alloc, round);
			#if ADP_GRAPHICAL
			//adaptive_environment_->visualizeEnvironment();
			#endif
			nIterations = round;
			//stat_->setFinalEps(-1.0);
			//stat_->setSuccess(false);
			//stat_->setNumIterations(nIterations+1);
			return planning_bRet;
			}*/

		// @avemula ADDED if no path found, grow all the spheres and try again		
		if(!planning_bRet || planning_stateV.size() == 0) {
		  adaptive_environment_->growAllSpheres();
		  continue;
		}

		// @avemula REMOVED checking whether the path is high-dimensional
		/*if(adaptive_environment_->isHighDPath(&planning_stateV)){
		  solution_stateIDs_V->clear();
		  for(unsigned int i = 0; i < planning_stateV.size(); i++){
		  solution_stateIDs_V->push_back(planning_stateV[i]);
		  }
		  SBPL_INFO("Iteration Time: %.3f sec (avg: %.3f)", MY_TIME_DIFF_S(MY_TIME_NOW, iter_start), MY_TIME_DIFF_S(MY_TIME_NOW, start_t) / (round+1.0));
		  SBPL_INFO("Done in: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
		  nIterations = round;
		  stat_->setFinalEps(planner->get_final_epsilon());
		  stat_->setFinalPlanCost(p_Cost);
		  stat_->setFinalTrackCost(p_Cost);
		  stat_->setNumIterations(nIterations+1);
		  stat_->setSuccess(true);
		  stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
		  return true;
		}*/

		// @avemula ADDED our version of tracking which is a little bit different since we just check whether the path found
		// is in collision with any dynamic obstacle or not
		#if ADP_GRAPHICAL
		adaptive_environment_->visualizeEnvironment();
		adaptive_environment_->visualizeStatePath(&planning_stateV, 0, 120, "planning_path");
		#endif
		
		vector<int> collision_location;
		if(adaptive_environment_->isCollisionFree(&planning_stateV, &collision_location)) {
		    solution_stateIDs_V->clear();
		    for(unsigned int i = 0; i < planning_stateV.size(); i++){
		      solution_stateIDs_V->push_back(planning_stateV[i]);
		    }
		    SBPL_INFO("Iteration Time: %.3f sec (avg: %.3f)", MY_TIME_DIFF_S(MY_TIME_NOW, iter_start), MY_TIME_DIFF_S(MY_TIME_NOW, start_t) / (round+1.0));
		    SBPL_INFO("Done in: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
		    nIterations = round;
		    SBPL_INFO("Number of iterations is %d\n", nIterations);
		    return true;
		  }
		else {
		  // push locations into new spheres list
		  for(int i : collision_location) {
		    new_sphere_locations.push_back(i);
		  }
		}
		
		

		
		SBPL_INFO("Iteration Time: %.3f sec (avg: %.3f)", MY_TIME_DIFF_S(MY_TIME_NOW, iter_start), MY_TIME_DIFF_S(MY_TIME_NOW, start_t) / (round+1.0));
		SBPL_INFO("Total Time so far: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
		round++;
	} while (true);

	/*stat_->setFinalEps(-1);
	stat_->setFinalPlanCost(INFINITECOST);
	stat_->setFinalTrackCost(INFINITECOST);
	stat_->setNumIterations(round+1);
	stat_->setSuccess(false);
	stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));*/

	return tracking_bRet;
}




int AdaptivePlanner::set_goal(int goal_stateID)
{
	GoalStateID = goal_stateID;
	SBPL_INFO("goal set (StateID: %d)", goal_stateID);
	return 1;
}


int AdaptivePlanner::set_start(int start_stateID)
{
	StartStateID = start_stateID;
	SBPL_INFO("start set (StateID: %d)", start_stateID);
	return 1;
}


int AdaptivePlanner::force_planning_from_scratch()
{
	if(planner != NULL && tracker != NULL){
		SBPL_INFO("Resetting planner and tracker!");
		planner->force_planning_from_scratch();
		tracker->force_planning_from_scratch();
	}
	adaptive_environment_->reset();
	stat_->reset();
	return 1;
}

int AdaptivePlanner::force_planning_from_scratch_and_free_memory()
{
	if(planner != NULL && tracker != NULL){
		SBPL_INFO("Resetting planner and tracker and freeing memory!");
		planner->force_planning_from_scratch_and_free_memory();
		tracker->force_planning_from_scratch_and_free_memory();
	}
	adaptive_environment_->reset();
	stat_->reset();
	return 1;
}


int AdaptivePlanner::set_search_mode(bool bSearchUntilFirstSolution)
{
	bsearchuntilfirstsolution = bSearchUntilFirstSolution;
	//set search mode
	planner->set_search_mode(bsearchuntilfirstsolution);
	tracker->set_search_mode(bsearchuntilfirstsolution);
	return 1;
}


void AdaptivePlanner::print_searchpath(FILE* fOut)
{
	SBPL_WARN("print_searchpath() NOT IMPLEMENTED YET!");
	return;
}

bool AdaptivePlanner::set_new_sphere_radius(double rad){
	if(rad <= 0) return false;
	newSphereRad = rad;
	return true;
}

bool AdaptivePlanner::set_tunnel_width(double w){
	if(w < 0) return false;
	tunnelWidth = w;
	return true;
}

bool AdaptivePlanner::set_time_per_retry(double t){
	timePerRetry = t;
	return true;
}

void AdaptivePlanner::set_initialsolution_eps(double initialsolution_eps){
  // @avemula ASK
	targetEPS = initialsolution_eps;
	//planningEPS = sqrt(initialsolution_eps);
	//trackingEPS = sqrt(initialsolution_eps);
	planningEPS = initialsolution_eps;
	trackingEPS = initialsolution_eps;
	planner->set_initialsolution_eps(planningEPS);
	tracker->set_initialsolution_eps(trackingEPS);
}

void AdaptivePlanner::set_initialsolution_eps_plan(double initialsolution_eps){
	targetEPS = initialsolution_eps;
	planningEPS = initialsolution_eps;
	trackingEPS = initialsolution_eps;
	planner->set_initialsolution_eps(planningEPS);
	tracker->set_initialsolution_eps(trackingEPS);
}

int AdaptivePlanner::replan(vector<int>* solution_stateIDs_V, ReplanParams params, int* solcost){
  set_initialsolution_eps(params.initial_eps);
  bsearchuntilfirstsolution = params.return_first_solution;
  return replan(params.max_time, params.repair_time, solution_stateIDs_V, solcost);
}
