/*
 * SCVStat.h
 *
 *  Created on: Sep 14, 2015
 *      Author: kalin
 */

#ifndef _SBPL_ADAPTIVE_SCVSTAT_H_
#define _SBPL_ADAPTIVE_SCVSTAT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>

class AdaptivePlannerCSVStat_c {
public:

	AdaptivePlannerCSVStat_c(){
		n_expansions = 0;
		n_expansions_low = 0;
		n_expansions_high = 0;
		n_expansions_near = 0;
		n_expansions_track = 0;
		n_iterations = 0;
		t_planning_phase = 0;
		t_tracking_phase = 0;
		t_total = 0;
		d_final_eps = 0;
		d_initial_eps = 0;
		cost_plan = 0;
		cost_track = 0;
		b_success = false;
	}

	~AdaptivePlannerCSVStat_c(){};

	bool writeToFile(std::string name){
		if(fileExists(name)){
			return appendToFile(name);
		} else {
			FILE* file = fopen(name.c_str(), "w");
			if(!file) return false;
			fprintf(file, "Success, Initial Eps, Planning Phase Time, Tracking Phase Time, Total Time, Num Iterations, Num Expansions LD, Num Expansions NearHD, Num Expansions HD, Num Expansions Track, Num Expansions Total, Final Plan Cost, Final Track Cost, Final Eps\n");
			fprintf(file, "%s, %.5f, %.5f, %.5f, %.5f, %u, %lu, %lu, %lu, %lu, %lu, %lu, %lu, %.5f\n",
					(b_success)?std::string("True").c_str():std::string("False").c_str(), d_initial_eps,
					t_planning_phase, t_tracking_phase, t_total,
					n_iterations,
					n_expansions_low, n_expansions_near, n_expansions_high, n_expansions_track, n_expansions,
					cost_plan, cost_track,
					d_final_eps);
			fclose(file);
			return true;
		}
	}

	bool appendToFile(std::string name){
		if(!fileExists(name)){
			return writeToFile(name);
		} else {
			FILE* file = fopen(name.c_str(), "a");
			if(!file) return false;
			//fprintf(file, "Success, Initial Eps, Planning Phase Time, Tracking Phase Time, Total Time, Num Iterations, Num Expansions LD, Num Expansions NearHD, Num Expansions HD, Num Expansions Track, Num Expansions Total, Final Plan Cost, Final Track Cost, Final Eps\n");
			fprintf(file, "%s, %.5f, %.5f, %.5f, %.5f, %u, %lu, %lu, %lu, %lu, %lu, %lu, %lu, %.5f\n",
					(b_success)?std::string("True").c_str():std::string("False").c_str(), d_initial_eps,
					t_planning_phase, t_tracking_phase, t_total,
					n_iterations,
					n_expansions_low, n_expansions_near, n_expansions_high, n_expansions_track, n_expansions,
					cost_plan, cost_track,
					d_final_eps);
			fclose(file);
			return true;
		}
	}

	bool fileExists(std::string name){
		if (FILE *file = fopen(name.c_str(), "r")) {
			fclose(file);
			return true;
		} else {
			return false;
		}
	}

	void recordLDExpansion(){
		n_expansions_low++;
		n_expansions++;
	}

	void recordTrackExpansion(){
		n_expansions_track++;
		n_expansions++;
	}

	void recordHDExpansion(){
		n_expansions_high++;
		n_expansions++;
	}

	void recordNearHDExpansion(){
		n_expansions_near++;
		n_expansions++;
	}

	void addPlanningPhaseTime(double t){
		t_planning_phase += t;
	}

	void addTrackingPhaseTime(double t){
		t_tracking_phase += t;
	}

	void setTotalPlanningTime(double t){
		t_total = t;
	}

	void setInitialEps(double e){
		d_initial_eps = e;
	}

	void setFinalEps(double e){
		d_final_eps = e;
	}

	void setFinalPlanCost(unsigned long c){
		cost_plan = c;
	}

	void setFinalTrackCost(unsigned long c){
		cost_track = c;
	}

	void setSuccess(bool success){
		b_success = success;
	}

	void setNumIterations(int n_iter){
		n_iterations = n_iter;
	}

	void reset(){
		n_expansions = 0;
		n_expansions_low = 0;
		n_expansions_high = 0;
		n_expansions_near = 0;
		n_expansions_track = 0;
		n_iterations = 0;
		t_planning_phase = 0;
		t_tracking_phase = 0;
		t_total = 0;
		d_final_eps = 0;
		d_initial_eps = 0;
		cost_plan = 0;
		cost_track = 0;
		b_success = false;
	}

        double getInitialEps() { return d_initial_eps; }
	double getFinalEps() { return d_final_eps; }

        double getTimePlanning() { return t_planning_phase; }
	double getTimeTracking() { return t_tracking_phase; }
	double getTimeTotal() { return t_total; }

	unsigned int getNumIterations() { return n_iterations; }

	unsigned long getNumExpLD() { return n_expansions_low; }
	unsigned long getNumExpNearHD() { return n_expansions_near; }
	unsigned long getNumExpHD() { return n_expansions_high; }
	unsigned long getNumExpTrack() { return n_expansions_track; }
	unsigned long getNumExpTotal() { return n_expansions; }

	unsigned long getPlanCost() { return cost_plan; }
	unsigned long getTrackCost() { return cost_track; }

private:
	//number of expansions
	unsigned long n_expansions;
	unsigned long n_expansions_low;
	unsigned long n_expansions_near;
	unsigned long n_expansions_high;
	unsigned long n_expansions_track;
	//planning times
	double t_planning_phase;
	double t_tracking_phase;
	double t_total;
	//num iterations
	unsigned int n_iterations;
	bool b_success;
	//epsilon
	double d_final_eps;
	double d_initial_eps;
	//cost
	unsigned long cost_track;
	unsigned long cost_plan;
	//planner config ???
};



#endif /* SRC_MRSLC_PLANNER_SBPL_ADAPTIVE_INCLUDE_SBPL_ADAPTIVE_SCVSTAT_H_ */
