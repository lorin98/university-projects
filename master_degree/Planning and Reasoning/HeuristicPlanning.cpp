#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Planner.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/config.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <valarray>
#include <fstream>
#include <limits>

#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>

namespace ob = ompl::base;
// assuming we do the geometric planning
namespace og = ompl::geometric;
namespace po = boost::program_options;

// return the clearance between the robot and the given obstacle
double check_obstacle(const ob::State *state, const double x_obs, const double y_obs, const double radius) {
	const auto *s = state->as<ob::SE2StateSpace::StateType>();
	double x=s->getX(), y=s->getY();
	
	return sqrt((x - x_obs)*(x - x_obs) + (y - y_obs)*(y - y_obs)) - radius - 0.2;
	
}

// return the MINIMUM clearance between the robot and every obstacles
double collision_obstacles(const ob::State *state) {
	// collecting all the clearances
	double clearances[18];
	clearances[0] = check_obstacle(state, -1.4, 1.45, 0.3);
	clearances[1] = check_obstacle(state, -2, 1.65, 0.2);
	clearances[2] = check_obstacle(state, -2, 0.75, 0.2);
	clearances[3] = check_obstacle(state, -1.625, 0.375, 0.2);
	clearances[4] = check_obstacle(state, -1.25, 0., 0.2);
	clearances[5] = check_obstacle(state, -1.625, -0.375, 0.2);
	clearances[6] = check_obstacle(state, -2, -0.75, 0.2);
	clearances[7] = check_obstacle(state, -2, -1.65, 0.2);
	clearances[8] = check_obstacle(state, -1.4, -1.45, 0.3);
	clearances[9] = check_obstacle(state, 1.4, 1.45, 0.3);
	clearances[10] = check_obstacle(state, 2, 1.65, 0.2);
	clearances[11] = check_obstacle(state, 2, 0.75, 0.2);
	clearances[12] = check_obstacle(state, 1.625, 0.375, 0.2);
	clearances[13] = check_obstacle(state, 1.25, 0., 0.2);
	clearances[14] = check_obstacle(state, 1.625, -0.375, 0.2);
	clearances[15] = check_obstacle(state, 2, -0.75, 0.2);
	clearances[16] = check_obstacle(state, 2, -1.65, 0.2);
	clearances[17] = check_obstacle(state, 1.4, -1.45, 0.3);
	
	// iterating over all the clearances to compute the minimum
	double min = clearances[0];
	for (int i = 1; i < 18; i++) {
		if (clearances[i] < min) {
			min = clearances[i]; 
		}
	}
	return min;
}

double collision_maze(const ob::State *state) {
	// collecting all the clearances with a brutal approach:
	// we define a set of control points along the walls of the maze
	double clearances[225];
	int c = 0;
	double x, y;
	for (int i = 0; i < 21; i++){
		y = i / 4.;
		x = 0.;
		clearances[c] = check_obstacle(state, x, y, 0.);
		c++;
		x = 5.;
		clearances[c] = check_obstacle(state, x, y, 0.);
		c++;
		if (y == 0) {
			for (int j = 1; j < 20; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c++;
			}
		}
		
		else if (y == 0.5 || y == 4.5) {
			for (int j = 2; j < 19; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c++;
			}
		}
		
		else if (y == 0.75 || y == 4.25) {
			x = 0.5;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			x = 4.5;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
		}
		
		else if (y == 1.) {
			x = 0.5;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			
			for (int j = 4; j < 11; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c++;
			}
			for (int j = 12; j < 19; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c++;
			}
		}
		
		else if (y == 1.25 || y == 3.75) {
			x = 0.5;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			x = 1.;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			x = 4.;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			x = 4.5;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
		}
		
		else if (y == 1.5 || y == 3.5) {
			x = 0.5;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			x = 1.;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			
			for (int j = 6; j < 15; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c++;
			}
			x = 4.;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			x = 4.5;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			
		}
		
		else if (y == 1.75 || y == 2 || y == 2.25 || y == 2.5 || y == 2.75 || y == 3 || y == 3.25) {
			x = 0.5;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			x = 1.;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			x = 1.5;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			if (y != 3.25) {
				x = 3.5;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c++;
			}
			if (y == 2.5) {
				x = 0.25;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c++;
				x = 3.75;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c++;
			}
			x = 4.;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			
			if (y != 2.75) {
				x = 4.5;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c++;
			}
		}
		
		else if (y == 4.) {
			x = 0.5;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
			
			for (int j = 4; j < 17; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c++;
			}
			x = 4.5;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c++;
		}
		
		else if (y == 5.) {
			for (int j = 0; j < 9; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c++;
			}
			for (int j = 10; j < 21; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c++;
			}
		}
		
	}
	
	// iterating over all the clearances to compute the minimum
	double min = clearances[0];
	for (int i = 1; i < 225; i++) {
		if (clearances[i] < min) {
			min = clearances[i]; 
		}
	}
	return min;
}

double collision_real(const ob::State *state) {
	double clearances[834];
	double x, y;
	int c = 0;
	// horizontal walls
	for (int i = 8; i < 89; i++) {
		y = i / 4.;
		
		if (y == 2.) {
			for (int j = 56; j < 81; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		
		else if (y == 6.) {
			for (int j = 8; j < 25; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
			for (int j = 36; j < 73; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
			for (int j = 80; j < 113; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		
		else if (y == 12.) {
			for (int j = 56; j < 73; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
			for (int j = 88; j < 113; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		
		else if (y == 14.) {
			for (int j = 8; j < 49; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
			for (int j = 96; j < 113; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		
		else if (y == 18.) {
			for (int j = 32; j < 57; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
			for (int j = 64; j < 73; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		
		else if (y == 22.) {
			for (int j = 8; j < 113; j++) {
				x = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
	}
	// vertical walls
	for (int i = 8; i < 113; i++) {
		x = i / 4.;
		
		if (x == 2.) {
			for (int j = 24; j < 89; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		
		else if (x == 8.) {
			for (int j = 72; j < 81; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		
		else if (x == 12.) {
			for (int j = 64; j < 73; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		
		else if (x == 14.) {
			for (int j = 8; j < 17; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
			for (int j = 24; j < 49; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
			for (int j = 72; j < 89; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		
		else if (x == 18.) {
			for (int j = 72; j < 89; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		
		else if (x == 20.) {
			for (int j = 8; j < 49; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		
		else if (x == 22.) {
			for (int j = 56; j < 89; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		
		else if (x == 28.) {
			for (int j = 24; j < 49; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
			for (int j = 56; j < 89; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
	}
	// obstacles
	// - rectangles
	for (int i = 26; i < 39; i++) {
		x = i / 4.;
		
		if (x == 6.5 || x == 9.5) {
			for (int j = 41; j < 48; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		else {
			y = 10.25;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
			y = 11.75;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
		}
	}
	for (int i = 8; i < 17; i++) {
		x = i / 4.;
		
		if (x == 2. || x == 4.) {
			for (int j = 53; j < 60; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		else {
			y = 13.25;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
			y = 14.75;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
		}
	}
	for (int i = 8; i < 17; i++) {
		x = i / 4.;
		
		if (x == 2. || x == 4.) {
			for (int j = 73; j < 89; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		else {
			y = 18.25;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
		}
	}
	for (int i = 52; i < 57; i++) {
		x = i / 4.;
		
		if (x == 13. || x == 14.) {
			for (int j = 8; j < 17; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		else {
			y = 2.;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
			y = 4.;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
		}
	}
	for (int i = 56; i < 63; i++) {
		x = i / 4.;
		
		if (x == 15.75) {
			for (int j = 24; j < 31; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		else {
			y = 7.45;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
		}
	}
	for (int i = 56; i < 63; i++) {
		x = i / 4.;
		
		if (x == 15.5) {
			for (int j = 24; j < 31; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		else {
			y = 7.45;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
		}
	}
	for (int i = 56; i < 60; i++) {
		x = i / 4.;
		
		if (x == 14.75) {
			for (int j = 40; j < 49; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		else {
			y = 10.;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
		}
	}
	for (int i = 72; i < 81; i++) {
		x = i / 4.;
		
		if (x == 20.) {
			for (int j = 85; j < 89; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		else {
			y = 21.25;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
		}
	}
	for (int i = 88; i < 92; i++) {
		x = i / 4.;
		
		if (x == 22.75) {
			for (int j = 80; j < 89; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		else {
			y = 20.;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
		}
	}
	for (int i = 109; i < 113; i++) {
		x = i / 4.;
		
		if (x == 27.25) {
			for (int j = 24; j < 33; j++) {
				y = j / 4.;
				clearances[c] = check_obstacle(state, x, y, 0.);
				c += 1;
			}
		}
		else {
			y = 8.;
			clearances[c] = check_obstacle(state, x, y, 0.);
			c += 1;
		}
	}
	// - circles
	clearances[c] = check_obstacle(state, 16, 20, 1.25);
	c += 1;
	clearances[c] = check_obstacle(state, 11, 20, 1.5);
	c += 1;
	clearances[c] = check_obstacle(state, 5.5, 16.25, 2);
	c += 1;
	clearances[c] = check_obstacle(state, 25, 19, 1.75);
	c += 1;
	clearances[c] = check_obstacle(state, 24, 9.5, 1.75);
	c += 1;
	
	// iterating over all the clearances to compute the minimum
	double min = clearances[0];
	for (int i = 1; i < 834; i++) {
		if (clearances[i] < min) {
			min = clearances[i]; 
		}
	}
	return min;
}

// state validity checker class
// it handles invalid states and clearances between obstacles
class ValidityChecker : public ob::StateValidityChecker
{
protected:
	ob::SpaceInformationPtr spi;
	std::string exp;
public:
	ValidityChecker(const ob::SpaceInformationPtr& si, const std::string exp) :
		ob::StateValidityChecker(si) {
			this->spi = si;
			this->exp = exp;
		}

	// Returns whether the given state's position overlaps the
	// circular obstacles or crosses the boundaries
	bool isValid(const ob::State* state) const
	{
		const auto *s = state->as<ob::SE2StateSpace::StateType>();
		double x=s->getX(), y=s->getY();
		
		// ONLY in "obstacles" env, we have a narrow corridor to take into account
		bool corridor = this->exp == "obstacles" ? (x < -0.7 || x > 0.7 || (y > -0.05 && y < 0.05)) : true;
		
		return this->spi->satisfiesBounds(state)
				&& corridor
				&& this->clearance(state) > 0.01;
	}

	// Returns the distance from the given state's position to the
	// boundary of the circular obstacles
	double clearance(const ob::State* state) const
	{	
		if (this->exp == "obstacles") return collision_obstacles(state);
		else if (this->exp == "maze") return collision_maze(state);
		else if (this->exp == "real") return collision_real(state);
		// to avoid compiler warinings
		else return 0.0;
	}
};

// optimization objectives:
// 1) path length minimization
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(100));
    return obj;
}

// 2) minimum clearance maximization
ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si, const bool use_informed)
{
    ob::OptimizationObjectivePtr obj(new ob::MaximizeMinClearanceObjective(si));
    obj->setCostThreshold(ob::Cost(0.01));
    // needed to make this objective actually working with informed sampling
    if (use_informed) obj->setCostToGoHeuristic(ob::goalRegionCostToGo);
    return obj;
}

// 3) combination of 1) and 2) (N.B. we need to define a slightly different clearance function)
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }
 
    ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};

ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si, const bool use_informed)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));
	
    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(lengthObj, 5.0);
    opt->addObjective(clearObj, 5.0);
	
	// needed to make this objective actually working with informed sampling
	if (use_informed) opt->setCostToGoHeuristic(ob::goalRegionCostToGo);
	
	opt->setCostThreshold(ob::Cost(1000));
    return ob::OptimizationObjectivePtr(opt);
}

// utility for stack of planner in benchmarking
void addPlanner(ompl::tools::Benchmark& benchmark, const ob::PlannerPtr& planner)
{
     ob::ParamSet& params = planner->params();
     benchmark.addPlanner(planner);
}

void planWithSimpleSetup(og::SimpleSetup ss,
						 const std::string output_path,
                         const std::string graph_path,
                         int runs) {
	
	// in case of single-run experiment we need to output the solution path and the search tree
	if (runs < 2){
		
		ss.setup();
		ss.print();

		// attempt to solve the problem within the specified seconds of planning time
		ob::PlannerStatus solved = ss.solve(150);

		// if the solution has been found
		if(solved)
		{

			std::cout << "Found solution." << std::endl;
			og::PathGeometric path = ss.getSolutionPath();
			path.interpolate();
			
			ob::PlannerPtr planner = ss.getPlanner();
			
			/*
			//double cost = planner->as<og::RRTstar>()->bestCost().value();
			int len = path.getStateCount();
			//std::cout << "With cost: " << cost << std::endl;
			std::cout << "With length: " << len << std::endl;
			// routine to run the algorithm until a minimum/maximum desired solution cost/length is found
            while ((len > 200) || !ss.haveExactSolutionPath()) {
                ss.clear();
                ss.solve(150);
                path = ss.getSolutionPath();
                path.interpolate();                
				//cost = planner->as<og::RRTstar>()->bestCost().value();
				len = path.getStateCount();
				//std::cout << "With cost: " << cost << std::endl;
				std::cout << "With length: " << len << std::endl;
            }
            */

			if (!ss.haveExactSolutionPath())
			{
				std::cout << "Solution is approximate. Distance to actual goal is "
					   << ss.getProblemDefinition()->getSolutionDifference() << "." << std::endl;
			}

			
			if (graph_path != "") {
				// extracting planner data from most recent solve attempt
				ob::PlannerData pd(ss.getSpaceInformation());
				ss.getPlannerData(pd);

				// printing into the GraphML file
				std::ofstream graph_file;
				graph_file.open(graph_path);
				pd.printGraphML(graph_file);
				graph_file.close();
				std::cout << "Planner data was written in the graphml file: " << graph_path << "." << std::endl;
			}
			
			// print the solution to the output file rather than to stdout
			if (output_path != "") {
				std::ofstream output_file;
				output_file.open(output_path);
				path.printAsMatrix(output_file);
				// also the desired goal
				ob::SE2StateSpace::StateType *goal = ss.getGoal()->as<ob::GoalState>()->getState()->as<ob::SE2StateSpace::StateType>();
				const double x = goal->getX();
				const double y = goal->getY();
				const double theta = goal->getYaw();
				output_file << x << " " << y << " " << theta;
				output_file.close();		
				std::cout << "Solution was written in the output file: " << output_path << "." << std::endl;
			}
		
		}
		else
			std::cout << "No solution found." << std::endl;

	}
	else {
		// if runs > 1 we get here
		// planning parameters:
		// 	    runtime_limit: experiment execution time
		//   	memory_limit:  experiment memory limit
		//   	runs:  how many times to run each planner
		double memory_limit = 100000, runtime_limit = 150.;
		ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, runs);
		ompl::tools::Benchmark b(ss, "obstacle avoidance"); 

		// add here all the planners you want to test
		//addPlanner(b, std::make_shared<og::RRT>(ss.getSpaceInformation()));
		addPlanner(b, std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
		addPlanner(b, std::make_shared<og::InformedRRTstar>(ss.getSpaceInformation()));

		b.benchmark(request);
		b.saveResultsToFile("solution.log");
		
	}

}

void planObstacles(const std::string output_path,
					const std::string graph_path,
					const std::string planner,
					const std::string obj,
					int runs){
						
	// construct the state space we are planning in
	auto space(std::make_shared<ob::SE2StateSpace>());

	// set the bounds for the R^2 part of SE(2)
	ob::RealVectorBounds bounds(2);
	bounds.setLow(-3);
	bounds.setHigh(3);

	space->setBounds(bounds);
	
	// define a simple setup class
	og::SimpleSetup ss(space);
	
	// set state validity checking for this space
	ss.setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(ss.getSpaceInformation(), "obstacles")));
	
	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);

	// start and goal for obstacles
	start[0] = -2.; // x
	start[1] = 0.; // y
	start[2] = 0.;  // theta

	goal[0] = 2.;	// x
	goal[1] = 0.;   // y
	goal[2] = -boost::math::constants::pi<double>(); // theta
	
	// set these states as start and goal for the SimpleSetup
	// the third parameter denotes the threshold
	ss.setStartAndGoalStates(start, goal, 0.2);
	
	bool use_informed = false;
	
	// instatiating requested planner
	if (planner == "RRTstar"){
		og::RRTstar *rrt = new og::RRTstar(ss.getSpaceInformation());
		ob::PlannerPtr plr(rrt);	
		ss.setPlanner(plr);
	}
	else if (planner == "InfRRTstar"){
		og::InformedRRTstar *rrt = new og::InformedRRTstar(ss.getSpaceInformation());
		ob::PlannerPtr plr(rrt);	
		ss.setPlanner(plr);
		use_informed = true;
	}
	else{
		og::RRT *rrt = new og::RRT(ss.getSpaceInformation());
		ob::PlannerPtr plr(rrt);	
		ss.setPlanner(plr);
	}
	
	// setting the optimization objective, if possible
	if (obj == "length") ss.setOptimizationObjective(getPathLengthObjective(ss.getSpaceInformation()));
	else if (obj == "clearance") ss.setOptimizationObjective(getClearanceObjective(ss.getSpaceInformation(), use_informed));
	else if (obj == "multi") ss.setOptimizationObjective(getBalancedObjective(ss.getSpaceInformation(), use_informed));
	
	planWithSimpleSetup(ss, output_path, graph_path, runs);
						
}


void planMaze(const std::string output_path,
			  const std::string graph_path,
			  const std::string planner,
			  const std::string obj,
			  int runs){
						
	// construct the state space we are planning in
	auto space(std::make_shared<ob::SE2StateSpace>());

	// set the bounds for the R^2 part of SE(2)
	ob::RealVectorBounds bounds(2);
	bounds.setLow(0);
	bounds.setHigh(0, 5);
	bounds.setHigh(1, 5);

	space->setBounds(bounds);
	
	// define a simple setup class
	og::SimpleSetup ss(space);
	
	// set state validity checking for this space
	ss.setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(ss.getSpaceInformation(), "maze")));
	
	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);

	// start and goal for maze
	start[0] = 2.5; // x
	start[1] = 2.5; // y
	start[2] = 0.;  // theta

	goal[0] = 0.25;	// x
	goal[1] = 0.25;   // y
	goal[2] = - boost::math::constants::pi<double>() / 2; // theta
	
	// set these states as start and goal for the SimpleSetup
	// the third parameter denotes the threshold
	ss.setStartAndGoalStates(start, goal, 0.2);

	bool use_informed = false;
	
	// instatiating requested planner
	if (planner == "RRTstar"){
		og::RRTstar *rrt = new og::RRTstar(ss.getSpaceInformation());
		ob::PlannerPtr plr(rrt);	
		ss.setPlanner(plr);
	}
	else if (planner == "InfRRTstar"){
		og::InformedRRTstar *rrt = new og::InformedRRTstar(ss.getSpaceInformation());
		ob::PlannerPtr plr(rrt);	
		ss.setPlanner(plr);
		use_informed = true;
	}
	else{
		og::RRT *rrt = new og::RRT(ss.getSpaceInformation());
		ob::PlannerPtr plr(rrt);	
		ss.setPlanner(plr);
	}
	
	// setting the optimization objective, if possible
	if (obj == "length") ss.setOptimizationObjective(getPathLengthObjective(ss.getSpaceInformation()));
	else if (obj == "clearance") ss.setOptimizationObjective(getClearanceObjective(ss.getSpaceInformation(), use_informed));
	else if (obj == "multi") ss.setOptimizationObjective(getBalancedObjective(ss.getSpaceInformation(), use_informed));
	
	planWithSimpleSetup(ss, output_path, graph_path, runs);
						
}


void planReal(const std::string output_path,
			  const std::string graph_path,
			  const std::string planner,
			  const std::string obj,
			  int runs){
						
	// construct the state space we are planning in
	auto space(std::make_shared<ob::SE2StateSpace>());

	// set the bounds for the R^2 part of SE(2)
	ob::RealVectorBounds bounds(2);
	bounds.setLow(0);
	bounds.setHigh(0, 32);
	bounds.setHigh(1, 24);

	space->setBounds(bounds);
	
	// define a simple setup class
	og::SimpleSetup ss(space);
	
	// set state validity checking for this space
	ss.setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(ss.getSpaceInformation(), "real")));
	
	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);

	// start and goal for real
	start[0] = 2.675; // x
	start[1] = 6.5; // y
	start[2] = boost::math::constants::pi<double>() / 2;  // theta

	// read an image
    cv::Mat image= cv::imread("solution_to_video/real/real_env.png");
    cv::resize(image, image, cv::Size(), 0.5, 0.5);
    cv::namedWindow("Real Environment", 1);
    // this variable will store the desired goal ID
    int id = 0;
    cv::createTrackbar("The robot is in its charging station at [0].\nPlease, move the slider to set a desired goal.\nThen, close the window to start planning.", "Real Environment", &id, 10, NULL);
    // show the image on window
    cv::imshow("Real Environment", image);
    cv::waitKey(0);

	// creating goal state according to the desired destination
	switch (id) {
		case 1:
			goal[0] = 8;
			goal[1] = 12.5;
			break;
		case 2:
			goal[0] = 5;
			goal[1] = 20;
			break;
		case 3:
			goal[0] = 9;
			goal[1] = 18.5;
			break;
		case 4:
			goal[0] = 17;
			goal[1] = 18.5;
			break;
		case 5:
			goal[0] = 27;
			goal[1] = 20;
			break;
		case 6:
			goal[0] = 25;
			goal[1] = 7;
			break;
		case 7:
			goal[0] = 17;
			goal[1] = 7;
			break;
		case 8:
			goal[0] = 23;
			goal[1] = 1;
			break;
		case 9:
			goal[0] = 17;
			goal[1] = 3;
			break;
		case 10:
			goal[0] = 1;
			goal[1] = 3;
			break;
		default:
			goal[0] = start[0];
			goal[1] = start[1];
			break;
	}
	
	// set these states as start and goal for the SimpleSetup
	// the third parameter denotes the threshold
	ss.setStartAndGoalStates(start, goal, 0.2);

	bool use_informed = false;
	
	// instatiating requested planner
	if (planner == "RRTstar"){
		og::RRTstar *rrt = new og::RRTstar(ss.getSpaceInformation());
		ob::PlannerPtr plr(rrt);	
		ss.setPlanner(plr);
	}
	else if (planner == "InfRRTstar"){
		og::InformedRRTstar *rrt = new og::InformedRRTstar(ss.getSpaceInformation());
		ob::PlannerPtr plr(rrt);	
		ss.setPlanner(plr);
		use_informed = true;
	}
	else{
		og::RRT *rrt = new og::RRT(ss.getSpaceInformation());
		ob::PlannerPtr plr(rrt);	
		ss.setPlanner(plr);
	}
	
	// setting the optimization objective, if possible
	if (obj == "length") ss.setOptimizationObjective(getPathLengthObjective(ss.getSpaceInformation()));
	else if (obj == "clearance") ss.setOptimizationObjective(getClearanceObjective(ss.getSpaceInformation(), use_informed));
	else if (obj == "multi") ss.setOptimizationObjective(getBalancedObjective(ss.getSpaceInformation(), use_informed));
	
	planWithSimpleSetup(ss, output_path, graph_path, runs);
						
}


int main(int argc, char **argv) {
	
	try
	{
		// parsing args from command line
		std::string path;
        std::string graph_path;
        std::string exp;
        std::string planner;
        std::string obj;
        int runs;

        po::options_description desc("Options");
        desc.add_options()
            ("help,h", "show help message")
            ("outfile", po::value<std::string>(&path)->default_value("solution.txt"), "path of the file to save the solution")
            ("graphfile", po::value<std::string>(&graph_path)->default_value("solution.graphml"), "path to the GraphML representation of the planner data")
			("exp", po::value<std::string>(&exp)->default_value("obstacles"), "type of experiment: {ostacles, maze, real}")
			("planner", po::value<std::string>(&planner)->default_value("RRT"), "type of planner: {RRT, RRTstar, InfRRTstar}")
			("obj", po::value<std::string>(&obj)->default_value(""), "type of optimization objective: {length, clearance, multi}")
            ("runs", po::value<int>(&runs)->default_value(1), "number of experiments to be run");

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if ((vm.count("help") != 0u))
        {
            std::cout << desc << "\n";
            return 1;
        }
		
		// checking args validity
		// - exp
		if (exp != "obstacles" && exp != "maze" && exp != "real"){
			std::cout << "Invalid argument for experiment type (--exp). Please, retry." << std::endl;
			return 1;
		}
		// - planner
		if (planner != "RRT" && planner != "RRTstar" && planner != "InfRRTstar"){
			std::cout << "Invalid argument for planner type (--planner). Please, retry." << std::endl;
			return 1;
		}
		// - obj
		if (obj != "" && obj != "length" && obj != "clearance" && obj != "multi"){
			std::cout << "Invalid argument for optimization objective type (--obj). Please, retry." << std::endl;
			return 1;
		}
		if (planner == "RRT" && obj != ""){
			std::cout << "Invalid use of optimization objective with a non-optimizing planner. Please, retry." << std::endl;
			return 1;
		}
		// - runs
		if (runs < 1){
			std::cout << "Invalid negative or zero value for <<runs>> (--runs). Please, retry." << std::endl;
			return 1;
		}
		
		// let's plan!
		if (exp == "obstacles") planObstacles(path, graph_path, planner, obj, runs);
		else if (exp == "maze") planMaze(path, graph_path, planner, obj, runs);
		else planReal(path, graph_path, planner, obj, runs);
		
	}
	catch(std::exception& e) {
		std::cerr << "error: " << e.what() << "\n";
		return 1;
	}
	
	return 0;
	
}
