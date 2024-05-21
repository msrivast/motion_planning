 #include <ompl/base/SpaceInformation.h>
 #include <ompl/base/spaces/SE2StateSpace.h>
 #include <ompl/geometric/planners/rrt/RRT.h>
 #include <ompl/geometric/SimpleSetup.h>
  
 #include <ompl/config.h>
 #include <iostream>
 #include <fstream>
 #include <time.h>
  
 namespace ob = ompl::base;
 namespace og = ompl::geometric;
  
 bool isStateValid(const ob::State *state)
 {
     // cast the abstract state type to the type we expect
     const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
  
     // extract the first component of the state and cast it to what we expect
     const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
  
     // extract the second component of the state and cast it to what we expect
     const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);
  
     // check validity of state defined by pos & rot
     double x = pos->values[0];
     double y = pos->values[1]; 
    // if (pos->values[0]>0 && pos->values[1]>0) return false;
    if(x<0.3 && x>-0.1 && abs(y)<0.5 && abs(y)>0.05) return false;
    if(x<0.9 && x>0.6 && abs(y)<0.1) return false;
    if (abs(rot->value) > 3.14 / 2.0)  return false;
     // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
     return (const void*)rot != (const void*)pos;
 }
  clock_t start, stop;
  
 void planWithSimpleSetup()
 {
     // construct the state space we are planning in
     auto space(std::make_shared<ob::SE2StateSpace>());
  
     // set the bounds for the R^3 part of SE(3)
     ob::RealVectorBounds bounds(2);
     bounds.setLow(0, -0.1);
     bounds.setHigh(0,1);
     bounds.setLow(1,-1);
     bounds.setHigh(1,1);
    
    //  bounds.setLow(-1);
    //  bounds.setHigh(1);
  
     space->setBounds(bounds);
  
     // define a simple setup class
     og::SimpleSetup ss(space);
  
     // set state validity checking for this space
     ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
  
     // create a random start state
    //  ob::ScopedState<> start(space);
    //  start.random();
    ob::ScopedState<ob::SE2StateSpace> start(ss.getSpaceInformation());
    start->setX(0.9);
    start->setY(-0.5);
    start->setYaw(3.14/4.0);

  
     // create a random goal state
    //  ob::ScopedState<> goal(space);
    //  goal.random();
    ob::ScopedState<ob::SE2StateSpace> goal(ss.getSpaceInformation());
    goal->setX(0.0);
    goal->setY(0.0);
    goal->setYaw(0);

  
     // set the start and goal states
    //  ss.setStartAndGoalStates(start, goal);
     ss.setStartAndGoalStates(goal, start);
  
     ss.setPlanner(std::make_shared<og::RRT>(ss.getSpaceInformation()));
     
     // this call is optional, but we put it in to get more output information
     ss.setup();
     ss.print();
    
 
     // attempt to solve the problem within one second of planning time
     start = clock();
     ob::PlannerStatus solved = ss.solve(1.0);
     stop = clock();
  
     if (solved)
     {
         std::cout << "Found solution:" << std::endl;
         // print the path to screen
        //  ss.simplifySolution();
        //  ss.getSolutionPath().print(std::cout);
         ss.getSolutionPath().printAsMatrix(std::cout);
        //  std::ofstream output("path_RRT.txt");
        //  ss.getSolutionPath().printAsMatrix(output);
         
         
     }
     else
         std::cout << "No solution found" << std::endl;
 }
  
 int main(int /*argc*/, char ** /*argv*/)
 {
     std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
  
     std::cout << std::endl << std::endl;
  
     planWithSimpleSetup();

     std::cout << "Time: " << (stop - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  
     return 0;
 }
