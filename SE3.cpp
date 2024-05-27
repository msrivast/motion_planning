 #include <omplapp/apps/SE3RigidBodyPlanning.h>
 #include <omplapp/config.h>
 #include <fstream>
 #include <ompl/geometric/planners/rrt/RRT.h>
 #include <ompl/geometric/planners/kpiece/KPIECE1.h>
 #include <time.h>
  
 using namespace ompl;

 base::StateValidityCheckerPtr myFclSvc;

  bool isStateValid(const base::State *state)
 {
    return (myFclSvc->isValid(state) || myFclSvc->clearance(state)>-0.01);
    // return myFclSvc->isValid(state);
    // return true;
 }

 int main()
 {
     // plan in SE3
     app::SE3RigidBodyPlanning setup;

  
     // load the robot and the environment
        // std::string robot_fname = std::string("cuboid_robot_coarse.STL"); // robot has breadth 0.18
     std::string robot_fname = std::string("cuboid_robot_tight_coarse.STL"); // robot has breadth 0.2
     std::string env_fname = std::string("env_coarse.dae"); // environment has gap 0.2
     setup.setRobotMesh(robot_fname);
     setup.setEnvironmentMesh(env_fname); 
  
     // define start state
     base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
     start->setX(0.5);
     start->setY(0.5);
     start->setZ(0.0);
     start->rotation().setIdentity();
     

     // define goal state
     base::ScopedState<base::SE3StateSpace> goal(start);
     goal->setX(0.0);
     goal->setY(0.0);
     goal->setZ(0.0);
     goal->rotation().setAxisAngle(1.0,0.0,0.0,3.14/2.0);
  
     // set the start & goal states
     setup.setStartAndGoalStates(start, goal);
    
     // setting collision checking resolution to 1% of the space extent
     setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

     setup.setPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
    //  setup.setPlanner(std::make_shared<geometric::KPIECE1>(setup.getSpaceInformation()));
    

     // Calling setup before replacing the FCL planner is essential!
     setup.setup();
     myFclSvc = setup.getFCLStateValidityChecker(); // Look at changes in AppBase.h
    auto si = setup.getSpaceInformation();
    si->setStateValidityChecker(isStateValid);

    // std::string s = myFclSvc->isValid(start.get()) ? "true" : "false";
    // std::cout<< s << "\n";
    // std::cout << myFclSvc->clearance(start.get())<<"\n"; // clearance() can only be called when a collision has definitely occured.
                                                            // Look at changes made in to distanceRequest and distanceResult constructors in FCLMethodWrapper.h
  
     
     clock_t startTime = clock();
     // try to solve the problem
     if (setup.solve(10))
     {
         // simplify & print the solution
         setup.simplifySolution();
         clock_t stopTime = clock();
         setup.getSolutionPath().printAsMatrix(std::cout);
         std::cout << "Time: " << (stopTime - startTime) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
         std::ofstream output("path_3d.txt");
         setup.getSolutionPath().printAsMatrix(output);
     }
  
     return 0;
 }