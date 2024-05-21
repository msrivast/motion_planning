   /* Author: Andreas Orthey */
  
 #include <ompl/base/spaces/SE2StateSpace.h>
 #include <ompl/base/spaces/RealVectorStateSpace.h>
 #include <ompl/base/SpaceInformation.h>
 #include <ompl/base/StateSpace.h>
 #include <ompl/multilevel/planners/qrrt/QRRT.h>
 #include <iostream>
 #include <boost/math/constants/constants.hpp>
 #include <fstream>
 #include <ompl/geometric/PathGeometric.h>
 #include <time.h>
 using namespace ompl::base;
  
 // Path Planning on fiber bundle SE2 \rightarrow R2
  
 bool boxConstraint(const double values[])
 {
    double x = values[0];
    double y = values[1]; 
    if(x<0.3 && x>-0.1 && abs(y)<0.5 && abs(y)>0.05) return false;
    if(x<0.9 && x>0.6 && abs(y)<0.1) return false;
    return true;
 }

 bool isStateValid_SE2(const State *state)
 {
     const auto *SE2state = state->as<SE2StateSpace::StateType>();
     const auto *R2 = SE2state->as<RealVectorStateSpace::StateType>(0);
     const auto *SO2 = SE2state->as<SO2StateSpace::StateType>(1);

     return boxConstraint(R2->values) && (SO2->value < boost::math::constants::pi<double>() / 2.0);
 }
 bool isStateValid_R2(const State *state)
 {
     const auto *R2 = state->as<RealVectorStateSpace::StateType>();
     return boxConstraint(R2->values);
 }
  
 int main()
 {  
    
     // Setup SE2
     auto SE2(std::make_shared<SE2StateSpace>());
     RealVectorBounds bounds(2);
     bounds.setLow(0, -0.1);
     bounds.setHigh(0,1);
     bounds.setLow(1,-1);
     bounds.setHigh(1,1);
     SE2->setBounds(bounds);
     SpaceInformationPtr si_SE2(std::make_shared<SpaceInformation>(SE2));
     si_SE2->setStateValidityChecker(isStateValid_SE2);
  
     // Setup Quotient-Space R2
     auto R2(std::make_shared<RealVectorStateSpace>(2));
     R2->setBounds(bounds);
     SpaceInformationPtr si_R2(std::make_shared<SpaceInformation>(R2));
     si_R2->setStateValidityChecker(isStateValid_R2);
  
     // Create vector of spaceinformationptr
     std::vector<SpaceInformationPtr> si_vec;
     si_vec.push_back(si_R2);
     si_vec.push_back(si_SE2);
  
     // Define Planning Problem
     using SE2State = ScopedState<SE2StateSpace>;
     SE2State start_SE2(SE2);
     SE2State goal_SE2(SE2);
     start_SE2->setXY(0, 0);
     start_SE2->setYaw(0);
     goal_SE2->setXY(0.9, -0.5);
     goal_SE2->setYaw(boost::math::constants::pi<double>() / 4.0);
  
     ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(si_SE2);
     pdef->setStartAndGoalStates(start_SE2, goal_SE2);
  
     // Setup Planner using vector of spaceinformationptr
     auto planner = std::make_shared<ompl::multilevel::QRRT>(si_vec);
  
     // Planner can be used as any other OMPL algorithm
     planner->setProblemDefinition(pdef);
     planner->setup();
    clock_t start = clock();
     PlannerStatus solved = planner->Planner::solve(1.0);
    clock_t stop = clock();
     if (solved)
     {
         std::cout << std::string(80, '-') << std::endl;
         std::cout << "Bundle Space Path (SE2):" << std::endl;
         std::cout << std::string(80, '-') << std::endl;
         pdef->getSolutionPath()->print(std::cout);
        //  std::ofstream output("path_QRRT.txt");
        //  pdef->getSolutionPath()->print(output);


  
         std::cout << std::string(80, '-') << std::endl;
         std::cout << "Base Space Path (R2)   :" << std::endl;
         std::cout << std::string(80, '-') << std::endl;
         const ProblemDefinitionPtr pdefR2 = planner->getProblemDefinition(0);
         pdefR2->getSolutionPath()->print(std::cout);
         std::cout << std::string(80, '-') << std::endl;
         std::cout << "Time: " << (stop - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
     }
     return 0;
 }