#include "Algorithm.h"
#include <iostream>

#include "AGVUnit.h"
#include "CNCUnit.h"
#include "LoadingTaskUnit.h"
#include "BasicPhysicalEngine.h"
#include "BasicDecisionMaking.h"

using namespace std;

bool InPath(const int &row, const int &col, const std::list<Node *> &path) {
  for (const auto &p : path) {
    if (row == p->robot_config_.x && col == p->robot_config_.y) {
      return true;
    }
  }
  return false;
}

int main() 
{

    //
    // AGV unit test
    //
    AbstractRobotUnit *agv1 = new AGVUnit();
    agv1->robotName = "agv1";
    agv1->currentLocationX = 1;
    agv1->currentLocationY = 1;
    agv1->showAllParameters();

    AbstractRobotUnit *agv2 = new AGVUnit();
    agv2->robotName = "agv2";
    agv2->currentLocationX = 2;
    agv2->currentLocationY = 1;
    agv2->showAllParameters();

    AbstractRobotUnit *agv3 = new AGVUnit();
    agv3->robotName = "agv3";
    agv3->currentLocationX = 3;
    agv3->currentLocationY = 3;
    agv3->showAllParameters();

    //
    // CNC unit test
    //
    AbstractTargetUnit *cnc = new CNCUnit();
    //cnc->test();
    cnc->currentLocationX = 1;
    cnc->currentLocationY = 10;
    cnc->showAllParameters();

    AbstractTargetUnit *cnc2 = new CNCUnit();
    cnc2->currentLocationX = 6;
    cnc2->currentLocationY = 1;
    cnc2->showAllParameters();

    AbstractTargetUnit *cnc3 = new CNCUnit();
    cnc3->currentLocationX = 6;
    cnc3->currentLocationY = 5;
    cnc3->showAllParameters();

    //
    // task test 
    //
    // add the instance of the robot and target
    AbstractTaskUnit *loadingTask2 = new LoadingTaskUnit();
    loadingTask2->addARobot(agv1);
    loadingTask2->addARobot(agv2);
    loadingTask2->addARobot(agv3);
    loadingTask2->showAllParameters();

    loadingTask2->addATarget(cnc);
    loadingTask2->addATarget(cnc2);
    loadingTask2->addATarget(cnc3);
    loadingTask2->showAllParameters();

    cout<<"!!!!!!!!!!!!!!!!!!!!"<<endl;
    cout<<"!!!!!!!!!!!!!!!!!!!!"<<endl;
    cout<<"!!!!!!!!!!!!!!!!!!!!"<<endl;
    cout<<"!!!!!!!!!!!!!!!!!!!!"<<endl;
    cout<<"!!!!!!!!!!!!!!!!!!!!"<<endl;

//    //set start node and end node for A*
//    RobotConfiguration start_config_;
//    start_config_.x = agv1->currentLocationX;
//    start_config_.y = agv1->currentLocationY;
//
//    Priority start_pri_;
//    start_pri_.g = 0;
//    start_pri_.h = 0;
//    start_pri_.f = 0;
//
//    Node start_node = Node(start_config_, start_pri_);
//
//    RobotConfiguration end_config_;
//    end_config_.x = cnc->currentLocationX;
//    end_config_.y = cnc->currentLocationY;
//
//    Priority end_pri_;
//    end_pri_.g = 0;
//    end_pri_.h = 0;
//    end_pri_.f = 0;
//
//    Node end_node = Node(end_config_, end_pri_);
//
//    // A*
//    std::list<Node *> path = astar.get_path(start_node, end_node, false);
//    // print path found
//    for (auto &p : path) {
//      std::cout << "(" << p->robot_config_.x << "," << p->robot_config_.y << ") ";
//    }
//    std::cout << "\n";

	//
	// PhysicalEngine
	//
	double timeStart = 0;
    double timeEnd   = 23;
	double timeWidth = 1;
	BasicPhysicalEngine PE(timeStart, timeEnd, timeWidth);
    PE.setATaskUnit(loadingTask2);

    // generate and set the map
    std::vector<std::vector<int>> map = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                         {1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1},
                                         {1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1},
                                         {1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1},
                                         {1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 1},
                                         {1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1},
                                         {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1},
                                         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
    PE.setTheMap(map);

    PE.startSimulation();


  return 0;
}
