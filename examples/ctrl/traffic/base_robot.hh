#include "/Users/lucyliu/stg/include/Stage-4.3/stage.hh"
#include <fstream>
using namespace Stg;


struct base_robot {
  ModelPosition *pos;
  ModelRanger *laser;
  ModelFiducial *fiducial;
  int goals_reached;
  Pose start_pos, goal_pos;
  bool stop;
  double stopdist, cruisespeed, r_lower, r_upper;
  // bool newgoals; // if yes, keep generating new goals for robot. else, move b&f between initial start and goal
  std::string outfile_name, addtl_data = ""; 
  std::ofstream outfile;
  bool verbose, newgoals, periodic, circle;
  uint64_t memory_interval = 1000000;
  ModelFiducial::Fiducial *closest = NULL;
};
typedef struct base_robot base_robot;