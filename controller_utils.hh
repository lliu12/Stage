#include "/Users/lucyliu/stg/include/Stage-4.3/stage.hh"
#include <cxxopts.hpp>
#include <random>
#include <chrono>
#include <fstream>
using namespace Stg;


struct base_robot {
  ModelPosition *pos;
  ModelRanger *laser;
  int goals_reached;
  Pose start_pos, goal_pos;
  bool stop;
  double stopdist, cruisespeed, r_lower, r_upper;
  // bool newgoals; // if yes, keep generating new goals for robot. else, move b&f between initial start and goal
  std::string outfile_name, addtl_data; 
  std::ofstream outfile;
  bool verbose, newgoals, periodic, circle;
  bool already_initialized = false;
  std::vector<bool> blocked_memory;
  unsigned int memory_index, memory_length, blocked_count;
  uint64_t memory_interval = 1000000;
};
typedef struct base_robot base_robot;

// Use rejection sampling to get a random point in a the ring between radius r_lower and r_upper (center at origin)
Pose random_goal(double r_lower, double r_upper, base_robot *robot) {

    bool done = 0;
    double rand_x;
    double rand_y;

    while (!done) {
        rand_x = r_upper * 2 * (drand48() - .5);
        rand_y = r_upper * 2 * (drand48() - .5);
        double dist = Pose(rand_x, rand_y, 0, 0).Distance(Pose(0,0,0,0));
        if (!robot->circle || (dist < r_upper && dist > r_lower)) { done = 1; }
    }
    if (robot->verbose) {
      printf("Random Pose generated for %s is [%.2f %.2f] \n", robot->pos->Token(), rand_x, rand_y);
    }    
    return Pose(rand_x, rand_y, 0, 0);
}

cxxopts::ParseResult cast_args(cxxopts::Options options, char * string_start) {
  enum { kMaxArgs = 64 };
  int argc = 0;
  char *argv[kMaxArgs];
  char *p2 = std::strtok(string_start, " ");
  while (p2 && argc < kMaxArgs-1)
    {
      argv[argc++] = p2;
      p2 = strtok(0, " ");
    }
  argv[argc] = 0;
  return options.parse(argc, argv); 
}

// initialize robot's start and goal positions
void gen_start_goal_positions(base_robot *robot) {

  if (robot->verbose) {
    printf("\nGenerating start and goal for robot %s... \n", robot->pos->Token());
  }

  robot->pos->SetGlobalPose(random_goal(robot->r_lower, robot->r_upper, robot)); // set start
  robot->start_pos = robot->pos->GetPose();
  robot->pos->AdjustPoseToFreeSpace(robot->stopdist, 10);

  robot->goal_pos = random_goal(robot->r_lower, robot->r_upper, robot); // set goal
}

// set up vectors for storing waypoint history, memory
void gen_robot_data(base_robot *robot) {
  std::vector<ModelPosition::Waypoint> wps;
  wps.push_back(ModelPosition::Waypoint(robot->goal_pos, robot->pos->GetColor()));
  robot->pos->waypoints = wps;

  robot->memory_index = 0;
  std::vector<bool> mem_vec(robot->memory_length, 0);
  robot->blocked_memory = mem_vec;
  robot->blocked_count = 0;
}

// update necessary variables and output data when a robot reaches goal 
void goal_updates(base_robot *robot) {
    if (robot->outfile_name != "NULL" && !robot->outfile_name.empty()) {
        robot->outfile.open(robot->outfile_name, std::ios_base::app);
        robot->outfile << (std::string("goal") + std::string(",") + std::to_string(robot->pos->GetWorld()->SimTimeNow()) + std::string(",") + std::string(robot->pos->Token()) + std::string(",") + robot->addtl_data) << std::endl;
        robot->outfile.close();
    }

    if (robot->newgoals) {
        robot->goal_pos = random_goal(robot->r_lower, robot->r_upper, robot);
    }
    else { 
        std::swap(robot->goal_pos, robot->start_pos); 
    }

    std::vector<ModelPosition::Waypoint> wps;
    wps.push_back(ModelPosition::Waypoint(robot->goal_pos, robot->pos->GetColor()));
    robot->pos->waypoints = wps;
    robot->goals_reached++;
}

/** If memory_interval has passed since last update, record whether robot is currently blocked. */
void memory_update(base_robot *robot) {
  if (robot->memory_length > 0 && robot->pos->GetWorld()->SimTimeNow() % robot->memory_interval == 0) {
    robot->blocked_count -= int(robot->blocked_memory[robot->memory_index]);
    robot->blocked_count += int(robot->stop);
    robot->blocked_memory[robot->memory_index] = robot->stop;
    robot->memory_index++; // increment index of current location in memory vector
    robot->memory_index %= robot->memory_length;

    if (robot->verbose) {
      printf("Memory updated for %s at %llu seconds to %u / %u blocked. \n", robot->pos->Token(), robot->pos->GetWorld()->SimTimeNow() / 1000000, robot->blocked_count, robot->memory_length);
    }
  }
}

// Return true if robot position is within stopdist of boundary 
// (Position can be outside of boundary bounds)
bool calc_near_boundary(base_robot *robot) {
  double s = 2 * robot->r_upper;
  Pose cur_pos = robot->pos->GetPose();
  // return (cur_pos.x < -s/2 + robot->stopdist || cur_pos.x > s/2 - robot->stopdist || cur_pos.y < -s/2 + robot->stopdist  || cur_pos.y > s/2 - robot->stopdist);
  return (abs(cur_pos.x - (-s/2)) < robot->stopdist || abs(cur_pos.x - s/2) < robot->stopdist || abs(cur_pos.y - (-s/2)) < robot->stopdist  || abs(cur_pos.y - s/2) < robot->stopdist);

}
