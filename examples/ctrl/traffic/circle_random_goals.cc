// travel towards goal, but turn right as far as necessary to not run into anything
// generate goal locations using rejection sampling for points at the outer part of the circle
// takes arguments from when controller is called in world file 

// #include "stage.hh"
#include "/Users/lucyliu/stg/include/Stage-4.3/stage.hh"
#include "/Users/lucyliu/stage4/Stage/examples/ctrl/circles/controller_utils.hh"
#include <cxxopts.hpp>
#include <fstream>
#include <chrono>
using namespace Stg;

static const bool verbose = false;
static const float tol = .3;

struct robot_t : base_robot { // base_robot imported from controller_utils
  ModelFiducial *fiducial;
  int fwd_count;
  double minfrontdist, avoidspeed, turnspeed;
};
typedef struct robot_t robot_t;

int LaserUpdate(Model *mod, robot_t *robot);
int PositionUpdate(Model *mod, robot_t *robot);

// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *args)
{
  robot_t *robot = new robot_t();

  // parse input params from worldfile
  cxxopts::Options options_wf("circle_random_goals", "avoid traffic via circling");
  options_wf.add_options()
    ("s, stopdist", "Stop moving when obstacle detected within this distance", cxxopts::value<double>())
    ("m, minfrontdist", "Slow down & begin avoidant turns when obstacle detected within this distance", cxxopts::value<double>())
    ("a, avoidspeed", "Speed when obstacle distance between stopdist, minfrontdist ", cxxopts::value<double>())
    ("c, cruisespeed", "Speed when no close obstacle", cxxopts::value<double>())
    ("t, turnspeed", "Speed for avoidant turns", cxxopts::value<double>())
    ("n, newgoals", "Keep generating new goals", cxxopts::value<bool>())
    ("u, r_upper", "Upper bound radius for goal generation", cxxopts::value<double>())
    ("l, r_lower", "Lower bound radius for goal generation", cxxopts::value<double>()->default_value("0"))
    // ("mem", "Length of memory", cxxopts::value<int>())
    ("d, data", "Data in string form to append to any data outputs", cxxopts::value<std::string>()->default_value(""))
    ;

  auto result_wf = cast_args(options_wf, &args->worldfile[0]);
  robot->stopdist = result_wf["stopdist"].as<double>();
  robot->minfrontdist = result_wf["minfrontdist"].as<double>();
  robot->avoidspeed = result_wf["avoidspeed"].as<double>();
  robot->cruisespeed = result_wf["cruisespeed"].as<double>();
  robot->turnspeed = result_wf["turnspeed"].as<double>();
  robot->newgoals = result_wf["newgoals"].as<bool>();
  robot->r_lower = result_wf["r_lower"].as<double>();
  robot->r_upper = result_wf["r_upper"].as<double>();
  // robot->memory_length = result_wf["mem"].as<int>();
  robot->addtl_data = result_wf["data"].as<std::string>();

  // parse input params from commandline
  cxxopts::Options options_cl("circle_random_goals", "pass commandline data into controller");
  options_cl.add_options()
    ("o, outfile", "file to save data to", cxxopts::value<std::string>()->default_value(""))
    ("d, data", "Data in string form to append to any data outputs", cxxopts::value<std::string>()->default_value(""))
    ("v, verbose", "Verbose output", cxxopts::value<bool>()->default_value("false")->implicit_value("true"))
    ;

  auto result_cl = cast_args(options_cl, &args->cmdline[0]);

  robot->outfile_name = result_cl["outfile"].as<std::string>();
  robot->addtl_data = robot->addtl_data + result_cl["data"].as<std::string>();
  robot->verbose = result_cl["verbose"].as<bool>();

  robot->pos = dynamic_cast<ModelPosition *>(mod);
  robot->pos->SetColor(Color::RandomColor());
  robot->goals_reached = 0;

  // set up range finder
  ModelRanger *laser = NULL;
  for( int i=1; i<17; i++ )
    {
      char name[32];
      snprintf( name, 32, "ranger:%d", i ); // generate sequence of model names
      if (robot->verbose) {
        printf( "  looking for a suitable ranger at \"%s:%s\" ... \n", robot->pos->Token(), name );
      }      
      laser = dynamic_cast<ModelRanger *>(robot->pos->GetChild( name ));
      
      if( laser && laser->GetSensors()[0].sample_count > 8 ) {
        break;
      }
    }
  
  if( !laser ) {
    PRINT_ERR("  Failed to find a ranger with more than 8 samples. Exit.");
    exit(2);
  }
  robot->laser = laser;
  robot->laser->AddCallback(Model::CB_UPDATE, model_callback_t(LaserUpdate), robot);
  robot->laser->Subscribe();

  gen_start_goal_positions(robot); // generate start and goal positions
  robot->fwd_count = 1;
  robot->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), robot);
  robot->pos->Subscribe(); // starts the position updates


  return 0; // ok
}

// inspect the ranger data and decide what to do
int LaserUpdate(Model *, robot_t *robot)
{
  // get the data
  const std::vector<meters_t> &scan = robot->laser->GetSensors()[0].ranges;
  uint32_t sample_count = scan.size();
  if (sample_count < 1)
    return 0;

  bool obstruction = false;
  robot->stop = false;

  for (uint32_t i = 0; i < sample_count; i++) {
    if (scan[i] < robot->minfrontdist) {
      obstruction = true;
    }
    if (scan[i] < robot->stopdist) {
      robot->stop = true;
    }
  }

  // memory_update(robot);
  if (robot->pos->GetPose().Distance(robot->goal_pos) < tol) {
    goal_updates(robot);
  }

  if (obstruction || robot->stop) {

    robot->pos->SetXSpeed(robot->stop ? 0.0 : robot->avoidspeed);
    robot->pos->SetTurnSpeed(robot->turnspeed); // turn right until no longer obstructed
    robot->fwd_count = 1; // once robot is no longer obstructed, it should move forward at least one step before attempting to turn back to goal

  } else {
    robot->pos->SetXSpeed(robot->cruisespeed);
    if (robot->fwd_count <= 0) {
        Pose goal_pos = robot->goal_pos;
        double x_error = goal_pos.x - robot->pos->GetPose().x;
        double y_error = goal_pos.y - robot->pos->GetPose().y;
        double goal_angle = atan2(y_error, x_error);
        double a_error = normalize(goal_angle - robot->pos->GetPose().a);
        robot->pos->SetTurnSpeed(a_error);
    } else {
        robot->fwd_count = robot->fwd_count - 1;
    }
  }
  return 0; // run again
}

int PositionUpdate(Model *, robot_t *robot)
{
  Pose pose = robot->pos->GetPose();
  return 0; // run again
}
