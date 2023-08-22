// robot travels straight ahead

#include "../../../libstage/stage.hh"
#include "controller_utils.hh"
#include <cxxopts.hpp>
using namespace Stg;

static const double cruisespeed = 0.4;

typedef struct robot_t : base_robot{
  ModelPosition *pos;
  ModelRanger *laser;
//   int avoidcount, randcount;
  double r_lower, r_upper, stopdist;
  bool periodic;
} robot_t;
typedef struct robot_t robot_t;

int LaserUpdate(Model *mod, robot_t *robot);
int PositionUpdate(Model *mod, robot_t *robot);

// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *args)
{
  robot_t *robot = new robot_t();

    // parse input params from worldfile
  cxxopts::Options options_wf("test_periodic_raytrace", "robot that just goes straight");
  options_wf.add_options()
    ("s, stopdist", "Stop moving when obstacle detected within this distance", cxxopts::value<double>())
    // ("c, cruisespeed", "Speed when no close obstacle", cxxopts::value<double>())
    // ("a, anglenoise", "Standard deviation of noise added to angles", cxxopts::value<double>()->default_value("0"))
    // ("b, bias", "Bias of noise added to angles", cxxopts::value<double>()->default_value("0"))
    // ("n, newgoals", "Keep generating new goals", cxxopts::value<bool>()) // implicit value is true
    // ("circle", "Generate goals in circle instead of square", cxxopts::value<bool>()) // implicit value is true
    ("u, r_upper", "Upper bound radius for goal generation (or s/2 for square)", cxxopts::value<double>())
    ("l, r_lower", "Lower bound radius for goal generation", cxxopts::value<double>()->default_value("0"))
    // ("r, runsteps", "Total steps in a run phase", cxxopts::value<int>())
    // ("t, tumblesteps", "Total steps in a tumble phase", cxxopts::value<int>())
    // ("mem", "Length of memory stored", cxxopts::value<int>())
    // ("d, data", "Data in string form to append to any data outputs", cxxopts::value<std::string>()->default_value(""))
    ;

  auto result_wf = cast_args(options_wf, &args->worldfile[0]);
  robot->stopdist = result_wf["stopdist"].as<double>();
//   robot->cruisespeed = result_wf["cruisespeed"].as<double>();
//   robot->anglebias = result_wf["bias"].as<double>();
//   robot->newgoals = result_wf["newgoals"].as<bool>();
//   // robot->periodic = result_wf["periodic"].as<bool>();
//   robot->circle = result_wf["circle"].as<bool>();
  robot->r_lower = result_wf["r_lower"].as<double>();
  robot->r_upper = result_wf["r_upper"].as<double>();
//   robot->runsteps = result_wf["runsteps"].as<int>();
//   robot->tumblesteps = result_wf["tumblesteps"].as<int>();
//   robot->anglenoise = result_wf["anglenoise"].as<double>();
//   robot->addtl_data = result_wf["data"].as<std::string>();
//   robot->memory_length = result_wf["mem"].as<int>();

  robot->periodic = mod->GetWorld()->IsPeriodic();

  robot->pos = dynamic_cast<ModelPosition *>(mod);
  
  robot->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), robot);
  robot->pos->Subscribe(); // starts the position updates

  ModelRanger *laser = NULL;

  printf( "\nWander ctrl for robot %s:\n",  robot->pos->Token() );
  for( int i=0; i<16; i++ )
    {
      char name[32];
      snprintf( name, 32, "ranger:%d", i ); // generate sequence of model names

      printf( "  looking for a suitable ranger at \"%s:%s\" ... ", robot->pos->Token(), name );      
      laser = dynamic_cast<ModelRanger *>(robot->pos->GetChild( name ));
      
      if( laser && laser->GetSensors()[0].sample_count > 8 )
      {
        puts( "yes." );
        break;
      }

      puts( "no." );
    }
  
  if( !laser ) {
    PRINT_ERR("  Failed to find a ranger with more than 8 samples. Exit.");
    exit(2);
  }
  
  robot->laser = laser;
  robot->laser->AddCallback(Model::CB_UPDATE, model_callback_t(LaserUpdate), robot);
  robot->laser->Subscribe(); // starts the ranger updates

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

    bool stop = false;
    for (uint32_t i = 0; i < sample_count; i++) {
    if (scan[i] < robot->stopdist) {
      stop = true;
      }
    }

    robot->pos->SetXSpeed(stop ? 0 : cruisespeed);
    robot->pos->SetTurnSpeed(0);

    if (robot->periodic) {
    double s = 2 * robot->r_upper;
    Pose cur_pos = robot->pos->GetPose();
    if (cur_pos.x < -s/2 || cur_pos.x > s/2 || cur_pos.y < -s/2 || cur_pos.y > s/2) { // if out of bounds
        double x = fmod(cur_pos.x + s/2, s) - s/2;
        double y = fmod(cur_pos.y + s/2, s) - s/2;
        robot->pos->SetGlobalPose(Pose(x > -s/2 ? x : x + s, y > -s/2 ? y : y + s, cur_pos.z, cur_pos.a));
    }
  }

  return 0; // run again
}

int PositionUpdate(Model *, robot_t *robot)
{
  Pose pose = robot->pos->GetPose();

//   printf("Pose: [%.2f %.2f %.2f %.2f]\n", pose.x, pose.y, pose.z, pose.a);

  return 0; // run again
}
