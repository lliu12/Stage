// travel towards goal noisily, stop moving forward to avoid collisions

// #include "stage.hh"
#include "/Users/lucyliu/stg/include/Stage-4.3/stage.hh"
#include "/Users/lucyliu/stage4/Stage/examples/ctrl/circles/controller_utils.hh"
#include <cxxopts.hpp>
#include <random>
#include <chrono>
#include <fstream>
// #include <cstdlib> 

using namespace Stg;

static const bool verbose = false;
static const float tol = .3;

struct robot_t : base_robot{ // base_robot imported from controller_utils
  bool running; // running or tumbling
  bool near_boundary; // is robot within stopdist of boundary
  int current_phase_count, runsteps, tumblesteps; // time so far spent running or tumbling, total length of a run or tumble period
  double anglenoise, goal_angle, anglebias;
};
typedef struct robot_t robot_t;

int LaserUpdate(Model *mod, robot_t *robot);
int PositionUpdate(Model *mod, robot_t *robot);
int Reset(Model *mod, robot_t *robot);

// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *args)
{
  robot_t *robot = new robot_t();

  // if (robot->already_initialized) { // if robot has been initialized before, reset everything to starting values (with new random locations, etc.)
  //   // set values back to starting values
  //   robot->goals_reached = 0;
  //   robot->running = false;
  //   robot->current_phase_count = 0;
  //   robot->memory_index = 0;

  //   // get new start goal locations 
  //   gen_start_goal_positions(robot);

  //   // wipe waypoints and memory vectors, then regenerate to new ones
  //   std::vector<ModelPosition::Waypoint>().swap(robot->pos->waypoints);
  //   std::vector<bool>().swap(robot->blocked_memory);
  //   gen_robot_data(robot);
  // }

  // else { // otherwise if robot is new, set up from scratch
  robot->already_initialized = true;
  // parse input params from worldfile
  cxxopts::Options options_wf("noise_random_goals", "traffic avoidance with variable noise");
  options_wf.add_options()
    ("s, stopdist", "Stop moving when obstacle detected within this distance", cxxopts::value<double>())
    ("c, cruisespeed", "Speed when no close obstacle", cxxopts::value<double>())
    ("a, anglenoise", "Standard deviation of noise added to angles", cxxopts::value<double>()->default_value("0"))
    ("b, bias", "Bias of noise added to angles", cxxopts::value<double>()->default_value("0"))
    ("n, newgoals", "Keep generating new goals", cxxopts::value<bool>()) // implicit value is true
    ("p, periodic", "Periodic boundary conditions?", cxxopts::value<bool>()) // implicit value is true
    ("circle", "Generate goals in circle instead of square", cxxopts::value<bool>()) // implicit value is true
    ("u, r_upper", "Upper bound radius for goal generation (or s/2 for square)", cxxopts::value<double>())
    ("l, r_lower", "Lower bound radius for goal generation", cxxopts::value<double>()->default_value("0"))
    ("r, runsteps", "Total steps in a run phase", cxxopts::value<int>())
    ("t, tumblesteps", "Total steps in a tumble phase", cxxopts::value<int>())
    ("mem", "Length of memory stored", cxxopts::value<int>())
    ("d, data", "Data in string form to append to any data outputs", cxxopts::value<std::string>()->default_value(""))
    ;

  auto result_wf = cast_args(options_wf, &args->worldfile[0]);
  robot->stopdist = result_wf["stopdist"].as<double>();
  robot->cruisespeed = result_wf["cruisespeed"].as<double>();
  robot->anglebias = result_wf["bias"].as<double>();
  robot->newgoals = result_wf["newgoals"].as<bool>();
  robot->periodic = result_wf["periodic"].as<bool>();
  robot->circle = result_wf["circle"].as<bool>();
  robot->r_lower = result_wf["r_lower"].as<double>();
  robot->r_upper = result_wf["r_upper"].as<double>();
  robot->runsteps = result_wf["runsteps"].as<int>();
  robot->tumblesteps = result_wf["tumblesteps"].as<int>();
  robot->anglenoise = result_wf["anglenoise"].as<double>();
  robot->addtl_data = result_wf["data"].as<std::string>();
  robot->memory_length = result_wf["mem"].as<int>();

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
  robot->running = false;
  robot->current_phase_count = 0;
  double s = 2 * robot->r_upper;
  Pose cur_pos = robot->pos->GetPose();
  robot->near_boundary = calc_near_boundary(robot);

  // set up range finder
  ModelRanger *laser = NULL;
  for( int i=0; i<16; i++ )
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
  gen_robot_data(robot); // set up vectors for storing waypoint history, memory
  
  robot->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), robot);
  robot->pos->AddCallback(Model::CB_RESET, model_callback_t(Reset), robot);
  robot->pos->Subscribe(); // starts the position updates
  // }

  return 0; // ok
}

int Reset(Model *, robot_t *robot) {
  printf("reset function in controller called \n");
  printf("pos b4 reset: [ %.4f %.4f %.4f ]\n", robot->pos->GetPose().x, robot->pos->GetPose().y, robot->pos->GetPose().a);
  robot->goals_reached = 0;
  robot->running = false;
  robot->current_phase_count = 0;
  robot->memory_index = 0;

  // get new start goal locations 
  gen_start_goal_positions(robot);

  // wipe waypoints and memory vectors, then regenerate to new ones
  std::vector<ModelPosition::Waypoint>().swap(robot->pos->waypoints);
  std::vector<bool>().swap(robot->blocked_memory);
  gen_robot_data(robot);

  printf("pos after reset: [ %.4f %.4f %.4f ]\n", robot->pos->GetPose().x, robot->pos->GetPose().y, robot->pos->GetPose().a);
  return 0;
}

// inspect the ranger data and decide what to do
int LaserUpdate(Model *, robot_t *robot)
{
  // get the data
  const std::vector<meters_t> &scan = robot->laser->GetSensors()[0].ranges;
  uint32_t sample_count = scan.size();
  if (sample_count < 1)
    return 0;

  robot->stop = false;

  for (uint32_t i = 0; i < sample_count; i++) {
  if (scan[i] < robot->stopdist) {
    robot->stop = true;
  }
  }

  double s = 2 * robot->r_upper;
  Pose cur_pos = robot->pos->GetPose();

  // // extra scans if robot locations are periodic to sense across boundaries
  // if (!robot->stop && robot->periodic && robot->near_boundary) {
  //   double xs [9] = {-s, -s, -s, 0, 0, s, s, s};
  //   double ys [9] = {-s, 0, s, -s, s, -s, 0, s};
  //   for ( int i=0; i<8; i++ ) {
  //     robot->pos->SetGlobalPose(cur_pos + Pose(xs[i], ys[i], 0, 0));
  //     if (calc_near_boundary(robot)) {
  //       // printf("I am here 2 \n");
  //       robot->laser->Update();
  //       const std::vector<meters_t> &scan = robot->laser->GetSensors()[0].ranges;
  //       uint32_t sample_count = scan.size();
  //       for (uint32_t i = 0; i < sample_count; i++) {
  //         if (scan[i] < robot->stopdist) {
  //           printf("I am here 3 \n");
  //           robot->stop = true;
  //         }
  //       }  
  //     }
  //   }
  //   robot->pos->SetGlobalPose(cur_pos);
  // }

  memory_update(robot);

  // updates for if the goal has been reached
  if (robot->pos->GetPose().Distance(robot->goal_pos) < tol) {
    goal_updates(robot);
  }
    
  // check if current run or tumble phase is over
  if (robot->current_phase_count >= (robot->running ? robot->runsteps : robot->tumblesteps)) {
    robot->running = !robot->running;
    robot->current_phase_count = 0;
  }

  if (!robot->running && robot->current_phase_count == 0) {
    // set goal_angle
    Pose goal_pos;
    if (!robot->periodic) {
      goal_pos = robot->goal_pos;
    }
    else {
      double s = 2 * robot->r_upper;
      double xs [9] = {-s, -s, -s, 0, 0, 0, s, s, s};
      double ys [9] = {-s, 0, s, -s, 0, s, -s, 0, s};
      int closest_pos = 0;
      double closest_dist = std::numeric_limits<double>::infinity();
      for ( int i=0; i<9; i++ ) {
        Pose diff = Pose(xs[i], ys[i], 0, 0);
        Pose test_pos = robot->goal_pos + diff;
        double dist = robot->pos->GetPose().Distance(test_pos);
        if (dist < closest_dist) {
          closest_dist = dist;
          closest_pos = i;
        }
      }

      goal_pos = Pose(robot->goal_pos.x + xs[closest_pos], robot->goal_pos.y + ys[closest_pos], 0, 0);
    }
    
    double x_error = goal_pos.x - robot->pos->GetPose().x;
    double y_error = goal_pos.y - robot->pos->GetPose().y;
    double goal_angle = atan2(y_error, x_error);

    // std::random_device rd{};
    // std::mt19937 gen{rd()};
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    // std::normal_distribution<double> distribution(robot->anglebias, robot->anglenoise);
    float frustration = (robot->memory_length == 0 ? 1 : float(robot->blocked_count) / float(robot->memory_length));
    std::normal_distribution<double> distribution(robot->anglebias * frustration, robot->anglenoise * frustration);
    robot->goal_angle = normalize(goal_angle += distribution(generator));
    // printf("%d \n", robot->goal_angle);
  }
  
  if (robot->running) {
    robot->pos->SetXSpeed(robot->stop ? 0 : robot->cruisespeed);
    robot->pos->SetTurnSpeed(0);
  }
  else {
    double a_error = normalize(robot->goal_angle - robot->pos->GetPose().a);
    robot->pos->SetTurnSpeed(a_error);
    robot->pos->SetXSpeed(0);
  }

  robot->near_boundary = calc_near_boundary(robot);
  if (robot->periodic && (cur_pos.x < -s/2 || cur_pos.x > s/2 || cur_pos.y < -s/2 || cur_pos.y > s/2)) {
    double x = fmod(cur_pos.x + s/2, s) - s/2;
    double y = fmod(cur_pos.y + s/2, s) - s/2;
    robot->pos->SetGlobalPose(Pose(x > -s/2 ? x : x + s, y > -s/2 ? y : y + s, cur_pos.z, cur_pos.a));
  }

  robot->current_phase_count++;
  return 0; // run again
}

int PositionUpdate(Model *, robot_t *robot)
{
  Pose pose = robot->pos->GetPose();
  return 0; // run again
}
