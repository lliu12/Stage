// initial attempt at implementing flocking

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
  int current_phase_count, avg_runsteps, runsteps, tumblesteps; // time so far spent running or tumbling, total length of a run or tumble period
  double anglenoise, goal_angle, anglebias;
  std::vector<bool> blocked_memory;
  unsigned int memory_index, memory_length, blocked_count;
  bool random_runsteps;
  ModelFiducial *fiducial;
  ModelFiducial::Fiducial *closest;
  radians_t closest_bearing;
  meters_t closest_range;
  radians_t closest_heading_error;
  double flockstrength;

};
typedef struct robot_t robot_t;

int LaserUpdate(Model *mod, robot_t *robot);
int PositionUpdate(Model *mod, robot_t *robot);
int FiducialUpdate(ModelFiducial *fid, robot_t *robot);
int Reset(Model *mod, robot_t *robot);

// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *args)
{
  robot_t *robot = new robot_t();

  // parse input params from worldfile
  cxxopts::Options options_wf("noise_random_goals", "traffic avoidance with variable noise");
  options_wf.add_options()
    ("s, stopdist", "Stop moving when obstacle detected within this distance", cxxopts::value<double>())
    ("c, cruisespeed", "Speed when no close obstacle", cxxopts::value<double>())
    ("a, anglenoise", "Standard deviation of noise added to angles", cxxopts::value<double>()->default_value("0"))
    ("b, bias", "Bias of noise added to angles", cxxopts::value<double>()->default_value("0"))
    ("n, newgoals", "Keep generating new goals", cxxopts::value<bool>()) // implicit value is true
    // ("p, periodic", "Periodic boundary conditions?", cxxopts::value<bool>()) // implicit value is true
    ("circle", "Generate goals in circle instead of square", cxxopts::value<bool>()) // implicit value is true
    ("u, r_upper", "Upper bound radius for goal generation (or s/2 for square)", cxxopts::value<double>())
    ("l, r_lower", "Lower bound radius for goal generation", cxxopts::value<double>()->default_value("0"))
    ("r, runsteps", "Total steps in a run phase", cxxopts::value<int>())
    ("random_runsteps", "Randomize runsteps each phase (average is the runsteps passed above)", cxxopts::value<bool>()) // implicit value is true
    ("t, tumblesteps", "Total steps in a tumble phase", cxxopts::value<int>())
    ("mem", "Length of memory stored", cxxopts::value<int>())
    ("d, data", "Data in string form to append to any data outputs", cxxopts::value<std::string>()->default_value(""))
    ("f, flockstrength", "flocking weight when choosing angle", cxxopts::value<double>()->default_value("0.5"))
    ;

  auto result_wf = cast_args(options_wf, &args->worldfile[0]);
  robot->stopdist = result_wf["stopdist"].as<double>();
  robot->cruisespeed = result_wf["cruisespeed"].as<double>();
  robot->anglebias = result_wf["bias"].as<double>();
  robot->newgoals = result_wf["newgoals"].as<bool>();
  // robot->periodic = result_wf["periodic"].as<bool>();
  robot->circle = result_wf["circle"].as<bool>();
  robot->r_lower = result_wf["r_lower"].as<double>();
  robot->r_upper = result_wf["r_upper"].as<double>();

  robot->runsteps = result_wf["runsteps"].as<int>();
  robot->avg_runsteps = result_wf["runsteps"].as<int>();
  robot->random_runsteps = result_wf["random_runsteps"].as<bool>();
  robot->tumblesteps = result_wf["tumblesteps"].as<int>();
  robot->flockstrength = result_wf["flockstrength"].as<double>();
  
  robot->anglenoise = result_wf["anglenoise"].as<double>();
  robot->addtl_data = result_wf["data"].as<std::string>();
  robot->memory_length = result_wf["mem"].as<int>();
  robot->periodic = mod->GetWorld()->IsPeriodic();

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
  gen_waypoint_data(robot); // set up vectors for storing waypoint history

  robot->memory_index = 0;
  std::vector<bool> mem_vec(robot->memory_length, 0);
  robot->blocked_memory = mem_vec;
  robot->blocked_count = 0;
  
  robot->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), robot);
  robot->pos->AddCallback(Model::CB_RESET, model_callback_t(Reset), robot);
  robot->pos->Subscribe(); // starts the position updates

  robot->fiducial = (ModelFiducial *)mod->GetUnusedModelOfType("fiducial");
  assert(robot->fiducial);
  robot->fiducial->AddCallback(Model::CB_UPDATE, (model_callback_t)FiducialUpdate, robot);
  robot->fiducial->Subscribe();

  return 0; // ok
}

int Reset(Model *, robot_t *robot) {
  // printf("reset function in controller called \n");
  // printf("pos b4 reset: [ %.4f %.4f %.4f ]\n", robot->pos->GetPose().x, robot->pos->GetPose().y, robot->pos->GetPose().a);
  robot->goals_reached = 0;
  robot->running = false;
  robot->current_phase_count = 0;
  robot->near_boundary = calc_near_boundary(robot);

  // get new start goal locations 
  gen_start_goal_positions(robot);

  // wipe waypoints and memory vectors, then regenerate to new ones
  std::vector<ModelPosition::Waypoint>().swap(robot->pos->waypoints);
  std::vector<bool>().swap(robot->blocked_memory);
  gen_waypoint_data(robot);

  robot->memory_index = 0;
  std::vector<bool> mem_vec(robot->memory_length, 0);
  robot->blocked_memory = mem_vec;
  robot->blocked_count = 0;

  // printf("pos after reset: [ %.4f %.4f %.4f ]\n", robot->pos->GetPose().x, robot->pos->GetPose().y, robot->pos->GetPose().a);
  return 0;
}

/** If memory_interval has passed since last update, record whether robot is currently blocked. */
inline void memory_update(robot_t *robot) {
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

int FiducialUpdate(ModelFiducial *fid, robot_t *robot)
{
  // find the closest teammate

  double dist = 1e6; // big

  robot->closest = NULL;

  FOR_EACH (it, fid->GetFiducials()) {
    ModelFiducial::Fiducial *other = &(*it);

    if (other->range < dist) {
      dist = other->range;
      robot->closest = other;
    }
  }

  if (robot->closest) // if we saw someone
  {
    robot->closest_bearing = robot->closest->bearing;
    robot->closest_range = robot->closest->range;
    robot->closest_heading_error = robot->closest->geom.a;
  }

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

  memory_update(robot);

  // updates for if the goal has been reached
  if (robot->pos->GetPose().Distance(robot->goal_pos) < tol) {
    if (robot->outfile_name != "NULL" && !robot->outfile_name.empty()) {
      robot->outfile.open(robot->outfile_name, std::ios_base::app);
      float frustration = (robot->memory_length == 0 ? 1 : float(robot->blocked_count) / float(robot->memory_length));
      robot->outfile << (std::string("goal") + std::string(",") + std::to_string(robot->pos->GetWorld()->SimTimeNow()) + std::string(",") + std::string(robot->pos->Token()) + std::string(",") + std::to_string(frustration) + std::string(",") + robot->addtl_data) << std::endl;
      robot->outfile.close();
    }
    goal_updates(robot);
  }

  // check if current run or tumble phase is over
  if (robot->current_phase_count >= (robot->running ? robot->runsteps : robot->tumblesteps)) {
    robot->running = !robot->running;
    robot->current_phase_count = 0;
  }

  if (robot->current_phase_count == 0) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    // if a new run phase is beginning, get random runlength between 1/2 and 3/2 of provided runsteps
    if (robot->random_runsteps && robot->running) {
      int lower = std::round(robot->avg_runsteps / 2);
      int higher = std::round(3 * robot->avg_runsteps / 2);
      std::uniform_int_distribution<uint32_t> unifrange(lower,higher);
      robot->runsteps = unifrange(generator);
    }

    // if a new tumble phase is beginning, set goal angle
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

      float frustration = (robot->memory_length == 0 ? 1 : float(robot->blocked_count) / float(robot->memory_length));
      std::normal_distribution<double> distribution(robot->anglebias * frustration, robot->anglenoise * frustration);

      // average between random angle and closest neighbor's from fiducials, if closest neighbor is found
      if (robot->closest) {
        // use pose.a or geom.a. pose.a is perfectly accurate but is kind of cheating; geom.a is the relative angle difference
        Stg::radians_t avg_angle = normalize(.5 * (robot->closest->geom.a + 2 * robot->pos->GetPose().a));
        robot->goal_angle = normalize((1 - robot->flockstrength) * (goal_angle + distribution(generator)) + robot->flockstrength * avg_angle);
        printf("%s goal angle: %f \n", robot->pos->Token(), robot->goal_angle);
      }
      else {
        robot->goal_angle = normalize(goal_angle + distribution(generator));
      }
    }
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

  if (robot->periodic) {
    robot->near_boundary = calc_near_boundary(robot);
    if (cur_pos.x < -s/2 || cur_pos.x > s/2 || cur_pos.y < -s/2 || cur_pos.y > s/2) { // if out of bounds
      double x = fmod(cur_pos.x + s/2, s) - s/2;
      double y = fmod(cur_pos.y + s/2, s) - s/2;
      robot->pos->SetGlobalPose(Pose(x > -s/2 ? x : x + s, y > -s/2 ? y : y + s, cur_pos.z, cur_pos.a));
    }
  }

  robot->current_phase_count++;
  return 0; // run again
}

int PositionUpdate(Model *, robot_t *robot)
{
  Pose pose = robot->pos->GetPose();
  return 0; // run again
}
