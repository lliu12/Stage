#include "../../../libstage/stage.hh"
#include <cxxopts.hpp>
#include "traffic_robots.hh"
#include <stdio.h>
#include "random.hh"
#include <random>
#include <chrono>
#include <fstream>

///////////////////////////////////////////////////////////////////////////
// Define BaseRobot class functions

// Constructor
BaseRobot::BaseRobot(Model *mod, CtrlArgs *args) {
    pos = dynamic_cast<ModelPosition *>(mod);
    goals_reached = 0;
    periodic = pos->GetWorld()->IsPeriodic();
    pos->SetColor(Color::RandomColor());
    goal_birth_time = pos->GetWorld()->SimTimeNow();
    closest = NULL;

    // Set up sensors
    // rangefinder
    ModelRanger *new_laser = NULL;
    for( int i=1; i<17; i++ )
      {
        char name[32];
        snprintf( name, 32, "ranger:%d", i ); // generate sequence of model names
        if (verbose) {
          printf( "  looking for a suitable ranger at \"%s:%s\" ... \n", pos->Token(), name );
        }      
        new_laser = dynamic_cast<ModelRanger *>(pos->GetChild( name ));
        
        if( new_laser && new_laser->GetSensors()[0].sample_count > 8 ) {
          break;
        }
      }
    
    if( !new_laser ) {
      PRINT_ERR("  Failed to find a ranger with more than 8 samples. Exit.");
      exit(2);
    }

    laser = new_laser;

    // fiducial
    fiducial = (ModelFiducial *)mod->GetUnusedModelOfType("fiducial");

}

// Destructor
BaseRobot::~BaseRobot(void){}

// Initialize
void BaseRobot::initialize(Model *mod, CtrlArgs *args) {
  // Parse argument parameters from worldfile
  cxxopts::Options options_wf("base robot", "robot heads to its goal");
  add_options(&options_wf);
  cxxopts::ParseResult result_wf = cast_parse_args(options_wf, &args->worldfile[0]);
  fill_in_params(result_wf);

  // Set up goals, waypoints
  gen_start_goal_positions();
  gen_waypoint_data();
}

// Cast our argument string to same form as argc, argv and apply parser to it
cxxopts::ParseResult BaseRobot::cast_parse_args(cxxopts::Options options, char * string_start) {
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

// Add options to cxxopts parser
void BaseRobot::add_options(cxxopts::Options *options_wf) {
    options_wf->add_options()
        ("s, stopdist", "Stop moving when obstacle detected within this distance", cxxopts::value<double>())
        ("c, cruisespeed", "Speed when no close obstacle", cxxopts::value<double>())
        ("n, newgoals", "Keep generating new goals", cxxopts::value<bool>()) // implicit value is true
        ("circle", "Generate goals in circle instead of square", cxxopts::value<bool>()) // implicit value is true
        ("u, r_upper", "Upper bound radius for goal generation (or s/2 for square)", cxxopts::value<double>())
        ("l, r_lower", "Lower bound radius for goal generation", cxxopts::value<double>()->default_value("0"))
        ("d, data", "Data in string form to append to any data outputs", cxxopts::value<std::string>()->default_value(""))
        ;
}

// Fill in fields using parser results
void BaseRobot::fill_in_params(cxxopts::ParseResult result_wf) {
    // Robot motion params
    stopdist = result_wf["stopdist"].as<double>();
    cruisespeed = result_wf["cruisespeed"].as<double>();

    // When and where random goals are generated
    newgoals = result_wf["newgoals"].as<bool>();
    circle = result_wf["circle"].as<bool>();
    r_lower = result_wf["r_lower"].as<double>();
    r_upper = result_wf["r_upper"].as<double>();

    addtl_data = result_wf["data"].as<std::string>();
}

// Use rejection sampling to get a random point in a the ring between radius r_lower and r_upper (center at origin)
Pose BaseRobot::random_goal() 
{
  bool done = 0;
  double rand_x;
  double rand_y;
  double rand_a = 2 * M_PI * (Random::get_unif_double(0, 1) - .5);

  while (!done) {
      rand_x = r_upper * 2 * (Random::get_unif_double(0, 1) - .5);
      rand_y = r_upper * 2 * (Random::get_unif_double(0, 1) - .5);
      double dist = Pose(rand_x, rand_y, 0, 0).Distance(Pose(0,0,0,0));
      if (!circle || (dist <= r_upper && dist >= r_lower)) { done = 1; }
  }

  if (verbose) {
    printf("Random Pose (x, y, angle) generated for %s is [%.2f %.2f %.2f] \n", pos->Token(), rand_x, rand_y, rand_a);
  }    

  return Pose(rand_x, rand_y, 0, rand_a);
}

// initialize robot's start and goal positions
void BaseRobot::gen_start_goal_positions() {
  if (verbose) {
    printf("\nGenerating start and goal for robot %s... \n", pos->Token());
  }
  pos->SetGlobalPose(random_goal());
  start_pos = pos->GetPose();
  goal_pos = random_goal(); // set goal
}

// set up vectors for storing waypoint history, memory
void BaseRobot::gen_waypoint_data() {
  std::vector<ModelPosition::Waypoint> wps;
  wps.push_back(ModelPosition::Waypoint(goal_pos, pos->GetColor()));
  pos->waypoints = wps;
}

// reset robot data (e.g. for a new trial)
void BaseRobot::reset() {
    goals_reached = 0;
    goal_birth_time = pos->GetWorld()->SimTimeNow();
    closest = NULL;

    // get new start goal locations 
    gen_start_goal_positions();

    // wipe waypoints and memory vectors by swapping their values with an empty vector, then regenerate to new ones
    std::vector<ModelPosition::Waypoint>().swap(pos->waypoints);
    gen_waypoint_data();
}

// make updates when robot reaches goal (increase goal counters, generate new goal, etc)
void BaseRobot::goal_updates() {
    if (newgoals) {
      goal_pos = random_goal();
    }
    else { 
        std::swap(goal_pos, start_pos); 
    }

    std::vector<ModelPosition::Waypoint> wps;
    wps.push_back(ModelPosition::Waypoint(goal_pos, pos->GetColor()));
    pos->waypoints = wps;
    goals_reached++;
    goal_birth_time = pos->GetWorld()->SimTimeNow();
}

//// Updates made each step for values other than robot speed and direction
//// Determine whether robot is blocked and update info about blockedness, closest neighbor, periodicity updates, etc.
void BaseRobot::member_update() {
  // find the closest teammate
  double dist = std::numeric_limits<double>::infinity();; // big
  closest = NULL;
  FOR_EACH (it, fiducial->GetFiducials()) {
    ModelFiducial::Fiducial *other = &(*it);
    if (other->range < dist) {
      dist = other->range;
      closest = other;
    }
  }

  // update whether robot is blocked or not
  stop = dist < stopdist;

  // update location if world is periodic and robot is now out of bounds
  if (periodic) {
    double s = 2 * r_upper;
    Pose cur_pos = pos->GetPose();

    if (cur_pos.x < -s/2 || cur_pos.x > s/2 || cur_pos.y < -s/2 || cur_pos.y > s/2) { // if out of bounds
      double x = fmod(cur_pos.x + s/2, s) - s/2;
      double y = fmod(cur_pos.y + s/2, s) - s/2;
      pos->SetGlobalPose(Pose(x > -s/2 ? x : x + s, y > -s/2 ? y : y + s, cur_pos.z, cur_pos.a));
    }
  }
}

//// Get (global) angle robot should move in to head straight to goal
double BaseRobot::angle_to_goal() {
      Pose goal_pos_helper; // will be true goal pos if world is not periodic
      if (!periodic) {
        goal_pos_helper = goal_pos;
      }

      // if space is periodic, figure out where robot should move to for shortest path to goal
      else {
        double s = 2 * r_upper;
        double xs [9] = {-s, -s, -s, 0, 0, 0, s, s, s};
        double ys [9] = {-s, 0, s, -s, 0, s, -s, 0, s};
        int closest_pos = 0;
        double closest_dist = std::numeric_limits<double>::infinity();
        for ( int i=0; i<9; i++ ) {
          Pose diff = Pose(xs[i], ys[i], 0, 0);
          Pose test_pos = goal_pos + diff;
          double dist = pos->GetPose().Distance(test_pos);
          if (dist < closest_dist) {
            closest_dist = dist;
            closest_pos = i;
          }
        }
        goal_pos_helper = Pose(goal_pos.x + xs[closest_pos], goal_pos.y + ys[closest_pos], 0, 0);
      }
      
      double x_error = goal_pos_helper.x - pos->GetPose().x;
      double y_error = goal_pos_helper.y - pos->GetPose().y;
      return atan2(y_error, x_error);
}

//// Set robot speed and turning angle
void BaseRobot::motion_update() {
  double goal_angle = angle_to_goal();
  double a_error = normalize(goal_angle - pos->GetPose().a);
  pos->SetTurnSpeed(a_error);
  pos->SetXSpeed(stop ? 0 : cruisespeed);
}







///////////////////////////////////////////////////////////////////////////
// Define NoiseRobot class functions

NoiseRobot::NoiseRobot(Model *mod, CtrlArgs *args) : BaseRobot(mod, args) {
  running = false;
  current_phase_count = 0;
}

// Destructor
NoiseRobot::~NoiseRobot(void){}

void NoiseRobot::add_options(cxxopts::Options *options_wf) {
  BaseRobot::add_options(options_wf);
  options_wf->add_options()
    ("a, anglenoise", "STD when random noise is added to tumble goal angles", cxxopts::value<double>()->default_value("0"))
    ("b, bias", "Bias of noise added to angles", cxxopts::value<double>()->default_value("0"))
    ("r, runsteps", "Total steps in a run phase", cxxopts::value<int>())
    ("random_runsteps", "Randomize runsteps each phase (average is the runsteps passed above)", cxxopts::value<bool>()) // implicit value is true
    ("t, tumblesteps", "Total steps in a tumble phase", cxxopts::value<int>())
    ;
}

void NoiseRobot::fill_in_params(cxxopts::ParseResult result_wf) {
  BaseRobot::fill_in_params(result_wf);
  anglenoise = result_wf["anglenoise"].as<double>();
  anglebias = result_wf["bias"].as<double>();
  runsteps = result_wf["runsteps"].as<int>();
  avg_runsteps = result_wf["runsteps"].as<int>();
  random_runsteps = result_wf["random_runsteps"].as<bool>();
  tumblesteps = result_wf["tumblesteps"].as<int>();

}

void NoiseRobot::reset() {
  BaseRobot::reset();
  running = false;
  current_phase_count = 0;
}


//// Determine angle for robot to steer in (after adding noise)
double NoiseRobot::get_travel_angle() {
  return angle_to_goal() + (anglenoise == -1 ? Random::get_unif_double(-M_PI, M_PI) : Random::get_normal_double(anglebias, anglenoise));
}

//// Set robot speed and turning angle
void NoiseRobot::motion_update() {
  // check if current run or tumble phase is over
  if (current_phase_count >= (running ? runsteps : tumblesteps)) {
    running = !running;
    current_phase_count = 0;
  }

  if (current_phase_count == 0) {
    // if a new run phase is beginning, get random runlength between 1/2 and 3/2 of provided runsteps
    if (random_runsteps && running) {
      int lower = std::round(avg_runsteps / 2);
      int higher = std::round(3 * avg_runsteps / 2);
      runsteps = Random::get_unif_int(lower, higher);
    }

    // if a new tumble phase is beginning, set goal angle
    if (!running && current_phase_count == 0) {
      travel_angle = get_travel_angle();
    }
  }

  if (running) {
    pos->SetXSpeed(stop ? 0 : cruisespeed);
    pos->SetTurnSpeed(0);
  }
  else {
    double a_error = normalize(travel_angle - pos->GetPose().a);
    pos->SetTurnSpeed(a_error);
    pos->SetXSpeed(0);
  }
  current_phase_count++;
}







///////////////////////////////////////////////////////////////////////////
// Define WithinLastFRobot class functions

WithinLastFRobot::WithinLastFRobot(Model *mod, CtrlArgs *args) : NoiseRobot(mod, args) {
  frustrated_samples_left = 0;
}


void WithinLastFRobot::add_options(cxxopts::Options *options_wf) {
  NoiseRobot::add_options(options_wf);
  options_wf->add_options()
    ("f, frustration_len", "Num. random steps to take after being blocked", cxxopts::value<int>())
    ;
}

void WithinLastFRobot::fill_in_params(cxxopts::ParseResult result_wf) {
  NoiseRobot::fill_in_params(result_wf);
  frustration_len = result_wf["frustration_len"].as<int>();
}

void WithinLastFRobot::reset() {
  NoiseRobot::reset();
  frustrated_samples_left = 0;
}

//// Determine angle for robot to steer in (after adding conditional noise)
double WithinLastFRobot::get_travel_angle() {
  if (anglenoise == -1) {return Random::get_unif_double(-M_PI, M_PI);}
  else {return angle_to_goal() + (frustrated_samples_left > 0 || frustration_len == 0 ? Random::get_normal_double(anglebias, anglenoise) : 0);}
}

void WithinLastFRobot::member_update() {
  NoiseRobot::member_update();
  if (pos->GetWorld()->SimTimeNow() % memory_interval == 0) {
    if (stop) {
        frustrated_samples_left = frustration_len;
    }
    else {
        frustrated_samples_left = std::max(frustrated_samples_left - 1, 0);
    }
  }
}