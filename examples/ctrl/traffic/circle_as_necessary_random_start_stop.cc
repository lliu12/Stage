// travel towards goal, but turn right as far as necessary to not run into anything
// generate start and goal locations using rejection sampling for points at the outer part of the circle

#include "../../../libstage/stage.hh"
#include <cxxopts.hpp>
using namespace Stg;

static const double stopdist = 0.5;
static const double minfrontdistance = 1; // 0.6
static const double avoidspeed = 0.1;
static const double cruisespeed = .3;
static const double turnspeed = -1;

static const bool verbose = false;
static const double rink_radius = 7; // the circle that all agent's positions lie within
static const float tol = .1;

typedef struct {
  ModelPosition *pos;
  ModelRanger *laser;
  int fwd_count;
  Pose start_pos, goal_pos;
} robot_t;

int LaserUpdate(Model *mod, robot_t *robot);
int PositionUpdate(Model *mod, robot_t *robot);

// Use rejection sampling to get a random point in a circle of radius r (center at origin)
Pose random_in_circle(int r) {
    bool done = 0;
    double rand_x;
    double rand_y;
    while (!done) {
        rand_x = r * 2 * (drand48() - .5);
        rand_y = r * 2 * (drand48() - .5);
        double dist = Pose(rand_x, rand_y, 0, 0).Distance(Pose(0,0,0,0));
        if (dist < r && dist > r / 2.0) { done = 1; }
    }
    printf("Random Pose: [%.2f %.2f] \n", rand_x, rand_y);
    return Pose(rand_x, rand_y, 0, 0);
}

// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *)
{
  robot_t *robot = new robot_t();
  robot->pos = dynamic_cast<ModelPosition *>(mod);
  if (!robot->pos) {
    PRINT_ERR("No position model given in wander controller."); 
    exit(1);
  }

  robot->pos->SetColor(Color::RandomColor());

  printf("Generating start and goal for robot %s...", robot->pos->Token());

  robot->goal_pos = random_in_circle(rink_radius);

  std::vector<ModelPosition::Waypoint> wps;
  wps.push_back(ModelPosition::Waypoint(robot->goal_pos, robot->pos->GetColor()));
  robot->pos->waypoints = wps;

  robot->pos->SetGlobalPose(random_in_circle(rink_radius));
  robot->start_pos = robot->pos->GetPose();

  robot->fwd_count = 1;
  
  robot->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), robot);
  robot->pos->Subscribe(); // starts the position updates

  // find a range finder

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

  bool obstruction = false;
  bool stop = false;

  // find the closest distance to the left and right and check if
  // there's anything in front
  double minleft = 1e6;
  double minright = 1e6;

  for (uint32_t i = 0; i < sample_count; i++) {

    if (scan[i] < minfrontdistance) {
      obstruction = true;
    }

    if (scan[i] < stopdist) {
      stop = true;
    }

    if (i > sample_count / 2)
      minleft = std::min(minleft, scan[i]);
    else
      minright = std::min(minright, scan[i]);
  }

  if (robot->pos->GetPose().Distance(robot->goal_pos) < tol) {
        std::swap(robot->goal_pos, robot->start_pos);
        std::vector<ModelPosition::Waypoint> wps;
        wps.push_back(ModelPosition::Waypoint(robot->goal_pos, robot->pos->GetColor()));
        robot->pos->waypoints = wps;
    }

  if (obstruction || stop) {

    robot->pos->SetXSpeed(stop ? 0.0 : avoidspeed);
    robot->pos->SetTurnSpeed(turnspeed); // turn right until no longer obstructed
    robot->fwd_count = 1; // once robot is no longer obstructed, it should move forward at least one step before attempting to turn back to goal

  } else {

    robot->pos->SetXSpeed(cruisespeed);

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

//   printf("Pose: [%.2f %.2f %.2f %.2f]\n", pose.x, pose.y, pose.z, pose.a);

  return 0; // run again
}
