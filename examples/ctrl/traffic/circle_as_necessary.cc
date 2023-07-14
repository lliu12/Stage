// travel towards goal, but turn right as far as necessary to not run into anything


// #include "stage.hh"
#include "/Users/lucyliu/stg/include/Stage-4.3/stage.hh"
using namespace Stg;

static const double cruisespeed = 1;
static const double avoidspeed = 0.1;
static const double minfrontdistance = 1; // 0.6
static const bool verbose = false;
static const double stopdist = 0.5;
static const double circle_radius = 5; // the circle that all agents' start and goal positions lie on

static const float tol = .1;

typedef struct {
  ModelPosition *pos;
  ModelRanger *laser;
  bool to_goal;
  int fwd_count;
  Pose start_pos, goal_pos;
} robot_t;

int LaserUpdate(Model *mod, robot_t *robot);
int PositionUpdate(Model *mod, robot_t *robot);

// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *)
{
  robot_t *robot = new robot_t();

  robot->pos = dynamic_cast<ModelPosition *>(mod);
  if (!robot->pos) {
    PRINT_ERR("No position model given in wander controller."); 
    exit(1);
  }

  robot->start_pos = robot->pos->GetPose();
  robot->goal_pos = robot->pos->GetPose() + Pose(2 * circle_radius,0,0,0);
  robot->to_goal = 1;
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

    if ((i > (sample_count / 3)) && (i < (sample_count - (sample_count / 3)))
        && scan[i] < minfrontdistance) {
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

  if (robot->pos->GetPose().Distance(robot->to_goal ? robot->goal_pos : robot->start_pos) < tol) {
        robot->to_goal = !robot->to_goal;
    }

  if (obstruction || stop) {

    robot->pos->SetXSpeed(stop ? 0.0 : avoidspeed);
    robot->pos->SetTurnSpeed(-.5); // turn right until no longer obstructed
    robot->fwd_count = 1; // once robot is no longer obstructed, it should move forward at least one step

  } else {

    robot->pos->SetXSpeed(cruisespeed);

    if (robot->fwd_count <= 0) {
        robot->pos->GoTo(robot->to_goal ? robot->goal_pos : robot->start_pos);
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
