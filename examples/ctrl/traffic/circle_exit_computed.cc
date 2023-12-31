// robot travels straight ahead and begins traveling in a circle when it sees an obstruction
// compute when to exit circle using closest point on circle to goal

#include "../../../libstage/stage.hh"
using namespace Stg;

static const double cruisespeed = 0.4;
static const double avoidspeed = 0.05;
static const double avoidturn = 0.5;
static const double minfrontdistance = 3.0; // 0.6
static const bool verbose = false;
static const double stopdist = 0.3;
static const int avoidduration = 10;

static const float tol = .1;

typedef struct {
  ModelPosition *pos;
  ModelRanger *laser;
  int avoidcount, randcount;
  bool to_goal;
  Pose start_pos, goal_pos;
} robot_t;

int LaserUpdate(Model *mod, robot_t *robot);
int PositionUpdate(Model *mod, robot_t *robot);

// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *)
{
  robot_t *robot = new robot_t();

  robot->avoidcount = 0;
  robot->randcount = 0;

  robot->pos = dynamic_cast<ModelPosition *>(mod);
  if (!robot->pos) {
    PRINT_ERR("No position model given in wander controller.");
    exit(1);
  }

  robot->start_pos = robot->pos->GetPose();
  robot->goal_pos = robot->pos->GetPose() + Pose(4,0,0,0);
  robot->to_goal = 1;
  
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

  if (obstruction || stop || (robot->avoidcount > 0)) {

    robot->pos->SetXSpeed(stop ? 0.0 : avoidspeed); // STOP ROBOT: this is the change made from original wander controller

  } else {

    if (robot->pos->GetPose().Distance(robot->to_goal ? robot->goal_pos : robot->start_pos) < tol) {
        robot->to_goal = !robot->to_goal;
        // printf("%d\n", robot->to_goal);
    }
    
    robot->avoidcount = 0;
    robot->pos->GoTo(robot->to_goal ? robot->goal_pos : robot->start_pos);
    // robot->pos->SetXSpeed(cruisespeed);
    // robot->pos->SetTurnSpeed(robot->circling ? .3 : 0);


  }

  return 0; // run again
}

int PositionUpdate(Model *, robot_t *robot)
{
  Pose pose = robot->pos->GetPose();

//   printf("Pose: [%.2f %.2f %.2f %.2f]\n", pose.x, pose.y, pose.z, pose.a);

  return 0; // run again
}
