// robot travels straight ahead and begins traveling in a circle when it sees an obstruction

#include "../../../libstage/stage.hh"
using namespace Stg;

static const double cruisespeed = 1;
static const double turnspeed = .25;

typedef struct {
  ModelPosition *pos;
  ModelRanger *laser;
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

  robot->pos->SetXSpeed(cruisespeed);
  robot->pos->SetTurnSpeed(turnspeed);

  return 0; // run again
}

int PositionUpdate(Model *, robot_t *robot)
{
  Pose pose = robot->pos->GetPose();

//   printf("Pose: [%.2f %.2f %.2f %.2f]\n", pose.x, pose.y, pose.z, pose.a);

  return 0; // run again
}
