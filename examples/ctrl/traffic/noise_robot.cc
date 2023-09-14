#include "../../../libstage/stage.hh"
#include <cxxopts.hpp>
#include <chrono>
#include <fstream>
#include "random.hh"
#include "base_robot.hh"


using namespace Stg;

static const bool verbose = false;
static const float tol = .3;

int LaserUpdate(Model *mod, BaseRobot *robot);
int PositionUpdate(Model *mod, BaseRobot *robot);
int FiducialUpdate(ModelFiducial *fid, BaseRobot *robot);
int Reset(Model *mod, BaseRobot *robot);


// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *args)
{
  NoiseRobot *robot = new NoiseRobot(mod, args);
  robot->initialize(mod, args);

  assert(robot->fiducial);
  robot->fiducial->AddCallback(Model::CB_UPDATE, model_callback_t(FiducialUpdate), robot);
  robot->fiducial->Subscribe();

  robot->laser->AddCallback(Model::CB_UPDATE, model_callback_t(LaserUpdate), robot);
  robot->laser->Subscribe();

  robot->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), robot);
  robot->pos->AddCallback(Model::CB_RESET, model_callback_t(Reset), robot);
  robot->pos->Subscribe(); // starts the position updates

  return 0; // ok
}

int Reset(Model *, BaseRobot *robot) {
  robot->reset();
  return 0;
}

int FiducialUpdate(ModelFiducial *fid, BaseRobot *robot)
{
  return 0;
}

int LaserUpdate(Model *, BaseRobot *robot)
{
  return 0; // run again
}

int PositionUpdate(Model *, BaseRobot *robot)
{
  robot->location_sensor_update();

  // check if robot has reached its goal and update variables accordingly
  if (robot->pos->GetPose().Distance(robot->goal_pos) < tol) {
    robot->goal_updates();
  }

  robot->motion_update();

    return 0;
}