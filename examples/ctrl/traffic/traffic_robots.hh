#ifndef BASEROBOT_HH
#define BASEROBOT_HH

#include "../../../libstage/stage.hh"
#include <fstream>
#include <cxxopts.hpp>
using namespace Stg;


// A basic robot which navigates to randomly generated individual goals
// Uses fiducial sensing

class BaseRobot {
  public:
    ModelPosition *pos;
    ModelRanger *laser;
    ModelFiducial *fiducial;
    int goals_reached;
    uint64_t goal_birth_time;
    Pose start_pos, goal_pos;
    bool stop;
    double stopdist, cruisespeed, r_lower, r_upper;
    std::string outfile_name, addtl_data = ""; 
    std::ofstream outfile;
    bool verbose = false, newgoals, periodic, circle;
    uint64_t memory_interval = 1000000;
    ModelFiducial::Fiducial *closest = NULL;

    //// Constructor
    BaseRobot(Model *mod, CtrlArgs *args);

    //// Destructor
    ~BaseRobot();

    //// Finish initializing robot (setup requiring virtual functions)
    virtual void initialize(Model *mod, CtrlArgs *args);

    //// Add options to argument parser
    virtual void add_options(cxxopts::Options *options_wf);

    //// Use parser results to fill in class fields
    virtual void fill_in_params(cxxopts::ParseResult results_wf);

    //// Cast our argument string to same form as argc, argv and apply parser to it
    cxxopts::ParseResult cast_parse_args(cxxopts::Options options, char * string_start);

    //// Use rejection sampling to get a random point in a the ring between radius r_lower and r_upper (center at origin)
    Pose random_goal();

    //// Set up start and goal positions
    virtual void gen_start_goal_positions();

    //// Set up waypoint storage (used to visualize next goal)
    virtual void gen_waypoint_data();

    //// Reset robot data for a new trial
    virtual void reset();

    //// Updates to make when robot reaches goal
    virtual void goal_updates();

    //// Updates made each step for values other than robot speed and direction
    //// Determine whether robot is blocked and update info about blockedness, closest neighbor, periodicity updates, etc.
    virtual void member_update();

    //// Update robot speed and turning angle
    virtual void motion_update();

    //// Get (global) angle robot should move in to head straight to goal
    double angle_to_goal();
    
};




///////////////////////////////////////////////////////////////////////////
// A robot which navigates to randomly generated individual goals, sometimes adding noise to its motion

class NoiseRobot : public BaseRobot {
  public:
    bool running; // is robot currently in run phase
    int current_phase_count, avg_runsteps, runsteps, tumblesteps; // time so far spent running or tumbling, total length of a run or tumble period
    double anglenoise, travel_angle, anglebias; // amount of noise and bias to add to random angles
    bool random_runsteps; // randomly vary length of run phases

    //// Constructor
    NoiseRobot(Model *mod, CtrlArgs *args);

    //// Destructor
    ~NoiseRobot();

    //// Add options to argument parser
    virtual void add_options(cxxopts::Options *options_wf) override;

    //// Use parser results to fill in class fields
    virtual void fill_in_params(cxxopts::ParseResult results_wf) override;

    //// Reset robot data for a new trial
    virtual void reset() override;

    //// Update robot speed and turning angle
    virtual void motion_update() override;

    //// Determine angle for robot to steer in (after adding noise)
    virtual double get_travel_angle();

};







///////////////////////////////////////////////////////////////////////////
// A robot which moves with rotational noise only if it has been blocked within the last F samples (F for length of frustration)

class WithinLastFRobot : public NoiseRobot {
  public:
    int frustration_len; // how many frustrated steps to take after being blocked
    int frustrated_samples_left; // how many frustrated steps left (decreases with each non-blocked sample)

    //// Constructor
    WithinLastFRobot(Model *mod, CtrlArgs *args);

    //// Destructor
    ~WithinLastFRobot();

    //// Add options to argument parser
    virtual void add_options(cxxopts::Options *options_wf) override;

    //// Use parser results to fill in class fields
    virtual void fill_in_params(cxxopts::ParseResult results_wf) override;

    //// Reset robot data for a new trial
    virtual void reset() override;

    //// Updates made each step for values other than robot speed and direction
    //// Determine whether robot is blocked and update info about blockedness, closest neighbor, periodicity updates, memory, etc.
    virtual void member_update() override;

    // //// Update robot speed and turning angle
    // virtual void motion_update() override;

    //// Determine angle for robot to steer in (after adding noise)
    virtual double get_travel_angle() override;

};

#endif