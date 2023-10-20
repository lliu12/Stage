#include <cstdlib>
#include <stdlib.h>
#include <string.h>
#include <iomanip>

#include "stage.hh"
#include "../examples/ctrl/traffic/traffic_robots.hh"
using namespace Stg;

// Constructor
DataCollector::DataCollector(std::ofstream *ofile, int sample_steps) {
    outfile = ofile;
    *outfile << std::fixed << std::setprecision(2); 
    steps_between_samples = sample_steps;
}

// Destructor
DataCollector::~DataCollector(void){}

// Count number of robots in world
int DataCollector::CountNumRobots(World *world) {
    int result = 0;
    FOR_EACH (m, world->models) {
        std::set<Model::cb_t> &reset_callbacks = (*m)->callbacks[Model::CB_RESET];
        FOR_EACH (it, reset_callbacks) {
            const Model::cb_t &cba = *it;
            result++;
        }
    }
    return result;
}

// Count number of blocked robots
int DataCollector::CountBlockedRobots(World *world) {
    int result = 0;
    FOR_EACH (m, world->models) {
        std::set<Model::cb_t> &reset_callbacks = (*m)->callbacks[Model::CB_RESET];
        FOR_EACH (it, reset_callbacks) {
            const Model::cb_t &cba = *it;
            if (((BaseRobot*)cba.arg)->stop) {
                result++;
            }
        }
    }
    return result;
}

// Get robot addtldata field for saving to outfile
std::string DataCollector::GetAddtlData(World *world) {
    std::string result;
    FOR_EACH (m, world->models) {
        std::set<Model::cb_t> &reset_callbacks = (*m)->callbacks[Model::CB_RESET];
        FOR_EACH (it, reset_callbacks) {
            const Model::cb_t &cba = *it;
            result = ((BaseRobot*)cba.arg)->addtl_data;
        }
    }
    return result;
}

// Collect data every X steps of simulation. A good default is probably 10 seconds which would be 100 steps? I think the default step was .1sec
// Later one I want to save ID/pos of every blocked robot and ID/pos of the robot blocking it.... 
// Should be fine because by definition any robot blocked by the jam will be in the jam. 
// But a robot blocking the jam that is itself not blocked will not be included.



// Simulate a world. Every this->sample_steps, save data about how many robots are blocked. 
void DataCollector::SimWorldCountBlocked(World *world, int trials) {
    int total_robots = CountNumRobots(world);
    std::string world_addtl_data = GetAddtlData(world);
    for (int trial = 0; trial < trials; trial++) {
        bool done = false;
        int since_last_sample = 0; // num updates since we last saved data
        do {
            // save data at steps_between_samples update intervals
            if (since_last_sample % steps_between_samples == 0) {
                int cur_blocked_robots = CountBlockedRobots(world);
                *outfile << std::to_string(trial) + std::string(",") +
                            std::to_string(world->SimTimeNow()) + std::string(",") +
                            world_addtl_data + std::string(",") +
                            std::to_string(cur_blocked_robots) + std::string(",") +
                            std::to_string(total_robots)
                         << std::endl;
                since_last_sample = 0;
            }
            done = world->Update();
            since_last_sample++;
        }
        while(!done);
        world->Reset();
    }
}


// Simulate a world. At sample steps, record which robots are stopped and all robots blocking them.
void DataCollector::SimWorldRecordStoppedRobots(World *world, int trials, bool save_positions) {

    std::string world_addtl_data = GetAddtlData(world);
    for (int trial = 0; trial < trials; trial++) {
        bool done = false;
        int since_last_sample = 0; // num updates since we last saved data
        do {
            // save data at steps_between_samples update intervals
            if (since_last_sample % steps_between_samples == 0) {
                FOR_EACH (m, world->models) {
                    std::set<Model::cb_t> &reset_callbacks = (*m)->callbacks[Model::CB_RESET];
                    FOR_EACH (it, reset_callbacks) {
                        const Model::cb_t &cba = *it;
                        BaseRobot *robot = (BaseRobot*)cba.arg;
                        if (!robot->stop) {
                            *outfile << std::to_string(trial) + std::string(",") +
                                std::to_string(world->SimTimeNow()) + std::string(",") +
                                world_addtl_data + 
                                std::to_string((*m)->GetFiducialReturn()) + std::string(",")
                                << robot->pos->GetPose().x << std::string(",")
                                << robot->pos->GetPose().y << std::string(",")
                                << robot->pos->GetPose().a << std::string(",") +
                                std::to_string(robot->goal_birth_time) + std::string(",") +
                                std::to_string(robot->goals_reached) + std::string(",") +
                                std::to_string(robot->stop) + std::string(",") +
                                std::to_string(-1) + std::string(",")
                                << std::endl;
                        }

                        else {
                            if (robot->fiducial->GetFiducials().size() == 0) {
                                printf("Error: Robot stopped but nothing in fiducials.... \n");
                                printf("Sim time: %llu, Robot ID: %i \n", world->SimTimeNow(), (*m)->GetFiducialReturn());
                            }

                            FOR_EACH (it, robot->fiducial->GetFiducials()) {
                                ModelFiducial::Fiducial *other = &(*it);
                                if (other->range < robot->stopdist) {
                                    *outfile << std::to_string(trial) + std::string(",") +
                                        std::to_string(world->SimTimeNow()) + std::string(",") +
                                        world_addtl_data + 
                                        std::to_string((*m)->GetFiducialReturn()) + std::string(",")
                                        << robot->pos->GetPose().x << std::string(",")
                                        << robot->pos->GetPose().y << std::string(",")
                                        << robot->pos->GetPose().a << std::string(",") +
                                        std::to_string(robot->goal_birth_time) + std::string(",") +
                                        std::to_string(robot->goals_reached) + std::string(",") +
                                        std::to_string(robot->stop) + std::string(",") +
                                        std::to_string(other->id) + std::string(",")
                                        << std::endl;
                                }
                            }
                        }
                    }
                }
                since_last_sample = 0;
            }
            done = world->Update();
            since_last_sample++;
        }
        while(!done);
        world->Reset();
    }
}

// Simulate a world. At sample steps, record which robots are stopped and the single closest robot blocking them.
void DataCollector::SimWorldRecordStoppedRobotsClosest(World *world, int trials, bool save_positions) {
    // int total_robots = CountNumRobots(world);
    std::string world_addtl_data = GetAddtlData(world);
    for (int trial = 0; trial < trials; trial++) {
        bool done = false;
        int since_last_sample = 0; // num updates since we last saved data
        do {
            // save data at steps_between_samples update intervals
            if (since_last_sample % steps_between_samples == 0) {
                FOR_EACH (m, world->models) {
                    std::set<Model::cb_t> &reset_callbacks = (*m)->callbacks[Model::CB_RESET];
                    FOR_EACH (it, reset_callbacks) {
                        const Model::cb_t &cba = *it;
                        BaseRobot *robot = (BaseRobot*)cba.arg;
                        *outfile << std::to_string(trial) + std::string(",") +
                                    std::to_string(world->SimTimeNow()) + std::string(",") +
                                    world_addtl_data + 
                                    robot->pos->Token() + std::string(",")
                                    << robot->pos->GetPose().x << std::string(",")
                                    << robot->pos->GetPose().y << std::string(",") +
                                    std::to_string(robot->stop) + std::string(",") +
                                    std::to_string((robot->closest ? robot->closest->id : -1)) + std::string(",")
                                    << std::endl;
                        
                    }
                }
                since_last_sample = 0;
            }
            done = world->Update();
            since_last_sample++;
        }
        while(!done);
        world->Reset();
    }
}
