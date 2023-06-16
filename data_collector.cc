#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <sys/time.h>
#include <cstdlib>
#include <stdlib.h>
#include <string.h>
#include <random>
#include <chrono>

#include "stage.hh"
#include "/Users/lucyliu/stage4/Stage/examples/ctrl/circles/base_robot.hh"
using namespace Stg;

// Constructor
DataCollector::DataCollector(std::ofstream *ofile, int sample_steps) {
    outfile = ofile;
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
            if (((struct base_robot*)cba.arg)->stop) {
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
            result = ((struct base_robot*)cba.arg)->addtl_data;
        }
    }
    return result;
}

// Collect data every X steps of simulation. A good default is probably 10 seconds which would be 100 steps? I think the default step was .1sec
// And this one should output stuff directly to outfile
// First pass: save num blocked robots at each step. 
// Later one I want to save ID/pos of every blocked robot and ID/pos of the robot blocking it.... 
// Should be fine because by definition any robot blocked by the jam will be in the jam. 
// But a robot blocking the jam that is itself not blocked will not be included.

// Simulate a world. Every this->sample_steps, save data about how many robots are blocked. 
void DataCollector::SimWorldCountBlocked(World *world, int trials = 1) {
    int total_robots = CountNumRobots(world);
    std::string world_addtl_data = GetAddtlData(world);
    for (int trial = 0; trial < trials; trial++) {
        bool done = false;
        int since_last_sample = 0; // num updaets since we last saved data
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
            }
            done = world->Update();
            since_last_sample++;
        }
        while(!done);
        world->Reset();
    }
}
