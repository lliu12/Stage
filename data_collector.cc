#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <sys/time.h>
#include <cstdlib>
#include <stdlib.h>
#include <string.h>
#include <random>
#include <chrono>
#include <fstream>

#include "stage.hh"
using namespace Stg;

// Constructor
DataCollector::DataCollector() {}

// Destructor
DataCollector::~DataCollector(void){}

// Count number of robots in world
int DataCollector::GetNumRobots(World *world) {
    int result = 0;
    FOR_EACH (m, world->models) {
        // (*it)->callbacks[CB_RESET]
        std::set<Model::cb_t> &reset_callbacks = (*m)->callbacks[Model::CB_RESET];
        FOR_EACH (it, reset_callbacks) {
            const Model::cb_t &cba = *it;
            // if (cba.arg->)
            result++;
        }
    }
    return result;
}
