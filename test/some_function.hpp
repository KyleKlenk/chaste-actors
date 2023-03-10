#pragma once
#include "caf/all.hpp"
#include "caf/io/all.hpp"
#include "json.hpp"
#include <iostream>
#include <fstream>


int read_json_file();

/**
 * Is the entry point of the actor system
*/
void caf_main(caf::actor_system& system);


/*
  * Creates the actor system
*/
int create_actor_system();
