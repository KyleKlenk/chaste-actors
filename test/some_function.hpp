#pragma once
#include "caf/all.hpp"
#include "caf/io/all.hpp"
#include "json.hpp"
#include <iostream>
#include <fstream>


struct parameters {
  int numDigits;
  int numTimeSteps;
  std::string dtTissue;
  std::string dtCell;
  std::string mesh;
  double timeDuration;
  std::string stimulusCurrent;
  double stimulusDuration;
  double stimulusStarttime;
  double stimulusRange_x;
  double stimulusRange_y;
  double stimulusRange_z;
  double electrodeposition_x;
  double electrodeposition_y;
  double electrodeposition_z;
  double sigma_ix;
  double sigma_iy;
  double sigma_iz;
  double sigma_ex;
  double sigma_ey;
  double sigma_ez;
  int Cm;
  int Chi;
  double xLength;
  double yLength;
  double zLength;
  std::string cellModel;
  std::string cellModelfileName;
  std::string cellModelConstructor;
  std::string odeSolver;
};

int read_json_file(parameters &p);

/**
 * Is the entry point of the actor system
*/
void caf_main(caf::actor_system& system);


/*
  * Creates the actor system
*/
int create_actor_system();
