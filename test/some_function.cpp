#include "some_function.hpp"

using namespace caf;
using json = nlohmann::json;

int read_json_file(parameters& p) {
  std::ifstream fin("/code/Chaste/projects/chaste-actors/test/parameters.json");

  // Check if the file was opened successfully
  if (!fin) {
      std::cerr << "Error opening file" << std::endl;
      return -1;
  }

  // Parse the JSON file
  json j;
  fin >> j;

  // Read in parameters
  p.numDigits = j["numDigits"];
  p.numTimeSteps = j["numTimeSteps"];
  p.dtTissue = j["dtTissue"];
  p.dtCell = j["dtCell"];
  p.mesh = j["mesh"];
  p.timeDuration = j["timeDuration"];
  p.stimulusCurrent = j["stimulusCurrent"];
  p.stimulusDuration = j["stimulusDuration"];
  p.stimulusStarttime = j["stimulusStarttime"];
  p.stimulusRange_x = j["stimulusRange_x"];
  p.stimulusRange_y = j["stimulusRange_y"];
  p.stimulusRange_z = j["stimulusRange_z"];
  p.electrodeposition_x = j["electrodeposition_x"];
  p.electrodeposition_y = j["electrodeposition_y"];
  p.electrodeposition_z = j["electrodeposition_z"];
  p.sigma_ix = j["sigma_ix"];
  p.sigma_iy = j["sigma_iy"];
  p.sigma_iz = j["sigma_iz"];
  p.sigma_ex = j["sigma_ex"];
  p.sigma_ey = j["sigma_ey"];
  p.sigma_ez = j["sigma_ez"];
  p.Cm = j["Cm"];
  p.Chi = j["Chi"];
  p.xLength = j["xLength"];
  p.yLength = j["yLength"];
  p.zLength = j["zLength"];
  p.cellModel = j["cellModel"];
  p.cellModelfileName = j["cellModelfileName"];
  p.cellModelConstructor=  j ["cellModelConstructor"]; 
	p.odeSolver= 	j ["odeSolver"];

  // Close the file
  fin.close();

	return 0;
}


void caf_main(actor_system& system) {
  scoped_actor self{system};

  parameters p;

  read_json_file(p);
  
  aout(self) << "Hello World From Actor Land" << std::endl;

}


int create_actor_system() {

  // Needed as arguments to the exec main function
  int argc = 0;
  char** argv = nullptr;

  core::init_global_meta_objects();
  return exec_main<>(caf_main, argc, argv);
}
