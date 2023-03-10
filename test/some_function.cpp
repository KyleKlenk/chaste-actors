#include "some_function.hpp"

using namespace caf;
using json = nlohmann::json;

int read_json_file() {
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
  int numDigits = j["numDigits"];
  int numTimeSteps = j["numTimeSteps"];
  std::string dtTissue = j["dtTissue"];
  std::string dtCell = j["dtCell"];
  std::string mesh = j["mesh"];
  double timeDuration = j["timeDuration"];
  std::string stimulusCurrent = j["stimulusCurrent"];
  double stimulusDuration = j["stimulusDuration"];
  double stimulusStarttime = j["stimulusStarttime"];
  double stimulusRange_x = j["stimulusRange_x"];
  double stimulusRange_y = j["stimulusRange_y"];
  double stimulusRange_z = j["stimulusRange_z"];
  double electrodeposition_x = j["electrodeposition_x"];
  double electrodeposition_y = j["electrodeposition_y"];
  double electrodeposition_z = j["electrodeposition_z"];
  double sigma_ix = j["sigma_ix"];
  double sigma_iy = j["sigma_iy"];
  double sigma_iz = j["sigma_iz"];
  double sigma_ex = j["sigma_ex"];
  double sigma_ey = j["sigma_ey"];
  double sigma_ez = j["sigma_ez"];
  int Cm = j["Cm"];
  int Chi = j["Chi"];
  double xLength = j["xLength"];
  double yLength = j["yLength"];
  double zLength = j["zLength"];
  std::string cellModel = j["cellModel"];
  std::string cellModelfileName = j["cellModelfileName"];
  std::string cellModelConstructor=  j ["cellModelConstructor"]; 
	std::string odeSolver= 	j ["odeSolver"];

  std::cout << "numDigits: " << numDigits << std::endl;
  std::cout << "numTimeSteps: " << numTimeSteps << std::endl;
  std::cout << "dtTissue: " << dtTissue << std::endl;
  std::cout << "dtCell: " << dtCell << std::endl;
  std::cout << "mesh: " << mesh << std::endl;
  std::cout << "timeDuration: " << timeDuration << std::endl;
  std::cout << "stimulusCurrent: " << stimulusCurrent << std::endl;
  std::cout << "stimulusDuration: " << stimulusDuration << std::endl;
  std::cout << "stimulusStarttime: " << stimulusStarttime << std::endl;
  std::cout << "stimulusRange_x: " << stimulusRange_x << std::endl;
  std::cout << "stimulusRange_y: " << stimulusRange_y << std::endl;
  std::cout << "stimulusRange_z: " << stimulusRange_z << std::endl;
  std::cout << "electrodeposition_x: " << electrodeposition_x << std::endl;
  std::cout << "electrodeposition_y: " << electrodeposition_y << std::endl;
  std::cout << "electrodeposition_z: " << electrodeposition_z << std::endl;
  std::cout << "sigma_ix: " << sigma_ix << std::endl;
  std::cout << "sigma_iy: " << sigma_iy << std::endl;
  std::cout << "sigma_iz: " << sigma_iz << std::endl;
  std::cout << "sigma_ex: " << sigma_ex << std::endl;
  std::cout << "sigma_ey: " << sigma_ey << std::endl;
  std::cout << "sigma_ez: " << sigma_ez << std::endl;
  std::cout << "Cm: " << Cm << std::endl;
  std::cout << "Chi: " << Chi << std::endl;
  std::cout << "xLength: " << xLength << std::endl;
  std::cout << "yLength: " << yLength << std::endl;
  std::cout << "zLength: " << zLength << std::endl;
  std::cout << "cellModel: " << cellModel << std::endl;
  std::cout << "cellModelfileName: " << cellModelfileName << std::endl;
  std::cout << "cellModelConstructor: " << cellModelConstructor << std::endl;
  std::cout << "odeSolver: " << odeSolver << std::endl;



	// Use the variables in your program
	// ...

	return 0;
}


void caf_main(actor_system& system) {
  scoped_actor self{system};
  read_json_file();
  aout(self) << "Hello World From Actor Land" << std::endl;

}


int create_actor_system() {

  // Needed as arguments to the exec main function
  int argc = 0;
  char** argv = nullptr;

  core::init_global_meta_objects();
  return exec_main<>(caf_main, argc, argv);
}
