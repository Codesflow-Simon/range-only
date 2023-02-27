
#include "data_emulator.h"
#include "data_real.h"
#include <fstream>
#include <nlohmann/json.hpp>

using namespace std;
typedef nlohmann::json json;


int main () {

  std::ifstream f("../params.json");
  json parameters = json::parse(f);

  RealSource* source = new RealSource(parameters["source"]);

  // sensor->sampleA2a();
  for(int i=0; i<205; i++) {
    cout << json(source->getJson()) << endl;
  }
}