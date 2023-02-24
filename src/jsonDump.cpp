
#include "dataEmulator.h"
#include "realSource.h"
#include "sensor.h"
#include <fstream>
#include <nlohmann/json.hpp>

using namespace std;
typedef nlohmann::json json;


int main () {

  std::ifstream f("../params.json");
  json parameters = json::parse(f);

  RealSource* dataSource = new RealSource(parameters["source"]);
  JsonSensor* sensor = new JsonSensor(dataSource);

  // sensor->sampleA2a();
  for(int i=0; i<205; i++) {
    cout << json(dataSource->getJson()) << endl;
  }
}