
#include "dataEmulator.h"
#include "sensor.h"
#include <fstream>
#include <nlohmann/json.hpp>

using namespace std;

int main () {
  DataSource* dataSource = new Emulator();
  JsonSensor* sensor = new JsonSensor(dataSource);

  sensor->sampleA2a();
  for(int i=0; i<205; i++) {
    cout << json(sensor->sample()) << endl;
  }
}