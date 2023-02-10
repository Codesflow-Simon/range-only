
#include "dataEmulator.h"
#include <fstream>

using namespace std;

int main () {
  DataSource data = Emulator();

  while(true) {
    cout << data.getJson() << endl;
  }
}