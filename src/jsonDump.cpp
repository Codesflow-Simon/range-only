
#include "dataEmulator.h"
#include <fstream>

using namespace std;

int main () {
  Emulator data = Emulator();

  for(int i=0; i<200; i++) {
    cout << data.getJson() << endl;
  }
}