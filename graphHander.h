#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/FastVector.h>

using namespace std;
// For marginalising variables far back in past
class GraphHandler {
    private:
        int maxLength
        vector<Graph*> graphs
}