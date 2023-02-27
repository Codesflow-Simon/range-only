/**
 * @brief Adds range measurements to graph. Will call the sample method of the sensor to get the keys and values of the measurements.
 * @param Graph* graph graph to write too
 * @param JsonSensor* sensor that makes measurements to write too
 * @param SharedNoiseModel distance noise
*/
void add_rangeFactors(Graph* graph, DataSource* sensor , SharedNoiseModel distNoise) {
  // Samples from sensors
  map<pair<Key,Key>,double> sample = sensor->sample();
  
  for (auto meas : sample) {  // ID-measurement pair from sample
    std::pair<Key,Key> measKeys = meas.first;

    assert(measKeys.first != 0 && measKeys.second != 0);

    auto factor = RangeFactor<Point3>(measKeys.first, measKeys.second, meas.second, distNoise);
    graph->add(factor);

    write_log("Added DistanceFactor " + keyToString(measKeys.first) + " and " + keyToString(int(measKeys.second)) + " with measurement " + 
               to_string(meas.second) + "\n");
  }
}
