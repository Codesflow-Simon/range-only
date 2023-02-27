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

/**
 * @brief Adds projection factors between the tag and camera to the provided graph
 * @param Graph* graph graph to write too]
 * @param list<CameraWrapper*> cameras
 * @param Anchor tag
 * @param Key key of the tag
 * @param SharedNoiseModel noise model
*/
void add_cameraFactors(Graph* graph, list<CameraWrapper*> cameras, Anchor tag, Key tagKey, SharedNoiseModel projNoise) {
  write_log("adding GenericProjectionFactors\n");
  int i=0;
  for (auto camera : cameras) {
    Point2 measurement = camera->sample(tag.location);

    write_log("Camera " + vecToString(camera->getCamera()->pose().translation()));
    write_log("Measured: (" + to_string(measurement.x()) + ", " + to_string(measurement.y()) + ")\n\n");

    auto factor = GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurement, projNoise, Symbol('c', i++), tagKey, camera->getParams());
    graph->add(factor);
  }