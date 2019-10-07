#ifndef VEHICLE_H
#define VEHICLE_H

#include "state_machine.h"

#include <utility>
#include <vector>

enum Lanes
{
  LEFT,
  MIDDLE,
  RIGHT
};

class Vehicle
{
public:
  /**
   * Constructor in Frenet's coordinates.
   */
  Vehicle(int id, double s, double vel_s, double accel_s, double d, double vel_d, double accel_d);

  /**
   * Generate predictions of vehicle based on constant velocity model along the lane.
   *
   * @param[horizon] prediction horizon in seconds
   * @param[counts] number of predictions within the horizon (excluding initial)
   * @return future states of the vehicle with timestamp
   */
  std::vector<std::pair<double, Vehicle>> generate_predictions(int horizon, int counts);

  /**
   * Generate endpoint for a trajectory
   *
   * @param[predictions] predicted trajectory of other vehicles
   * @return the endpoint of the trajectory in the form of required time to reach and vehicle state
   */
  std::pair<double, Vehicle> generate_endpoint(std::vector<std::vector<std::pair<double, Vehicle>>> predictions);

  /**
   * Determine whether a vehicle is ahead of ego vehicle in the given lane
   *
   * @param[distance] distance to check in meters
   * @param[lane] lane to check
   * @param[predictions] predicted trajectory of other vehicles
   * @param[vehicle_ahead] vehicle that is ahead of ego vehicle, only valid when function return true
   * @return true if a vehicle is ahead, false otherwise
   */
  bool get_vehicle_ahead(double distance,
                         Lanes lane,
                         std::vector<std::vector<std::pair<double, Vehicle>>> predictions,
                         Vehicle& vehicle_ahead);

  /**
   * Determine whether a vehicle is ahead of ego vehicle in the given lane
   *
   * @param[distance] distance to check in meters
   * @param[lane] lane to check
   * @param[predictions] predicted trajectory of other vehicles
   * @param[vehicle_ahead] vehicle that is behind of ego vehicle, only valid when function return true
   * @return true if a behind is ahead, false otherwise
   */
  bool get_vehicle_behind(double distance,
                          Lanes lane,
                          std::vector<std::vector<std::pair<double, Vehicle>>> predictions,
                          Vehicle& vehicle_behind);

  /**
   * Generate vehicle with new states after moving for a certain duration from now.
   * Vehicle is assumed to move with constant velocity model along the lane.
   *
   * @param[duration] duration in seconds
   * @return state of Vehicle
   */
  Vehicle states_after(float duration);

  /**
   * Getter function for lane;
   *
   * @return current lane
   */
  Lanes get_lane();

  /**
   * Getter function for s;
   *
   * @return current s;
   */
  double get_s();

private:
  int id_;  // id of the Vehicle
  double s_;  // longitudinal position in Frenet's coordinates
  double d_;  // lateral position in Frenet's coordinates
  double vel_s_;  // longitudinal velocity in Frenet's coordinates
  double vel_d_;  // lateral velocity in Frenet's coordinates
  double accel_s_;  // longitudinal acceleration in Frenet's coordinates
  double accel_d_;  // lateral acceleration in Frenet's coordinates
  Lanes current_lane_;  // current lane of the vehicle
  StateMachine current_state_;  // current state of the vehicle
};

#endif // VEHICLE_H
