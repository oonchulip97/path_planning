#include "vehicle.h"

#include "state_machine.h"

#include <cmath>
#include <utility>
#include <vector>

Vehicle::Vehicle(int id, double s, double vel_s, double accel_s, double d, double vel_d, double accel_d)
  : id_(id),
    s_(s),
    vel_s_(vel_s),
    accel_s_(accel_s),
    d_(d),
    vel_d_(vel_d),
    accel_d(accel_d),
{
   // TODO(CHULIP): Implement conversion to map coordinates here
   // TODO(CHULIP): Initialize other variables missing
   // TODO(CHULIP): Implement lane check here
}

std::vector<std::pair<<double, Vehicle>> Vehicle::generate_predictions(int horizon, int counts);
{
  // Initialize prediction using current state
  std::vector<std::pair<double,Vehicle>> predictions;
  predictions.push_back(std::pair<double, Vehicle>(0.0, *this));

  // Predict until horizon
  // Use constant velocity model in Frenet's coordinates
  for (size_t i = 1; i <= counts; i++)
  {
    double future_time = (double)i / counts  * horizon;
    Vehicle future_vehicle = states_after(future_time);
    predictions.push_back(std::pair<double, Vehicle>(future_time, future_vehicle));
  }

  return predictions;
}

std::pair<double, Vehicle> generate_endpoint(std::vector<std::vector<std::pair<double, Vehicle>>> predictions)
{
  std::pair<double, Vehicle> endpoint;

  if (current_state_->get_state() == KEEP_LANE)
  {
    endpoint = keep_lane_endpoint(predictions);
  }
  else if (current_state_->get_state() == PREPARE_LANE_CHANGE_LEFT ||
           current_state_->get_state() == PREPARE_LANE_CHANGE_RIGHT)
  {
    // TODO(CHULIP): Implement prepare lane change endpoints
  }
  else if (current_state_->get_state() == LANE_CHANGE_LEFT ||
           current_state_->get_state() == LANE_CHANGE_RIGHT)
  {
    // TODO(CHULIP): Implement lane change endpoints
  }

  return endpoint;
}

vector<double> Vehicle::get_lane_kinematics(Lanes lane, std::vector<std::vector<std::pair<double, Vehicle>>> predictions)
{
  double end_s;
  double end_vel_s;
  double end_accel_s;
  double distance_to_check = 30.0;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(distance_to_check, lane, predictions, vehicle_ahead))
  {
    if (get_vehicle_behind(distance_to_check, lane, predictions, vehicle_behind))
    {
      end_vel_s = vehicle_ahead.get_vel_s(); // TODO(CHULIP): Implement as class method
    }
    else
    {
      end_s = vehicle_ahead.get_s() - buffer_distance_; // TODO(CHULIP): Implement as class attribute
      end_vel_s = std::min(vehicle_ahead.get_vel_s(), target_vel_); // TODO(CHULIP): Implement as class method
      end_accel_s = 0.0;
    }
  }
  else
  {
    end_s = ;
    end_vel_s = target_vel;  // TODO(CHULIP): Implement as class attribute
    end_accel_s = 
  }

  return {end_s, end_vel_s, 0};
}

bool Vehicle::get_vehicle_ahead(double distance,
                                Lanes lane,
                                std::vector<std::vector<std::pair<double, Vehicle>>> predictions,
                                Vehicle& vehicle_ahead);
{
  bool found_vehicle = false;
  double min_distance = distance;
  Vehicle aux_vehicle;

  for (std::vector<std::vector<std::pair<double, Vehicle>>>::iterator it = predictions.begin(),
       it != predictions.end(),
       it++)
  {
    aux_vehicle = it->at(0)->second;
    // TODO(CHULIP): Implement loop checking
    if (aux_vehicle.get_lane() == lane &&
        aux_vehicle.get_s() > s_ &&
        (aux_vehicle.get_s() - s_) < min_distance)
    {
      min_distance = aux_vehicle.get_s() - s_;
      vehicle_ahead = aux_vehicle;
      found_vehicle = true;
    }
  }

  return found_vehicle;
}

bool Vehicle::get_vehicle_behind(double distance,
                                 Lanes lane,
                                 std::vector<std::vector<std::pair<double, Vehicle>>> predictions,
                                 Vehicle& vehicle_behind);
{
  bool found_vehicle = false;
  double min_distance = distance;
  Vehicle aux_vehicle;

  for (std::vector<std::vector<std::pair<double, Vehicle>>>::iterator it = predictions.begin(),
       it != predictions.end(),
       it++)
  {
    aux_vehicle = it->at(0)->second;
    // TODO(CHULIP): Implement loop checking
    if (aux_vehicle.get_lane() == lane &&
        s_ > aux_vehicle.get_s() &&
        (s_ - aux_vehicle.get_s()) < min_distance)
    {
      min_distance = s_ - aux_vehicle.get_s();
      vehicle_behind = aux_vehicle;
      found_vehicle = true;
    }
  }

  return found_vehicle;
}

Vehicle states_after(float duration)
{
  // TODO(CHULIP): Implement loop checking
  double future_s = s_ + vel_s_ * duration + 0.5 * accel_s_ * pow(duration, 2);
  double future_vel_s = vel_s_ * duration;
  double future_accel_s = accel_s_;
  double future_d = d_ + vel_d_ * duration + 0.5 * accel_d_ * pow(duration, 2);
  double future_vel_d = vel_d_ * duration;
  double future_accel_d = accel_d_;

  return Vehicle(id_, future_s, future_vel_s, future_accel_s, future_d, future_vel_d, future_accel_d);
}

Lanes Vehicle::get_lane()
{
  return current_lane_;
}

double Vehicle::get_s()
{
  return s_;
}
