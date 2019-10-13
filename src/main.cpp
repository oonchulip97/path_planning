#include <uWS/uWS.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  enum Lanes
  {
    LEFT,
    MIDDLE,
    RIGHT
  };

  // Starting at middle lane;
  Lanes lane = MIDDLE;

  // Reference velocity
  double ref_vel = 0.0;  // mph

  // Constant
  const double LANE_WIDTH = 4.0;  // m
  const double TIME_INCREMENT = 0.02;  // s
  const double ANCHOR_DIST = 30.0;  // m
  const double MAX_SPEED = 49.5;  // mph
  const double MAX_ACC = 0.1;  // mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &max_s, &lane, &ref_vel,
               &LANE_WIDTH, &TIME_INCREMENT, &ANCHOR_DIST, &MAX_SPEED, &MAX_ACC]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // Previous path size
          size_t prev_size = previous_path_x.size();

          // Prevent collision by using future state
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          /**
           * Prediction Module:
           * Determine whether each of the lane is safe to navigate or not using future state.
           */
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;
          double ahead_dist = std::numeric_limits<double>::infinity();
          double ahead_speed = std::numeric_limits<double>::infinity();
          for (size_t compare_index = 0; compare_index < sensor_fusion.size(); compare_index++)
          {
            // Determine the lane driven by compared car
            double compare_d = sensor_fusion[compare_index][6];
            Lanes compare_lane;
            if (compare_d >= 0 && compare_d < LANE_WIDTH)
              compare_lane = LEFT;
            else if (compare_d >= LANE_WIDTH && compare_d < 2 * LANE_WIDTH)
              compare_lane = MIDDLE;
            else if (compare_d >= 2 * LANE_WIDTH && compare_d <= 3 * LANE_WIDTH)
              compare_lane = RIGHT;
            else
              continue;  // compared car is at other side of the road

            // Estimate compared car's future state after executing previous path
            double compare_vx = sensor_fusion[compare_index][3];
            double compare_vy = sensor_fusion[compare_index][4];
            double compare_speed = sqrt(compare_vx * compare_vx + compare_vy * compare_vy);
            double compare_s = sensor_fusion[compare_index][5];
            compare_s += (double)prev_size * TIME_INCREMENT * compare_speed;
            compare_s = std::fmod(compare_s, max_s);  // loop check

            // Calculate interval bound, using anchor distance as safety margin
            double upper_bound, lower_bound;
            bool close_to_finish = false;
            bool close_to_start = false;
            if (car_s + ANCHOR_DIST < max_s)
            {
              upper_bound = car_s + ANCHOR_DIST;
            }
            else
            {
              upper_bound = std::fmod(car_s + ANCHOR_DIST, max_s);
              close_to_finish = true;
            }
            if (car_s - ANCHOR_DIST > 0)
            {
              lower_bound = car_s - ANCHOR_DIST;
            }
            else
            {
              lower_bound = car_s + max_s - ANCHOR_DIST;
              close_to_start = true;
            }

            // Set flag for state machine
            if (compare_lane == lane)
            {
              if (!close_to_finish)
              {
                car_ahead |= (compare_s < upper_bound) && (compare_s >= car_s);

                if (car_ahead && ((compare_s - car_s) < ahead_dist))
                {
                  ahead_dist = compare_s - car_s;
                  ahead_speed = compare_speed;
                }
              }
              else
              {
                car_ahead |= ((compare_s < upper_bound) && (compare_s >= 0.0)) ||
                             ((compare_s < max_s ) && (compare_s >= car_s));

                if (car_ahead)
                {
                  if (compare_s < upper_bound)
                  {
                    if ((max_s - car_s + compare_s) < ahead_dist)
                    {
                      ahead_dist = max_s - car_s + compare_s;
                      ahead_speed = compare_speed;
                    }
                  }
                  else
                  {
                    if (compare_s - car_s < ahead_dist)
                    {
                      ahead_dist = compare_s - car_s;
                      ahead_speed = compare_speed;
                    }
                  }
                }
              }
            }
            else if (compare_lane - lane == -1)
            {
              if (!close_to_finish)
              {
                car_left |= (compare_s < upper_bound) && (compare_s >= car_s);
              }
              else
              {
                car_left |= ((compare_s < upper_bound) && (compare_s >= 0.0)) ||
                            ((compare_s < max_s) && (compare_s >= car_s));
              }

              if (!close_to_start)
              {
                car_left |= (compare_s <= car_s) && (compare_s > lower_bound);
              }
              else
              {
                car_left |= ((compare_s <= car_s) && (compare_s >= 0.0)) ||
                            ((compare_s < max_s) && (compare_s > lower_bound));
              }
            }
            else if (compare_lane - lane == 1)
            {
              if (!close_to_finish)
              {
                car_right |= (compare_s < upper_bound) && (compare_s >= car_s);
              }
              else
              {
                car_right |= ((compare_s < upper_bound) && (compare_s >= 0.0)) ||
                             ((compare_s < max_s ) && (compare_s >= car_s));
              }

              if (!close_to_start)
              {
                car_right |= (compare_s <= car_s) && (compare_s > lower_bound);
              }
              else
              {
                car_right |= ((compare_s <= car_s) && (compare_s >= 0.0)) ||
                            ((compare_s < max_s) && (compare_s > lower_bound));
              }
            }
          }

          /**
           * Behavior Planning:
           * Adjust target lane based on prediction.
           */
          double speed_diff = 0.0;
          if (car_ahead)
          {
            if (lane != LEFT && !car_left)
              lane = (lane == MIDDLE) ? LEFT : MIDDLE;
            else if (lane != RIGHT && !car_right)
              lane = (lane == MIDDLE) ? RIGHT : MIDDLE;
            else  // cannot change lane, slow down if necessary
              if (ref_vel > ahead_speed)
                speed_diff -= MAX_ACC;
          }
          else
          {
            if ((lane == LEFT && !car_right) || (lane == RIGHT && !car_left))
                lane = MIDDLE;

            if (ref_vel < MAX_SPEED)
              speed_diff += MAX_ACC;
          }

          /**
           * Path Planning:
           * Create spline path using anchor points.
           */
          // Anchor points for making spline
          vector<double> anchor_x;
          vector<double> anchor_y;

          // Reference state
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);  // degrees

          // Set up current anchor points
          if (prev_size < 2)
          {
            double prev_x = car_x - cos(ref_yaw);
            double prev_y = car_y - sin(ref_yaw);

            anchor_x.push_back(prev_x);
            anchor_x.push_back(car_x);
            anchor_y.push_back(prev_y);
            anchor_y.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            anchor_x.push_back(ref_x_prev);
            anchor_x.push_back(ref_x);
            anchor_y.push_back(ref_y_prev);
            anchor_y.push_back(ref_y);
          }

          // Set up future anchor points
          vector<double> next_wp0 = getXY(car_s + ANCHOR_DIST, 2 + 4 * lane, map_waypoints_s, map_waypoints_x,
                                          map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 2 * ANCHOR_DIST, 2 + 4 * lane, map_waypoints_s, map_waypoints_x,
                                          map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 3 * ANCHOR_DIST, 2 + 4 * lane, map_waypoints_s, map_waypoints_x,
                                          map_waypoints_y);

          anchor_x.push_back(next_wp0[0]);
          anchor_x.push_back(next_wp1[0]);
          anchor_x.push_back(next_wp2[0]);

          anchor_y.push_back(next_wp0[1]);
          anchor_y.push_back(next_wp1[1]);
          anchor_y.push_back(next_wp2[1]);

          // Transform anchor points from global map coordinates to local car coordinates to prevent vertical line
          for (size_t i = 0; i < anchor_x.size(); i++)
          {
            double trans_x = anchor_x[i] - ref_x;
            double trans_y = anchor_y[i] - ref_y;

            anchor_x[i] = trans_x * cos(ref_yaw) + trans_y * sin(ref_yaw);
            anchor_y[i] = - trans_x * sin(ref_yaw) + trans_y * cos(ref_yaw);
          }

          // Create spline
          tk::spline spl;
          spl.set_points(anchor_x, anchor_y);

          // Output path points from previous path for continuity
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (size_t i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Output path points from spline
          const double target_x = 30.0;  // m
          double target_y = spl(target_x);
          double target_dist = distance(target_x, target_y, 0.0, 0.0);
          double x_add_on = 0.0;
          for (size_t i = 1; i < 50 - prev_size; i++)
          {
            ref_vel += speed_diff;
            ref_vel = std::max(1.0, std::min(MAX_SPEED, ref_vel));

            // Calculate interval spacing
            double N = target_dist / (TIME_INCREMENT * ref_vel / 2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = spl(x_point);
            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;
            // Transform path points from local car coordinate to global map coordinate
            x_point = x_ref * cos (-ref_yaw) + y_ref * sin(-ref_yaw);
            y_point = - x_ref * sin (-ref_yaw) + y_ref * cos(-ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
