#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include "Helper.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// NOTE: A safe speed limit 😀 is between 50-70 mph.
const double SPEED_LIMIT = 65.0;  // mph

int main()
{
  uWS::Hub h;

  PID steering_pid;
  // Steering PID init params
  double  Kp = 0.13;
  double  Ki = 0.0002;
  double  Kd = 2.0;
  // Throttle PID and params
  PID throttle_pid;
  double  TKp = 0.12;
  double  TKi = 0.00001;
  double  TKd = 2.5;


  std::cout << "Initializing steering_pid and throttle_pid..." << std::endl;
  // TODO: Initialize the pid variable.
  steering_pid.Init(Kp, Ki, Kd);
  throttle_pid.Init(TKp, TKi, TKd);

  h.onMessage([&steering_pid, &throttle_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // 1 - j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          double speed_cte =  speed - SPEED_LIMIT;

          double steer_value;     // calculated Streering angle
          double throttle_value;  // calculated Throttle speed
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // 2.1 - Update Steering CTE
          steering_pid.UpdateError(cte);
          // 2.2 - Get new steer value based on CTE
          steer_value = steering_pid.TotalError();  
          // smoothly clip the steering value: 
          // NOTE: This smoothens out the rough/jerky motion of the vehicle
          // NOTE: Even though the steering values are [-1, 1], we sometimes need sharper angles, 
          // hence the values are slightly more than [-1, 1]
          steer_value = sigmoid(steer_value, 1.25, -1.25);  

          // 3.1 - Update Throttle CTE
          throttle_pid.UpdateError(speed_cte);
          throttle_value  = throttle_pid.TotalError();
          // smoothly clip the throttle values
          throttle_value = sigmoid(throttle_value, 1.0, -1.0);
          
          // DEBUG
          std::cout << "STEERING CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << "SPEED    CTE: " << speed_cte << "  Throttle Value: " << throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;       // ORIG:  0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
