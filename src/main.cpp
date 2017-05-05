#include "uWS/uWS.h"
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <map>

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
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (s.compare("null") == 0) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

#define CAP(x,l,h)  (x > h ? h : x < l ? l : x)

typedef std::pair<double, double> FactorKey;

bool operator < (const FactorKey& p1, const FactorKey& p2) {
  return p1.first < p2.first ? true : p1.second < p2.second;
}
static std::map<FactorKey, double> known_errors;

#define DELTA_P 0.1

class twiddler : public PID
{
public:

  twiddler(double Kp, double Ki, double Kd) :PID(Kp, Ki, Kd)
  {
    p[0] = Kp;
    p[1] = Kd;
  }

  // Compute next update for Kp, Kd
  // Call adjust when running the robot comes to an end (crash / completion of track)
  // Returns true when optimization is completed
  bool optimize()
  {
    std::cout << "Optimze " << mse << ", " << n << std::endl;
    double err = mse / (n*n);
    bool res;

    known_errors.insert(std::pair<FactorKey, double>(FactorKey(Kp, Kd), err));

    res = iterate(err);
    while (known_errors.find(FactorKey(Kp, Kd)) != known_errors.end()) {
      err = known_errors[FactorKey(Kp, Kd)];
      std::cout << "Lookup " << err << std::endl;
      res = iterate(err);
    }
    mse = 0;
    n = 0;

    return res;
  }

  int limit()
  {
    if (dp[0] + dp[1] > 0.2) return 500;
    if (dp[0] + dp[1] > 0.02) return 1000;
    return 2000;
  }

protected:

  bool iterate(double err) {
    std::cout << "adjust(" << err << ")" << std::endl;
    if (state == 0) {
      p[i] += dp[i];
      state = 1;
      apply();
      return false;
    }
    else if (state == 1) {
      if (err < best_err) {
        best_err = err;
        dp[i] *= 1 + DELTA_P;

      }
      else {
        p[i] -= 2 * dp[i];
        state = 2;
        apply();
        return false;
      }
    }
    else if (state == 2) {
      if (err < best_err) {
        best_err = err;
        dp[i] *= 1 + DELTA_P;

      }
      else {
        p[i] += dp[i];
        dp[i] *= 1 - DELTA_P;

      }
    }

    // loop again
    state = 0;
    i++;
    if (i == 2) {
      i = 0;
      apply();
      // termination check
      bool done = (dp[0] + dp[1] < 0.002);
      if (done) {
        // do something 
      }
      return done;
    }

    p[i] += dp[i];
    state = 1;
    apply();
    return false;
  }

  int state = 0;
  int i = 0;
  double best_err = 1e5;
  double p[2] = { 0, 0 };
  double dp[2] = { 1, 1 };

  void apply()
  {
    Kp = p[0];
    Kd = p[1];
    std::cout << "Kp=" << Kp << ", Ki=" << Ki << ", Kd=" << Kd << ", dp: [" << dp[0] << ", " << dp[1] << "]" << std::endl;
  }

};

int main()
{
  uWS::Hub h;


#define NUM_ITERATIONS_PER_RUN  1000
  double target_speed = 40;
  //PID steering_pid(0.122567, 1e-5, 4.56273);
  twiddler steering_pid(0, 0, 0);
  //PID steering_pid(1.22316, 0, 1.63106);
  PID speed_pd(0.12, 0, 0.8);


  h.onMessage([&steering_pid, &speed_pd, &target_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // message from previous run (queued before reset)
          if (steering_pid.n == 0 && abs(cte) > 5) {
            std::cout << "** n=" << steering_pid.n << ", cte=" << abs(cte) << std::endl;
            steer_value = 0;
          }
          else {

            if (steering_pid.n > steering_pid.limit()) { // || abs(cte) > 5 || (steering_pid.n > 10 && speed < 0.1)) {
              std::cout << "n=" << steering_pid.n << ", cte=" << abs(cte) << ", speed=" << speed << std::endl;

              if (steering_pid.optimize()) {
                // Completed - stop ?
              }

              std::string msg = "42[\"reset\"]";

              std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              return;
            }

            // Compute steering
            steer_value = steering_pid.ComputeControl(cte);
            steer_value = CAP(steer_value, -1, 1);
            steering_pid.UpdateError(cte);
          }

          // Compute throttle
          double speed_cte = speed - target_speed;
          double throttle = speed_pd.ComputeControl(speed_cte);
          throttle = CAP(throttle, -1, 1);
          speed_pd.UpdateError(speed_cte);

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << ", error:" << steering_pid.TotalError() << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
        else {
          std::cout << event << std::endl;
        }
      }
      else {
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
  if (h.listen("0.0.0.0", port))
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
