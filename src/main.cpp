#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "MPC.h"
#include "json.hpp"
#include <time.h>

// for convenience
using json = nlohmann::json;
const double Lf = 2.67;
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}
Eigen::VectorXd cap_to_map(double xp, double yp, double psi, double xc, double yc){
  Eigen::VectorXd xy_c(3);
  xy_c << xc, yc, 1;
  Eigen::MatrixXd M(3, 3);
  M << cos(psi), -sin(psi), xp,
    sin(psi), cos(psi), yp,
    0, 0, 1;
  return M * xy_c;
}

Eigen::VectorXd map_to_car(double xp, double yp, double psi, double xm, double ym){
  //Eigen::VectorXd xy_m(3);
  //xy_m << xm, ym, -1;
  //Eigen::MatrixXd M(3, 3);
  //M << cos(psi), -sin(psi), xp,
  //  sin(psi), cos(psi), yp,
  //  0, 0, 1;
  //return M.colPivHouseholderQr().solve(xy_m);
  Eigen::VectorXd xy_m(2);
  xy_m << xm-xp, ym-yp;
  Eigen::MatrixXd M(2, 2);
  M << cos(-psi), -sin(-psi),
    sin(-psi), cos(-psi);
  return M *xy_m;
}

const double mph_to_ms(0.44704);
  /* code */

unsigned long getTime()
{
  struct timeval detail_time;
  gettimeofday(&detail_time,NULL);
  return (detail_time.tv_usec); /* microseconds */
}

int main(int argc, char* argv[])
{
  uWS::Hub h;
  size_t N;
  double dt;
  double latency;
  std::vector<double> weight(7);
  if (argc!=11){
    // The parameter setting is hard-coded as default here
    N=10;
    dt=0.2;
    latency=0.2;
    weight[0]=2;
    weight[1]=1;
    weight[2]=10;
    weight[3]=1000;
    weight[4]=1000;
    weight[5]=1000;
    weight[6]=1000;
  } else {
    // take in command line argument as the parameter setting. This is used to avoid rebuliding the code each time during parameter optimization.
    std::cout << "command line" << std::endl;
    N=atoi(argv[1]);
    dt=atof(argv[2]);
    latency=atof(argv[3]);
    for (size_t i=0; i<7; i++){
      weight[i] = atof(argv[i+4]);
    }

  }

  // MPC is initialized here!


  MPC mpc(N, dt, weight);

  h.onMessage([&mpc, latency](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          //std::cout << "ptsx size: " << ptsx.size() << std::endl;
          //time_t timer=time(NULL);
          //double seconds = difftime(timer, timer_0);
          std::cout << "t: " << getTime() << std::endl;

          double px0 = j[1]["x"];
          double py0 = j[1]["y"];
          double psi0 = j[1]["psi"];
          double v0 = (double)j[1]["speed"] * mph_to_ms;
          //double v0 = (double)j[1]["speed"];

          double delta0 = -(double)j[1]["steering_angle"];
          //delta0 = (-delta0);
          double a0 = j[1]["throttle"];
          //v0 *= mph_to_ms;
          std::cout << "v0 " << v0 << " delta0 " << delta0 << " a0 " << a0 << std::endl;
          // account for latency
          double px = px0 + v0 * cos(psi0) * latency;
          double py = py0 + v0 * sin(psi0) * latency;
          double psi = psi0 + v0 * delta0 / Lf * latency;
          double v = v0 + a0 * latency;

          //std::cout << "px:" << px << " py:" << py << " psi:" << psi << " v:" << v << " angle:" << current_steering_angle << " throttle:" << current_throttle << std::endl;

          // Convert to Vehicle coordinate
          Eigen::VectorXd ptsx_c(ptsx.size());
          Eigen::VectorXd ptsy_c(ptsy.size());
          for (size_t i=0; i<ptsx.size(); i++){
            Eigen::VectorXd xy_c = map_to_car(px, py, psi, ptsx[i], ptsy[i]);
            ptsx_c[i] = xy_c[0];
            ptsy_c[i] = xy_c[1];
          }
          //std::cout << "ptsx_c:" << ptsx_c << " ptsy_c:" << ptsy_c << std::endl;
          double px_c = 0;
          double py_c = 0;
          double psi_c = 0;

          Eigen::VectorXd coeffs = polyfit(ptsx_c, ptsy_c, 3);
          //std::cout << "coef: " << coeffs << std::endl;
          double cte = py_c - polyeval(coeffs, px_c);
          // TODO: calculate the orientation error
          double epsi = psi_c - atan(coeffs[1] + 2 * coeffs[2] * px_c + 3 * coeffs[3] * px_c * px_c);
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          Eigen::VectorXd state(6);
          state << px_c, py_c, psi_c, v, cte, epsi;
          //std::cout << "state: " << state << std::endl;
          std::vector<double> vars = mpc.Solve(state,  coeffs);
          std::cout << "var: " << vars[0] <<" " <<  vars[1] << std::endl;

          double steer_value = vars[0];
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -(steer_value / deg2rad(25));
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          size_t output_size = (vars.size()-2)/3;
          vector<double> mpc_x_vals(output_size);
          vector<double> mpc_y_vals(output_size);
          //vector<double> psi_vals(output_size);
          for (size_t i=0; i < output_size; i++){
            mpc_x_vals[i] = vars[2+i*3];
            mpc_y_vals[i] = vars[2+i*3+1];
            //psi_vals[i] = vars[2+i*3+2];
          }
          //for (size_t i=0; i < output_size; i++){
            //mpc_x_vals[i] = i*10; //vars[2+i*3];
            //mpc_y_vals[i] = 0; //vars[2+i*3+1];
            //Eigen::VectorXd xy_new_car_coordinate = cap_to_map(0, 0, delta0 * latency, i*10, 0);
            //mpc_x_vals[i] = xy_new_car_coordinate[0]; //vars[2+i*3];
            //mpc_y_vals[i] = xy_new_car_coordinate[1]; //vars[2+i*3+1];

            //psi_vals[i] = vars[2+i*3+2];
          //}
          //double delayed_x=mpc_x_vals[0];
          //double delayed_y=mpc_y_vals[0];
          //double delayed_psi=psi_vals[0];
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals(ptsx_c.size());
          vector<double> next_y_vals(ptsx_c.size());
          for (size_t i=0; i < ptsx_c.size(); i++){
            //Eigen::VectorXd xy_new_car_coordinate = map_to_car(0, 0, 0, ptsx_c[i], ptsy_c[i]);
            next_x_vals[i] = ptsx_c[i];
            next_y_vals[i] = ptsy_c[i];
          }
          //for (size_t i=0; i<ptsx.size(); i++){
          //  Eigen::VectorXd xy_c = map_to_car(px, py, psi, ptsx[i], ptsy[i]);
          //  ptsx_c[i] = xy_c[0];
          //  ptsy_c[i] = xy_c[1];
          //}
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
