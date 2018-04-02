#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
const double ref_v = 40;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  size_t x_start;
  size_t y_start;
  size_t psi_start;
  size_t v_start;
  size_t cte_start;
  size_t epsi_start;
  size_t delta_start;
  size_t a_start;
  size_t N;
  double dt;
  // weight of cost function terms
  double w_cte;
  double w_epsi;
  double w_v;
  // weight of contraint
  double w_delta;
  double w_a;
  double w_d_delta;
  double w_d_a;
  FG_eval(Eigen::VectorXd coeffs, size_t N, double dt, std::vector<double> weight) {

    this->coeffs = coeffs;
    this->N = N;
    this->dt = dt;
    this->x_start = 0;
    this->y_start = x_start + N;
    this->psi_start = y_start + N;
    this->v_start = psi_start + N;
    this->cte_start = v_start + N;
    this->epsi_start = cte_start + N;
    this->delta_start = epsi_start + N;
    this->a_start = delta_start + N - 1;
    if (weight.size()!= 7){
      throw std::invalid_argument( "weight should be of size 7" );
    }
    this->w_cte = weight[0];
    this->w_epsi = weight[1];
    this->w_v = weight[2];
    this->w_delta = weight[3];
    this->w_a = weight[4];
    this->w_d_delta = weight[5];
    this->w_d_a = weight[6];
   }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // fg[0] is the cost function for optimization
    fg[0] = 0;
    for (size_t t = 0; t< N; t++){
      fg[0] += w_cte*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += w_epsi*CppAD::pow(vars[epsi_start + t], 2);
      //fg[0] -= w_v*CppAD::log(1+vars[v_start + t]);
      fg[0] += w_v*CppAD::pow(vars[v_start + t]-ref_v, 2);

    }

    for (size_t t = 0; t< N-1; t++){
      fg[0] += w_delta*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += w_a*CppAD::pow(vars[a_start + t], 2);
    }

    for (size_t t = 0; t< N-2; t++){
      fg[0] += w_d_delta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += w_d_a * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // for the starting value to be the same as input
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (size_t t = 1; t < N; t++) {
      AD<double> x1 = vars[x_start + t];
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y1 = vars[y_start + t];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> psi0 = vars[psi_start + t -1];
      AD<double> v1 = vars[v_start + t];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> cte0 = vars[cte_start + t -1];
      AD<double> epsi1 = vars[epsi_start + t];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psi_desired = CppAD::atan(coeffs[1] + 2* coeffs[2] * x0 + 3* coeffs[3] * CppAD::pow(x0, 2));

      fg[1 + x_start + t] = x1- (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1- (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1- (psi0 + v0 * delta0 /Lf * dt);
      fg[1 + v_start + t] = v1- (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - (y0 - f0 + v0 *  CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - (psi0 -  psi_desired + (v0 * delta0 / Lf * dt));
    }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC(size_t N, double dt, std::vector<double> weight){
  this->N = N;
  this->dt = dt;
  this->x_start = 0;
  this->y_start = x_start + N;
  this->psi_start = y_start + N;
  this->v_start = psi_start + N;
  this->cte_start = v_start + N;
  this->epsi_start = cte_start + N;
  this->delta_start = epsi_start + N;
  this->a_start = delta_start + N - 1;
  this->weight = weight;
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  //std::cout << "MPC Solve start " << state << std::endl;
  // Set the number of model variables
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Set the number of contraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial state
  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[cte_start] = state[4];
  vars[epsi_start] = state[5];


  //std::cout << "Bound" << vars.size() << std::endl;
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // constrints for delta values
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -25 * M_PI / 180.0;
    vars_upperbound[i] = 25 * M_PI / 180.0;
  }

  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_lowerbound[psi_start] = state[2];
  constraints_lowerbound[v_start] = state[3];
  constraints_lowerbound[cte_start] = state[4];
  constraints_lowerbound[epsi_start] = state[5];

  constraints_upperbound[x_start] = state[0];
  constraints_upperbound[y_start] = state[1];
  constraints_upperbound[psi_start] = state[2];
  constraints_upperbound[v_start] = state[3];
  constraints_upperbound[cte_start] = state[4];
  constraints_upperbound[epsi_start] = state[5];


  // account for delay


  //std::cout << "state size" << state.size() << std::endl;
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, N, dt, weight);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  //auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;
  vector<double> sol_vec(2+3*N);
  //std::cout << "size" << solution.x.size() << std::endl;
  sol_vec[0]=solution.x[delta_start];
  sol_vec[1]=solution.x[a_start];
  for (size_t i=0; i<N; i++){
    sol_vec[2+i*3] = solution.x[x_start+i];
    sol_vec[2+i*3+1] = solution.x[y_start+i];
    sol_vec[2+i*3+2] = solution.x[psi_start+i];
  }

  return sol_vec;
  //{solution.x[delta_start], solution.x[a_start]};
}
