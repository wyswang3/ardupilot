#pragma once

#include <vector>
#include <algorithm>

class MPC_Controller {
public:
    // Constructor
    MPC_Controller();

    // Initialize the controller with system matrices, prediction horizon, and sampling time
    void init(int prediction_horizon, double sampling_time, const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B, const std::vector<std::vector<double>>& Q, const std::vector<std::vector<double>>& R);

    // Set the desired state for the MPC to track
    void set_desired_state(const std::vector<double>& state);

    // Update the current state of the vehicle
    void update_current_state(const std::vector<double>& state);

    // Calculate control outputs based on the desired state and current state
    void calculate_control(double& roll_output, double& pitch_output, double& yaw_output, double& throttle_output);

    // Set control gains for the MPC controller
    void set_control_gains(const std::vector<double>& gains);

    // Set control bounds for the outputs (min and max values)
    void set_control_bounds(const std::vector<double>& bounds);

private:
    // Current state of the vehicle
    std::vector<double> current_state;

    // Desired state for the vehicle to track
    std::vector<double> desired_state;

    // Control gains for the MPC controller
    std::vector<double> control_gains;

    // Control bounds for the outputs (min and max values)
    std::vector<double> control_bounds;  // {lower_bound, upper_bound}

    // System matrices for the model predictive control
    std::vector<std::vector<double>> A;  // State transition matrix
    std::vector<std::vector<double>> B;  // Control input matrix
    std::vector<std::vector<double>> Q;  // State cost matrix
    std::vector<std::vector<double>> R;  // Control cost matrix

    // Prediction horizon and sampling time
    int prediction_horizon;
    double sampling_time;
};
