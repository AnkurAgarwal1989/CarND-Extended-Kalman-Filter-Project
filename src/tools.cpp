#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth) {
    //We know our state is a 4D vector
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
        std::cout << "Incorrect data for RMSE calculation" << std::endl;
        return rmse;
    }

    for (auto i = 0; i < estimations.size(); ++i) {
        VectorXd error = ground_truth[i] - estimations[i];
        error = error.array()*error.array();
        rmse += error;
    }

    rmse /= estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    /**
    TODO:
      * Calculate a Jacobian here.
    */
    return MatrixXd(3, 3);
}
