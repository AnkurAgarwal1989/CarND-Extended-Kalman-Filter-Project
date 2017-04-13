#include<iostream>
#include"tools.h"
#include <vector>
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;
void noDataCheck_RMSE(Tools &tools) {
  vector<VectorXd> a;
  vector<VectorXd> b;

  tools.CalculateRMSE(a, b);
}

void sameDataCheck_RMSE(Tools &tools) {
  vector<VectorXd> a;
  vector<VectorXd> b;
  VectorXd data(4);
  data << 1.0, 2.0, 3.0, 4.0;
  for (int i = 0; i < 3; ++i) {
    a.push_back(data);
    b.push_back(data);
  }
  VectorXd rmse = tools.CalculateRMSE(a, b);

  cout << rmse << endl;
}

void validCalcCheck_RMSE(Tools &tools) {
  vector<VectorXd> a;
  vector<VectorXd> b;
  VectorXd data_a(4);
  data_a << 0, 0, 0, 0;
  VectorXd data_b(4);
  data_b << 1.3, 4.5, 3.5, 9.0;
  a.push_back(data_a);
  b.push_back(data_b);

  VectorXd rmse = tools.CalculateRMSE(a, b);
  if (rmse != data_b)
    cout << "RMSE calculation is not valid" <<endl;
  cout << rmse << endl;
}

void validCalcCheck_Jacobian(Tools &tools) {
  VectorXd state(4);
  state << 3, 5, 1.2, 2.4;

  MatrixXd Hj = tools.CalculateJacobian(state);

  cout << "Hj:" << endl << Hj << endl;
}

void runToolsTests() {
  Tools tools_test;
  cout << "No data check" << endl;
  noDataCheck_RMSE(tools_test);

  cout << "Same data check" << endl;
  sameDataCheck_RMSE(tools_test);

  cout << "Valid Calculation check for RMSE" << endl;
  validCalcCheck_RMSE(tools_test);

  cout << "Valid Calculation check for Jacobian" << endl;
  validCalcCheck_Jacobian(tools_test);
}