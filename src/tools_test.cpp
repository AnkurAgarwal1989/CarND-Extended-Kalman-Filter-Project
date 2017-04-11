#include<iostream>
#include"tools.h"
#include <vector>
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;
void noDataCheck(Tools &tools) {
    vector<VectorXd> a;
    vector<VectorXd> b;

    tools.CalculateRMSE(a, b);
}

void validCalcCheck(Tools &tools) {
    vector<VectorXd> a;
    vector<VectorXd> b;
    VectorXd data_a(4);
    data_a << 0, 0, 0, 0;
    VectorXd data_b(4);
    data_b << 1.3, 4.5, 3.5, 9.0;
    a.push_back(data_a);
    b.push_back(data_b);

    VectorXd rmse = tools.CalculateRMSE(a, b);
    assert(rmse == data_b, "RMSE calculation is not valid");
    std::cout << rmse << std::endl;
}

void sameDataCheck(Tools &tools) {
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

void runToolsTests() {
    Tools tools_test;
    cout << "No data check" << endl;
    noDataCheck(tools_test);
    cout << "Same data check" << endl;
    sameDataCheck(tools_test);
    cout << "Valid Calculation check" << endl;
    validCalcCheck(tools_test);

}