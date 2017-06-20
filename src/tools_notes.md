# Tools.cpp Notes

This file is relatively straight forward. You will implement functions to calculate root mean squared error and the Jacobian matrix:

```c++
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
```

HINT: You implemented these already in the coding quizzes.
