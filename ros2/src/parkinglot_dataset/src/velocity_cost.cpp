#include <iostream>

#include <steam.hpp>

class VelocityConstraintEval : public steam::ErrorEvaluator<6, 6>::type {
public:
  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Shared pointer typedefs for readability
  //////////////////////////////////////////////////////////////////////////////////////////////
  using Ptr = std::shared_ptr<VelocityConstraintEval>;
  using ConstPtr = std::shared_ptr<const VelocityConstraintEval>;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Construct from a 1D state vector
  //////////////////////////////////////////////////////////////////////////////////////////////
  VelocityConstraintEval(const steam::VectorSpaceStateVar::ConstPtr &stateVec,
                         const Eigen::Array<double, 6, 1> &coeff)
      : stateVec_(stateVec), coeff_(coeff) {
    if (stateVec_->getPerturbDim() != 6) {
      throw std::invalid_argument("Dimension was improper size");
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Returns whether or not an evaluator contains unlocked state
  /// variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual bool isActive() const { return !stateVec_->isLocked(); }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the error
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double, 6, 1> evaluate() const {

    // Get value of state variable
    auto x = stateVec_->getValue().array();

    auto err = x * x * coeff_;

    return err.matrix();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Evaluate the error and Jacobians wrt state variables
  //////////////////////////////////////////////////////////////////////////////////////////////
  virtual Eigen::Matrix<double, 6, 1>
  evaluate(const Eigen::Matrix<double, 6, 6> &lhs,
           std::vector<steam::Jacobian<6, 6>> *jacs) const {

    // Get value of state variable
    auto x = stateVec_->getValue().array();

    // Check for null ptr and clear jacobians
    if (jacs == NULL) {
      throw std::invalid_argument(
          "Null pointer provided to return-input 'jacs' in evaluate");
    }
    jacs->clear();

    // If state not locked, add Jacobian
    if (!stateVec_->isLocked()) {

      // Create Jacobian object
      jacs->push_back(steam::Jacobian<6, 6>());
      steam::Jacobian<6, 6> &jacref = jacs->back();
      jacref.key = stateVec_->getKey();

      auto diag = 2 * x * coeff_;

      // Fill out matrix
      Eigen::Matrix<double, 6, 6> jacobian =
          Eigen::Matrix<double, 6, 6>::Zero();
      jacobian.diagonal() = diag;
      jacref.jac = lhs * jacobian;
    }

    // Construct error and return
    auto err = x * x * coeff_;
    return err.matrix();
  }

private:
  steam::VectorSpaceStateVar::ConstPtr stateVec_;

  Eigen::Array<double, 6, 1> coeff_;
};

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Setup a fresh problem with unsolve variables
//////////////////////////////////////////////////////////////////////////////////////////////
void setupDivergenceProblem() {

  // Create vector state variable
  Eigen::Matrix<double, 6, 1> initial;
  initial << 1, 2, 3, 4, 5, 1;
  auto stateVar = std::make_shared<steam::VectorSpaceStateVar>(initial);

  // Setup shared noise and loss function
  auto sharedNoiseModel = std::make_shared<steam::StaticNoiseModel<6>>(
      Eigen::Matrix<double, 6, 6>::Identity());
  steam::L2LossFunc::Ptr sharedLossFunc(new steam::L2LossFunc());

  // Setup cost term
  Eigen::Matrix<double, 6, 1> coeff;
  coeff << 1, 2, 3, 4, 5, 6;
  VelocityConstraintEval::Ptr errorfunc(
      new VelocityConstraintEval(stateVar, coeff));
  auto costTerm = std::make_shared<steam::WeightedLeastSqCostTerm<6, 6>>(
      errorfunc, sharedNoiseModel, sharedLossFunc);

  // Init problem
  steam::OptimizationProblem problem;
  problem.addStateVariable(stateVar);
  problem.addCostTerm(costTerm);

  std::cout << stateVar->getValue().transpose() << std::endl;

  steam::DoglegGaussNewtonSolver::Params params;
  params.verbose = true;
  params.maxIterations = 100;
  steam::DoglegGaussNewtonSolver solver(&problem, params);
  solver.optimize();
  std::cout << "Powell's Dogleg Terminates from: "
            << solver.getTerminationCause() << " after "
            << solver.getCurrIteration() << " iterations." << std::endl;

  std::cout << stateVar->getValue().transpose() << std::endl;
}

int main(int argc, char **argv) {

  // Solve using Powell's Dogleg Solver

  setupDivergenceProblem();

  return 0;
}
