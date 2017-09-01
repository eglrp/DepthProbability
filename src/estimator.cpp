//
// Created by jianping on 17-9-1.
//

#include "estimator.hpp"


void DistributionEstimator::estimate()
{
    ceres::Problem problem;

    //problem.AddParameterBlock(&params[0],3);
    //problem.SetParameterBlockConstant(&params[0]);

    for (int i = 0; i < _data.size(); ++i) {
        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<distributionCostFunctor, 1, 3>(
                        new distributionCostFunctor(_data[i],_min,_max));
        problem.AddResidualBlock(cost_function, NULL, &params[0]);
    }

    //problem.AddParameterBlock(&params[0],1);
    //problem.SetParameterBlockConstant(&params[0]);

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout<<params[0]<<" "<<params[1]<<" "<<params[2]<<"\n";
}