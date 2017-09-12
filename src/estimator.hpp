//
// Created by jianping on 17-9-1.
//

#ifndef DEPTHPROBABILITY_ESTIMATOR_HPP
#define DEPTHPROBABILITY_ESTIMATOR_HPP

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <algorithm>
#include <math.h>

class DistributionEstimator {
public:
    struct distributionCostFunctor {

        distributionCostFunctor(double ob_x,double min, double max)
        :_ob_x(ob_x),_min(min),_max(max)
        {

        }

        template <typename T>
        bool operator()(const T* const params, T* residual) const {
            T part_normal = T(1.0)/(T(std::sqrt(2.0*M_PI))*params[2])
                            *ceres::exp(-(T(_ob_x)-params[1])*(T(_ob_x)-params[1])/(T(2)*params[2]*params[2]))*params[0];
            T part_uniform = T(1)/(_max-_min)*(T(1.0)-params[0]);
            residual[0] = -ceres::log(part_normal+part_uniform);
            if(params[0] < T(0.3) || params[0]>T(1))
            {
                return false;
            }
            return true;
        }

        double _ob_x;
        double _min;
        double _max;

    };

public:
    DistributionEstimator(const std::vector<double>& data)
    :_data(data)
    {
        _min = *std::min_element(_data.begin(),_data.end());
        _max = *std::max_element(_data.begin(),_data.end());


        //init
        params.resize(3);
        params[0] = 1;
        params[1] = std::accumulate(_data.begin(),_data.end(),0.0)/_data.size();
        params[2] = 0;
        for (int i = 0; i < _data.size(); ++i) {
            params[2] += (_data[i]-params[1])*(_data[i]-params[1]);
        }
        params[2]/=_data.size();
        params[2] = std::sqrt(params[2]);
        std::cout<<" mu:"<<params[1]<<std::endl;
        std::cout<<" sigma:"<<params[2]<<std::endl;
    }

    void estimate();
    std::vector<double> params;//pi mu sigma
private:
    const std::vector<double>& _data;
    double _min;
    double _max;

};


#endif //DEPTHPROBABILITY_ESTIMATOR_HPP
