//
// Created by jianping on 17-8-31.
//
#include "common.hpp"
#include "estimator.hpp"
#include <iostream>

//u,sigma,n
boost::python::list common::RandomGaussion(double mu,double sigma, unsigned int n){
    std::random_device rd;
    std::mt19937 gen(rd());


    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution<> d(mu,sigma);

    boost::python::list hist;
    for(int i=0; i <n; i++) {
        hist.append(d(gen));
    }
    return hist;
}

boost::python::list common::RandomUniform(double min,double max, unsigned int n)
{
    boost::python::list hist;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(min,max);
    for (int i=0; i<n; ++i)
    {
        hist.append(distribution(generator));
    }
    return hist;
}

boost::python::list common::GetGaussionSamples(const double& mu, const double& sigma, const boost::python::list& sample_Xs)
{
    boost::python::list samples_Ys;
    std::function< double(double) > f_gaussion = [=](double x)->double{
        return 1.0/(std::sqrt(2*M_PI)*sigma)*std::exp(-(x-mu)*(x-mu)/(2*sigma*sigma));
    };
    for (int j = 0; j < boost::python::len(sample_Xs); ++j) {
        double x = boost::python::extract<double>(sample_Xs[j]);
        samples_Ys.append(f_gaussion(x));
    }
    return samples_Ys;
}


boost::python::list common::solveParams(const boost::python::list& data)
{
    std::vector<double> input_data;
    for (int j = 0; j < boost::python::len(data); ++j) {
        input_data.push_back(boost::python::extract<double>(data[j]));
    }
    DistributionEstimator estimator(input_data);
    estimator.estimate();


    boost::python::list result;
    result.append(estimator.params[0]);
    result.append(estimator.params[1]);
    result.append(estimator.params[2]);

    return result;
}

