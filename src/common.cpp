//
// Created by jianping on 17-8-31.
//
#include "common.hpp"
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

