//
// Created by jianping on 17-8-31.
//
#include "common.hpp"


boost::python::list common::RandomGaussion(unsigned int n){
    std::random_device rd;
    std::mt19937 gen(rd());

    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution<> d(1,0);

    boost::python::list hist;
    for(int i=0; i <n; i++) {
        hist.append(d(gen));
    }
    return hist;
}

