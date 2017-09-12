//
// Created by jianping on 17-8-31.
//

#ifndef DEPTHPROBABILITY_COMMON_HPP
#define DEPTHPROBABILITY_COMMON_HPP

#include <Eigen/Dense>
#include <boost/python.hpp>


namespace common{
    //u,sigma,n
    boost::python::list RandomGaussion(double mu,double sigma, unsigned int n);

    boost::python::list RandomUniform(double min,double max, unsigned int n);

    boost::python::list GetGaussionSamples(const double& mu, const double& sigma, const boost::python::list& sample_Xs);

    boost::python::list solveParams(const boost::python::list& data);

}

BOOST_PYTHON_MODULE(libdepthProb)
{
    boost::python::def("RandomGaussion", common::RandomGaussion);
    boost::python::def("RandomUniform", common::RandomUniform);
    boost::python::def("GetGaussionSamples", common::GetGaussionSamples);
    
    boost::python::def("solveParams", common::solveParams);
}

#endif //DEPTHPROBABILITY_COMMON_HPP
