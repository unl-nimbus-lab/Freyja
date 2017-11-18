/* Aj's classy filter library */

#ifndef AJ_FILTER_COLLECTION_H
#define AJ_FILTER_COLLECTION_H

#include <ros/ros.h>
#include <vector>
#include <algorithm>

namespace AjFilterCollection
{
  std::vector<double> gauss_filter_coeff;
  int filter_len;
  
  void initGaussianFilter( std::vector<double> &, const int & );
  void filterObservations( const std::string &,
                           std::vector<double> &,
                           double & );
}


void AjFilterCollection::initGaussianFilter( std::vector<double> &fc, const int &len )
  {
    /* For the moment there is no scope of "automatically" doing this using
      mean and standard deviation. I do not have the time. These values are 
      copied from Matlab (or any online generator for that matter)
    */
    filter_len = len;
    gauss_filter_coeff = fc;
  }
  
void AjFilterCollection::filterObservations( const std::string &filter_name,
                                          std::vector<double> &obs,
                                          double &retVal )
{
    /* Convolve the filter over the observations and generate the result */
    if( filter_name == "gauss" )
    {
      /* Make sure there are equal number of observations, else return the
        most recent observation (meaning, no filtering) */
      if( obs.size() == filter_len )
      {
        retVal = std::inner_product( gauss_filter_coeff.begin(),
                                    gauss_filter_coeff.end(), obs.begin(), 0.0 );
      }
      else
      {
        ROS_WARN( "Filter length does not match!" );
        retVal = obs.back();
      } 
    }
    else
    {
      /* If it is not gauss, then don't filter */
      retVal = obs.back();
    }
}
#endif
