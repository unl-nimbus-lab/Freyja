/* Aj's classy filter library */

#ifndef AJ_FILTER_COLLECTION_H
#define AJ_FILTER_COLLECTION_H

#include <ros/ros.h>
#include <vector>
#include <algorithm>

namespace AjFilterCollection
{
  std::vector<double> gauss_filter_coeff;
  std::vector<double> lwma_filter_coeff;
  int filter_len;
  
  void initLwmaFilter( const std::string &, const int & );
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
  
void AjFilterCollection::initLwmaFilter( const std::string &weight_type,
                                         const int &window_size )
{
  /* A locally weighted moving average filter is simply a moving average
     filter with non-uniform weights. It is essentially a loess/lowess
     regression. I am not sure how much "better" this is yet.
     Weight type determines the way weights are associated:
      simple: linearly decreasing weights as we go away from the current point
      cubic : loess' preferred way of doing it
  */
  filter_len = window_size;
  
  double win = window_size;
  
  lwma_filter_coeff.resize( window_size );
  for( int idx = 0; idx < window_size; idx++ )
    lwma_filter_coeff[idx] = (idx+1)/win;
  if( weight_type == "sqaured" )
  {
    /* multiply once more by i/w */
    for( int idx = 0; idx < window_size; idx++ )
    {
      lwma_filter_coeff[idx] *= (idx+1)/win;
    }
  }
  else if( weight_type == "cubic" )
  {
    /* compute the cube: multiply by i^2/w^2 */
    win = win*win;
    for( int idx = 0; idx < window_size; idx++ )
    {
      lwma_filter_coeff[idx] *= ((idx+1)*(idx+1))/win;
    }
  }
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
    else if( filter_name == "lwma")
    {
      if( obs.size() == filter_len )
      {
        retVal = std::inner_product( lwma_filter_coeff.begin(),
                                  lwma_filter_coeff.end(), obs.begin(), 0.0 );
        retVal/=double(filter_len);
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
