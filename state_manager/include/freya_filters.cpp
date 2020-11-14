/* Aj's classy filter library.
This implements a set of commonly used data filters, each suited for
its own purpose. An instance of this class can be created anywhere
(usually in state managers), and the object is passed the vector to be
filtered. The vector (and shuffling) is *not* maintained by this class!
That is: no previous data memory is held here, the implementation is
re-entrant (strictly Markovian, in this respect). Only the filter
properties (coeffs, type, len. etc) are held here.
*/

#ifndef FREYJA_FILTER_COLLECTION_H
#define FREYJA_FILTER_COLLECTION_H

#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <static_sort.h>

const int MEDIAN_FILTER_LEN = 13;
const int MEDIAN_IDX = 6;
class FreyjaFilters
{
  /* Only instantiate the filter once per object. This prevents having to check
  the filter type every time the filter function is called */
  int filter_len_;
  std::string filter_type_;
  std::vector<double> filter_coeffs_;
  double weight_scaler_;
  
  bool using_conv_filters_;
  
  /* For median filter */
  //  const int MEDIAN_FILTER_LEN = 13;
  //  const int MEDIAN_IDX = MEDIAN_FILTER_LEN/2;
  StaticSort<MEDIAN_FILTER_LEN> sorter_instance_;
  std::vector<double> unsorted_temp;

  void initGaussianFilter( const std::vector<double> &fc, const int &len )
  {
    /* For the moment there is no scope of "automatically" doing this using
      mean and standard deviation. I do not have the time. These values are 
      copied from Matlab (or any online generator for that matter)
    */
    filter_len_ = len;
    filter_coeffs_ = fc;
    weight_scaler_ = std::accumulate( fc.begin(), fc.end(), 0.0 );
  }
  
  void initLwmaFilter( const std::string &weight_type, const int &window_size )
  {
    /* A locally weighted moving average filter is simply a moving average
       filter with non-uniform weights. It is essentially a loess/lowess
      regression. I am not sure how much "better" this is yet.
      Weight type determines the way weights are associated:
       simple: linearly decreasing weights as we go away from the current point
       cubic : loess' preferred way of doing it
    */
    filter_len_ = window_size;
    weight_scaler_ = 0.0;

    double win = window_size;

    filter_coeffs_.resize( window_size );
    for( int idx = 0; idx < window_size; idx++ )
    {
      filter_coeffs_[idx] = (idx+1)/win;
      weight_scaler_ += (idx+1)/win;
    }
    if( weight_type == "squared" )
    {
      weight_scaler_ = 0.0;
      /* multiply once more by i/w */
      for( int idx = 0; idx < window_size; idx++ )
      {
        filter_coeffs_[idx] *= (idx+1)/win;
        weight_scaler_ += ((idx+1)*(idx+1)/(win*win));
      }
    }
    else if( weight_type == "cubic" )
    {
      weight_scaler_ = 0.0;
      /* compute the cube: multiply by i^2/w^2 */
      win = win*win;
      for( int idx = 0; idx < window_size; idx++ )
      {
        filter_coeffs_[idx] *= ((idx+1)*(idx+1))/win;
        weight_scaler_ += ((idx+1)*(idx+1)*(idx+1)/(win*window_size));
      }
    }
  }

  void initMedianFilter( )
  {
    /*
      Use a constant time sorting network that is constructed at compile-time
      to sort a fixed-length vector. The algorithm's template instantiation is
      all copy-pasta from the internets. See static_sort.h for details & license.
     */
     filter_len_ = MEDIAN_FILTER_LEN;
     unsorted_temp.resize( MEDIAN_FILTER_LEN );
     ROS_WARN( "Median filter init! Length: %d", filter_len_ );
  }

  public:
    /* Filter instantiation needs the length of the filter and the name.
      Sometimes a filter may have additional type as a string argument,
      or the object may pass in the coeffs that it wants to use.  */

    FreyjaFilters();
    FreyjaFilters( const int&, const std::string&, const std::string&,
                        const std::vector<double> );

    int getCurrentFilterLen();
    void filterObservations( const std::vector<double> &obs, double &retVal );
    void filterObservations( const std::vector<double> &obs1, 
                                             const std::vector<double> &obs2, 
                                             const std::vector<double> &obs3, 
                                             double &retVal1,
                                             double &retVal2,
                                             double &retVal3 );
};

FreyjaFilters::FreyjaFilters( )
{
  /* Perhaps a default filter uses this function call style? */
}

FreyjaFilters::FreyjaFilters( const int& len, const std::string &f_name,
                              const std::string &f_type,
                              const std::vector<double> coeffs = {0} )
{
  bool filter_initialised = false;

  /* Check for the name and the type */
  if( f_name == "lwma" )
  {
      initLwmaFilter( f_type, len );
      using_conv_filters_ = true;
      filter_initialised = true;
  }

  if( f_name == "gauss" )
  {
    if( coeffs.size() == 1 )
    {
      /* Init default filter: mean 10, stddev = 5 */
      ROS_WARN( "[FILTER]: No coeffs given for Gaussian. Picking defaults: len=21, stddev=5." );
      int len = 21;
      std::vector<double> fc = { 0.0108, 0.0158, 0.0222, 0.0299, 0.0388, 0.0484, 
                                0.0579, 0.0666, 0.0737, 0.0782, 0.0798, 0.0782,
                                0.0737, 0.0666, 0.0579, 0.0484, 0.0388, 0.0299,
                                0.0222, 0.0158, 0.0108};
      initGaussianFilter( fc, len );
    }
    else
      initGaussianFilter( coeffs, len );

    using_conv_filters_ = true;
    filter_initialised = true;
   }

  if( f_name == "woltring" )
  {
    /* @TODO: implement this .. */
    ROS_ERROR( "Woltring filter not implemented in this function call! Picking median-filter." );
    FreyjaFilters( -1, "median", "~" );

    using_conv_filters_ = false;
    filter_initialised = true;
  }

  if( f_name == "median" )
  {
    initMedianFilter( );
    using_conv_filters_ = false;
    filter_initialised = true;
  }

  if( !filter_initialised )
    ROS_ERROR( "Filter not initialized: %s", f_name.c_str() );
}

void FreyjaFilters::filterObservations( const std::vector<double> &obs,
                                        double &retVal )
{
  if( using_conv_filters_ )
  {
    if( obs.size() == filter_len_ )
      retVal = std::inner_product( filter_coeffs_.begin(), filter_coeffs_.end(),
                                    obs.begin(), 0.0 )/weight_scaler_;
    else
    {
      ROS_WARN( "Filter length does not match!" );
      retVal = obs.back();
    }
  }
  else
  {
    /* For the moment, this means we are using median filter */
    std::vector<double> unsorted_temp = obs;
    sorter_instance_( unsorted_temp );
    retVal = unsorted_temp[ MEDIAN_IDX ];
  }
}

void FreyjaFilters::filterObservations( const std::vector<double> &obs1, 
                                        const std::vector<double> &obs2, 
                                        const std::vector<double> &obs3, 
                                        double &retVal1,
                                        double &retVal2,
                                        double &retVal3 )
{
  if( using_conv_filters_ )
  {
    /* First input */
    if( obs1.size() == filter_len_ )
      retVal1 = std::inner_product( filter_coeffs_.begin(), filter_coeffs_.end(),
                                    obs1.begin(), 0.0 )/weight_scaler_;
    else
    {
      ROS_WARN( "Filter length does not match!" );
      retVal1 = obs1.back();
    }
    
    /* Second input */
    if( obs2.size() == filter_len_ )
      retVal2 = std::inner_product( filter_coeffs_.begin(), filter_coeffs_.end(),
                                    obs2.begin(), 0.0 )/weight_scaler_;
    else
    {
      ROS_WARN( "Filter length does not match!" );
      retVal2 = obs2.back();
    }
    
    /* Third input */
    if( obs3.size() == filter_len_ )
      retVal3 = std::inner_product( filter_coeffs_.begin(), filter_coeffs_.end(),
                                    obs3.begin(), 0.0 )/weight_scaler_;
    else
    {
      ROS_WARN( "Filter length does not match!" );
      retVal3 = obs3.back();
    }
  }
  else
  {
    /* For the moment, this means we are using median filter */
    std::vector<double> unsorted_temp = obs1;
    sorter_instance_( unsorted_temp );
    retVal1 = unsorted_temp[ MEDIAN_IDX ];
    
    unsorted_temp = obs2;
    sorter_instance_( unsorted_temp );
    retVal2 = unsorted_temp[ MEDIAN_IDX ];
    
    unsorted_temp = obs3;
    sorter_instance_( unsorted_temp );
    retVal3 = unsorted_temp[ MEDIAN_IDX ];
  }
}

int FreyjaFilters::getCurrentFilterLen()
{
  return filter_len_;
}
#endif
