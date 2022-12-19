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

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <string>
#include <functional>
#include "eigen3/Eigen/Dense"


namespace FreyjaUtils
{
  class Filter
  {
    protected:
      unsigned int filter_len_;
      std::string filter_name_;

    public:
      Filter();

      // derived classes must implement an init function
      virtual void init() = 0;

      unsigned int getFilterLen() { return filter_len_; }

      // for markov-type state filters
      virtual void getStateEstimate( Eigen::VectorXd &x_est );
      virtual void getStateEstimate( Eigen::VectorXd &x_est, const int& n_states );
      virtual void getStateEstimate( std::vector<double> &x_est );
      virtual void getStateEstimate( std::vector<double> &x_est, const int& n_states );

      // Eigen versions
      virtual void filterObservations( const Eigen::VectorXd &obs, double &retVal );
      virtual void filterObservations( const Eigen::MatrixXd &obs, Eigen::VectorXd &retVal );

      // STL versions
      virtual void filterObservations( const std::vector<double> &obs, double &retVal );
      virtual void filterObservations(  const std::vector<double> &obs1, 
                                        const std::vector<double> &obs2, 
                                        const std::vector<double> &obs3, 
                                        double &retVal1,
                                        double &retVal2,
                                        double &retVal3 );
  };
}
#include "freyja_conv_filters.cpp"
#include "freyja_median_filter.cpp"
#include "freyja_kalman_filter.cpp"


#endif
