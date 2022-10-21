  /* Implementation for spatial-type filters: median filtering */
  
  /* -DMEDIAN_WINDOW can be provided at compile. Default=13.
    A fully-optimised implementation uses compile-time sorting network to
    create highly efficient sort (almost half the time taken by std::sort).
    This requires the 'window size' to be known at compile.

    A run-time adjustable implementation uses std::sort, and is selectable
    by a parameter passed during instantiation of this class.
  */

#include <static_sort.h>

namespace FreyjaUtils
{
  #ifndef MEDIAN_WINDOW
    #define MEDIAN_WINDOW 13
  #endif

  class MedianFilter : public Filter
  {
    StaticSort<MEDIAN_WINDOW> static_sorter_;
    std::vector<double> obs_vec_;
    bool using_static_sort_;
    int median_idx_;

    std::function<void(std::vector<double>&)> sorter_;

    public:
      using Filter::Filter;
    
      MedianFilter( );
      MedianFilter( const int &len );
      void init();
      void filterObservations( const std::vector<double> &obs, double &retVal );
      void filterObservations( const std::vector<double> &obs1, 
                                const std::vector<double> &obs2, 
                                const std::vector<double> &obs3, 
                                double &retVal1,
                                double &retVal2,
                                double &retVal3 );
  };

  MedianFilter::MedianFilter()
  {
    filter_len_ = MEDIAN_WINDOW;
    using_static_sort_ = true;
    sorter_ = static_sorter_;
    init();
  }
  MedianFilter::MedianFilter( const int &len )
  {
    filter_len_ = len;
    using_static_sort_ = false;
    sorter_ = [](auto &v){ std::sort( v.begin(), v.end() ); };
    init();
  }

  void MedianFilter::init()
  {
    filter_name_ = "median";
    median_idx_ = std::floor( float(filter_len_)/2.0 );
    std::string user_disp_hint;
    if( using_static_sort_ )
      user_disp_hint = "def-static-fast";
    else
      user_disp_hint = "dyn-std.sort-slow";
    
    std::cout << "Median Filter [" << user_disp_hint.c_str() <<"] window: "<< filter_len_ << std::endl;
  }

  void MedianFilter::filterObservations( const std::vector<double> &obs, double &retVal )
  {
    obs_vec_ = obs;
    sorter_( obs_vec_ );
    retVal = obs_vec_[median_idx_];
  }

  void MedianFilter::filterObservations( const std::vector<double> &obs1, 
                              const std::vector<double> &obs2, 
                              const std::vector<double> &obs3, 
                              double &retVal1,
                              double &retVal2,
                              double &retVal3 )
  {
    /* First input */
    filterObservations( obs1, retVal1 );
    
    /* Second input */
    filterObservations( obs2, retVal2 );
    
    /* Third input */
    filterObservations( obs3, retVal3 );
  }

}
