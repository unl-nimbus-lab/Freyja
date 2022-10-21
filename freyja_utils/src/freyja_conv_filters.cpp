namespace FreyjaUtils
{
  class ConvFilters : public Filter
  {
    std::vector<double> filter_coeffs_;
    double weight_scaler_;

    public:
      using Filter::Filter;
      ConvFilters() { }
      ConvFilters( const std::string&, const unsigned int&, const std::vector<double> );
      void init();
      void initGenericFilter( const std::string&, const int&, const std::vector<double>&, double );
      
      void filterObservations( const Eigen::VectorXd &obs, double &retVal );
      void filterObservations( const Eigen::MatrixXd &obs, Eigen::VectorXd &retVal );
      void filterObservations( const std::vector<double> &obs, double &retVal );
      void filterObservations(  const std::vector<double> &obs1, 
                                const std::vector<double> &obs2, 
                                const std::vector<double> &obs3, 
                                double &retVal1,
                                double &retVal2,
                                double &retVal3 );
  };

  void ConvFilters::init()
  {
    std::cout << "Initialised filter type: " << filter_name_ << std::endl;
  }

  void ConvFilters::initGenericFilter( const std::string &fname, const int &len, 
                                      const std::vector<double> &fc,
                                      double _weight=-1.0 )
  {
    filter_name_ = fname;
    filter_len_ = len;
    filter_coeffs_ = fc;
    weight_scaler_ = _weight > 0 ? 
                      _weight :
                      std::accumulate( fc.begin(), fc.end(), 0.0 );
    init();
  }

  ConvFilters::ConvFilters( const std::string &f_name, const unsigned int& len, 
                            const std::vector<double> _coeffs = std::vector<double>() )
  {
    if( _coeffs.size() > 0 )
    {
      /* User has provided the coeffs to be used. No need to do anything else */
      if( _coeffs.size() != len )
        std::cout << "WARNING: length does not match number of filter coeffs. Ignoring len!" << std::endl;

      initGenericFilter( f_name, len, _coeffs );
    }
    else
    {
      /* No coeffs provided, we must compute them */
      std::vector<double> fcoeffs;
      int flength;
      double scaler = -1.0;
      if( f_name == "gauss" )
      {
              flength = 21;
              fcoeffs = {0.0108, 0.0158, 0.0222, 0.0299, 0.0388, 0.0484, 
                        0.0579, 0.0666, 0.0737, 0.0782, 0.0798, 0.0782,
                        0.0737, 0.0666, 0.0579, 0.0484, 0.0388, 0.0299,
                        0.0222, 0.0158, 0.0108} ;
        
        }
        else if( f_name == "lwma-cubic" ) 
            {
              flength = len;
              scaler = 0.0;
              fcoeffs.resize(flength);
              /* compute the cube: multiply by i^2/w^2 */
              double len2 = flength*flength;
              for( int idx = 0; idx < flength; idx++ )
              {
                fcoeffs[idx] = (idx+1)/flength;
                weight_scaler_ += (idx+1)/flength;
              }
              for( int idx = 0; idx < flength; idx++ )
              {
                fcoeffs[idx] *= ((idx+1)*(idx+1))/len2;
                scaler += ((idx+1)*(idx+1)*(idx+1)/(len2*flength));
              }
            
            }

        else if( f_name == "savit-golay" )
            {
              flength = len;
              if( flength == 5 )
                fcoeffs = {-3.0, 12.0, 17.0, 12.0, -3.0};
              else if( flength == 7 )
                fcoeffs = {-2.0, 3.0, 6.0, 7.0, 6.0, 3.0, -2.0};
              else if (flength == 9 )
                fcoeffs = {-21.0, 14.0, 39.0, 54.0, 59.0, 54.0, 39.0, 14.0, -21.0};
              else
                std::cout << "Filter length not supported for savit-golay!" << std::endl;

            }
      
      initGenericFilter( f_name, flength, fcoeffs, scaler );
    }
  }

  void ConvFilters::filterObservations( const std::vector<double> &obs,
                                        double &retVal )
  {
    if( obs.size() == filter_len_ )
      retVal = std::inner_product( filter_coeffs_.begin(), filter_coeffs_.end(),
                                    obs.begin(), 0.0 )/weight_scaler_;
    else
    {
      std::cout << "Filter length does not match!\n" ;
      retVal = obs.back();
    }
  }

  void ConvFilters::filterObservations( const std::vector<double> &obs1, 
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
