/* Convenience functions
*/
#include <algorithm>

inline float Mean( const std::vector<float> &d )
{
  return std::accumulate( d.begin(), d.end(), 0.0 )/d.size();
}

inline float MeanFixedLength( const std::vector<float> &d, const int &LEN )
{
  return std::accumulate( d.begin(), d.end(), 0.0 )/LEN;
}

inline float Variance( const std::vector<float> &d )
{
  float m = Mean( d );
  float sq = std::inner_product( d.begin(), d.end(), d.begin(), 0.0 );
  return (sq/d.size() - m*m);
}

inline float VarianceFixedLength( const std::vector<float> &d, const int &LEN )
{
  float m = MeanFixedLength( d, LEN );
  float sq = std::inner_product( d.begin(), d.end(), d.begin(), 0.0 );
  return (sq/LEN - m*m);
}

inline float VarianceFixedLength( const std::vector<float> &d, const int &LEN, const float &m )
{
  float sq = std::inner_product( d.begin(), d.end(), d.begin(), 0.0 );
  return (sq/LEN - m*m);
}
