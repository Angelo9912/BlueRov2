#ifndef GEODETIC_FUNCTIONS_H_
#define GEODETIC_FUNCTIONS_H_

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

// #-------------------------------------------------------------------------------------------
// # convert from geodetic 2D to Cartesian (North-East) 2D
// #-------------------------------------------------------------------------------------------
Vector2d ll2ne(const Vector2d& ll0, const Vector2d& ll)
{      
  double lat0 = ll0(0);
  double lon0 = ll0(1);
  double lat  = ll(0);
  double lon  = ll(1);

  Vector2d ne = Vector2d::Zero();

  lat   = lat * M_PI/180;
  lon   = lon * M_PI/180;
  lat0  = lat0 * M_PI/180;
  lon0  = lon0 * M_PI/180;

  double dlat = lat - lat0;
  double dlon = lon - lon0;

  double a = 6378137.0;
  double f = 1 / 298.257223563;
  double Rn = a / sqrt(1 - (2 * f - f * f) * sin(lat0) * sin(lat0));
  double Rm = Rn * (1 - (2 * f - f * f)) / (1 - (2 * f - f * f) * sin(lat0) * sin(lat0));

  ne(0) = dlat / atan2(1, Rm);
  ne(1) = dlon / atan2(1, Rn * cos(lat0));

  return ne;      
}


// #-------------------------------------------------------------------------------------------
// # convert from Cartesian (North-East) 2D to geodetic 2D
// #-------------------------------------------------------------------------------------------
Vector2d ne2ll(const Vector2d& ll0, const Vector2d& ne)
{
  double lat0 = ll0(0);
  double lon0 = ll0(1);

  Vector2d ll = Vector2d::Zero();

  lat0 = lat0 * M_PI/180;
  lon0 = lon0 * M_PI/180;

  double a  = 6378137.0;
  double f  = 1 / 298.257223563;
  double Rn = a / sqrt(1 - (2 * f - f * f) * sin(lat0) * sin(lat0));
  double Rm = Rn * (1 - (2 * f - f * f)) / (1 - (2 * f - f * f) * sin(lat0) * sin(lat0));

  ll(0) = (lat0 + atan2(1, Rm)*ne(0)) * 180/M_PI;
  ll(1) = (lon0 + atan2(1, Rn*cos(lat0)) * ne(1)) * 180/M_PI;

  return ll;
}


// #-------------------------------------------------------------------------------------------
// # convert from geodetic 3D to Cartesian (North-East) 3D
// #-------------------------------------------------------------------------------------------
Vector3d lld2ned(const Vector3d& lld0, const Vector3d& lld) 
{
    Vector3d ned = Vector3d::Zero();
    ned.block(0,0,2,1) = ll2ne(lld0.block(0,0,2,1), lld.block(0,0,2,1));
    ned(2) = lld(2) - lld0(2);
    return ned;
}


// #-------------------------------------------------------------------------------------------
// # convert from Cartesian (North-East-Down) 3D to geodetic 3D
// #-------------------------------------------------------------------------------------------
Vector3d ned2lld( const Vector3d& lld0, const Vector3d& ned) 
{
    Vector3d lld = Vector3d::Zero();
    lld.block(0,0,2,1) = ne2ll(lld0.block(0,0,2,1), ned.block(0,0,2,1));
    lld(2) = lld0(2) + ned(2);
    return lld;
}

// #-------------------------------------------------------------------------------------------
// # Given 2 geodetic 3D positions give back the DIRECTION between the two
// #-------------------------------------------------------------------------------------------
double lld2direction(const Vector3d& lld0, const Vector3d& lld) 
{
    Vector3d tmp_ned = lld2ned(lld0, lld);
    return atan2( tmp_ned(1),tmp_ned(0) );
}

// #-------------------------------------------------------------------------------------------
// # Given 2 geodetic 3D positions give back the DISTANCE between the two
// #-------------------------------------------------------------------------------------------
double lld2distance( const Vector3d& lld0, const Vector3d& lld) 
{
    Vector3d tmp_ned = lld2ned(lld0, lld);
    return sqrt( pow(tmp_ned(2),2) + pow(tmp_ned(1),2) + pow(tmp_ned(0),2) );
}

// #-------------------------------------------------------------------------------------------
// # Given 2 geodetic 2D positions give back the DIRECTION between the two
// #-------------------------------------------------------------------------------------------
double ll2direction( const Vector2d& ll0, const Vector2d& ll) 
{
    Vector2d tmp_ned = ll2ne(ll0, ll);
    return  atan2( tmp_ned(1),tmp_ned(0) );
}

// #-------------------------------------------------------------------------------------------
// # Given 2 geodetic 2D positions give back the DISTANCE between the two
// #-------------------------------------------------------------------------------------------
double ll2distance( const Vector2d& ll0, const Vector2d& ll) 
{
    Vector2d tmp_ned = ll2ne(ll0, ll);
    return sqrt( pow(tmp_ned(1),2) + pow(tmp_ned(0),2) );
}

#endif //GEODETIC_FUNCTIONS_H_
