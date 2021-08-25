#ifndef _GNSS_UTILS_H
#define _GNSS_UTILS_H

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace GNSS_utils
{

const double RADIANS_PER_DEGREE = M_PI/180.0;
const double DEGREES_PER_RADIAN = 180.0/M_PI;

// WGS84 Parameters
const double WGS84_A = 6378137.0;		// major axis
const double WGS84_B = 6356752.31424518;	// minor axis
const double WGS84_F = (WGS84_A - WGS84_B)/WGS84_A;		// ellipsoid flattening
const double WGS84_E_SQUARE = WGS84_F*(2-WGS84_F);		// first eccentricity square

void LLAtoECEF(const double Lat, const double Long, const double Alt, double &ECEFX, double &ECEFY, double &ECEFZ);

void ECEFtoENU(const double X, const double Y, const double Z, const double Lat0, const double Long0, const double Alt0, double &xEast, double &yNorth, double &zUp);

void LLAtoENU(const double Lat, const double Long, const double Alt, const double Lat0, const double Long0, const double Alt0, double &xEast, double &yNorth, double &zUp);

void ECEFtoLLA(const double X, const double Y, const double Z, double &Lat, double &Lon, double &Height);

void ENUtoECEF(const double enu_x, const double enu_y, const double enu_z, const double Lat0, const double Long0, const double Alt0, double &ecef_x, double &ecef_y, double &ecef_z);
} // end namespace gnss_utils

#endif // _GNSS_UTILS_H
