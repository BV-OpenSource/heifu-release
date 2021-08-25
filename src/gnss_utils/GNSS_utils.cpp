#include "gnss_utils/GNSS_utils.h"

void GNSS_utils::LLAtoECEF(const double Lat, const double Long, const double Alt,
                           double &ECEFX, double &ECEFY, double &ECEFZ)
{
    double a = WGS84_A;
    double eccSquared = WGS84_E_SQUARE;

    double LatRad = Lat*RADIANS_PER_DEGREE;
    double LongRad = Long*RADIANS_PER_DEGREE;

    double N=a/sqrt(1-eccSquared*pow(sin(LatRad), 2));
    ECEFX = (N+Alt)*cos(LatRad)*cos(LongRad);
    ECEFY = (N+Alt)*cos(LatRad)*sin(LongRad);
    ECEFZ = (N*(1-eccSquared) + Alt)*sin(LatRad);

}

void GNSS_utils::ECEFtoENU(const double X, const double Y, const double Z, const double Lat0, const double Long0, const double Alt0,
                           double &xEast, double &yNorth, double &zUp)
{
    double X0, Y0, Z0;
    LLAtoECEF(Lat0, Long0, Alt0, X0, Y0, Z0);

    double phi = Lat0*RADIANS_PER_DEGREE;
    double lamb = Long0*RADIANS_PER_DEGREE;

    double sin_lambda = sin(lamb);
    double cos_lambda = cos(lamb);
    double sin_phi = sin(phi);
    double cos_phi = cos(phi);

    double xd = X - X0;
    double yd = Y - Y0;
    double zd = Z - Z0;

    xEast = -sin_lambda * xd + cos_lambda * yd;
    yNorth = -sin_phi * cos_lambda * xd - sin_phi * sin_lambda * yd + cos_phi * zd;
    zUp = cos_phi * cos_lambda * xd + cos_phi * sin_lambda * yd + sin_phi * zd;
}

void GNSS_utils::LLAtoENU(const double Lat, const double Long, const double Alt, const double Lat0, const double Long0, const double Alt0,
                          double &xEast, double &yNorth, double &zUp){
    double ECEFX, ECEFY, ECEFZ;
    LLAtoECEF(Lat, Long, Alt, ECEFX, ECEFY, ECEFZ);
    ECEFtoENU(ECEFX, ECEFY, ECEFZ, Lat0, Long0, Alt0, xEast, yNorth, zUp);
}

void GNSS_utils::ECEFtoLLA(const double X, const double Y, const double Z,
                           double &Lat, double &Lon, double &Height)
{
    double a = WGS84_A;
    double b = WGS84_B;
    double e_sq = WGS84_E_SQUARE;
    double r = sqrt(X*X + Y*Y);
    double e2 = (a*a -b*b)/(b*b);
    double F = 54*b*b*Z*Z;
    double G = r*r + (1-e_sq)*Z*Z - e_sq*(a*a - b*b);
    double c = e_sq*e_sq * F * r * r / (G * G * G);
    double s = pow(1 + c + sqrt(c*c + 2*c), 1/3);
    double P = F / (3*G*G*(pow(1+s+1/s,2)));
    double Q = sqrt(1+2*P*e_sq*e_sq);
    double r0 = (-P*e_sq*r)/(1+Q) + sqrt( a*a*(1+1/Q)/2 - (P*(1-e_sq)*Z*Z)/(Q*(1+Q)) - P*r*r/2);
    double U = sqrt(pow(r - e_sq*r0,2) + Z*Z);
    double V = sqrt(pow(r - e_sq*r0,2) + (1-e_sq)*Z*Z );
    double z0 = b*b*Z/(a*V);
    Height = U*(1-b*b/(a*V));
    Lat = atan( (Z + e2*z0) / r) * 180/M_PI;
    Lon = atan2(Y,X) * 180 / M_PI;
}

void GNSS_utils::ENUtoECEF(const double enu_x, const double enu_y, const double enu_z, const double Lat0, const double Long0, const double Alt0,
                           double &ecef_x, double &ecef_y, double &ecef_z)
{
    double X0, Y0, Z0;
    LLAtoECEF(Lat0, Long0, Alt0, X0, Y0, Z0);

    double phi = Lat0*RADIANS_PER_DEGREE;
    double lamb = Long0*RADIANS_PER_DEGREE;

    double sin_lambda = sin(lamb);
    double cos_lambda = cos(lamb);
    double sin_phi = sin(phi);
    double cos_phi = cos(phi);

    ecef_x = X0 - sin_lambda * enu_x - sin_phi * cos_lambda * enu_y + cos_phi * cos_lambda * enu_z;
    ecef_y = Y0 + cos_lambda * enu_x - sin_phi * sin_lambda * enu_y + cos_phi * sin_lambda * enu_z;
    ecef_z = Z0 + cos_phi * enu_y + sin_phi * enu_z;
}
