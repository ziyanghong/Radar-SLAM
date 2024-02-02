#include <cmath>

#include <math.h>
#define DEG2RAD(a)   ((a) / (180 / M_PI))
#define RAD2DEG(a)   ((a) * (180 / M_PI))
#define EARTH_RADIUS 6378137


// From http://wiki.openstreetmap.org/wiki/Mercator#Elliptical_Mercator

/*
 * Mercator transformation
 * accounts for the fact that the earth is not a sphere, but a spheroid
 */

#define D_R (M_PI / 180.0)
#define R_D (180.0 / M_PI)
#define R_MAJOR 6378137.0
#define R_MINOR 6356752.3142
#define RATIO (R_MINOR/R_MAJOR)
#define ECCENT (sqrt(1.0 - (RATIO * RATIO)))
#define COM (0.5 * ECCENT)

static double deg_rad (double ang) {
        return ang * D_R;
}

double merc_x (double lon) {
        return R_MAJOR * deg_rad (lon);
}

double merc_y (double lat) {
        lat = fmin (89.5, fmax (lat, -89.5));
        double phi = deg_rad(lat);
        double sinphi = sin(phi);
        double con = ECCENT * sinphi;
        con = pow((1.0 - con) / (1.0 + con), COM);
        double ts = tan(0.5 * (M_PI * 0.5 - phi)) / con;
        return 0 - R_MAJOR * log(ts);
}

static double rad_deg (double ang) {
        return ang * R_D;
}

double merc_lon (double x) {
        return rad_deg(x) / R_MAJOR;
}

double merc_lat (double y) {
        double ts = exp ( -y / R_MAJOR);
        double phi = M_PI_2 - 2 * atan(ts);
        double dphi = 1.0;
        int i;
        for (i = 0; fabs(dphi) > 0.000000001 && i < 15; i++) {
                double con = ECCENT * sin (phi);
                dphi = M_PI_2 - 2 * atan (ts * pow((1.0 - con) / (1.0 + con), COM)) - phi;
                phi += dphi;
        }
        return rad_deg (phi);
}




/* The following functions take their parameter and return their result in degrees */

double y2lat_d(double y)   { return RAD2DEG( atan(exp( DEG2RAD(y) )) * 2 - M_PI/2 ); }
double x2lon_d(double x)   { return x; }

double lat2y_d(double lat) { return RAD2DEG( log(tan( DEG2RAD(lat) / 2 +  M_PI/4 )) ); }
double lon2x_d(double lon) { return lon; }

/* The following functions take their parameter in something close to meters, along the equator, and return their result in degrees */

double y2lat_m(double y)   { return RAD2DEG(2 * atan(exp( y/EARTH_RADIUS)) - M_PI/2); }
double x2lon_m(double x)   { return RAD2DEG(              x/EARTH_RADIUS           ); }

/* The following functions take their parameter in degrees, and return their result in something close to meters, along the equator */

double lat2y_m(double lat) { return log(tan( DEG2RAD(lat) / 2 + M_PI/4 )) * EARTH_RADIUS; }
double lon2x_m(double lon) { return          DEG2RAD(lon)                 * EARTH_RADIUS; }