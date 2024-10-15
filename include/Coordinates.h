#ifndef COORDINATES
#define COORDINATES
#include <cmath>


class Coordinates
{
public :
Coordinates(double lat=0, double longi=0);

void setCoordinates(double lat, double longi);

double getLat() const;
double getLongi ()const;


private:
 double lat;
 double longi;

};

#endif
