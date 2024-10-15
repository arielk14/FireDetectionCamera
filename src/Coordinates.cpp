#include "Coordinates.h"

// Constructor to initialize latitude and longitude
Coordinates::Coordinates(double lat, double lon) : lat(lat), longi(lon) {}

// Setter for latitude and longitude together
void Coordinates::setCoordinates(double lat, double lon) {
    lat = lat;
    longi = lon;
}

// Getter for latitude
double Coordinates::getLat() const {
    return lat;
}

// Getter for longitude
double Coordinates::getLongi() const {
    return longi;
}




