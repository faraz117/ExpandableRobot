#include "./TinyGPS.h" 
TinyGPS gps;

void gpsdump(TinyGPS &gps) {

}

// Feed data as it becomes available 
bool feedgps() {
  while (GPSSerial.available()) {
    if (gps.encode(GPSSerial.read()))
      return true;
  }
  return false;
}


double calc_distance(float lat1 , float lon1 , float lat2 , float lon2)
{
  float dist_calc;
  //------------------------------------------ distance formula below. Calculates distance from current location to waypoint
  dist_calc=sqrt((((lon1)-(lon2))*((lon1)-(lon2)))+(((lat2-lat1)*(lat2-lat1))));
   dist_calc*=110567 ; //Converting to meters
   return dist_calc;
   
}

int courseToWaypoint(float lat1 , float lon1 , float lat2 , float lon2) 
{
  float dlon = radians(lon2-lon1);
  float cLat = radians(lat1);
  float tLat = radians(lat2);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  float targetHeading = degrees(a2);
  return targetHeading;
}   // courseToWaypoint()

float calcDesiredTurn(float targetHeading , float currentHeading)
{
    // calculate where we need to turn to head to destination
    float headingError = targetHeading - currentHeading;
    
    // adjust for compass wrap
    if (headingError < -180)      
      headingError += 360;
    if (headingError > 180)
      headingError -= 360;
  
    // calculate which way to turn to intercept the targetHeading
    return headingError;
 
}  // calcDesiredTurn()
