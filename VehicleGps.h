/*
  VehicleGps - a small GPS library for Arduino providing basic NMEA parsing.
Based on work by Maarten Lamers and Mikal Hart.
Copyright (C) 2011-2014 J.A. Woltjer.
All rights reserved.
 
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef VehicleGps_h
#define VehicleGps_h

#include <Arduino.h>
#include <EEPROM.h>

// software version of this library
#define GPS_VERSION 1.0

// conversion constants
#define GPS_MS_PER_KNOT 0.51444444
#define GPS_KMH_PER_KNOT 1.852
#define GPS_MILES_PER_METER 0.00062137112
#define GPS_KM_PER_METER 0.001

#define GPGGA_TERM   "GPGGA"
#define GPVTG_TERM   "GPVTG"
#define GPXTE_TERM   "GPXTE"
#define ROXTE_TERM   "ROXTE"
#define CAN_POS_TERM "0CFEF31C"
#define CAN_SPD_TERM "0CFEE81C"
#define CAN_XTE_TERM "0CFFFF2A"

#define GPS_INVALID_FLOAT 999999.9
#define GPS_INVALID_LONG 0xFFFFFFFF

#define GPS_NO_STATS

#define MINSPEED 0.5f

class VehicleGps {
private:
  //-------------
  // data members
  //-------------
  
  // configuration items
  //byte gps_type;
  byte datarate;
  
  // nmea items
  float time, new_time;
  unsigned long date, new_date;
  float latitude, new_latitude;
  float longitude, new_longitude;
  float altitude, new_altitude;
  float speed, new_speed;
  float course, new_course;
  int xte, new_xte;
  byte quality, new_quality;

  // timekeepers
  unsigned long last_GGA_fix, new_GGA_fix;
  unsigned long last_VTG_fix, new_VTG_fix;
  unsigned long last_XTE_fix, new_XTE_fix;

  // parsing state variables
  char term[20];
  byte term_number;
  byte term_offset;
  byte parity;
  byte checksum;
  int sum;
  bool is_checksum_term;
  
  // sentence type of decoded message
  enum types{
    GGA, VTG, XTE, XTE2, CAN_POS, CAN_SPD, CAN_XTE, OTHER
  };
  types sentence_type;

#ifndef GPS_NO_STATS
  // statistics
  unsigned long encoded_characters;
  unsigned long good_sentences;
  unsigned long failed_checksum;
  unsigned long passed_checksum;
#endif
  //-------------------------------------------------------
  // private member functions implemented in VehicleGps.cpp
  //-------------------------------------------------------
  float parseDecimal(const char *_c);
  float parseDegrees(const char *_c);
  int parseInteger(const char *_c);
  
  bool strcmp(const char *_str1, const char *_str2);
  byte hexToInt(char _c);
  
  bool parseTerm();
  
public:
  // -----------------------------------------------------
  // public member functions implemented in VehicleGps.cpp
  // -----------------------------------------------------

  //Constructor
  VehicleGps();

  bool update();
  static float distanceBetween(float lat1, float long1, float lat2, float long2);

#ifndef GPS_NO_STATS
  void stats(unsigned long *chars, unsigned short *sentences, unsigned short *failed_cs);
#endif

  // ----------------------------------------------------------
  // public inline member functions implemented in VehicleGps.h
  // ----------------------------------------------------------
  inline boolean minSpeed(){
    return GPS_KMH_PER_KNOT * speed > MINSPEED;
  }

  inline void readBaudrate(){
    byte rates[8] = {
    1, 2, 3, 4, 6, 8, 12, 24 };
    
    datarate = EEPROM.read(10);
    
    if (datarate > 7) datarate = 7;
/*
#if defined(__AVR_ATmega32U4__)    
    Serial1.begin(long(4800) * rates[datarate]);
#else
    Serial.begin(long(4800) * rates[datarate]);
#endif*/
  }
  
  inline void commitBaudrate(byte _rate){
    datarate = _rate;
    
    EEPROM.write(10, _rate);
  }
  
  // -------
  // Getters
  // -------
  inline byte getBaudrate(){
    return datarate;
  }

  // date as ddmmyy, time as hhmmsscc, and age in milliseconds
  inline void getDatetime(unsigned long *outdate, unsigned long *outtime) {
    if (outdate) *outdate = date;
    if (outtime) *outtime = time;
  }

  // date as dd, mm, yyyy, time as hh, mm, ss, cc, and age in milliseconds
  inline void getDatetimeDetails(int *outyear, byte *outmonth, byte *outday,
  byte *outhour, byte *outminute, byte *outsecond, byte *outhundredths = 0) {
    unsigned long _d, _t;
    getDatetime(&_d, &_t);
    if (outyear) {
      *outyear = _d % 100;
      *outyear += *outyear > 80 ? 1900 : 2000;
    }
    if (outmonth) *outmonth = (_d / 100) % 100;
    if (outday) *outday = _d / 10000;
    if (outhour) *outhour = _t / 1000000;
    if (outminute) *outminute = (_t / 10000) % 100;
    if (outsecond) *outsecond = (_t / 100) % 100;
    if (outhundredths) *outhundredths = _t % 100;
  }

  // lat/long in degrees and age of fix in milliseconds
  inline void getPosition(float *outlatitude, float *outlongitude) {
    if (outlatitude) *outlatitude = latitude;
    if (outlongitude) *outlongitude = longitude;
  }

  // altitude in last full GPGGA sentence in centimeters
  inline float getAltitude() {
    return altitude;
  }

  // quality of the GPS data from GGA string
  inline byte getQuality() {
    return quality;
  }

  // course in last full GPVTG sentence in degrees
  inline float getCourse() {
    return course;
  }

  // speed in last full GPVTG sentence in knots
  inline float getSpeed() {
    return speed;
  }

  // xte in last full GPXTE sentence in meters
  inline int getXte() {
    return xte;
  }

  //-------------------
  //special conversions
  //-------------------

  // altitude in centimeters
  inline int getAltitudeCm(){
    return int(altitude * 100);
  }

  // speed in meters per second
  inline float getSpeedMs() {
    return GPS_MS_PER_KNOT * speed;
  }

  // speed in kilometers per hour
  inline float getSpeedKmh() {
    return GPS_KMH_PER_KNOT * speed;
  }

  // cross track error in meters
  inline float getXteM() {
    return float(xte) / 100;
  }
  
  //-------------
  //age & version
  //-------------
  
  //returns age of sentence
  inline unsigned long getGgaFixAge(){
    return last_GGA_fix;
  }

  //returns age of sentence
  inline unsigned long getVtgFixAge(){
    return last_VTG_fix;
  }

  //returns age of sentence
  inline unsigned long getXteFixAge(){
    return last_XTE_fix;
  }

  // library version
  inline static float libraryVersion() {
    return GPS_VERSION;
  }
};
#endif