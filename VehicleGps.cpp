/*
  VehicleGps - a small GPS library for Arduino providing basic NMEA parsing.
Based on work by Maarten Lamers and Mikal Hart.
Copyright (C) 2011-2013 J.A. Woltjer.
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

#include "VehicleGps.h"

//------------
// Constructor
//------------
VehicleGps::VehicleGps(){
  time = GPS_INVALID_FLOAT;
  date = GPS_INVALID_LONG;
  latitude = GPS_INVALID_FLOAT;
  longitude = GPS_INVALID_FLOAT;
  altitude = GPS_INVALID_FLOAT;
  speed = GPS_INVALID_FLOAT;
  course = GPS_INVALID_FLOAT;
  xte = GPS_INVALID_FLOAT;
  quality = 0;

  last_GGA_fix = 0;
  last_VTG_fix = 0;
  last_XTE_fix = 0;

  term[0] = '\0';
  term_number = 0;
  term_offset = 0;
  parity = 0;
  sum = 0;
  checksum = 0;
  is_checksum_term = false;
  sentence_type = OTHER;

#ifndef GPS_NO_STATS
  encoded_characters = 0;
  good_sentences = 0;
  failed_checksum = 0;
  passed_checksum = 0;
#endif

  Serial1.begin(GPS_DATARATE);
}

//----------------------------------------
// private member functions implementation
//----------------------------------------

// ---------------------------------
// Method for parsing ascii to float
// ---------------------------------
float VehicleGps::parseDecimal(const char *_c) {
  //atof from stdlib.h
  float _f = atof(_c);
  return _f;
}

// -----------------------------------
// Method for parsing ascii to integer
// -----------------------------------
int VehicleGps::parseInteger(const char*_c) {
  //atoi from stdlib.h
  int _i = atoi(_c);
  return _i;
}

// -----------------------------------------------------
// Method for parsing ascii deg/min.dec to float degrees
// -----------------------------------------------------
float VehicleGps::parseDegrees(const char *_c) {
  //atof from stdlib.h
  float _f = atof(_c);

  int _left = _f / 100;
  float _right = _f - _left * 100;

  return _left + _right / 60.0;
}

// ---------------------------------------------------------
// Method for comparing two strings returns true if the same
// ---------------------------------------------------------
bool VehicleGps::strcmp(const char *_str1, const char *_str2) {
  // Loop trough 2 strings
  while (*_str1 == *_str2) {
    //check for null character
    if (!*_str1){
      return true;
    }
    _str1++, _str2++;
  }
  return false;
}

// ------------------------------------------
// Method for converting hex ascii to integer
// ------------------------------------------
int VehicleGps::hexToInt(char _c) {
  // Parse hex
  if (_c >= 'A' && _c <= 'F')
    return _c - 'A' + 10;
  else if (_c >= 'a' && _c <= 'f')
    return _c - 'a' + 10;
  else
    return _c - '0';
}

// ---------------------------------------------------------------------------
// Method for processing a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
// ---------------------------------------------------------------------------
bool VehicleGps::parseTerm() {
  if (is_checksum_term) {
    if (sentence_type == XTE2) {
      //checksum already checked in feedgps() for Trimble strings...
      checksum = parity;
    }
    else {
      // Process checksum and update state
      checksum = 16 * hexToInt(term[0]) + hexToInt(term[1]);
    }
    if (checksum == parity) {
#ifndef GPS_NO_STATS
      good_sentences++;
#endif
      switch (sentence_type) {
      case GGA:
        altitude = new_altitude;
        time = new_time;
        latitude = new_latitude;
        longitude = new_longitude;
        quality = new_quality;
        last_GGA_fix = millis();
        new_GGA_data = true;
        break;
      case VTG:
        course = new_course;
        speed = new_speed;
        last_VTG_fix = millis();
        new_VTG_data = true;
        break;
      case XTE:
      case XTE2:
        xte = new_xte;
        last_XTE_fix = millis();
        new_XTE_data = true;
        break;
      }
      return true;
    }
#ifndef GPS_NO_STATS
    else {
      failed_checksum++;
    }
#endif
    return false;  // no "else if" needed because of return statements
  }
  
  if (term_number == 0) {
  // The first term determines the sentence type
#ifdef GPGGA_TERM
    if (strcmp(term, GPGGA_TERM))
      sentence_type = GGA;
    else
#endif
#ifdef GPVTG_TERM
    if (strcmp(term, GPVTG_TERM))
      sentence_type = VTG;
    else
#endif
#ifdef GPXTE_TERM
    if (strcmp(term, GPXTE_TERM))
      sentence_type = XTE;
    else
#endif
#ifdef GPRMC_TERM
    if (strcmp(term, GPRMC_TERM))
      sentence_type = RMC;
    else
#endif
#ifdef ROXTE_TERM
    if (strcmp(term, ROXTE_TERM))
      sentence_type = XTE2;
    else
#endif
      sentence_type = OTHER;
    return false;
  }
  
  if (term[0]){
  // Check if char array is filled
    switch (sentence_type) {
    case GGA:
      switch (term_number) {
      case 1: //Time
        new_time = parseDecimal(term);
        break;
      case 2: // Latitude
        new_latitude = parseDegrees(term);
        break;
      case 3: // N/S
        if (term[0] == 'S') {
          new_latitude = -new_latitude;
        }
        break;
      case 4: // Longitude
        new_longitude = parseDegrees(term);
        break;
      case 5: // E/W
        if (term[0] == 'W') {
          new_longitude = -new_longitude;
        }
        break;
      case 6: // Fix data quality
        new_quality = parseInteger(term);
        break;
      case 9: // Altitude
        new_altitude = parseDecimal(term);
        break;
      }
      break;
    case VTG:
      switch (term_number) {
      case 1: // Course
        new_course = parseDecimal(term);
        break;
      case 5: // Speed
        new_speed = parseDecimal(term);
        break;
      }
      break;
    case XTE:
      switch (term_number) {
      case 3: // XTE
        new_xte = parseDecimal(term);
        break;
      }
      break;
    case XTE2:
      switch (term_number) {
      case 1: // Trimble XTE
        new_xte = parseDecimal(term);
        break;
      }
      break;
    case OTHER:
      break;
    }
  }
  return false;
}

//--------------------------------------
//public member functions implementation
//--------------------------------------

// ------------------------------------------------
// Method for receiving characters from serial port
// ------------------------------------------------
bool VehicleGps::update() {
  // temporary variables
  char _c;
  bool _valid_sentence = false;
  //byte _t = 0;
  
  while(Serial1.available()){
    _c = Serial1.read();
  
    // keep track of encoded characters
    encoded_characters++;

    //start decoding, split sentence into terms separated by ","', "/r", "/n", "*" or "$".
    switch (_c) {
    // trimble id (reset sum)
    case 191:
      term_number = term_offset = 0;
      sum = 0;
      break;
    // sentence start
    case '$':
    case '@':
      // sentence begin, reset decoding process
      term_number = term_offset = 0;
      parity = 0;
      sum += byte(_c);
      sentence_type = OTHER;
      is_checksum_term = false;
      break;
    // bitbucket for unwanted trimble and in NMEA unused characters
    case 20:
    case 0:
    case ' ':
      sum += byte(_c);
      break;
    // term terminators, decode term by term
    case ',':
      parity ^= _c;
    case ':':
    case '*':
    case '\r':
    case '\n':
      sum += byte(_c);
      term[term_offset] = '\0';
      // pass completed term off to processing
      _valid_sentence = parseTerm();
      // reset parsing state for new term
      term_number++;
      term_offset = 0;
      is_checksum_term = _c == '*';
      break;
    // trimble specific term terminator and parity check
    // ascii 3 is terminator when preceded by ascii 16
    // last 3 digits before ascii 3 are: number of characters send
    // 2 byte hex sum of all characters after trimble id 
    case 3:
      if (term[term_offset - 1] == 16 && !is_checksum_term) {
        sum -= byte(term[term_offset - 1]);
        sum -= byte(term[term_offset - 2]);
        sum -= byte(term[term_offset - 3]);
      
        // check trimble checksum
        if (sum - byte(term[term_offset - 2]) - (256 * byte(term[term_offset - 3])) == 0) {
          term[term_offset - 3] = '\0';
          parseTerm();
          is_checksum_term = true;
          _valid_sentence = parseTerm();
        }
        term_number++;
        term_offset = 0;
        break;
      }
      else {
        // ordinary character
      }
    // ordinary characters
    default:
      if (term_offset < sizeof (term) - 1)
        term[term_offset++] = _c;
      if (!is_checksum_term)
        parity ^= _c;
      sum += byte(_c);
      break;
    }
  }
  return _valid_sentence;
}

float VehicleGps::distanceBetween(float lat1, float long1, float lat2, float long2) {
  // returns distance in meters between two positions, both specified 
  // as signed decimal-degrees latitude and longitude. Uses great-circle 
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1 - long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

#ifndef GPS_NO_STATS

void VehicleGps::stats(unsigned long *chars, unsigned short *sentences, unsigned short *failed_cs) {
  if (chars) *chars = encoded_characters;
  if (sentences) *sentences = good_sentences;
  if (failed_cs) *failed_cs = failed_checksum;
}

#endif