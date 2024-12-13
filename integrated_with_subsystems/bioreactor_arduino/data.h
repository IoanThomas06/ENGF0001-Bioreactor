#ifndef DATA_H
#define DATA_H

/* Stores:
 *  - pH as a String, representing a 4 digit integer (1000 times actual value).
 *  - Temperature as a String, representing a 4 digit integer (100 times actual value).
 *  - RPM as a String, representing a 4 digit integer (1 times actual value).
 */
typedef struct Data {
  String ph;
  String temp;
  String rpm;
} Data;

#endif