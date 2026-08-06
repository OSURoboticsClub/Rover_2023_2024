#include <SoftwareSerial.h>

#include <Arduino.h>
#include <math.h>
#include <string>

SoftwareSerial GPSSerial(26, 0); // GPIOs used for RX, TX on ESP32-WROOM-32D
uint8_t RoverOutput[8] = {0};
uint8_t AstroOutput[8] = {0};

double convertToDecimal(uint8_t sign, String coordinate) {
  double decimal_coordinate = 0; 
  // Deg min coordinate of the form DDMM.MMMMM or DDDMM.MMMMMM
  double degree, minutes;
  // Extract degrees, minutes parts
  int pos = coordinate.indexOf('.');

  // handles both two and three digit degrees
  degree = coordinate.substring(0, pos-2).toDouble(); // Extract degrees part
  minutes = coordinate.substring(pos-2).toDouble(); // Extract minutes part

  // Calculate decimal degrees
  decimal_coordinate = degree + (minutes / 60.0);

  if(sign == 1){ // coordinate is W or S so decimal deg should be negative
    decimal_coordinate = -decimal_coordinate;
  }
  return decimal_coordinate;
}

void separateCoordinate(double decimalDegree, byte& degree, unsigned short& fraction) {
  double fractpart, intpart;

  fractpart = modf (decimalDegree , &intpart)* (1 << 16);
  intpart = abs(intpart);
  degree = (byte)intpart;

  fractpart = modf(fabs(decimalDegree), &intpart);
  fraction = static_cast<unsigned short>(fabs(fractpart) * (1 << 16));
}

bool readRover(uint8_t* roverCoord){
  String rover_latitude = "";
  String rover_longitude = "";
  uint8_t rover_lat_sign = 0; 
  uint8_t rover_long_sign = 0;
  byte rover_lat_deg = 0;
  unsigned short rover_lat_frac = 0;
  byte rover_long_deg = 0;
  unsigned short rover_long_frac = 0;
  int numBytes = 45;
  int counter = 0;
  String gpsData = "";

  while (GPSSerial.available()) {
    gpsData = GPSSerial.readStringUntil('\n');
    int start = gpsData.indexOf("$GNRMC");
    if (start == -1) {
      gpsData = "";
      continue;
    }
    gpsData = gpsData.substring(start);
    break;
  }

  if (gpsData.length() == 0){
    Serial.println("Rover data not found");
    return false;
  }

  while(GPSSerial.available()) GPSSerial.read();
  Serial.print("GPS input: "); 
  for(int i = 0; i < numBytes; i++){
    char c = gpsData[i];
    
    Serial.print(c);
    if(c == ','){
      counter++;
    }
    if((counter >= 3) && (counter < 5)){ // latitude coordinate
      if((c >= '0' && c <= '9') || c == '.'){
        rover_latitude += c;
      }
      if(c == 'N'){
        rover_lat_sign = 0;
      }
      if(c == 'S'){
        rover_lat_sign = 1;
      }
    }
    if((counter >= 5) && (counter < 7)){ // longitude coordinate
      if((c >= '0' && c <= '9') || c == '.'){
        rover_longitude += c;
      }
      if(c == 'E'){
        rover_long_sign = 0;
      }
      if(c == 'W'){
        rover_long_sign = 1;
      }
    }
  }

      
  // Convert parsed data SOM packet format
  Serial.print("\nRover Latitude (deg min sec): ");
  Serial.println(rover_latitude);
  
  Serial.print("Rover Longitude (deg min sec): ");
  Serial.println(rover_longitude);
  // Print the formatted coordinates
  double rover_latitude_decimal = convertToDecimal(rover_lat_sign, rover_latitude);
  double rover_longitude_decimal = convertToDecimal(rover_long_sign, rover_longitude);

  Serial.print("\nRover Latitude (decimal): ");
  Serial.println(rover_latitude_decimal);
  
  Serial.print("Rover Longitude (decimal): ");
  Serial.println(rover_longitude_decimal);

  separateCoordinate(rover_latitude_decimal, rover_lat_deg, rover_lat_frac);
  separateCoordinate(rover_longitude_decimal, rover_long_deg, rover_long_frac);

  roverCoord[0] = rover_lat_sign;
  roverCoord[1] = rover_lat_deg;
  roverCoord[2] = rover_lat_frac >> 8;
  roverCoord[3] = rover_lat_frac & 0xff;
  roverCoord[4] = rover_long_sign;
  roverCoord[5] = rover_long_deg;
  roverCoord[6] = rover_long_frac >> 8;
  roverCoord[7] = rover_long_frac & 0xff;
      
  Serial.print("Rover Coord: [");
  for (int i = 0; i < 8; i++) {
    Serial.print(roverCoord[i]);
    if (i < 7) {
      Serial.print(", ");
    }
  }
  Serial.println("]"); 
  return true;
}

void setup() {
  pinMode(22, OUTPUT); // set enable pin high
  digitalWrite(22, HIGH);
  //initialize the serial port to 115200 baud
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  GPSSerial.begin(38400); 
}

void loop() {
//  read_extractContent(AstroOutput);
  if(!readRover(RoverOutput)) return;

  
  //send Rover coordinates over UART (in uint16_t)
  uint8_t toSOM[18] = {0};

  toSOM[0] = 255;
  toSOM[1] = RoverOutput[0];
  toSOM[2] = RoverOutput[1];
  toSOM[3] = RoverOutput[2];
  toSOM[4] = RoverOutput[3];
  toSOM[5] = RoverOutput[4];
  toSOM[6] = RoverOutput[5];
  toSOM[7] = RoverOutput[6];
  toSOM[8] = RoverOutput[7];
  toSOM[9] = AstroOutput[0];
  toSOM[10] = AstroOutput[1];
  toSOM[11] = AstroOutput[2];
  toSOM[12] = AstroOutput[3];
  toSOM[13] = AstroOutput[4];
  toSOM[14] = AstroOutput[5];
  toSOM[15] = AstroOutput[6];
  toSOM[16] = AstroOutput[7];
  toSOM[17] = 255;
  Serial2.write(toSOM, sizeof(toSOM)); // Write Rover and Astronaut coordinates to SOM


  Serial.print("Data sent to SOM: [");
  for (int i = 0; i < 18; i++) {
    Serial.print(toSOM[i]);
    if (i < 17) {
      Serial.print(", ");
    }
  }
  Serial.println("]"); 

}
