#include <Wire.h>
#include <MechaQMC5883.h>

MechaQMC5883 qmc;
int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;
int XAxis, YAxis, z;
  
void setup() {
  Serial.begin(9600);
  Wire.begin();
  qmc.init();
  qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);
}

void loop() {
  qmc.read(&XAxis,&YAxis,&z);

  if (XAxis < minX) minX = XAxis;
  if (XAxis > maxX) maxX = XAxis;
  if (YAxis < minY) minY = YAxis;
  if (YAxis > maxY) maxY = YAxis;

  // Calculate offsets
  offX = (maxX + minX)/2;
  offY = (maxY + minY)/2;
  
  Serial.print(XAxis);
  Serial.print(":");
  Serial.print(YAxis);
  Serial.print(":");
  Serial.print(minX);
  Serial.print(":");
  Serial.print(maxX);
  Serial.print(":");
  Serial.print(minY);
  Serial.print(":");
  Serial.print(maxY);
  Serial.print(":");
  Serial.print(offX);
  Serial.print(":");
  Serial.print(offY);
  Serial.print("\n");
  delay(100);
}
