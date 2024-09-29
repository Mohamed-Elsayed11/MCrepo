#include <Arduino.h>
#include "FloodFill.h"
void setup()
{
   Serial.begin(9600);
   initialize();
}

void loop()
{
  solve();

}