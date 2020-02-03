#include "pickup.h"

pickup main_pickup;

void setup() 
{
  main_pickup.init();
}

void loop()
{
  main_pickup.spin();
  delay(10);
}
