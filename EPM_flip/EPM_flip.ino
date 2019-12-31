#include "pedro.h"

pedro main_pedro;

void setup() 
{
  main_pedro.init();
}

void loop()
{
  main_pedro.spin();
  delay(10);
}
