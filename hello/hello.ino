#include "hello.h"

void setup()
{
	Serial.begin();
}
void loop()
{
 {
	Hello obj1 {};
	obj1.run();
 }
	delay(500);
 {
	Hello obj2 {1};
	obj2.run();
 }
	delay(500);
}
