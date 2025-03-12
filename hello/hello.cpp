#include "hello.h"
#include "Arduino.h" //Serial
Hello::Hello( int v ) {
	id = ( v > 0 ? v : 0);
	Serial.print("Created id ");
	Serial.println(id);
}
Hello::~Hello() {
	Serial.print("Destroyed id ");
	Serial.println(id);
}
	void Hello::run() {
	Serial.print("Hello by id ");
	Serial.println(id);
}