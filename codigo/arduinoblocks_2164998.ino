double volatje;

int fnc_dynamic_analogRead(int _pin){
	pinMode(_pin,INPUT);
	return analogRead(_pin);
}

void setup()
{


	Serial.begin(115200);
	Serial.flush();
	while(Serial.available()>0)Serial.read();

}


void loop()
{
	yield();

  	volatje = fnc_dynamic_analogRead(2);
  	Serial.println(volatje);
  	delay(100);

}