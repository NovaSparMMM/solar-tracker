double luz;

void setup()
{
    pinMode(A0, INPUT);

  Serial.begin(9600);
  Serial.flush();
  while(Serial.available()>0)Serial.read();

}


void loop()
{

    luz = map(analogRead(A0),0,1023,0,100);
    Serial.println(luz);
    delay(1000);

}
