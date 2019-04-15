uint8_t data[6] = {0, 0, 0, 0, 0 , 0};
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.flush();
  Serial.write(28);
  while (Serial.available() <= 0);
//  for (int i = 0; i < 6; i++)
//  {
//    if (Serial.available() > 0)
//    {
//      data[i] = Serial.read();
//    }
//  }
    if (Serial.available() > 0)
    {
      data[0] = Serial.read();
      if(data[0] == 1)
      {
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
  delay(100);
  for (int i = 0; i < 6; i++)
  {
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

