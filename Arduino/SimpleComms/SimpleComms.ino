HardwareSerial *curPort;

int commPin = 2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1000000);
  Serial1.begin(1000000);
  Serial2.begin(1000000);
  while (!Serial);

  //Serial.println("Comms relay Ready");
  pinMode(commPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    /*byte option = Serial.read();
    switch (option) {
      case 1:
        curPort = &Serial1;
        break;
      case 2:
        curPort = &Serial2;
        break;
    }*/

    byte buffer[2];
    Serial.readBytes(buffer, 2);
    int len = buffer[1];
    len |= buffer[0] << 8;

    //Serial.println(len);

    // output (echo)
    digitalWrite(commPin, 1);
    for (int i = 0; i < len; i++) {
      // Busy wait for each byte
      while (!Serial.available());
      // Pass-through (MAXIMUM SPEED)
      Serial1.write(Serial.read());
    }
    // make sure everything is written before the comm mode switches
    Serial1.flush();

    // receive (echo)
    digitalWrite(commPin, 0);
    while (Serial1.available()) {
      Serial1.read();
    }

    //Serial.println("Done");
  }

  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
}
