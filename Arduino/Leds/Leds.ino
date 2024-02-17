int ledR = 6;
int ledG = 5;
int ledB = 3;  

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  Serial.begin(115200);
}

// the loop routine runs over and over again forever:
void loop() {
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    // look for the next valid integer in the incoming serial stream:
    Serial.println(str);
    int red = (int)str.substring(0,3).toInt();
    // do it again:
    int green = (int)str.substring(3,6).toInt();
    // do it again:
    int blue = (int)str.substring(6,9).toInt();

      red = constrain(red, 0, 255);
      green = constrain(green, 0, 255);
      blue = constrain(blue, 0, 255);

      // fade the red, green, and blue legs of the LED:
      analogWrite(ledR, red);
      analogWrite(ledG, green);
      analogWrite(ledB, blue);

      // print the three numbers in one string as hexadecimal:
      Serial.print(red, HEX);
      Serial.print(green, HEX);
      Serial.println(blue, HEX);
    
  }

}