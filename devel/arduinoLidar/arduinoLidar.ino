

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  if (Serial1.available()) {
    char ch=Serial1.read();
    Serial.write(ch);
  }
  if (Serial.available()) {
    char ch=Serial.read();
    Serial1.write(ch);
  }
}
