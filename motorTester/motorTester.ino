
const int minRange = 4;
const int maxRange = 13;
const int pwm = 200;
const int interval = 2000;
int port = minRange - 1;

void setup() {
  Serial.begin(9600);
}

void loop() {

  if (port < maxRange) {
    port++;
    analogWrite(port, pwm);
    Serial.println(port);
    delay(interval);
    analogWrite(port, 0);
  }

  if (port == maxRange)
    port = minRange;

}
