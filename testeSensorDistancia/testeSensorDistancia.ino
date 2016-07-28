void setup() {
  pinMode(23, OUTPUT);
  pinMode(22, INPUT);
  pinMode(53, OUTPUT);
  pinMode(52, INPUT);

  Serial.begin(9600);
}

void loop() {
  Serial.print(getDistance(23, 22));
  Serial.print("  ");
  Serial.print(getDistance(53, 52));
  Serial.println("  ");

}

float getDistance(int trigPin, int echoPin) {
  float duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 7000); // 7000 Delay Microsseconds to limit range

  distance = (duration / (float)2) / 29.1;

  return distance;
}
