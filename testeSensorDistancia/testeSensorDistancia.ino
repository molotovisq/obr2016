int frontSensorTrigger = 53;
int frontSensorEcho = 54;

int sideSensorTrigger = 23;
int sideSensorEcho = 24;


void setup() {
  pinMode(frontSensorTrigger, OUTPUT);
  pinMode(frontSensorEcho, INPUT);
  pinMode(sideSensorTrigger, OUTPUT);
  pinMode(sideSensorEcho, INPUT);

  Serial.begin(9600);
}

void loop() {
  Serial.print(getDistance(frontSensorTrigger, frontSensorEcho));
  Serial.print("  ");
  Serial.print(getDistance(sideSensorTrigger, sideSensorEcho));
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
