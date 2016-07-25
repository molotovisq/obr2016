const int quantityOfSensors = 8;
const int sensorPort[quantityOfSensors] = {A7, A6, A5, A4, A3, A2, A1, A0};
bool withNames = 0;
int sensorValue[quantityOfSensors];


void setup() {
  Serial.begin(9600);
}


void loop(){
  for (int i = 0; i < quantityOfSensors; i++) {
    sensorValue[i] = analogRead(sensorPort[i]);
  }

  if (withNames == 1) {
    for (int j = 0; j < quantityOfSensors; j++) {
      Serial.print(" Sensor ");
      Serial.print(j);
      Serial.print(": ");
      Serial.print(sensorValue[j]);

      if (j == 7)
        Serial.println("");
    }
    
  } else {
    
    for (int j = 0; j < quantityOfSensors; j++) {
      Serial.print(sensorValue[j]);
      Serial.print(" ");

      if (j == 7)
        Serial.println("");
    }
  }
}
