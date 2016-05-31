/*Criar condição para retornar frente quando encontrar preto em todos os sensores. !!!*/
/*
- seguir linha (com gap também)
- desviar de obstaculo (caixa de leite)
- encontrar vitima (bola de isopor revestida de papel aluminio) - mais de uma vítima
- encontrar base (arena preta triangular - 6cm altura)
- identificar se é seguidor, ou busca (colocar a vitima e dar um sinal de finalizado)

- verifica posição da linha na barra - atribui um valor (-7 a 7)

*/


int esperado = 0;

const float batteryMultiplier = 1.0;

const int motorLF = 7;
const int motorLB = 6;
const int motorRF = 5;
const int motorRB = 4;

int pwm_inicial_D = 90;
int pwm_inicial_E = 130;

int pwmD = 0;
int pwmE = 0;

const int quantityOfSensors = 8;

const int sensors[quantityOfSensors] = {A7, A6, A5, A4, A3, A2, A1, A0};
int sensorValues[quantityOfSensors];

const int blackLimit = 650; // >= 540
const int whiteLimit = 150; // <= 55
const int minGreenValue = 100;
const int maxGreenValue = 450;
const int silverLimiar = 200; // Atualizar este valor

const float kP = 18;
const float kD = 4;


int pwmR = 0;
int pwmL = 0;
int globalError = 0;
int lastGlobalError = 0;

bool rescuing = false;

void setup() {
  Serial.begin(9600);

  //Motors
  pinMode(motorLF, OUTPUT);
  pinMode(motorLB, OUTPUT);
  pinMode(motorRF, OUTPUT);
  pinMode(motorRB, OUTPUT);
}

void loop() {

  if (!rescuing) { // Seguir Linha e desviar de obstáculo

    // Avaliar condição da pista
    //   Avaliar condição do verde
    //   Avaliar condição da linha preta
    //   Se tiver verde, curva para o lado do verde
    //   Senão se tiver linha preta do lado guardar uma memoria apontando para curva
    //     Se não tiver linha depois, curva para o lado da memoria
    //     Senão limpa a memoria da curva 
    
    for (int i = 0; i < quantityOfSensors; i++) { // Leitura dos valores dos sensores e armazenamento no vetor "sensorValues[]"
      sensorValues[i] = analogRead(sensors[i]);
    }
    globalError = readLine(sensorValues, quantityOfSensors);
    float pid = calculatePD(globalError, lastGlobalError);

    pwmD = pwm_inicial_D - pid;
    pwmE = pwm_inicial_E + pid;
    pwmD = constrain(pwmD, 0, 255);
    pwmE = constrain(pwmE, 0, 255);

    analogWrite(motorLF, pwmE);
    analogWrite(motorLB, 0);

    analogWrite(motorRF, pwmD);
    analogWrite(motorRB, 0);


    if (pwmE < 40 && pwmD > 40) {

      analogWrite(motorLF, 0);
      analogWrite(motorLB, 120);
      analogWrite(motorRF, 80);
      analogWrite(motorRB, 0);

    } else if (pwmE > 40 && pwmD < 40) {

      analogWrite(motorLF, 120);
      analogWrite(motorLB, 0);
      analogWrite(motorRF, 0);
      analogWrite(motorRB, 80);

    }  
    

    //Serial.print(pwmE);
    //Serial.print('\t');
    // Serial.print(globalError);
    //Serial.print('\t');
    //Serial.println(pwmD);
    Serial.print("\tglobalError: ");
    Serial.print(globalError);
    Serial.print("\tcrossLine: ");

  } else {
  

  }

}


float calculatePD(int error, int lastError) {
  int P = error;
  int D = error - lastError;

  lastError = error;
  updateLastError(lastError);

  return (P * kP) + (D * kD);
}

void updateLastError(int lastError) {
  lastGlobalError = lastError;
}

void updateError(int error) {
  globalError = error;
}

long readLine(int sensors[], int sensorQuantity) {
  long numerator = 0;
  long denominator = 0;

  for (int i = 0; i < sensorQuantity; i++) {
    numerator += (i * 1000 * (long)sensors[i]);
    denominator += sensors[i];
  }

  return map((numerator / denominator) , 950 , 6000 , -7 , 7);

}


void Front() {

  analogWrite(motorLF, 130);
  analogWrite(motorLB, 0);
  analogWrite(motorRF, 90);
  analogWrite(motorRB, 0);

}

void Back() {

  analogWrite(motorLF, 0);
  analogWrite(motorLB, 130);
  analogWrite(motorRF, 0);
  analogWrite(motorRB, 90);

}

void SpinL() {

  analogWrite(motorLF, 0);
  analogWrite(motorLB, 130);
  analogWrite(motorRF, 90);
  analogWrite(motorRB, 0);

}

void SpinR() {

  analogWrite(motorLF, 130);
  analogWrite(motorLB, 0);
  analogWrite(motorRF, 0);
  analogWrite(motorRB, 90);

}
void Stop() {

  analogWrite(motorLF, 0);
  analogWrite(motorLB, 0);
  analogWrite(motorRF, 0);
  analogWrite(motorRB, 0);

}
