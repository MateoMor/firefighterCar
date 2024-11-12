#include <ESP32Servo.h>

Servo servo;

int pos = 0;    

/*-------defining Inputs------*/
#define Left_S 33      // left sensor
#define Forward_S 32 //forward sensor
#define Right_S 34      // right sensor

/*-------defining Outputs------*/
#define LM1 13       // left motor
#define LM2 12       // left motor
#define RM1 14       // right motor
#define RM2 27       // right motor
#define pump 25      // pump
#define servoPin 26     // servo

void setup() {
  Serial.begin(115200); // Iniciar comunicación serial a 115200 baudios
  pinMode(Left_S, INPUT);
  pinMode(Right_S, INPUT);
  pinMode(Forward_S, INPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(pump, OUTPUT);

  servo.attach(servoPin, 500, 2400);
  servo.write(90); 
  digitalWrite(pump, LOW);
  Serial.println("Robot listo.");
}

// Function to start fire extinguisher
void put_off_fire() {

  Serial.println("Apagando fuego...");

  digitalWrite(pump, HIGH); 
  delay(500);

  for (pos = 50; pos <= 130; pos += 1) { 
    servo.write(pos); 
    delay(12);  
  }
  for (pos = 130; pos >= 50; pos -= 1) { 
    servo.write(pos); 
    delay(12);
  }

  digitalWrite(pump, LOW);
  servo.write(90);

  Serial.println("Agua dispensada");
}

// Function to ask if all sensors are up
bool areAllUp() {
  return digitalRead(Left_S) == 0 && digitalRead(Right_S) == 0 && digitalRead(Forward_S) == 0;
}

void loop() {
  servo.write(90); // Sweep_Servo();

  // Leer el estado de cada pin
  int leftState = digitalRead(Left_S);
  int rightState = digitalRead(Right_S);
  int forwardState = digitalRead(Forward_S);

  // Imprimir el estado de cada pin
  Serial.print("Left Sensor: ");
  Serial.println(leftState); // Imprime 0 o 1 dependiendo del estado del pin

  Serial.print("Forward Sensor: ");
  Serial.println(forwardState);

  Serial.print("Right Sensor: ");
  Serial.println(rightState);

  if (areAllUp()) {
    // Si no hay fuego detectado
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
    delay(100);
    put_off_fire();
    // If fire didnt extinguish
    if(areAllUp()) {
      Serial.println("Fuego no apagado, moviendo adelante");
      digitalWrite(LM1, HIGH);
      digitalWrite(LM2, LOW);
      digitalWrite(RM1, HIGH);
      digitalWrite(RM2, LOW);
      delay(120);
    }
  } 
  else if (digitalRead(Left_S) == 0 && digitalRead(Right_S) == 0) { 
    // Si el del centro no está encendido pero a los lados si puede que el fuego esté debajo
    // en ese caso ir hacia atras
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, HIGH);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, HIGH);
    delay(800);
  }
  else if (digitalRead(Left_S) == 0) { // Si el fuego está a la izquierda mover llanda derecha
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
    Serial.println("Fuego detectado a la izquierda. Moviendo hacia la izquierda.");
  } 
  else if (digitalRead(Right_S) == 0) { // Si el fuego está a la derecha mover llanta izquierda
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
    Serial.println("Fuego detectado a la derecha. Moviendo hacia la derecha.");
  }
  else if (digitalRead(Forward_S) == 0) { // Si el fuego está al frente
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
    Serial.println("Fuego detectado al frente. Moviendo hacia adelante.");
  } else {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
  }
}
