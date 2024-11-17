#include <ESP32Servo.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

/*-------defining Inputs------*/
#define Left_S 33         // left sensor
#define Forward_S 32      // forward sensor
#define Right_S 34        // right sensor 

/*-------defining Outputs------*/
#define LM1 13            // left motor
#define LM2 12            // left motor
#define RM1 26            // right motor
#define RM2 27            // right motor
#define pump 25           // pump
#define servoPin 23       // servo

/*-------Variables------*/
String device_name = "FireFighter"; // Bluetooth device name
Servo servo;              // servo motor
int pos = 0;              // servo position
bool automatic = true;    // current mode
BluetoothSerial SerialBT; // Bluetooth connection

/*-------Manual Controls------*/
bool manualForward = false;   // Flag for forward movement
bool manualBackward = false;  // Flag for backward movement

// function prototypes
void put_off_fire();
bool areAllUp();
String readBluetoothMessage();
void automaticMode();
void manualMode();

void setup() {
  Serial.begin(115200);
  pinMode(Left_S, INPUT);
  pinMode(Right_S, INPUT);
  pinMode(Forward_S, INPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(pump, OUTPUT);

  SerialBT.begin(device_name); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  servo.attach(servoPin, 500, 2400);
  servo.write(90); 
  digitalWrite(pump, LOW);
  Serial.println("Robot listo.");
}

void loop() {
  String message = readBluetoothMessage();
  if (message.length() > 0) {
    Serial.println("Command: " + message);
  }
  message.trim();

  // Auto: Toggle automatic
  if(message == "Auto"){
    automatic = !automatic;
    if(automatic) {
      SerialBT.println("Auto Mode");
    } else {
      SerialBT.println("Manual Mode");
    }
  }
  
  if(automatic) {
    automaticMode();
  } else {
    manualMode(message);
  }
}

// Definición de las funciones debajo de setup() y loop()

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

bool areAllUp() {
  return digitalRead(Left_S) == 0 && digitalRead(Right_S) == 0 && digitalRead(Forward_S) == 0;
}

String readBluetoothMessage() {
  String message = "";

  // Verificar si hay datos disponibles en el puerto Bluetooth
  while (SerialBT.available()) {
    char incomingChar = SerialBT.read();  // Leer el carácter recibido

    // Agregar el carácter al mensaje
    message += String(incomingChar);
  }

  return message;
}

void automaticMode() {
  servo.write(90);

/*
  int leftState = digitalRead(Left_S);
  int rightState = digitalRead(Right_S);
  int forwardState = digitalRead(Forward_S);

  Serial.print("Left Sensor: ");
  Serial.println(leftState);

  Serial.print("Forward Sensor: ");
  Serial.println(forwardState);

  Serial.print("Right Sensor: ");
  Serial.println(rightState);
*/

  if (areAllUp()) {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
    delay(100);
    put_off_fire();
    if (areAllUp()) {
      // Serial.println("Fuego no apagado, moviendo adelante");
      digitalWrite(LM1, HIGH);
      digitalWrite(LM2, LOW);
      digitalWrite(RM1, HIGH);
      digitalWrite(RM2, LOW);
      delay(120);
    }
  } 
  else if (digitalRead(Left_S) == 0 && digitalRead(Right_S) == 0) {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, HIGH);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, HIGH);
    delay(800);
  }
  else if (digitalRead(Left_S) == 0) {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
    // Serial.println("Fuego detectado a la izquierda. Moviendo hacia la izquierda.");
  } 
  else if (digitalRead(Right_S) == 0) {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
    // Serial.println("Fuego detectado a la derecha. Moviendo hacia la derecha.");
  } 
  else if (digitalRead(Forward_S) == 0) {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
    // Serial.println("Fuego detectado al frente. Moviendo hacia adelante.");
  } else {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
  }
}

void manualMode(String command) {
  // Valid {command} values
  // UP: Active move forward
  // DOWN: Active move backward
  // LEFT: Turn left
  // RIGHT: Turn right

  // Active the signals for forward and backward
  if (command == "UP"){
    if (manualForward) {
      manualForward = false;
    } else {
      manualForward = true;
    }
    manualBackward = false;
  } else if (command == "DOWN") {
    if (manualBackward) {
      manualBackward = false;
    } else {
      manualBackward = true;
    }
    manualForward = false;
  }

  // Turn the car left or right
  if (command == "LEFT") {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, HIGH);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
    delay(160);
  } else if (command == "RIGHT") {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, HIGH);
    delay(160);
  } 

  // Stop the car
  if(command == "Stop") {
    manualForward = false;
    manualBackward = false;
  }

  // Stop the car if both forward and backward are active
  if (manualForward == manualBackward){
    manualForward = false;
    manualBackward = false;
  }

  // Move the car according to the command
  if(manualForward) {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
  } else if(manualBackward) {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, HIGH);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, HIGH);
  } 

  // Stop the car if no command is active
  if(!manualForward && !manualBackward) {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, LOW);
  }
}
