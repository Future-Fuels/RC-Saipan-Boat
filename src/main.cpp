#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <Servo.h>
#include <nRF24L01.h>

int left_in1 = 22;
int left_in2 = 24;
int right_in3 = 26;
int right_in4 = 28;
int ena = 30;
int enb = 32;

int button1 = 48;
int button2 = 46;
int ledRed = 9;
int ledBlue = 34;

// receiver module (testing)
int rxCE = 36;
int rxCSN = 38;
int rxSCK = 52;
int rxMOSI = 51;
int rxMISO = 50;
int rxIR = 53;

RF24 rx(rxCE, rxCSN);
const byte address[6] = "06967";

// transmitter module (testing)
int txCE = 2;
int txCSN = 3;
int txSCK = 13;
int txMOSI = 11;
int txMISO = 12;
int txIR = 10;

RF24 tx(txCE, txCSN);

// servo speed
int SPEED = 45;

Servo servo;
const int servoPin = A5;
const int pOut = A6;

void setup() {
  pinMode(left_in1, OUTPUT);
  pinMode(left_in2, OUTPUT);
  pinMode(right_in3, OUTPUT);
  pinMode(right_in4, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);

  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(ledRed, OUTPUT);
  pinMode(ledBlue, OUTPUT);

  servo.attach(servoPin);
  pinMode(pOut, INPUT);

  Serial.begin(9600);

  digitalWrite(left_in1, LOW);
  digitalWrite(left_in2, LOW);
  digitalWrite(right_in3, LOW);
  digitalWrite(right_in4, LOW);
  analogWrite(ena, 0);
  analogWrite(enb, 0);

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();

  if (rx.begin()) {
    Serial.println("rx works");
  } else {
    Serial.println("rx doesnt work");
  }

  if (tx.begin()) {
    Serial.println("tx works");
  } else {
    Serial.println("tx doesnt work");
  }

  // rx.begin();
  // rx.openReadingPipe(0, address);
  // rx.setPALevel(RF24_PA_LOW);
  // rx.setChannel(123);
  // rx.startListening();

  // tx.begin();
  // tx.openWritingPipe(address);
  // tx.setPALevel(RF24_PA_LOW);
}

bool runningFwd = false;
int lastFwd = HIGH;
bool runningRev = false;
int lastRev = HIGH;

void loop() {
  //   if (rx.isChipConnected()) {
  //   Serial.println("rx chip is responding!");
  // } else {
  //   Serial.println("Receiver chip not found. Check wiring of pins 50-52, 11,
  //   and 12.");
  // }

  //   if (tx.isChipConnected()) {
  //   Serial.println("tx chip is responding!");
  // } else {
  //   Serial.println("tx chip not found. Check wiring of pins 50-52, 11,
  //   and 12.");
  // }
  // int fwd = digitalRead(button1);
  // if (fwd == LOW && lastFwd == HIGH) {

  //   runningFwd = !runningFwd;
  //   runningRev = false;

  //   delay(50);
  // }
  // lastFwd = fwd;

  // int rev = digitalRead(button2);
  // if (rev == LOW && lastRev == HIGH) {

  //   runningRev = !runningRev;
  //   runningFwd = false;

  //   delay(50);
  // }
  // lastRev = rev;

  // Serial.print("last: ");
  // Serial.print(lastFwd);
  // Serial.print(" ");
  // Serial.print(lastRev);
  // Serial.print(" running: ");
  // Serial.print(runningFwd);
  // Serial.print(" ");
  // Serial.println(runningRev);

  // if (runningFwd) {
  //   digitalWrite(ledRed, HIGH);
  //   digitalWrite(ledBlue, LOW);

  //   digitalWrite(left_in1, LOW);
  //   digitalWrite(left_in2, HIGH);
  //   analogWrite(ena, SPEED);

  //   digitalWrite(right_in3, LOW);
  //   digitalWrite(right_in4, HIGH);
  //   analogWrite(enb, SPEED);

  // } else if (runningRev) {
  //   digitalWrite(ledRed, LOW);
  //   digitalWrite(ledBlue, HIGH);

  //   digitalWrite(left_in1, HIGH);
  //   digitalWrite(left_in2, LOW);
  //   analogWrite(ena, SPEED);

  //   digitalWrite(right_in3, HIGH);
  //   digitalWrite(right_in4, LOW);
  //   analogWrite(enb, SPEED);
  // } else {
  //   digitalWrite(ledRed, LOW);
  //   digitalWrite(ledBlue, LOW);

  //   digitalWrite(left_in1, LOW);
  //   digitalWrite(left_in2, LOW);
  //   analogWrite(ena, 0);

  //   digitalWrite(right_in3, LOW);
  //   digitalWrite(right_in4, LOW);
  //   analogWrite(enb, 0);
  // }

  // Serial.print("tx available:");
  // Serial.print(tx.available());
  // Serial.print(",  rx avail:");
  // Serial.println(rx.available());

  // if (tx.available()) {
  //   char txText[32];

  //   tx.write(&txText, sizeof(txText));
  //   Serial.print("TX: ");
  //   Serial.print(txText);
  //   Serial.println();
  // }

  // if (rx.available()){
  //   char rxText [32] = {0};
  //   rx.read(&rxText, sizeof(rxText));
  //   Serial.print("RX: ");
  //   Serial.print(rxText);
  //   Serial.println();
  // }

  // delay(500);
}
