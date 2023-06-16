//www.elegoo.com
//2016.12.09

// Arduino pin numbers
const int SI_pin = 8; // digital pin connected to switch output
const int ClockOut_pin = 13;
const int ledTes_pin = 7; 

byte pShiftOut(int clockInput, int SI_inputPin){

byte byteStore = 0;
byte bitInputStore = 0;

for(int i = 0; i < 8; i++){

digitalWrite(clockInput,HIGH);
delay(500);
bitInputStore = digitalRead(SI_inputPin);
bitInputStore <<= 7;
byteStore >>= 1;
byteStore |= bitInputStore;
digitalWrite(clockInput,LOW);
delay(4000);

}

return byteStore;
}

void setup() {
  pinMode(SI_pin, INPUT);
  pinMode(ClockOut_pin, OUTPUT);
  pinMode(ledTes_pin, OUTPUT);
  Serial.begin(9600);
}

byte aByte = 0;
void loop() {
  
digitalWrite(ledTes_pin,HIGH);
delay(5000);
digitalWrite(ledTes_pin,LOW);

aByte = pShiftOut(ClockOut_pin, SI_pin);

Serial.print(aByte , BIN);
  
}
