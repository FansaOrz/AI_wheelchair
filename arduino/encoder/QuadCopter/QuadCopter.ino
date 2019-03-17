 int encoder0PinA =7;
 int encoder0PinB = 8;
 int n = LOW;
 int m = LOW;

 void setup() { 
   pinMode (encoder0PinA,INPUT);
   pinMode (encoder0PinB,INPUT);
   Serial.begin (9600);
 } 

 void loop() {
   delay(10);
   n = digitalRead(encoder0PinA);
   Serial.print(n);
   delay(10);
   m = digitalRead(encoder0PinB);
   Serial.println(m);
   
 } 
