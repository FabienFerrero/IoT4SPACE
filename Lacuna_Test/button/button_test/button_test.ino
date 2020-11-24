
  // variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

 

void setup() {
Serial.begin(9600);
  
  // initialize the LED pin as an output:
  pinMode(LS_LED_BLUE, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(LS_USER_BUTTON, INPUT);

 
}

void loop() {
  // put your main code here, to run repeatedly:

  
   // read the state of the pushbutton value:
  buttonState = digitalRead(LS_USER_BUTTON);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(LS_LED_BLUE, HIGH);
  } else {
    // turn LED off:
    digitalWrite(LS_LED_BLUE, LOW);
  }


}
