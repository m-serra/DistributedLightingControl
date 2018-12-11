volatile boolean occupancy_state = false;
volatile boolean occupancy_state_prev = false;
const byte interruptPin = 2;
volatile int buttonState = 0;
const int debounceDelay = 2;
volatile float previous_t = 0.0, curr_t = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  attachInterrupt(0, button_pressed, RISING);
  //attachInterrupt(0,KeyPress,FALLING);

}

void loop() {
  // put your main code here, to run repeatedly:  
  if(occupancy_state != occupancy_state_prev){
    if(occupancy_state){Serial.println("OCCUPIED!");}
    else{Serial.println("Empty");}
  }

  occupancy_state_prev = occupancy_state;
}

void button_pressed(){

  curr_t = micros();
  buttonState = digitalRead(interruptPin);
  if(buttonState == 1 && ((curr_t - prev_t)> 10000)){occupancy_state = !occupancy_state;}

  prev_t = micros();
   
}

boolean debounce(int pin)
{
  boolean state;
  boolean previousState;
  
  // store switch state
  previousState = digitalRead(pin);
  
  for(int counter=0; counter < debounceDelay; counter++) {
    // wait for 1 millisecond
    delay(1);
    
    // read the pin
    state = digitalRead(pin);
    
    if( state != previousState) {
      
      // reset the counter if the state changes
      counter = 0;
      
      // and save the current state
      previousState = state;
    }
  }
  
  // here when the switch state has been stable
  // longer than the debounce period
  return state;
}
