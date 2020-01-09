/*
* Junior Design 1 - Project 2
* Group PID 11 - Bao, Zavi, Emilio
* Code typed, designed, and documented by Bao Nguyen
*/

long int  targetAngle;
long int  store;
double targetVolt;
double currentVolt;
double feedBack;

int Kp = 100;	// Require calibration
int Ki = 1.1;	// Require calibration 
int Kd = 70;	// Require calibration

double P;
double I;
double D;

// Var for the computePID func
unsigned long currentTime;
unsigned long previousTime;
double        lastError;

// For current reading
int storeCurrent;

void setup() {
  // put your setup code here, to run once:
pinMode(8, OUTPUT);   //  set direction
pinMode(9, OUTPUT);   //  set direction
pinMode(10, OUTPUT);  //  speed control

pinMode(A0, INPUT);		// for feedback
pinMode(A1, INPUT);		// for giving input - target
pinMode(A2, INPUT);		// for reading current

Serial.begin(9600);
// keep the DC motor at its current position
feedBack = analogRead(A0); 
}

void loop() {
  // Read in angle from potentiometer
  targetAngle = analogRead(A1)*0.264;	
  
  // Avarage out current reading
  for(int i = 0; i <20; i++)
  {
	  storeCurrent += analogRead(A2);
  }
  // Calculate to milli ampere
  storeCurrent = storeCurrent*0.1675; 		// Convert measured analog to miliamp 
  
  targetVolt = targetAngle*0.0185;          // Convert from angle => volt
        
  // Read in current voltage
  currentVolt = analogRead(A0)*0.0049;     // Map to 5V
  
  // Calculate feedBack to feed to the H-Bridge
  feedBack = computePID(currentVolt);
  // Set direction based on the sign of the feedBack
  if(abs(feedBack)>255){
    if(feedBack > 0){
      feedBack = 255;
    }
    if(feedBack < 0){
      feedBack = -255;
    }
  } 
  
  if(feedBack < 0.0){  // change the greater than equal potentially
      feedBack = abs(feedBack);
      digitalWrite(8, LOW);
      digitalWrite(9, HIGH);
      analogWrite(10, (int)feedBack);
  }
  else
  {   

      digitalWrite(8, HIGH);
      digitalWrite(9, LOW);
      analogWrite(10, (int)feedBack);
  }

  //Start printing target angle
  Serial.print(targetAngle);
  Serial.print(" ");
  //Print out the actual angle
  Serial.print(currentVolt*54);  
  Serial.print(" ");
  //Print out motor current
  Serial.println(storeCurrent*0.1675);
  // reset storeCurrent
  storeCurrent = 0;
  //Serial.print(" mA ");
  //Serial.println(feedBack);
  /*
  // Convert angle to target voltage
  //  270degree = 9.9k   ohm
  //  0 degree  = 0      ohm 
  */
}

double computePID(double input)
{
    // Compute P: Expect - current
    // Compute I(integral)  : Add to the Sum of CurrentError* Elapsed Time
    // Compute D(derivative): (CurrentError - LastError)/ Elapsed Time
    currentTime = millis();
    unsigned long elapsedTime = currentTime - previousTime;
    
    P = targetVolt - currentVolt;
    I = I + P * float(elapsedTime)*0.02;
    //Serial.print(I); 
    //Serial.print(" ");
    D = (P - lastError)/elapsedTime;
    lastError = P;
    double out = Kp*P + Ki*I + Kd*D; // modify KP, KI , and KD
    
    previousTime = currentTime;
    return out;
}
