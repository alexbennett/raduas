#define THRO_PIN 3
#define AILE_PIN 4
#define ELEV_PIN 5
#define RUDD_PIN 6
#define GEAR_PIN 7
#define AUX_PIN  8

void setup() 
{
  Serial.begin(9600);
  
  pinMode(THRO_PIN, INPUT);
  pinMode(AILE_PIN, INPUT);
  pinMode(ELEV_PIN, INPUT);
  pinMode(RUDD_PIN, INPUT);
  pinMode(GEAR_PIN, INPUT);
  pinMode(AUX_PIN, INPUT);
}

void loop() 
{
  int thro = pulseIn(THRO_PIN, HIGH, 25000);
  int aile = pulseIn(AILE_PIN, HIGH, 25000);
  int elev = pulseIn(ELEV_PIN, HIGH, 25000);
  int rudd = pulseIn(RUDD_PIN, HIGH, 25000);
  int gear = pulseIn(GEAR_PIN, HIGH, 25000);
  int aux = pulseIn(AUX_PIN, HIGH, 25000);
  
  Serial.println("Throttle: " + String(thro) + ", Aileron: " + String(aile) + ", Elevation: " + String(elev) + ", Rudder: " + String(rudd) + ", Gear: " + String(gear) + ", Aux: " + String(aux)); 
}
