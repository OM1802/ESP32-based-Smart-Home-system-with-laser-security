/*
This code is for an IoT project using an ESP32 microcontroller. It involves reading data from various sensors like DHT22 (temperature and humidity sensor) and 
an LDR (light-dependent resistor) and controlling output devices like an LED, a fan, a buzzer, and a laser module. The project is connected to the Blynk IoT platform 
for remote monitoring and control.
*/

#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL3uU_6KK1A"
#define BLYNK_TEMPLATE_NAME "HomeSecurity"
#define BLYNK_AUTH_TOKEN "6o-ayDgrmyjdsUVb0u6aQtEsc4dWcoCi"

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
float t,h;
#define DHTPIN 12     // Connect the DHT11 sensor's pin to GPIO 21
#define DHTTYPE DHT22 // DHT 22
#define FANPIN 2      // Connect the relay module to GPIO 2
// VIRTUALFAN V0 // Virtual pin for the fan control
///ALERT V1  // Virtual pin for fan alert
#define LDRPIN 15      // Connect LDR to GPIO 4
#define LEDPIN 23     // Connect LED to GPIO 26
#define LASERPIN 18   // Connect the laser module to GPIO 18
#define BUZZERPIN 4   // Connect the buzzer to GPIO 4
//#define INTRUDER_ALERT V2 // Virtual pin for intruder alert


char auth[] = BLYNK_AUTH_TOKEN; // Your Blynk Auth Token
char ssid[] = "om";
char pass[] = "hello12345";

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows

#define BUZZER_CONTROL V3 // Virtual pin for buzzer control

bool buzzerState = false; // Initialize buzzer state as off

BLYNK_WRITE(BUZZER_CONTROL) {
  // Update buzzer state based on value received from Blynk app
  buzzerState = param.asInt();
}


 int fanstate = 0;
BLYNK_WRITE(V0) {
  fanstate =param.asInt();
  if(fanstate==1){
    digitalWrite(FANPIN, LOW); // Control the fan based on the received value
  }
  
  else{
    digitalWrite(FANPIN,HIGH);
  }
}

// Send temperature data to Blynk app
BLYNK_WRITE(V2){
  Blynk.virtualWrite(V2, t);
}

// Send humidity data to Blynk app
BLYNK_WRITE(V1){
  Blynk.virtualWrite(V1, h);
}


void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  dht.begin();
  Blynk.begin(auth, ssid, pass);
  pinMode(FANPIN, OUTPUT);
  digitalWrite(FANPIN, LOW); // Ensure fan is initially turned off
  pinMode(LEDPIN, OUTPUT);   // Set LED pin as output
  pinMode(LASERPIN, INPUT); // Set the laser pin as input
  pinMode(LDRPIN, INPUT); 
  pinMode(BUZZERPIN, OUTPUT); // Set the buzzer pin as output


  analogReadResolution(10);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  Blynk.run();
  
   h = dht.readHumidity();
   t = dht.readTemperature();
Blynk.virtualWrite(V2, t);
  Blynk.virtualWrite(V1, h);
  if (isnan(h) || isnan(t)) {
    lcd.setCursor(5, 0);
    lcd.print("Error!");
    return;
  }

  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.print(t);
  lcd.print(" 'C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity:");
  lcd.print(h);
  lcd.print("%");
  delay(2000);
  lcd.clear();

  if (t > 25) {
    lcd.setCursor(0,0);
    lcd.print("Turn On Fan!");
  }

  // Read LDR value
 int ldrValue = digitalRead(LDRPIN); // Read digital value from LDR module output pin
  Serial.println(ldrValue); // Print LDR value to serial monitor

  // Convert digital value to LED brightness control
  int brightness = ldrValue == HIGH ? 255 : 0; // If LDR output is HIGH, set LED brightness to maximum, else turn off LED
  analogWrite(LEDPIN, brightness); // Set LED brightness
  delay(25); // Add a short delay for stability

 int laserState = digitalRead(LASERPIN); // Read the state of the laser module
if (laserState == LOW && buzzerState == true) { // If the laser beam is broken and the buzzer is off
  buzzerState = false; // Set buzzer state to on
  for (int i = 0; i < 5; i++) { // Make the buzzer beep 5 times
    lcd.setCursor(0,0);
    lcd.print("INTRUDER ALERT!");
    Blynk.virtualWrite(V2,1);
    digitalWrite(BUZZERPIN, HIGH);
    delay(2000);
    digitalWrite(BUZZERPIN, LOW);
    delay(2000);
  }
} else if (buzzerState == false) { // If the buzzer is turned off from the Blynk app
  digitalWrite(BUZZERPIN, LOW); // Turn off the buzzer
}

}
