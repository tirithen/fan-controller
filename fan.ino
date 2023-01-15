const byte temperatureSensorPinA = A0;
const byte temperatureSensorPinB = A1;
const float minTemperature = 25;
const float maxTemperature = 45;

const byte OC1A_PIN = 9;
const byte OC1B_PIN = 10;

const word PWM_FREQ_HZ = 25000; //Adjust this value to adjust the frequency
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ);

void setPwmDuty(byte duty) {
  OCR1A = (word)(duty * TCNT1_TOP) / 100;
}

float readTemperature(byte pin) {
  // Get a reading from the temperature sensor:
  int reading = analogRead(pin);

  // Convert the reading into millivolt:
  float milliVolt = reading * (1100.0 / 1024.0);

  // Convert the voltage into the temperature in degree Celsius:
  float temperature = milliVolt / 10;

  return temperature;
}

void setup() {
  pinMode(temperatureSensorPinA, INPUT);
  pinMode(temperatureSensorPinB, INPUT);
  analogReference(INTERNAL);
  
  Serial.begin(115200);

  pinMode(OC1A_PIN, OUTPUT);

  // Clear Timer1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
}

byte printCounter = 0;

void loop() {
  const int samples = 100;
  float totalA = 0.0;
  float totalB = 0.0;
  for (int i = 0; i < samples; i++) {
    totalA += readTemperature(temperatureSensorPinA);
    totalB += readTemperature(temperatureSensorPinB);
  }
  float temperatureA = totalA / samples;
  float temperatureB = totalB / samples;
  float temperature = max(temperatureA, temperatureB);

  int constrained = (int) (constrain(temperature, minTemperature, maxTemperature) * 100);
  byte fanSpeed = map(constrained, minTemperature * 100, maxTemperature * 100, 0, 100);
  setPwmDuty(fanSpeed);

  printCounter += 1;
  if (printCounter > 70) {
    Serial.print("Temp: ");
    Serial.print(temperature);
    Serial.print("*C");
    Serial.print(", Speed: ");
    Serial.print(fanSpeed);
    Serial.println("%");
    printCounter = 0;
  }
}
