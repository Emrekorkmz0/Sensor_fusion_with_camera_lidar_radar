#include <FreqMeasure.h>

void setup() {
  Serial.begin(9600);
  FreqMeasure.begin();
}

double sum = 0;
int count = 0;

void loop() {
  if (FreqMeasure.available()) {
    sum += FreqMeasure.read();
    count++;

    if (count > 30) {
      float frequency = FreqMeasure.countToFrequency(sum / count);
      float speed = frequency / 31.0; // HB100 için genel oran

      
      if (frequency > 30.0) {
        Serial.print("Hareket algılandı -> ");
        Serial.print("frequency: ");
        Serial.print(frequency);
        Serial.print(" Hz, speed: ");
        Serial.print(speed);
        Serial.println(" m/s");
      } else {
        Serial.println("Hareket yok.");
      }

      delay(50);
      sum = 0;
      count = 0;
    }
  }
}
