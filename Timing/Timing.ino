/* 
Code for setting sampling rate
https://forum.arduino.cc/t/struggling-to-obtain-fixed-sampling-frequency/278312/2
輸出十筆資料
*/


//unsigned long t0;
//float sampling_rate = 1000*1000; // 1000 ms
//static long prevMicros = 0;

void setup() {
  Serial.begin(115200);

  for(int counter = 0; counter < 10; counter++) {
    Serial.println(counter);
  }
}

void loop() {

}

//  for(int counter = 0; counter < 10; counter ++){
//    t0=micros();
//    if (micros() - prevMicros >= sampling_rate) {
//    Serial.print("time");
//    Serial.println(micros()/1000000);
//    prevMicros += sampling_rate;
//    }
//    }
