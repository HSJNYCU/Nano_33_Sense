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
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  

}

void loop() {
Serial.println("Hello");
}

//  for(int counter = 0; counter < 10; counter ++){
//    t0=micros();
//    if (micros() - prevMicros >= sampling_rate) {
//    Serial.print("time");
//    Serial.println(micros()/1000000);
//    prevMicros += sampling_rate;
//    }
//    }
