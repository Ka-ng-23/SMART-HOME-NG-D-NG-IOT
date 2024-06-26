#define BLYNK_TEMPLATE_ID "TMPL6933I3B1i"
#define BLYNK_TEMPLATE_NAME "SMART HOME"
#define BLYNK_AUTH_TOKEN "-w65-MkipNMVWb7zDQM4pJh0k9sr-lkK"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>

#define SOFT_RX 5  
#define SOFT_TX 4  

char ssid[] = "...";  
char pass[] = "11111111";  

SoftwareSerial mySerial(SOFT_RX, SOFT_TX); 
String data; 

void setup() {
  Serial.begin(9600);
  mySerial.begin(115200); 
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  Blynk.run();
  
  if (mySerial.available()) {
    data = mySerial.readStringUntil('\n');
    float gasValue = getValue(data, ';', 0).toFloat(); 
    float t = getValue(data, ';', 1).toFloat(); 
    Blynk.virtualWrite(V0, gasValue); 
    Blynk.virtualWrite(V1, t);
    Serial.println("Giá trị khí gas: " + String(gasValue)); 
    Serial.println("Giá trị t: " + String(t)); 
    
    if (gasValue > 100) {
      Blynk.logEvent("canhbao", String("Cảnh báo! Khí gas=" + String(gasValue) + " vượt quá mức cho phép!"));
      Serial.println("CẢNH BÁO: Nồng độ khí gas vượt quá ngưỡng an toàn!");
    }
  }
}

String getValue(String data, char separator, int index){
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
      if (data.charAt(i) == separator || i == maxIndex) {
          found++;
          strIndex[0] = strIndex[1] + 1;
          strIndex[1] = (i == maxIndex) ? i+1 : i;
      }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
