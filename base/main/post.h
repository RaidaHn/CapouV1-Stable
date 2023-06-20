/*
  Rui Santos
  Complete project details at Complete project details at https://RandomNerdTutorials.com/esp32-http-get-post-arduino/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <WiFi.h>
#include <HTTPClient.h>

//Your Domain name with URL path or IP address with path
const char* postServerName = "http://10.82.118.236:8000/bdd/receiveData/"; /* sends values           */
const char* postValveState = "http://10.82.118.236:8000/bdd/valveState/";  /* sends back valve state */


// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long postLastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long postTimerDelay = 5000;

void PostLoop(char postData[200], int postCode) {
  //Send an HTTP POST request every 10 minutes
  if ((millis() - postLastTime) > postTimerDelay) {
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      WiFiClient client;
      HTTPClient http;
      String jsonData;
    
      if(postCode == 1){
        http.begin(client, postServerName);
      }
      if((postCode == 2)||(postCode == 3)){
        printf("postCode = %d\n", postCode);
        http.begin(client, postValveState);
      }

      /* Header for JSON request */
      http.addHeader("Content-Type", "application/json");      
      
      /* Data to be sent */
      // String jsonData = "{\"node_id\":\"0x1B\",\"longitude\":\"1.313292\",\"latitude\":\"44.031909\",\"internal_humidity\":\"15\",\"external_humidity\":\"37\",\"internal_temperature\":\"27\",\"external_temperature\":\"33\"}"; /* example to send values */
      // String jsonData = "{\"node_id\": \"0x21\", \"valve_state\": \"on\"}"; /* example to send valve state */
      if(postCode == 1){
        jsonData = postData;  /* to send data received from probe */
        // String jsonData = "{\"node_id\":\"0x1B\",\"longitude\":\"1.313292\",\"latitude\":\"44.031909\",\"internal_humidity\":\"15\",\"external_humidity\":\"37\",\"internal_temperature\":\"27\",\"external_temperature\":\"33\"}"; /* example to send values */
      }
      else if(postCode == 2){
        jsonData = "{\"node_id\": \"0x21\", \"valve_state\": \"on\"}";
      }
      else if(postCode == 3){
        jsonData = "{\"node_id\": \"0x21\", \"valve_state\": \"off\"}";
      }
     
      int httpResponseCode = http.POST(jsonData);      /* POST request */

      Serial.print("HTTP POST Response code: ");
      Serial.println(httpResponseCode);
        
      // Free resources
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    postLastTime = millis();
  }
}