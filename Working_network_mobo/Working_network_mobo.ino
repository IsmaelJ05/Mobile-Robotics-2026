#include <WiFi.h>
#include <HTTPClient.h>

// Wi-Fi details
const char* ssid     = "iot";
const char* password = "tiling6whillilew";

// Server details
const char* server = "http://3.250.38.184:8000";
const char* TEAM_ID = "diyh4437";   // Your team ID

String getRoute() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return "";
  }

  HTTPClient http;
  String url = String(server) + "/api/getRoute/" + TEAM_ID;

  Serial.print("Requesting route from: ");
  Serial.println(url);

  http.begin(url);
  int code = http.GET();

  String body = "";
  if (code > 0) {
    body = http.getString();
    body.trim();  // remove \r\n and spaces
    Serial.print("HTTP Status: ");
    Serial.println(code);
    Serial.print("Route body: ");
    Serial.println(body);
  } else {
    Serial.print("GET failed, error: ");
    Serial.println(code);
  }

  http.end();
  return body;
}

void connectToWiFi() {
  Serial.print("Connecting to network: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void sendArrival(int position) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return;
  }

  HTTPClient http;

  String url = String(server) + "/api/arrived/" + TEAM_ID;
  String postData = "position=" + String(position);

  Serial.println("Sending POST to server...");
  Serial.println(postData);

  http.begin(url);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  int httpResponseCode = http.POST(postData);

  if (httpResponseCode > 0) {
    String response = http.getString();

    Serial.print("HTTP Status: ");
    Serial.println(httpResponseCode);

    Serial.print("Server response: ");
    Serial.println(response);

    if (response == "Finished") {
      Serial.println("Route complete!");
    } else {
      int nextDestination = response.toInt();
      Serial.print("Next destination: ");
      Serial.println(nextDestination);
    }
  } 
  else {
    Serial.print("POST failed, error: ");
    Serial.println(httpResponseCode);
  }

  http.end();  // close connection
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  connectToWiFi();

  String route = getRoute();   // e.g. "3,1,4,2" or similar

  // Now notify server you arrived at the start
  sendArrival(0);

}
void loop() {
  // Later you can call sendArrival(newPosition);
}

