#include <Wire.h>
#include "MAX30105.h"
#include <HeartRate.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// AWS IoT configuration for secure cloud connectivity
#define THINGNAME "ESP32"
const char WIFI_SSID[] = "Mindset";
const char WIFI_PASSWORD[] = "Mindset#";
const char AWS_IOT_ENDPOINT[] = "a8w8clg929m6t-ats.iot.eu-north-1.amazonaws.com";

// AWS certificates for encrypted MQTT communication
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM8MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUY+cmmKz6gpnWVX1DsqZA43FPbpAwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI1MDUwNzA4NTQ1
MFoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBANZgzQzNClGemHQt3uVQ
+qc129Y7OKk+0LNNlYBDUUXV7J6v93TUSNDEZC85n2LW8vQtoDML1Slg/eElUKVu
oblIv8E88JTVbg8FOOKbCmZR0B967k+b35JpaOrMDXdERIvB2fjbMJ51ZPRV17qa
GVBEp6t8AWznEC8uMQy8hOGQ48YN7qqfZ6v2PViyQ1ONkNnrpmV6tfOQSeBtk1k9
zINPSBx7MGMhWVZ6nXVUokn049M2eBMaOHmxD7zON4ak/3COZQl0N1IASPSZwNUb
dyEsMOYo+orZG1Z6LOz3CbNvmo8ybzs6K8VsHBuQ+5on0pNxxXJLOaEWIM92ODAQ
PX0CAwEAAaNgMF4wHwYDVR0jBBgwFoAUy7INyaIJNNkpjNDc4l3tnMZpSxkwHQYD
VR0OBBYEFOdhBpcLQxWJcATz2iB4VO94ifFHMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQCh5GZo23/lHz3drYq5AQJ/VJ7x
vEQ8VqOuRelpHyDcHX1OXgWaxze1okT0B9xcc5lAfI9eKtZjR6saTcGahZt4GIki
XsPvUw/XYaj3lzmaCP3GRTYd1usLl9ZIA2l8MV1M9F1T4EEHqgx7F+AD9vaipqhF
RLbH9nrkUEKdE8x3OiYyGn/q8arzaG/5pM0RzPeO6kbDcxQzR3aRVPu7HQL4clzL
ZouzKFo2uc4iYXrk1UZBspPnyfkmznRJkfBrJbsnfIJJ973Te4al100RivsofLHO
9kudl3NBYOo3+CIQWFvtpwQz2cpQXC6bDKxt32IyI8KC0Q8xcm5UsXulxSvq
-----END CERTIFICATE-----
)KEY";

static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEA1mDNDM0KUZ6YdC3e5VD6pzXb1js4qT7Qs02VgENRRdXsnq/3
dNRI0MRkLzmfYtby9C2gMwvVKWD94SVQpW6huUi/wTzwlNVuDwU44psKZlHQH3ru
T5vfkmlo6swNd0REi8HZ+NswnnVk9FXXupoZUESnq3wBbOcQLy4xDLyE4ZDjxg3u
qp9nq/Y9WLJDU42Q2eumZXq185BJ4G2TWT3Mg09IHHswYyFZVnqddVSiSfTj0zZ4
Exo4ebEPvM43hqT/cI5lCXQ3UgBI9JnA1Rt3ISww5ij6itkbVnos7PcJs2+ajzJv
OzorxWwcG5D7mifSk3HFcks5oRYgz3Y4MBA9fQIDAQABAoIBABSeJLppceedqC3s
R3aojiLMxELbWTy1xN6pxIocb1GXHHW6mMMMs6AYc0YzdYfNAnrHdJPUPIEJSgGA
VDxE4pruSzrrgHvf9wDM0MbCgIKMtcaBTTAs3pi3OJb0Xwssbqc+UpdjhU6IHdh9
9QyGS47zu2h7uateoajX+XGvI9yJpJ4LnK74fdkCUPnALSC59qd1gb4jHyYWHbAI
8AUyfRY603XyJxxfDV+GFhAL53LIferns13VvitQZuO9GBtHUlG1VB8lLj31yTfY
A/7sti6QKsxkrFlxXIHnG1L7AfQSJ9IL551Mfb6yPc49ijSE+T9dGLYXTZS0auf1
8A09boECgYEA/IoFw7lWO85zH/U6wMb8hhZgQ5lNl3k21RwaEfmTovVWpbhCAIWq
m2vKofOso0+PCkGfNdJkbQVK48c6oEJuuWMDiPByELd5J2smqHS88nRn72gPR9Nz
N2+qY0Z2CfgW/kJKCJv0nFZBfr8knbc7VqaBjSau3GgyXnFfKvCaxqECgYEA2VDm
JJU0BJw8fCRYFBQjPiAZ0QruzZqWY3/IRd3Lx83nWDyZKMbVlvFpNM9GIgrEswyM
unQAz8HzoKRi+vZqcL0Y5jt6j209h0DcLJr9R72G1JRWrCwt9vCI0rRCVPCrguhQ
bl8YRasyZHrKK67jUH5ozfTGK8CLuHElJRT+9V0CgYAX7fQx4aP8MsnR42jXz4Mq
KYChpyslCUVa2DjYLzAAJwM43MbUSdYquAIRaoeXSfih3in57Z+6fN/lyYESonjf
dDRni3EjiF8gjxSwra8hwkn+83tPMQgf3qLkmU6iDzIOkbV2L2D0V3AJVI9hqnUi
9tAb0eiL1gu/yzpbVUU1QQKBgQCsfBJ+/NHRyvQTEIG8RhOG7tGu4v+Rv14YUUbz
dUyWJO63UTm7bL4A1VHMsauXv3ZKKu7T2tj+DZJCcdeCG+112BEYRCNODcWlQay/
1rCrrVmEBs0YUOSWuGnQ23Q+mNt4xs14HRBV08DmkbM/ossXi43B0sMr3OLJHTz/
Rr/qnQKBgQDCiQ9azIqaRWWonXmyThsCBAofyWdaI4uM4nhQFfE3WFduZw3OCHSv
9bLKGt09ZaCq8IZoHZPMIBSorKwdNx8Yi5goCw6KmtLXrqbJZ38XVQgPgmkJgp3a
yI41Ok32bOZzTLQTl9y9/FtpZSWt6IKUyxVnYFFqbpK+WUNILt0sGw==
-----END RSA PRIVATE KEY-----
)KEY";

// Initialize sensors for cardiac monitoring
MAX30105 particleSensor;
HeartRate heartRate;

// AD8232 ECG pin configuration
#define ECG_PIN A0
#define LO_PLUS 10
#define LO_MINUS 11

// Data storage for sensor readings
float heartRateBPM = 0;
unsigned long lastBeatTime = 0;
float ecgSignal[100]; // Raw ECG buffer
float filteredEcgSignal[100]; // Filtered ECG buffer
int ecgIndex = 0;
float oldpeak = 0;
int slope = 1; // 0=downsloping, 1=flat, 2=upsloping
int restecg = 0; // 0=normal, 1=ST-T abnormality, 2=hypertrophy

// User input variables
float user_age = 0.0;
int user_sex = 0;
int user_cp = 0;

// MQTT client for AWS IoT Core
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
const char* mqttTopic = "cardioGuard/data";

// Moving average filter for ECG noise reduction
#define FILTER_WINDOW 5
float filterBuffer[FILTER_WINDOW];
int filterIndex = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("CardioGuard: Real-time Cardiac Monitoring System");

  // Collect user inputs with validation
  Serial.println("Enter age (18-100):");
  while (!getValidAge()) { delay(100); }
  
  Serial.println("Enter sex (0=female, 1=male):");
  while (!getValidSex()) { delay(100); }
  
  Serial.println("Enter chest pain type (0=none, 1=typical angina, 2=atypical, 3=non-anginal):");
  while (!getValidCp()) { delay(100); }

  // Initialize MAX30102 for heart rate monitoring
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 initialization failed!");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);

  // Configure AD8232 pins for ECG
  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MINUS, INPUT);
  pinMode(ECG_PIN, INPUT);

  // Initialize filter buffer
  for (int i = 0; i < FILTER_WINDOW; i++) {
    filterBuffer[i] = 0.0;
  }

  // Establish Wi-Fi connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected");

  // Set up AWS IoT certificates
  wifiClient.setCACert(AWS_CERT_CA);
  wifiClient.setCertificate(AWS_CERT_CRT);
  wifiClient.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to AWS IoT Core
  mqttClient.setServer(AWS_IOT_ENDPOINT, 8883);
  connectToAWS();

  Serial.println("System ready.");
}

// Validate and store age input
bool getValidAge() {
  if (Serial.available()) {
    float input = Serial.parseFloat();
    if (input >= 18 && input <= 100) {
      user_age = input;
      Serial.println(user_age);
      while (Serial.available()) Serial.read();
      return true;
    }
    Serial.println("Invalid age! Enter 18-100:");
    while (Serial.available()) Serial.read();
  }
  return false;
}

// Validate and store sex input
bool getValidSex() {
  if (Serial.available()) {
    int input = Serial.parseInt();
    if (input == 0 || input == 1) {
      user_sex = input;
      Serial.println(user_sex);
      while (Serial.available()) Serial.read();
      return true;
    }
    Serial.println("Invalid sex! Enter 0 or 1:");
    while (Serial.available()) Serial.read();
  }
  return false;
}

// Validate and store chest pain type input
bool getValidCp() {
  if (Serial.available()) {
    int input = Serial.parseInt();
    if (input >= 0 && input <= 3) {
      user_cp = input;
      Serial.println(user_cp);
      while (Serial.available()) Serial.read();
      return true;
    }
    Serial.println("Invalid cp! Enter 0-3:");
    while (Serial.available()) Serial.read();
  }
  return false;
}

// Connect to AWS IoT Core with retry
void connectToAWS() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to AWS IoT...");
    if (mqttClient.connect(THINGNAME)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying...");
      delay(5000);
    }
  }
}

// Validate ECG signal quality
bool isValidSignal(float* signal, int size) {
  float mean = 0, variance = 0;
  for (int i = 0; i < size; i++) {
    mean += signal[i];
  }
  mean /= size;
  
  for (int i = 0; i < size; i++) {
    variance += pow(signal[i] - mean, 2);
  }
  variance /= size;
  
  return variance > 0.01 && variance < 1.0; // Tuned for ECG stability
}

// Apply moving average filter to ECG signal
float applyMovingAverageFilter(float newSample) {
  filterBuffer[filterIndex] = newSample;
  filterIndex = (filterIndex + 1) % FILTER_WINDOW;
  float sum = 0;
  for (int i = 0; i < FILTER_WINDOW; i++) {
    sum += filterBuffer[i];
  }
  return sum / FILTER_WINDOW;
}

void loop() {
  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    connectToAWS();
  }
  mqttClient.loop();

  // Verify AD8232 electrode connection
  if (digitalRead(LO_PLUS) || digitalRead(LO_MINUS)) {
    Serial.println("ECG leads disconnected!");
    return;
  }

  // Measure heart rate with MAX30102
  long irValue = particleSensor.getIR();
  if (heartRate.checkForBeat(irValue)) {
    long delta = millis() - lastBeatTime;
    lastBeatTime = millis();
    float tempBPM = 60.0 / (delta / 1000.0);
    if (tempBPM >= 30 && tempBPM <= 200) {
      heartRateBPM = tempBPM;
      Serial.print("Heart Rate: ");
      Serial.print(heartRateBPM);
      Serial.println(" bpm");
    }
  }

  // Read and filter ECG signal from AD8232
  float ecgValue = analogRead(ECG_PIN) * (5.0 / 1023.0);
  ecgSignal[ecgIndex] = ecgValue;
  filteredEcgSignal[ecgIndex] = applyMovingAverageFilter(ecgValue);
  ecgIndex = (ecgIndex + 1) % 100;

  // Analyze ECG data when buffer is full
  if (ecgIndex == 0) {
    if (!isValidSignal(filteredEcgSignal, 100)) {
      Serial.println("Invalid ECG signal!");
      return;
    }

    // Calculate ECG metrics
    float maxEcg = filteredEcgSignal[0];
    float minEcg = filteredEcgSignal[0];
    for (int i = 1; i < 100; i++) {
      if (filteredEcgSignal[i] > maxEcg) maxEcg = filteredEcgSignal[i];
      if (filteredEcgSignal[i] < minEcg) minEcg = filteredEcgSignal[i];
    }

    // Estimate ST-segment
    float baseline = (maxEcg + minEcg) / 2.0;
    float stLevel = 0;
    int stCount = 0;
    for (int i = 20; i < 80; i++) { // Narrowed window for ST segment
      stLevel += (filteredEcgSignal[i] - baseline);
      stCount++;
    }
    stLevel /= stCount;

    // Compute oldpeak (ST depression)
    oldpeak = abs(stLevel * 10.0);
    if (oldpeak > 5.0) {
      Serial.println("Invalid ST depression!");
      return;
    }
    Serial.print("Oldpeak: ");
    Serial.print(oldpeak);
    Serial.println(" mm");

    // Compute ST slope
    float stStart = filteredEcgSignal[20] - baseline;
    float stEnd = filteredEcgSignal[70] - baseline;
    if (stEnd > stStart + 0.1) slope = 2;
    else if (stEnd < stStart - 0.1) slope = 0;
    else slope = 1;
    Serial.print("ST Slope: ");
    Serial.println(slope);

    // Determine resting ECG status
    restecg = (oldpeak > 1.0) ? 1 : 0;
    Serial.print("Resting ECG: ");
    Serial.println(restecg);

    // Publish data to AWS IoT Core
    StaticJsonDocument<256> doc;
    doc["heartRateBPM"] = heartRateBPM;
    doc["oldpeak"] = oldpeak;
    doc["slope"] = slope;
    doc["restecg"] = restecg;
    doc["age"] = user_age;
    doc["sex"] = user_sex;
    doc["cp"] = user_cp;

    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);

    if (mqttClient.publish(mqttTopic, jsonBuffer)) {
      Serial.println("Data published to AWS IoT Core");
    } else {
      Serial.println("Publish failed");
    }
  }

  delay(10); // 100 Hz sampling
}