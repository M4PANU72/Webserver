#include <Arduino.h>

/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-web-bluetooth/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <DHT.h>

//Constants
#define DHTPIN D9   // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

#define RED A0
#define GREEN A2
#define BLUE A1

unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
const long interval = 10000;
const long interval2 = 3000;

const int buttonPin = D3;  // the number of the pushbutton pin


float temp; //Stores temperature value
int buttonState;       
int buttoncounter = 0;
     // the current reading from the input pin


BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

const int ledPin = D2; // Use the appropriate GPIO pin for your setup

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pLedCharacteristic) {
    std::string ledvalue  = pLedCharacteristic->getValue(); 
    String value = String(ledvalue.c_str());
    if (value.length() >=0) {
      Serial.print("Characteristic event, written: ");
      Serial.println(static_cast<int>(value[0])); // Print the integer value

      int receivedValue = static_cast<int>(value[0]);
      if (receivedValue == 1) {
        digitalWrite(ledPin, HIGH);
      } 
      else {
        digitalWrite(ledPin, LOW);
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  dht.begin();// LED connection pins to be set as an output
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(buttonPin, INPUT);

  // Create the BLE Device
  BLEDevice::init("ESP32Manu");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
                      SENSOR_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create the ON button Characteristic
  pLedCharacteristic = pService->createCharacteristic(
                      LED_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  // Register the callback for the ON button characteristic
  pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pLedCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}


void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(RED, 255 - redValue);
  analogWrite(GREEN, 255 - greenValue);
  analogWrite(BLUE, 255 - blueValue);
}


double getTemp(){
  temp= dht.readTemperature();
   //Print temp and humidity values to serial monitor
    Serial.print("Temp: ");
    Serial.print(temp);
    Serial.println(" Celsius"); 
    if (temp>= 15.00 && temp<30.00 )
  {
    setColor(0,255,0);
  }
  else if (temp>= 30.00 )
  {
    setColor(255,0,0);
  }
  else if (temp< 15.00)
  {
    setColor(0,0,255);
  }
    return temp;
}



void loop() {
  // notify changed value
  
  unsigned long currentMillis1 = millis();
  unsigned long currentMillis2 = millis();

  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH && buttoncounter == 0) { 
    buttoncounter = 1;
    Serial.println("button pressed");
    getTemp();
    Serial.print("New value notified: ");
    Serial.println(temp);
    }
  if (buttonState == LOW){
    buttoncounter = 0;
    }

  if (currentMillis1 - previousMillis1 >= interval) {
    previousMillis1 = currentMillis1;
    getTemp();
  }
  if (deviceConnected) {
    if (currentMillis2 - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis2;
    pSensorCharacteristic->setValue(String(temp).c_str());
    pSensorCharacteristic->notify();
    
    Serial.print("New value notified: ");
    Serial.println(temp);
    } 
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
  }
}