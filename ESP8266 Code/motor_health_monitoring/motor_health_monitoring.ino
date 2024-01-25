#include <Wire.h>
#include <Adafruit_Sensor.h>   // Adafruit unified sensor  // ver- 1.1.9   https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_ADXL345_U.h> // Adafruit ADXL345  // ver-1.3.2   https://github.com/adafruit/Adafruit_ADXL345
#include "arduinoFFT.h"  //arduinoFFT  //ver-1.6.0  https://github.com/kosme/arduinoFFT
#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h> //Firebase Arduino client library for esp8266 and esp32   // ver-4.3.9  https://github.com/mobizt/Firebase-ESP-Client
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define API_KEY "AIzaSyByTWjYsnZQ4zSUH7ZEYJabD5XQxSppE3w" // Firebase API Key
#define DATABSE_URL "https://motor-health-analysis-default-rtdb.firebaseio.com/" // Firebase Database URL

//Details of wifi network you want esp to connect to 
char ssid[] = "Dinesh";
char password[] = "18032001";

Adafruit_ADXL345_Unified accel;  //creating the object of accelerometer
arduinoFFT FFT = arduinoFFT();   // fft object to calculate to fft

//firebase objects for its configuration
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

//firebase onjects to convert array to json
FirebaseJson json;
FirebaseJsonArray arr;


//used in printvector()
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

unsigned long sendDataPrevMillis = 0;  
bool signupOK = false;  //sets true when firebase is succesfully connected

const int samples = 1024;  //number of samples we are taking to for one observation
int samplingFrequency = 493;  //this is the rate at which accelerometer samples in one second
unsigned long StartTime;  //used for calculating dynamic sampling rate

double xReal[samples];  //for storing the acceleration values in x direction
double xImag[samples];  //for calculation of fft we need imaginary values also
//double yReal[samples];
//double yImag[samples];
//double zReal[samples];
//double zImag[samples];



void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(19200);
  //////////////////////////////////////////////////
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");

    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_8_G);
  

   
  /////////////////////////////////////////////////////

  // Connect to Wi-Fi////////////////////////////////
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  
  /////////////////////////////////////////////////////  
  //Confirgureing and connecting to firebase
  
  config.api_key = API_KEY;
  config.database_url = DATABSE_URL;
  if(Firebase.signUp(&config, &auth, "", "")){
    Serial.println("signUp OK");
    signupOK = true;
  }else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  
}

void loop() {
  // put your main code here, to run repeatedly:

  //taking data from the adxl///////////////////////////////////
  int i =0;
  
  digitalWrite(LED_BUILTIN, LOW); // turn on the led when we start taking samples
  StartTime=millis();  
  while(i<samples){
    sensors_event_t event; 
    accel.getEvent(&event);

      xReal[i]=event.acceleration.x;
      
      xImag[i] = 0.0; // we set all imaginary values to zero as we dont need it
      
      i=i+1;
      //here we are dynamically calculating the sampling frequency
      if(millis()-StartTime<=1000){
        samplingFrequency=i;    
        
      }
    
  }
  digitalWrite(LED_BUILTIN, HIGH); //turn the led off after taking the samples
  
  Serial.print("Sampling Frequency :-> ");Serial.print(",");Serial.println(samplingFrequency);
  
 
  //PrintVector(xReal, samples, SCL_TIME);  //this function print timestamp and values in xReal
  FFT.Windowing(xReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(xReal, xImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(xReal, xImag, samples); /* Compute magnitudes */

//  FFT.Windowing(yReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
//  FFT.Compute(yReal, xImag, samples, FFT_FORWARD); /* Compute FFT */
//  FFT.ComplexToMagnitude(yReal, xImag, samples); /* Compute magnitudes */
//
//  FFT.Windowing(zReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
//  FFT.Compute(zReal, xImag, samples, FFT_FORWARD); /* Compute FFT */
//  FFT.ComplexToMagnitude(zReal, xImag, samples); /* Compute magnitudes */
  Serial.println("/////////// ");
  
  //PrintVector(xReal, (samples >> 1), SCL_FREQUENCY);
    double fundamentalfreq = FFT.MajorPeak(xReal, samples, samplingFrequency);  // this function is used to calculate the major peak in the fft graph
  Serial.println(fundamentalfreq, 6);

////////////////////////////////////////////////////////////////////////////////////////////////////////

//Sending Data to Firebase (if call it after every 5 sec delay or if its running for the first time
  if(Firebase.ready() && signupOK && (millis()-sendDataPrevMillis>5000|| sendDataPrevMillis ==0)){
    sendDataPrevMillis = millis();

    
    //fragmenting data into array of length 100 to send to firebase
    //As it has limitation on the size of json obj we can send at a time
    
      for(i=0;i<100;i++){ // we read the first 100 values and store it into the arr obj
        arr.add(xReal[i]);
      }
      Serial.printf("Set array... %s\n", Firebase.RTDB.setArray(&fbdo, "fft/fft_arr/f1", &arr) ? "ok" : fbdo.errorReason().c_str()); //what path you write will make the structure of you firebase database
      arr.clear();// we clear the array for next iteration

      for(i=100;i<200;i++){
        arr.add(xReal[i]);
      }
      Serial.printf("Set array... %s\n", Firebase.RTDB.setArray(&fbdo, "fft/fft_arr/f2", &arr) ? "ok" : fbdo.errorReason().c_str());
      arr.clear();

      for(i=200;i<300;i++){
        arr.add(xReal[i]);
      }
      Serial.printf("Set array... %s\n", Firebase.RTDB.setArray(&fbdo, "fft/fft_arr/f3", &arr) ? "ok" : fbdo.errorReason().c_str());
      arr.clear();

      for(i=300;i<400;i++){
        arr.add(xReal[i]);
      }
      Serial.printf("Set array... %s\n", Firebase.RTDB.setArray(&fbdo, "fft/fft_arr/f4", &arr) ? "ok" : fbdo.errorReason().c_str());
       arr.clear();
       
       
      for(i=400;i<500;i++){
        arr.add(xReal[i]);
      }
      Serial.printf("Set array... %s\n", Firebase.RTDB.setArray(&fbdo, "fft/fft_arr/f5", &arr) ? "ok" : fbdo.errorReason().c_str());
      arr.clear();
      
      for(i=500;i<512;i++){
        arr.add(xReal[i]);
      }
      
    Serial.printf("Set array... %s\n", Firebase.RTDB.setArray(&fbdo, "fft/fft_arr/f6", &arr) ? "ok" : fbdo.errorReason().c_str());
        arr.clear();


  //Sending the major peak and the sampling rate
      json.add("fundamental frequency", fundamentalfreq);
      json.add("Sampling Rate", samplingFrequency);
      Serial.printf("Sending Meta %s\n", Firebase.RTDB.set(&fbdo, "fft/fft_meta", &json) ? "ok" : fbdo.errorReason().c_str());
      json.clear();
     
  }
  
Serial.println("////////////");

}




//this function is from the arduino fft library it prints according to the three arguments we pass on
void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print(" Hz");
    Serial.print(",");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
