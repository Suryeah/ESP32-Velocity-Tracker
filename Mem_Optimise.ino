#include "arduinoFFT.h"
#include "Protocentral_ADS1220.h"
#include <Integrator.h>
#include <TimeStep.h>
#include <SPI.h>
#include <WiFi.h>
#include <stdio.h>
#include <stdlib.h>

Protocentral_ADS1220 pc_ads1220;
arduinoFFT FFT = arduinoFFT();         /* Create FFT object */

#define ADS1220_CS_PIN    5
#define ADS1220_DRDY_PIN  4
#define ALERT_PIN         2

const double samplingFrequency = 1163; /* Hz, must be less than 10000 due to ADC */
uint32_t LTAW = 4096;                  /* MUST ALWAYS be a power of 2 */
double vReal[4096];
double vImag[4096];
volatile double max_v = 0.0;

int culp, peakcount, tc, LSC, STAW;
uint32_t sensorValue;
double adc, Acc, STA, LTA1, ST_LT, peak, thresh, x = 0;
bool SET, LED;
bool startup = 1;
long cur, lap;

const char* ssid = "Senpronics";
const char* password =  "Senpronics@2019";
WiFiServer wifiServer(80);

const IPAddress local_IP(192, 168, 1, 120);
const IPAddress gateway(192, 168, 1, 1);
const IPAddress subnet(255, 255, 255, 0);

void setup()
{
  pinMode(ALERT_PIN , OUTPUT);
  WiFi.config(local_IP, gateway, subnet);

  pc_ads1220.begin(ADS1220_CS_PIN, ADS1220_DRDY_PIN);
  pc_ads1220.writeRegister(CONFIG_REG0_ADDRESS, 0x00);
  pc_ads1220.writeRegister(CONFIG_REG1_ADDRESS, 0xB4);
  pc_ads1220.writeRegister(CONFIG_REG2_ADDRESS, 0x50);
  pc_ads1220.writeRegister(CONFIG_REG3_ADDRESS, 0x00);
  pc_ads1220.select_mux_channels(MUX_SE_CH3);
  Serial.begin(115200);
  delay(100);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
  wifiServer.begin();
}

void loop()
{
  String line;
  int temp;
  volatile uint32_t z = 0;
  int steps;

  WiFiClient client = wifiServer.available();
  if (client) {
    while (client.connected()) {
      if (startup)
      {
        Serial.println("Clinet Connected ");
        client.print("Set LTA (1/2/3) : ");
        while (!client.available()) {}
        line = client.readStringUntil('\n');
        Serial.println(line);
        LTAW = line.toInt();
        client.print("LTA counts is at: ");
        client.println(LTAW);

        client.print("Set STA : ");
        while (!client.available()) {}
        line = client.readStringUntil('\n');
        Serial.println(line);
        STAW = line.toInt();
        client.print("STA counts is at: ");
        client.println(STAW);

        LSC = LTAW / STAW;

        client.print("Set Trigger Count : ");
        while (!client.available()) {}
        line = client.readStringUntil('\n');
        Serial.println(line);
        tc = line.toInt();
        client.print("Trigger Count is: ");
        client.println(tc);

        client.print("Set Threshold : ");
        while (!client.available()) {}
        line = client.readStringUntil('\n');
        Serial.println(line);
        thresh = line.toFloat();
        client.print("Trigger Threshold Point is at: ");
        client.println(thresh, 4);

        startup = 0 ;
        client.println(" Setup Complete ");
        delay(1000);
      }

      if (startup == 0)
      {
        z = 0;
        LTA1 = 0;
        Serial.println("Sampling .. ");
        for (uint32_t i = 0; i < LTAW; )
        {
          sensorValue = pc_ads1220.Read_WaitForData();
          if (sensorValue)
          {
            adc = (sensorValue);
            Acc = (adc * 0.000000536);
            if (Acc)
            {
              Serial.println(Acc, 4);
              vReal[i] = Acc;
              LTA1 += Acc;
              i++;
            }
          }
        }
        Serial.println("End .. ");

        LTA1 = (LTA1 / LTAW);

        Serial.print("analyse...   LTA: " );
        Serial.println(LTA1, 4);

        for (int w = 1; w <= LSC; w++)
        {
          steps = STAW * w;
          STA = 0;
          Serial.println("w: " + String(w) + " steps: " + String(steps));

          for (z ; z < steps; z++)
          {
            STA += vReal[z];
          }
          STA = (STA / STAW);

          Serial.print("STA: ");
          Serial.println(STA, 4);

          ST_LT = (STA / LTA1);
          Serial.print("ST_LT: ");
          Serial.println(ST_LT, 4);

          if (ST_LT >= thresh)
          {
            peakcount++;
            Serial.println("Current Peak Count = " + String(peakcount));
          }

          if (peakcount >= tc)
          {
            digitalWrite(ALERT_PIN, HIGH);

            culp = 1;
            LED = 1;
            peakcount = 0;
            max_v = 0;

            Serial.println("Array DATA");
            for(uint32_t s = 0; s<LTAW; s++){
              Serial.println(vReal[s]);
            }
            
            FFT.Windowing(vReal, LTAW, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
            FFT.Compute(vReal, vImag, LTAW, FFT_FORWARD);                  /* Compute FFT */
            FFT.DCRemoval(vReal, LTAW);
            FFT.ComplexToMagnitude(vReal, vImag, LTAW);                    /* Compute magnitudes */
            x = FFT.MajorPeak(vReal, LTAW, samplingFrequency);
            Serial.print("Frequency: ");
            Serial.println(x, 4);                                          /* Print out what frequency is the most dominant.*/
            max_v = 0;
            for (int i = 0; i < LTAW; i++)
            {
              if (vReal[i] > max_v)
                max_v = vReal[i];
            }
            max_v = (max_v / samplingFrequency);
            Serial.print("Velocity: ");
            Serial.println(max_v, 4);
            client.println(String(x, 4) + '\n' + "@" + String(max_v, 4) + '\n' + "@" + "N/A" + '\n');
            SET = 1;
            LTA1, STA, peakcount, x, max_v = 0;
            digitalWrite(ALERT_PIN, LOW);
            break;
          }
        }
      }
    }
    delay(10);
  }
  client.stop();
}
