#include <PulseSensorPlayground.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include "BluetoothSerial.h"
#include "esp_bt.h" 

BluetoothSerial ESP_BT;
LiquidCrystal_I2C lcd(0x27, 16, 2);
PulseSensorPlayground pulseSensor;
const int OUTPUT_TYPE = SERIAL_PLOTTER;

const int PULSE_INPUT = 15; // Use any available GPIO
const int PULSE_BLINK = 2; // Use an available GPIO for LED
const int THRESHOLD = 535; // Initial value to avoid noise
const int numValues = 20;   // Number of values for sampling
bool ligaMotor = false;
const int motor = 4; // Use an available GPIO for the motor
const int tempoLimite = 5000; // 5s in ms
const int forcaMotor = 130; // Motor intensity
unsigned long ligaTempo = 0; // Time when motor was turned on
bool motorLigado = false; // Motor state
int acionaAmostra = 0;
int Bot1 = 25;

int bpmValues[numValues];  // Array to store BPM values
int bpmIndex = 0;          // Current index in the array
float totalbpm = 0;
int values[numValues];  // Array to store analog values
int i = 0; // Index
float total = 0; // Sum of values
unsigned long previousMillis = 0;
int bpm = 0;
int amostra = 80;
int bpmElevado = round(amostra * 1.3);
int acionaRemoto = 0;

String receive;
String modoMotor = "FREQ_FIXA";

unsigned long lastUpdate = 0; // For timing control
unsigned long enviaDados = 0;
int updateCount = 0; // Update counter
float threshold = THRESHOLD; // Dynamic threshold
float signalMediaMovel = 0;

bool messageProcessed = true;

void setup() {
    Serial.begin(115200);
    ESP_BT.begin("Esp32");

    pulseSensor.analogInput(PULSE_INPUT);
    pulseSensor.blinkOnPulse(PULSE_BLINK);
    pulseSensor.setSerial(Serial);
    pulseSensor.setOutputType(OUTPUT_TYPE);
    pulseSensor.setThreshold(threshold);

    lcd.init();
    lcd.clear();
    lcd.backlight();
    lcd.setCursor(2, 0);
    lcd.print("Bem vindo!");
    delay(500);
    lcd.clear();

    lcd.setCursor(0, 1);
    lcd.print("rlx:");
    lcd.print(amostra);

    lcd.setCursor(9, 0);
    lcd.print("lim:");
    lcd.print(bpmElevado);

    lcd.setCursor(13, 1);
    lcd.print("DES");

    pinMode(motor, OUTPUT);
    pinMode(Bot1, INPUT_PULLUP);

    if (!pulseSensor.begin()) {
        for (;;) {
            digitalWrite(PULSE_BLINK, LOW);
            delay(50);
            Serial.println('!');
            digitalWrite(PULSE_BLINK, HIGH);
            delay(50);
        }
    }
}

void loop() {
    unsigned long currentMillis = millis();

  // Verifies if there's available data for readings to avoid errors
  if (ESP_BT.available()) {
    receive = ESP_BT.readString();
    Serial.println(receive);
    messageProcessed = false;
  }

    // SAMPLES USER AVERAGE BPM
   if (receive == "AMOSTRA" && !messageProcessed) {
    acionaAmostra = 1;
    messageProcessed = true;
   }
    // TURNS THE DETECTION ON AND OFF
   if (receive == "LIGA" && !messageProcessed) {
    acionaRemoto = 1;
    messageProcessed = true;
}
   if (receive == "DESLIGA" && !messageProcessed) {
    acionaRemoto = 0;
    messageProcessed = true;
    }

    // MANUALLY ACTIVATE AND DEACTIVATE VIBRATION
    if (receive == "ATIVA" && !messageProcessed) {
    ligaMotor = 1;
    messageProcessed = true;
}
   if (receive == "DESATIVA" && !messageProcessed) {
    ligaMotor = 0;
    messageProcessed = true;
    }

if ((currentMillis - enviaDados >=500) && messageProcessed){
    enviaDados = currentMillis;
    ESP_BT.print(String(bpm) + "|" + String (amostra) + "|" + String (ligaMotor));
    receive = "";
  }

    // PulseSensor logic
    if (pulseSensor.UsingHardwareTimer) {
        delay(20);
        pulseSensor.outputSample();
    } else {
        if (pulseSensor.sawNewSample()) {
            if (--pulseSensor.samplesUntilReport == (byte)0) {
                pulseSensor.samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;
                pulseSensor.outputSample();
            }
        }
    }

    if (pulseSensor.sawStartOfBeat()) {
        pulseSensor.outputBeat();
    }

    bpm = pulseSensor.getBeatsPerMinute();

    Serial.print("BPM: ");
    Serial.println(bpm);
    Serial.print("Amostra: ");
    Serial.println(amostra);
    Serial.print("Receive: ");
    Serial.println(receive);
    lcd.setCursor(0, 0);
    lcd.print("bpm:");
    lcd.print(bpm);
    lcd.print("  ");

    // Store analog value for moving average
    int analogValue = analogRead(PULSE_INPUT);
    total = total - values[i];
    values[i] = analogValue;
    total = total + values[i];
    i = (i + 1) % numValues;

    // Update threshold each 1.2s
    if (currentMillis - lastUpdate >= 2000) { 
        lastUpdate = currentMillis;
        updateCount++;
        
        // Gets the Simple Moving Average
        signalMediaMovel = total / numValues;

        // Update threshold
        if (updateCount >= 1) {
            threshold = signalMediaMovel * 1.03; // STILL NEEDS TO BE FINE-TUNED
            pulseSensor.setThreshold(threshold);
            Serial.print("Threshold atualizada! ");
            updateCount = 0; // Reset counter
        }
    }

    int aux = digitalRead(Bot1);
    if (aux == LOW) { // Button pressed (active low)
        Serial.print("MODO AMOSTRA ACIONADO");
        acionaAmostra = 1;
    }

    if (acionaAmostra == 1) { // Condition for getting the sample value
        previousMillis = currentMillis;

        totalbpm = totalbpm - bpmValues[bpmIndex]; // Remove oldest value
        bpmValues[bpmIndex] = bpm;                  // Add new value
        totalbpm = totalbpm + bpmValues[bpmIndex];  // Update sum
        bpmIndex = (bpmIndex + 1) % numValues;

        if (bpmIndex == 0) {
            acionaAmostra = 0; // Reset
            amostra = round(totalbpm / numValues); // Average BPM
            bpmElevado = round(amostra * 1.3);
            lcd.clear();
            lcd.setCursor(0, 1);
            lcd.print("rlx:");
            lcd.print(amostra);
            lcd.setCursor(9, 0);
            lcd.print("lim:");
            lcd.print(bpmElevado);
        }
    }

    if (acionaRemoto == 1 ) deteccaoBPM(bpm, bpmElevado, tempoLimite);
    acionaMotor(bpm, motor, ligaMotor, forcaMotor, amostra);
}


// FUNCTION TO DETECT IF A PERSON'S BPM IS TOO HIGH
void deteccaoBPM(int bpm, int bpmElevado, unsigned long tempo) {
    static unsigned long tempoInicialLigado = 0;
    static unsigned long tempoInicialDesligado = 0;

    if (bpm >= bpmElevado) {
        if (tempoInicialLigado == 0) {
            tempoInicialLigado = millis();
        }
        unsigned long tempoLigado = millis() - tempoInicialLigado;
        if (tempoLigado >= tempo) {
            lcd.setCursor(12, 1);
            lcd.print("LIGA");
            ligaMotor = true;
            tempoInicialLigado = 0;
        }
        tempoInicialDesligado = 0;
    } else {
        if (tempoInicialDesligado == 0) {
            tempoInicialDesligado = millis();
        }
        unsigned long tempoDesligado = millis() - tempoInicialDesligado;
        if (tempoDesligado >= tempo) {
            lcd.setCursor(12, 1);
            lcd.print("DESL");
            ligaMotor = false;
            tempoInicialDesligado = 0;
        }
        tempoInicialLigado = 0;
    }
}

// FUNCTION TO SIMULATE A HEARTBEAT-LIKE VIBRATION
void acionaMotor(int bpm, int motor, int ligaMotor, int tempoLigado, float amostra) {
    float freqMotor, seg;
    int liga = ligaMotor;

    // DIFFERENT VIBRATION MODES
    if (modoMotor == "FREQ_FIXA") freqMotor = 60; // Vibrates at fixed 60 BPM
    if (modoMotor == "FREQ_VARIA") freqMotor = bpm * 0.8; // Vibrates -20% to the actual heartbeat captured
    if (modoMotor == "FREQ_AMOSTRA") freqMotor = amostra; // Vibrates at the user's fixed average heartbeat

    seg = 1000; // Fixed for testing
    unsigned long currentMillis = millis();

    if (liga == 0) {
        digitalWrite(motor, LOW); // Turn motor off
    }

    // Condition to keep the motor on only for the time set on tempoLigado
    if (currentMillis - ligaTempo >= tempoLigado && liga) {
        digitalWrite(motor, LOW); // Turn motor off
        liga = false;
    }

    // System to turn on motor according to frequency
    if (currentMillis - previousMillis >= seg && !liga) {
        previousMillis = currentMillis;
        digitalWrite(motor, HIGH); // Turn motor on
        ligaTempo = millis();
    }
}
