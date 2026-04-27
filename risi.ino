/*
  This example reads audio data from the on-board PDM microphones, and prints
  out the samples to the Serial console. The Serial Plotter built into the
  Arduino IDE can be used to plot the audio data (Tools -> Serial Plotter)

  Please try with the Circuit Playground Bluefruit

  This example code is in the public domain.
*/

#include <PDM.h>
#include <RISI_inferencing.h>

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];
// number of samples read
volatile int samplesRead;

// true if sound sample reaches THR
bool triggered = false;

// Constants:
/**
 * THR = 1000 <- This is just for testing. 
 * For Actual gunfire detection, use 32000
 * (out of max 32767)
 **/
const int THR = 1000;
const int SAMPLE_RATE = 16000;
const int PRE_TRIGGER_MS = 100;
const int POST_TRIGGER_MS = 900;
const int TOTAL_MS = PRE_TRIGGER_MS + POST_TRIGGER_MS; // 1000ms
const int PRE_SAMPLES = (SAMPLE_RATE * PRE_TRIGGER_MS) / 1000; // 800 samples
const int TOTAL_SAMPLES = (SAMPLE_RATE * TOTAL_MS) / 1000; // 3200 samples
const float CLASSIFICATION_THR = 0.8; // Threshold for classification accuracy
float inputBuffer[TOTAL_SAMPLES]; 

int ringHead = 0; 
int postTriggerCount = 0;

// Samples buffers:
short ringBuffer[PRE_SAMPLES]; //
short captureBuffer[TOTAL_SAMPLES]; // samples after THR reached

void setup() {
  Serial.begin(9600);
  while (!Serial) yield();

  // configure the data receive callback
  PDM.onReceive(onPDMdata);

  // optionally set the gain, defaults to 20
  // PDM.setGain(30);

  // initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1) yield();
  }
}

void loop() {
  // wait for samples to be read
  if (samplesRead == 0) return;

  for (int i = 0; i < samplesRead; i++) {
    const int sample = sampleBuffer[i];
    //Serial.println(sample);
    
    if(!triggered){
      // Check if read sample reached the threshold
      if (sample >= THR) {
        Serial.println(sample);
        Serial.println("Sound level reached threshold");
        triggered = true;

        ringBuffer[ringHead] = sample;
        ringHead = (ringHead + 1) % PRE_SAMPLES;

        for (int j = 0; j < PRE_SAMPLES; j++) {
          int idx = (ringHead - PRE_SAMPLES + j + PRE_SAMPLES) % PRE_SAMPLES;
          captureBuffer[j] = ringBuffer[idx];
        }
        captureBuffer[PRE_SAMPLES] = sample; // the trigger sample itself
        postTriggerCount = 1;
      }
    }else{
      // If sound THR reached:
      // Keeps recording some more samples for classificaition

      int pos = PRE_SAMPLES + postTriggerCount;
      if (pos < TOTAL_SAMPLES) {
        captureBuffer[pos] = sample;
        postTriggerCount++;
      }

      // buffer full — you have your 200ms clip
      if (postTriggerCount >= TOTAL_SAMPLES - PRE_SAMPLES) {
        triggered = false;
        classifySound();
      }

    }

    // clear the read count
    samplesRead = 0;
  }
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

void classifySound() {
  Serial.println("Sample gathering finished:");
  // Prints gathered samples:
  for (int i = 0; i < TOTAL_SAMPLES; i++) {
    Serial.print(captureBuffer[i]);
    Serial.print(" ");
  }
  Serial.println("");

  // 0 = nothing, 1 = gunshot, 2 = other
  uint8_t classResult = runClassifier(); // your ML result
  
  if (classResult == 1) {
    Serial.println("Sample classified as ");
    sendLoRaPayload(classResult);
  }
}

// Logic for sample classification with <RISI_inferencing.h>
uint8_t runClassifier() {
  Serial.println("Activating classification...");

  // Convert short buffer to float, normalized to [-1.0, 1.0]
  for (int i = 0; i < TOTAL_SAMPLES; i++) {
    inputBuffer[i] = (float)captureBuffer[i] / 32768.0f;
  }

  signal_t signal;
  numpy::signal_from_buffer(
    inputBuffer,
    TOTAL_SAMPLES,
    &signal
  );

  ei_impulse_result_t result = {0};
  
  // Run classifier
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  
  if (err != EI_IMPULSE_OK) {
    Serial.printf("Classifier error: %d\n", err);
    return 0;
  }

  // Print results
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.print(result.classification[i].label);
    Serial.print(": ");
    Serial.println(result.classification[i].value);

  }

  // Trigger LoRa alert if gunshot confidence high enough
  return (result.classification[0].value > CLASSIFICATION_THR);
}

void sendLoRaPayload(uint8_t result) {
  // Single byte payload — extremely LoRaWAN friendly
  // e5.print() syntax depends on your LoRa module library
  Serial1.print("AT+SEND=1,1,0");  // port 1, confirmed, 1 byte
  Serial1.write(result);
  Serial1.println();
}