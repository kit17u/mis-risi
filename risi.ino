/*
  This example reads audio data from the on-board PDM microphones, and prints
  out the samples to the Serial console. The Serial Plotter built into the
  Arduino IDE can be used to plot the audio data (Tools -> Serial Plotter)

  Please try with the Circuit Playground Bluefruit

  This example code is in the public domain.
*/

#include <PDM.h>

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];
// number of samples read
volatile int samplesRead;

// true if sound sample reaches THR
bool triggered = false;

// Constants:
const int THR = 1000; //32000; // threshold for gunfire detection(out of max 32767)
const int SAMPLE_RATE = 16000;
const int PRE_TRIGGER_MS = 50;
const int POST_TRIGGER_MS = 150;
const int TOTAL_MS = PRE_TRIGGER_MS + POST_TRIGGER_MS; // 200ms
const int PRE_SAMPLES = (SAMPLE_RATE * PRE_TRIGGER_MS) / 1000; // 800 samples
const int TOTAL_SAMPLES = (SAMPLE_RATE * TOTAL_MS) / 1000; // 3200 samples

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
  // TODO: Put classification function here
  Serial.println("Sample gathering finished, activated classification (unimplemented)");

  for (int i = 0; i < TOTAL_SAMPLES; i++) {
    Serial.print(captureBuffer[i]);
    Serial.print(" ");
  }
  Serial.println("");


}