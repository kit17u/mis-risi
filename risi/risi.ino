#include <PDM.h>
#include <RISI_inferencing.h>
#include "LSM6DS3.h"
#include "Wire.h"

LSM6DS3 myIMU(I2C_MODE, 0x6A);

// Constants:
const int THR = 1000;
const float CLASSIFICATION_THR = 0.5;

bool trackingMovement = false;
unsigned long trackingStartTime = 0;
unsigned long lastPrintTime = 0;
const unsigned long TRACKING_DURATION = 5 * 60 * 1000; // 5 min
const unsigned long PRINT_INTERVAL = 30 * 1000; // 30 sec
bool movementThisMinute = false;
static float lastAccel = 1.0;

// Edge Impulse inference struct
typedef struct {
  int16_t *buffers[2];
  uint8_t buf_select;
  uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;

static inference_t inference;
static volatile bool record_ready = false;
static signed short sampleBuffer[2048];
static bool debug_nn = false;

void setup() {
  Serial.begin(9600);
  while (!Serial) yield();

  delay(2000);

  //LoRaWan modem setup
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  Serial1.begin(9600);
  while (!Serial1) {}

  delay(2000);

  Serial1.println("AT+ID=AppEui,\"0000000000000001\"");
  delay(1000);
  Serial1.println("AT+ID=DevEui,\"70B3D57ED0076B25\"");
  delay(1000);
  Serial1.println("AT+KEY=AppKey,\"15AC9D90B16A2E62E3F7057BF0ED81F6\"");
  delay(1000);
  Serial1.println("AT+MODE=LWOTAA");
  delay(1000);

  Serial1.write("AT+ADR=OFF");
  delay(1000);
  Serial1.write("AT+DR=DR0");
  delay(1000);
  Serial1.println("AT+JOIN");
  delay(5000);
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  Serial.println("LoRaWAN ready.");

  Serial.println("Gunshot detector starting...");
  // summary of inferencing settings (from model_metadata.h)
  ei_printf("Inferencing settings:\n");
  ei_printf("\tInterval: ");
  ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
  ei_printf(" ms.\n");
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
  ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                          sizeof(ei_classifier_inferencing_categories[0]));

  run_classifier_init();
  if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) {
      ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
      return;
  }
  // Initialize continuous classifier
  run_classifier_init();
  Serial.println("Listening...");

  if (myIMU.begin() != 0) {
    Serial.println("IMU error");
  } else {
    Serial.println("IMU OK!");
  }
}

void loop() {

  // MOVEMENT TRACKING
  if (trackingMovement) {

    float ax = myIMU.readFloatAccelX();
    float ay = myIMU.readFloatAccelY();
    float az = myIMU.readFloatAccelZ();

    float accelMagnitude = sqrt(ax*ax + ay*ay + az*az);

    float diff = abs(accelMagnitude - lastAccel);
    lastAccel = accelMagnitude;

    if (diff > 0.15) { //tweak this based on lynx movements!
      movementThisMinute = true;
    }

    // every 30 seconds print result
    if (millis() - lastPrintTime > PRINT_INTERVAL) {

      if (movementThisMinute) {
        Serial.println("30s RESULT: MOVING");
      } else {
        Serial.println("30s RESULT: STILL");
      }

      movementThisMinute = false;
      lastPrintTime = millis();
    }

    // stop after 5 minutes
    if (millis() - trackingStartTime > TRACKING_DURATION) {
      Serial.println("Tracking finished.");
      trackingMovement = false;
    }
  }

  // Wait for a slice to be ready
  if (inference.buf_ready == 0) return;

  inference.buf_ready = 0;

  // Check if any sample in this slice exceeds threshold
  bool sliceTriggered = false;
  for (int i = 0; i < EI_CLASSIFIER_SLICE_SIZE; i++) {
    if (abs(inference.buffers[inference.buf_select ^ 1][i]) >= THR) {
      Serial.println(abs(inference.buffers[inference.buf_select ^ 1][i]));
      sliceTriggered = true;
      break;
    }
  }

  if (!sliceTriggered) return; // skip classification if quiet

  // Build signal from the ready buffer
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
  signal.get_data = &microphone_audio_signal_get_data;

  // Run continuous classifier
  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR err = run_classifier_continuous(&signal, &result, debug_nn);

  if (err != EI_IMPULSE_OK) {
    Serial.print("Classifier error: ");
    Serial.println(err);
    return;
  }

  // Only print results when classifier has enough context

  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.print(result.classification[i].label);
    Serial.print(": ");
    Serial.println(result.classification[i].value);
  }

  // Check for gunshot — adjust index to match your label order
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (
      strcmp(result.classification[i].label, "Strel") == 0 &&
      result.classification[i].value > CLASSIFICATION_THR
    ) {
      Serial.println("GUNSHOT DETECTED");
      sendLoRaPayload(1);
      
      trackingMovement = true;
      trackingStartTime = millis();
      lastPrintTime = millis();
      movementThisMinute = false;

      Serial.println("Started 5-minute movement tracking...");
      break;
    }
  }
}

// Called by Edge Impulse to get audio data from the ready buffer
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
  return 0;
}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);
  int samplesRead = bytesRead >> 1; // divide by 2, 16-bit samples

  for (int i = 0; i < samplesRead; i++) {
    inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

    if (inference.buf_count >= inference.n_samples) {
      inference.buf_select ^= 1; // swap buffers
      inference.buf_count = 0;
      inference.buf_ready = 1;
    }
  }
}

static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);

    PDM.setBufferSize(2048);
    delay(250);

    // initialize PDM with:
    // - one channel (mono mode)
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("ERR: Failed to start PDM!");
        return false;
    }

    // optionally set the gain, defaults to 24
    // Note: values >=52 not supported
    //PDM.setGain(40);

    record_ready = true;

    return true;
}

/**
 * @brief      PDM buffer full callback
 *             Copy audio data to app buffers
 */
static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if ((inference.buf_ready == 0) && (record_ready == true)) {
        for(int i = 0; i < bytesRead>>1; i++) {
            inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

            if (inference.buf_count >= inference.n_samples) {
                inference.buf_select ^= 1;
                inference.buf_count = 0;
                inference.buf_ready = 1;
                break;
            }
        }
    }
}
String buildPayload(ei_impulse_result_t result) {
    String json = "{";

    for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {

        // skrajšani ključi
        char key = result.classification[i].label[0];

        json += "\"";
        json += key;
        json += "\":";
        json += String(result.classification[i].value, 2);

        if (i < EI_CLASSIFIER_LABEL_COUNT - 1) json += ",";
    }

    json += "}";
    return json;
}

void sendLoRaPayload(uint8_t result) {
    Serial1.print("AT+MSG=\"");
    Serial1.print(result);
    Serial1.println("\"");

    delay(5000);

    while (Serial1.available()) {
        Serial.write(Serial1.read());
    }

    Serial.print("Sent to TTN: ");
    Serial.println(result);
}
