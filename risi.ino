#include <PDM.h>
#include <RISI_inferencing.h>

// Constants:
const int THR = 1000;
const float CLASSIFICATION_THR = 0.8;

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
}

void loop() {
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
      strcmp(result.classification[i].label, "gunshot") == 0 &&
      result.classification[i].value > CLASSIFICATION_THR
    ) {
      Serial.println("GUNSHOT DETECTED");
      sendLoRaPayload(1);
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
void sendLoRaPayload(uint8_t result) {
  Serial1.print("AT+SEND=1,1,0");
  Serial1.write(result);
  Serial1.println();
}