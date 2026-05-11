#include "BluetoothA2DPSource.h"
#include <math.h>

BluetoothA2DPSource a2dp_source;

#define SAMPLE_RATE 44100
#define TARGET_DEVICE "WH-1000XM6"

// 危險警報 (快速 beep)
int32_t get_data_frames(Frame *frame, int32_t frame_count) {

  static float t = 0.0;

  for (int i = 0; i < frame_count; i++) {

    float phase = fmod(t, 0.3);   // 0.3秒一個週期

    int16_t sample = 0;

    if (phase < 0.1) {
      sample = 14000 * sin(2 * PI * 1500 * t); // 高頻警報,一秒1500Hz
    }

    frame[i].channel1 = sample;
    frame[i].channel2 = sample;

    t += 1.0 / SAMPLE_RATE;
  }

  return frame_count;
}

void setup() {

  Serial.begin(115200);

  Serial.println("ESP32 Danger Alarm Test");
  Serial.println("Connecting to WH-1000XM6...");

  a2dp_source.set_data_callback_in_frames(get_data_frames);

  a2dp_source.set_volume(70); //設定音量 音量範圍：0 ~ 100

  a2dp_source.start(TARGET_DEVICE);
}

void loop() {
  delay(10);
}