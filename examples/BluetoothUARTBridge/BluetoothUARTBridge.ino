/**************************************************************************//**
 * @file     BluetoothUARTBridge.ino
 * @brief    UART to Bluetooth A2DP Warning Bridge for Visually Impaired System
 *
 * This sketch receives warning signals from M55M1 via UART and converts them
 * into A2DP audio alerts sent to Bluetooth headphones (e.g., WH-1000XM6).
 *
 * UART Protocol:
 *   START_BYTE | DIRECTION | CLASS_ID | SEVERITY | CHECKSUM | END_BYTE
 *      0xAA   |  1 byte   | 1 byte   | 1 byte   | 1 byte   |  0x55
 *
 * Connection:
 *   M55M1 PB3/D1 (TX) -> ESP32 GPIO26 (RX)
 *   M55M1 PB2/D0 (RX) <- ESP32 GPIO25 (TX)
 *   M55M1 GND       -> ESP32 GND
 *
 * Audio Output:
 *   - SAFE:     No sound
 *   - CAUTION:  1000 Hz beep @ 0.5 sec cycle
 *   - DANGER:   1500 Hz beep @ 0.3 sec cycle (fast)
 *   - UNKNOWN:  2000 Hz beep @ 0.4 sec cycle
 *
 * @author  ObjectTracker Vision Team
 * @date    2024-2025
 ******************************************************************************/

#include "BluetoothA2DPSource.h"
#include <HardwareSerial.h>
#include "SPIFFS.h"
#include "FS.h"

/* Set to 1 while testing M55M1 -> ESP32 UART. This keeps Bluetooth off. */
#define UART_ONLY_TEST 0

/* Set to 1 to print every received byte during UART bring-up. */
#define UART_RAW_BYTE_LOG 0

/* Printed at boot so Serial Monitor proves which firmware is actually running. */
#define FW_VERSION "voice-16k-resample-v4"

// ===== UART Configuration =====
#define TX_PIN 25                        /* ESP32 TX -> M55M1 PB2/D0 RX */
#define RX_PIN 26                        /* ESP32 RX <- M55M1 PB3/D1 TX */
#define BAUD_RATE 115200

// ===== UART Protocol =====
#define UART_START_BYTE 0xAA
#define UART_END_BYTE 0x55
#define UART_PACKET_SIZE 6
#define UART_TIMEOUT_MS 5000

// ===== Bluetooth Configuration =====
#define TARGET_DEVICE "WH-1000XM6"      /* Sony WH-1000XM6 headphones */
#define SAMPLE_RATE 44100.0f
#define AUDIO_AMPLITUDE 12000
#define VOICE_QUEUE_SIZE 10
#define VOICE_GAIN 3
#define VOICE_SELF_TEST_ON_CONNECT 1

// ===== Alarm Types =====
enum AlarmType {
    ALARM_SAFE = 0,
    ALARM_CAUTION = 1,
    ALARM_DANGER = 2,
    ALARM_UNKNOWN = 3
};

// ===== Warning Data Structure =====
struct Warning {
    uint8_t direction;  /* 0=LEFT, 1=CENTER, 2=RIGHT */
    uint8_t class_id;   /* 0-79 or 0xFF for unknown */
    uint8_t severity;   /* 0=SAFE, 1=CAUTION, 2=DANGER */
};

// ===== Global Variables =====
BluetoothA2DPSource a2dp_source;
HardwareSerial uart_serial(1);
static Warning current_warning = {0, 0, 0};
static AlarmType current_alarm = ALARM_SAFE;
static unsigned long last_warning_time = 0;
static bool spiffs_ready = false;

// ===== Voice WAV Segment Playback =====
struct VoiceSegment {
    const char *path;
};

static VoiceSegment voice_queue[VOICE_QUEUE_SIZE];
static uint8_t voice_q_head = 0;
static uint8_t voice_q_tail = 0;
static File voice_file;
static bool voice_playing = false;
static uint16_t voice_channels = 0;
static uint32_t voice_sample_rate = 0;
static uint32_t voice_data_remaining = 0;
static float voice_resample_phase = 1.0f;
static int16_t voice_last_sample = 0;
static bool voice_self_test_done = false;

uint16_t read_le16(File &f) {
    uint8_t b[2];
    if (f.read(b, 2) != 2) return 0;
    return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

uint32_t read_le32(File &f) {
    uint8_t b[4];
    if (f.read(b, 4) != 4) return 0;
    return (uint32_t)b[0] |
           ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) |
           ((uint32_t)b[3] << 24);
}

bool voice_file_exists(const char *path) {
    return spiffs_ready && SPIFFS.exists(path);
}

bool voice_queue_is_empty() {
    return voice_q_head == voice_q_tail;
}

bool voice_queue_push(const char *path) {
    if (!voice_file_exists(path)) {
        Serial.printf("[VOICE] missing %s\n", path);
        return false;
    }

    uint8_t next_tail = (voice_q_tail + 1) % VOICE_QUEUE_SIZE;
    if (next_tail == voice_q_head) {
        Serial.println("[VOICE] queue full");
        return false;
    }

    voice_queue[voice_q_tail].path = path;
    voice_q_tail = next_tail;
    Serial.printf("[VOICE] queued %s\n", path);
    return true;
}

void voice_queue_clear(bool stop_current) {
    voice_q_head = 0;
    voice_q_tail = 0;

    if (stop_current) {
        if (voice_file) {
            voice_file.close();
        }
        voice_playing = false;
        voice_data_remaining = 0;
        voice_resample_phase = 1.0f;
        voice_last_sample = 0;
    }
}

bool wav_open_segment(const char *path) {
    if (voice_file) {
        voice_file.close();
    }

    voice_file = SPIFFS.open(path, "r");
    if (!voice_file) {
        Serial.printf("[VOICE] open failed: %s\n", path);
        return false;
    }

    char id[5] = {0};
    if (voice_file.read((uint8_t *)id, 4) != 4 || strcmp(id, "RIFF") != 0) {
        Serial.printf("[VOICE] bad RIFF: %s\n", path);
        voice_file.close();
        return false;
    }

    read_le32(voice_file);
    if (voice_file.read((uint8_t *)id, 4) != 4 || strcmp(id, "WAVE") != 0) {
        Serial.printf("[VOICE] bad WAVE: %s\n", path);
        voice_file.close();
        return false;
    }

    bool saw_fmt = false;
    uint16_t audio_format = 0;
    uint16_t bits_per_sample = 0;
    uint32_t sample_rate = 0;

    while (voice_file.available()) {
        if (voice_file.read((uint8_t *)id, 4) != 4) break;
        uint32_t chunk_size = read_le32(voice_file);

        if (strncmp(id, "fmt ", 4) == 0) {
            audio_format = read_le16(voice_file);
            voice_channels = read_le16(voice_file);
            sample_rate = read_le32(voice_file);
            read_le32(voice_file);
            read_le16(voice_file);
            bits_per_sample = read_le16(voice_file);
            if (chunk_size > 16) {
                voice_file.seek(voice_file.position() + (chunk_size - 16));
            }
            saw_fmt = true;
        } else if (strncmp(id, "data", 4) == 0) {
            if (!saw_fmt || audio_format != 1 || bits_per_sample != 16 ||
                (voice_channels != 1 && voice_channels != 2) ||
                sample_rate < 8000 || sample_rate > 48000) {
                Serial.printf("[VOICE] unsupported WAV: %s fmt=%u ch=%u bits=%u rate=%lu\n",
                              path,
                              audio_format,
                              voice_channels,
                              bits_per_sample,
                              (unsigned long)sample_rate);
                voice_file.close();
                return false;
            }
            voice_data_remaining = chunk_size;
            voice_sample_rate = sample_rate;
            voice_resample_phase = 1.0f;
            voice_last_sample = 0;
            voice_playing = true;
            Serial.printf("[VOICE] playing %s rate=%lu\n", path, (unsigned long)voice_sample_rate);
            return true;
        } else {
            voice_file.seek(voice_file.position() + chunk_size);
        }
    }

    Serial.printf("[VOICE] no data chunk: %s\n", path);
    voice_file.close();
    return false;
}

bool voice_start_next_segment() {
    while (!voice_queue_is_empty()) {
        const char *path = voice_queue[voice_q_head].path;
        voice_q_head = (voice_q_head + 1) % VOICE_QUEUE_SIZE;
        if (wav_open_segment(path)) {
            return true;
        }
    }

    voice_playing = false;
    return false;
}

bool voice_read_pcm_sample(int16_t *sample) {
    if (!voice_playing && !voice_start_next_segment()) {
        *sample = 0;
        return false;
    }

    if (!voice_file || voice_data_remaining < (voice_channels * 2)) {
        if (voice_file) voice_file.close();
        voice_playing = false;
        return voice_start_next_segment() ? voice_read_pcm_sample(sample) : false;
    }

    int16_t left = (int16_t)read_le16(voice_file);
    voice_data_remaining -= 2;

    if (voice_channels == 2) {
        read_le16(voice_file);
        voice_data_remaining -= 2;
    }

    *sample = left;
    return true;
}

int16_t voice_read_sample() {
    int16_t out = 0;

    if (voice_sample_rate == (uint32_t)SAMPLE_RATE) {
        int16_t sample = 0;
        voice_read_pcm_sample(&sample);
        out = sample;
    } else {
        while (voice_resample_phase >= 1.0f) {
            int16_t sample = 0;
            if (!voice_read_pcm_sample(&sample)) {
                return 0;
            }
            voice_last_sample = sample;
            voice_resample_phase -= 1.0f;
        }

        voice_resample_phase += ((float)voice_sample_rate / SAMPLE_RATE);
        out = voice_last_sample;
    }

    int32_t boosted = (int32_t)out * VOICE_GAIN;
    if (boosted > 32767) return 32767;
    if (boosted < -32768) return -32768;
    return (int16_t)boosted;
}

const char *direction_voice_path(uint8_t dir) {
    switch (dir) {
        case 0: return "/voice/left.wav";
        case 1: return "/voice/center.wav";
        case 2: return "/voice/right.wav";
        default: return "/voice/center.wav";
    }
}

const char *severity_voice_path(uint8_t sev) {
    switch (sev) {
        case 1: return "/voice/caution.wav";
        case 2: return "/voice/danger.wav";
        default: return "/voice/safe.wav";
    }
}

const char *class_voice_path(uint8_t cls) {
    switch (cls) {
        case 0: return "/voice/person.wav";
        case 1: return "/voice/bicycle.wav";
        case 2: return "/voice/car.wav";
        case 3: return "/voice/motorcycle.wav";
        case 5: return "/voice/bus.wav";
        case 7: return "/voice/truck.wav";
        case 15: return "/voice/cat.wav";
        case 16: return "/voice/dog.wav";
        case 56: return "/voice/chair.wav";
        case 0xFF: return "/voice/unknown.wav";
        default: return nullptr;
    }
}

bool enqueue_voice_warning(uint8_t dir, uint8_t cls, uint8_t sev) {
    static Warning last_voice_warning = {0xFF, 0xFF, 0xFF};
    static unsigned long last_voice_enqueue_time = 0;
    const unsigned long now = millis();
    const bool same_warning =
        last_voice_warning.direction == dir &&
        last_voice_warning.class_id == cls &&
        last_voice_warning.severity == sev;

    if (sev == 0) {
        voice_queue_clear(true);
        last_voice_warning = {dir, cls, sev};
        last_voice_enqueue_time = now;
        return false;
    }

    if (same_warning && (now - last_voice_enqueue_time) < 3000) {
        return false;
    }

    voice_queue_clear(false);
    last_voice_warning = {dir, cls, sev};
    last_voice_enqueue_time = now;

    bool queued_any = false;

    queued_any |= voice_queue_push(direction_voice_path(dir));

    const char *path = class_voice_path(cls);
    if (path) {
        queued_any |= voice_queue_push(path);
    } else {
        queued_any |= voice_queue_push("/voice/object.wav");
    }

    queued_any |= voice_queue_push(severity_voice_path(sev));

    return queued_any;
}

// ===== A2DP Audio Frame Generation =====
bool tone_on_for_alarm(AlarmType alarm, float phase) {
    switch(alarm) {
        case ALARM_CAUTION:
            /* Slow double-tap: two short beeps, then a long pause. */
            return (phase < 0.12f) || (phase >= 0.28f && phase < 0.40f);

        case ALARM_DANGER:
            /* Urgent triple-tap: three rapid beeps. */
            return (phase < 0.09f) ||
                   (phase >= 0.16f && phase < 0.25f) ||
                   (phase >= 0.32f && phase < 0.41f);

        case ALARM_UNKNOWN:
            /* Distinct long-short-long pattern. */
            return (phase < 0.22f) ||
                   (phase >= 0.38f && phase < 0.48f) ||
                   (phase >= 0.64f && phase < 0.86f);

        default:
            return false;
    }
}

int32_t get_data_frames(Frame *frame, int32_t frame_count) {
    static float t = 0.0;
    
    for (int i = 0; i < frame_count; i++) {
        const bool voice_active = voice_playing || !voice_queue_is_empty();
        int16_t voice_sample = voice_active ? voice_read_sample() : 0;
        int16_t beep_sample = 0;
        float freq = 0.0f;
        float cycle = 1.0f;
        
        switch(current_alarm) {
            case ALARM_SAFE:
                break;
                
            case ALARM_CAUTION:
                freq = 880.0f;
                cycle = 1.20f;
                break;
                
            case ALARM_DANGER:
                freq = 1760.0f;
                cycle = 0.62f;
                break;
                
            case ALARM_UNKNOWN:
                freq = 660.0f;
                cycle = 1.35f;
                break;
        }

        if (current_alarm != ALARM_SAFE && tone_on_for_alarm(current_alarm, fmod(t, cycle))) {
            beep_sample = (int16_t)(AUDIO_AMPLITUDE * sin(2 * PI * freq * t));
        }
        
        frame[i].channel1 = voice_sample;  /* Left ear: voice notification */
        frame[i].channel2 = beep_sample;   /* Right ear: warning beep */
        t += 1.0 / SAMPLE_RATE;
    }
    
    return frame_count;
}

// ===== UART Packet Parsing =====
uint8_t calculate_checksum(uint8_t dir, uint8_t cls, uint8_t sev) {
    return (dir ^ cls ^ sev);
}

void parse_uart_packet(uint8_t *packet) {
    // Verify packet format
    if (packet[0] != UART_START_BYTE || packet[5] != UART_END_BYTE) {
        Serial.println("[ERROR] Invalid packet format");
        return;
    }
    
    uint8_t dir = packet[1];
    uint8_t cls = packet[2];
    uint8_t sev = packet[3];
    uint8_t checksum = packet[4];
    
    // Verify checksum
    if (checksum != calculate_checksum(dir, cls, sev)) {
        Serial.println("[ERROR] Checksum mismatch");
        return;
    }
    
    // Update warning state
    current_warning.direction = dir;
    current_warning.class_id = cls;
    current_warning.severity = sev;
    last_warning_time = millis();
    
    // Set alarm based on severity and class
    if (sev == 2) {
        current_alarm = ALARM_DANGER;
    } else if (sev == 1) {
        if (cls == 0xFF) {
            current_alarm = ALARM_UNKNOWN;  /* Unknown obstacle */
        } else {
            current_alarm = ALARM_CAUTION;  /* Known object */
        }
    } else {
        current_alarm = ALARM_SAFE;
    }
    
    // Print debug info
    const char *dir_names[] = {"LEFT", "CENTER", "RIGHT"};
    const char *sev_names[] = {"SAFE", "CAUTION", "DANGER"};
    const char *class_name = (cls == 0xFF) ? "UNKNOWN" : "KNOWN";
    
    Serial.printf("[UART RX] dir=%s, cls=%s(%02X), sev=%s\n",
                  dir < 3 ? dir_names[dir] : "INVALID",
                  class_name,
                  cls,
                  sev < 3 ? sev_names[sev] : "INVALID");

    enqueue_voice_warning(dir, cls, sev);
}

void handle_uart_receive() {
    static uint8_t packet_buffer[UART_PACKET_SIZE];
    static uint8_t buffer_index = 0;
    
    while (uart_serial.available()) {
        uint8_t byte = uart_serial.read();

#if UART_RAW_BYTE_LOG
        Serial.printf("[UART BYTE] %02X", byte);
        if (byte >= 32 && byte <= 126) {
            Serial.printf(" '%c'", byte);
        }
        Serial.println();
#endif
        
        // Wait for START_BYTE
        if (buffer_index == 0) {
            if (byte == UART_START_BYTE) {
                packet_buffer[0] = byte;
                buffer_index = 1;
            }
            continue;
        }
        
        // Receive subsequent bytes
        packet_buffer[buffer_index] = byte;
        buffer_index++;
        
        // Packet complete
        if (buffer_index == UART_PACKET_SIZE) {
            parse_uart_packet(packet_buffer);
            buffer_index = 0;  /* Reset buffer */
        }
    }
}

// ===== Bluetooth Connection Callback =====
void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
    const char *state_str = a2dp_source.to_str(state);
    Serial.printf("[A2DP] Connection state: %s\n", state_str);
    
    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        Serial.println("[A2DP] ✓ Connected to WH-1000XM6!");
        current_alarm = ALARM_SAFE;  /* Reset alarm on connection */
#if VOICE_SELF_TEST_ON_CONNECT
        if (!voice_self_test_done) {
            voice_self_test_done = true;
            Serial.println("[VOICE] self-test queued");
            voice_queue_clear(true);
            voice_queue_push("/voice/center.wav");
            voice_queue_push("/voice/person.wav");
            voice_queue_push("/voice/caution.wav");
        }
#endif
    } else {
        Serial.println("[A2DP] ✗ Disconnected");
    }
}

// ===== Setup & Loop =====
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n");
    Serial.println("========================================");
    Serial.println(" ESP32 Bluetooth UART Warning Bridge");
    Serial.printf(" FW_VERSION: %s\n", FW_VERSION);
    Serial.println("========================================");
    Serial.println("[INFO] Initializing UART...");

    spiffs_ready = SPIFFS.begin(false);
    Serial.printf("[SPIFFS] %s\n", spiffs_ready ? "ready" : "mount failed");
    
    // Initialize UART (M55M1 PB3/PB2 <-> ESP32 GPIO26/25)
    pinMode(RX_PIN, INPUT_PULLUP);
    uart_serial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.printf("[UART] Initialized at %d baud\n", BAUD_RATE);
    Serial.printf("[UART] RX=GPIO%d (M55M1 PB3/D1), TX=GPIO%d (M55M1 PB2/D0)\n", RX_PIN, TX_PIN);
    
#if !UART_ONLY_TEST
    // Initialize Bluetooth A2DP
    Serial.println("[INFO] Initializing Bluetooth A2DP...");
    a2dp_source.set_on_connection_state_changed(connection_state_changed);
    a2dp_source.set_data_callback_in_frames(get_data_frames);
    a2dp_source.set_volume(70);  /* 0-127 */
    a2dp_source.start(TARGET_DEVICE);
    
    Serial.printf("[A2DP] Connecting to %s...\n", TARGET_DEVICE);
#else
    Serial.println("[INFO] UART_ONLY_TEST=1, Bluetooth A2DP is disabled");
#endif
    Serial.println("========================================\n");
}

void loop() {
    static unsigned long last_heartbeat_ms = 0;
    static int last_rx_level = HIGH;
    static uint32_t rx_edge_count = 0;

    int rx_level = digitalRead(RX_PIN);
    if (rx_level != last_rx_level) {
        last_rx_level = rx_level;
        rx_edge_count++;
    }

    // Continuously receive UART packets
    handle_uart_receive();

#if UART_ONLY_TEST
    if (millis() - last_heartbeat_ms >= 2000) {
        last_heartbeat_ms = millis();
        Serial.printf("[HEARTBEAT] waiting for UART bytes... RX GPIO%d=%s edges=%lu\n",
                      RX_PIN,
                      rx_level ? "HIGH" : "LOW",
                      (unsigned long)rx_edge_count);
    }
#endif
    
    // Timeout clear (5 seconds no signal -> SAFE)
    if (millis() - last_warning_time > UART_TIMEOUT_MS && current_alarm != ALARM_SAFE) {
        current_alarm = ALARM_SAFE;
        Serial.println("[TIMEOUT] No warning signal for 5s, returning to SAFE");
    }
    
    delay(10);  /* Prevent watchdog timeout */
}
