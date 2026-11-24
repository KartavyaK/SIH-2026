#include <Arduino.h>
#include "sdkconfig.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled for this board! In Arduino IDE, make sure you selected 'ESP32 Dev Module' (Tools -> Board -> esp32 -> ESP32 Dev Module)."
#endif

#include "BluetoothSerial.h"
#include <Wire.h>
#include <U8g2lib.h>
#include <driver/i2s_std.h>
#include <math.h>

// =====================================================
// BLUETOOTH & THREAD SAFETY
// =====================================================
BluetoothSerial SerialBT;
SemaphoreHandle_t btSendMutex = NULL;

#define CONTROLLER_NAME "CONTROLLER_ESP32"
uint8_t robotAddress[6] = { 0x68, 0x09, 0x47, 0xB4, 0x7A, 0x7A };

// =====================================================
// CONTROLS & PTT BUTTON
// =====================================================
#define JOY_X          34
#define JOY_Y          35
#define JOY_MS         16   // Joystick press = Emergency Stop
#define MODE_SWITCH    27   // Drive mode switch (LOW = Land, HIGH = Water)
#define PTT_BUTTON_PIN 23   // 4-Pin Push-To-Talk Button (Pressed = LOW)

#define MODE_LAND  0
#define MODE_WATER 1

int centerX = 2048, centerY = 2048;

// =====================================================
// OLED DISPLAY (Universal 128x64 I2C: SDA=18, SCL=19)
// =====================================================
#define OLED_SDA 18
#define OLED_SCL 19

// Universal SSD1306 / SH1106 128x64 HW I2C constructor
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// =====================================================
// AUDIO HARDWARE (Controller: MAX98357A + INMP441)
//   SHARED CLOCKS: BCLK/SCK=GPIO 14, WS/LRC=GPIO 25
//   MIC DATA:      SD=GPIO 33
//   SPK DATA:      DIN=GPIO 26
// =====================================================
#define I2S_BCLK_PIN    14   // Connect to MAX98357A BCLK & INMP441 SCK
#define I2S_WS_PIN      25   // Connect to MAX98357A LRC  & INMP441 WS
#define I2S_MIC_SD_PIN  33   // Connect to INMP441 SD
#define I2S_SPK_DIN_PIN 26   // Connect to MAX98357A DIN

#define AUDIO_SAMPLE_RATE    8000
#define AUDIO_CHUNK_SAMPLES  64

i2s_chan_handle_t tx_handle = NULL;
i2s_chan_handle_t rx_handle = NULL;

float hpfX = 0.0f, hpfY = 0.0f;

// =====================================================
// PACKET TYPES
// =====================================================
#define PKT_CMD   1
#define PKT_C2R   2
#define PKT_R2C   3
#define PKT_DIST  4

uint8_t  rxBuf[512];
uint8_t  rxState = 0, rxType = 0;
uint16_t rxLen = 0, rxIdx = 0;

int16_t  distCM = -1;
unsigned long audioRecv = 0, audioSent = 0;
unsigned long lastAudioRecvTime = 0;
int      ctrlMicPeak = 0;
unsigned long lastJoySend = 0, lastOLED = 0;

// =====================================================
// THREAD-SAFE SEND PACKET
// =====================================================
void sendPacket(uint8_t type, const uint8_t *data, uint16_t len)
{
  if (!SerialBT.connected() || len > 255) return;
  if (btSendMutex && xSemaphoreTake(btSendMutex, pdMS_TO_TICKS(15)) == pdTRUE)
  {
    uint8_t hdr[4] = { 0xAA, 0x55, type, (uint8_t)len };
    SerialBT.write(hdr, 4);
    if (len && data) SerialBT.write(data, len);
    xSemaphoreGive(btSendMutex);
  }
}

// =====================================================
// STARTUP SHORT BEEP (0.2s clean tone on MAX98357A)
// =====================================================
void playStartupBeep()
{
  if (!tx_handle) return;
  Serial.println("Controller: Playing startup beep on MAX98357A...");
  int32_t buf[AUDIO_CHUNK_SAMPLES * 2];
  int total = AUDIO_SAMPLE_RATE * 200 / 1000; // 200 ms @ 1000 Hz

  for (int chunk = 0; chunk < total / AUDIO_CHUNK_SAMPLES; chunk++)
  {
    for (int i = 0; i < AUDIO_CHUNK_SAMPLES; i++)
    {
      int idx = chunk * AUDIO_CHUNK_SAMPLES + i;
      int32_t s = (int32_t)(sinf(2.0f * PI * 1000.0f * idx / AUDIO_SAMPLE_RATE) * 25000.0f) << 16;
      buf[i * 2]     = s;
      buf[i * 2 + 1] = s;
    }
    size_t w = 0;
    i2s_channel_write(tx_handle, buf, sizeof(buf), &w, portMAX_DELAY);
  }
  Serial.println("Controller: Startup beep complete.");
}

// =====================================================
// JOYSTICK & DRIVE CONTROL
// =====================================================
void calibrateJoystick()
{
  long sx = 0, sy = 0;
  for (int i = 0; i < 20; i++) { sx += analogRead(JOY_X); sy += analogRead(JOY_Y); delay(10); }
  centerX = sx / 20; centerY = sy / 20;
  Serial.printf("Joystick center X=%d Y=%d\n", centerX, centerY);
}

void sendJoystick()
{
  if (!SerialBT.connected()) return;

  const int DEAD = 250;
  int dx = analogRead(JOY_X) - centerX;
  int dy = centerY - analogRead(JOY_Y); // Forward is up

  int turn     = 0;
  int throttle = 0;

  if (abs(dx) > DEAD)
  {
    if (dx > 0)
      turn = map(dx, DEAD, 4095 - centerX, 0, 255);
    else
      turn = map(dx, -centerX, -DEAD, -255, 0);
  }

  if (abs(dy) > DEAD)
  {
    if (dy > 0)
      throttle = map(dy, DEAD, 4095 - centerY, 0, 255);
    else
      throttle = map(dy, -centerY, -DEAD, -255, 0);
  }

  turn     = constrain(turn, -255, 255);
  throttle = constrain(throttle, -255, 255);

  // Differential drive
  int left  = throttle + turn;
  int right = throttle - turn;

  left  = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (digitalRead(JOY_MS) == LOW)
  {
    left = 0; right = 0; // Emergency Stop
  }

  // Exact Mode Alignment: LOW = Land (0), HIGH = Water (1)
  uint8_t currentMode = (digitalRead(MODE_SWITCH) == LOW) ? MODE_LAND : MODE_WATER;

  uint8_t pkt[5];
  pkt[0] = currentMode;
  int16_t l = (int16_t)left;
  int16_t r = (int16_t)right;
  memcpy(&pkt[1], &l, 2);
  memcpy(&pkt[3], &r, 2);
  sendPacket(PKT_CMD, pkt, 5);
}

// =====================================================
// AUDIO SETUP (Full-Duplex I2S on GPIO 14, 25, 33, 26)
// =====================================================
void setupAudio()
{
  i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan.dma_desc_num  = 6;
  chan.dma_frame_num = AUDIO_CHUNK_SAMPLES;
  chan.auto_clear    = true;

  esp_err_t ret = i2s_new_channel(&chan, &tx_handle, &rx_handle);
  if (ret != ESP_OK) { Serial.printf("Controller i2s_new_channel fail: %x\n", ret); return; }

  // TX Configuration (MAX98357A Speaker Output)
  i2s_std_config_t tx_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK_PIN,
      .ws   = (gpio_num_t)I2S_WS_PIN,
      .dout = (gpio_num_t)I2S_SPK_DIN_PIN,
      .din  = I2S_GPIO_UNUSED,
      .invert_flags = { false, false, false },
    },
  };
  ret = i2s_channel_init_std_mode(tx_handle, &tx_cfg);
  if (ret != ESP_OK) { Serial.printf("Controller TX init fail: %x\n", ret); return; }

  // RX Configuration (INMP441 Microphone Input)
  i2s_std_config_t rx_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = I2S_GPIO_UNUSED,
      .ws   = I2S_GPIO_UNUSED,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)I2S_MIC_SD_PIN,
      .invert_flags = { false, false, false },
    },
  };
  ret = i2s_channel_init_std_mode(rx_handle, &rx_cfg);
  if (ret != ESP_OK) { Serial.printf("Controller RX init fail: %x\n", ret); return; }

  i2s_channel_enable(tx_handle);
  i2s_channel_enable(rx_handle);

  // Pre-fill DMA silence
  int32_t sil[AUDIO_CHUNK_SAMPLES * 2] = {0};
  size_t  w = 0;
  i2s_channel_write(tx_handle, sil, sizeof(sil), &w, pdMS_TO_TICKS(100));

  Serial.println("Controller I2S initialized (BCLK:14, WS:25, SD:33, DIN:26)");

  playStartupBeep();
}

// =====================================================
// PLAY AUDIO (From Robot Mic to Controller MAX98357A Speaker)
// =====================================================
void playAudio(const uint8_t *data, uint16_t len)
{
  if (!len || !tx_handle) return;
  audioRecv++;
  lastAudioRecvTime = millis();

  uint16_t n = min(len, (uint16_t)AUDIO_CHUNK_SAMPLES);
  int32_t  buf[AUDIO_CHUNK_SAMPLES * 2];
  for (uint16_t i = 0; i < n; i++)
  {
    int16_t s16 = ((int16_t)data[i] - 128) << 8;
    int32_t s32 = ((int32_t)s16) << 16;
    buf[i * 2]     = s32; // Left
    buf[i * 2 + 1] = s32; // Right
  }
  size_t w = 0;
  i2s_channel_write(tx_handle, buf, n * 2 * sizeof(int32_t), &w, pdMS_TO_TICKS(50));
}

// =====================================================
// CAPTURE CONTROLLER MIC (HIGH-GAIN PTT SPEECH BOOST)
// =====================================================
void processMic()
{
  bool isPttActive = (digitalRead(PTT_BUTTON_PIN) == LOW);

  if (!SerialBT.connected() || !rx_handle || !isPttActive)
  {
    ctrlMicPeak = 0;
    return;
  }

  int32_t raw[AUDIO_CHUNK_SAMPLES * 2];
  size_t  bytes = 0;
  if (i2s_channel_read(rx_handle, raw, sizeof(raw), &bytes, pdMS_TO_TICKS(20)) != ESP_OK) return;
  if (!bytes) return;

  uint16_t n = bytes / (sizeof(int32_t) * 2);
  uint8_t  pcm[AUDIO_CHUNK_SAMPLES];
  int peak = 0;

  for (uint16_t i = 0; i < n; i++)
  {
    int32_t s32 = (abs(raw[i * 2]) > abs(raw[i * 2 + 1])) ? raw[i * 2] : raw[i * 2 + 1];

    float x = (float)(s32 >> 14);

    float y = x - hpfX + 0.95f * hpfY;
    hpfX = x;
    hpfY = y;

    // High Digital Speech Gain (24x boost)
    int32_t s = (int32_t)(y * 24.0f);
    s = constrain(s, -32768, 32767);

    // Noise Gate: silence ambient mic noise
    if (abs(s) < 300) s = 0;

    pcm[i] = (uint8_t)((s + 32768) >> 8);
    if (abs(s) > peak) peak = abs(s);
  }

  ctrlMicPeak = peak;
  audioSent++;
  sendPacket(PKT_C2R, pcm, n);
}

// =====================================================
// PACKET RECEIVE
// =====================================================
void processPacket(uint8_t type, uint8_t *data, uint16_t len)
{
  if (type == PKT_DIST && len >= 2) memcpy(&distCM, data, 2);
  else if (type == PKT_R2C)         playAudio(data, len);
}

void receiveBT()
{
  while (SerialBT.available())
  {
    uint8_t b = SerialBT.read();
    switch (rxState)
    {
      case 0: if (b == 0xAA) rxState = 1; break;
      case 1: if (b == 0x55) rxState = 2; else rxState = (b == 0xAA) ? 1 : 0; break;
      case 2: rxType = b; rxState = 3; break;
      case 3:
        rxLen = b; rxIdx = 0;
        if (rxLen > sizeof(rxBuf)) rxState = 0;
        else if (!rxLen) { processPacket(rxType, rxBuf, 0); rxState = 0; }
        else rxState = 4;
        break;
      case 4:
        rxBuf[rxIdx++] = b;
        if (rxIdx >= rxLen) { processPacket(rxType, rxBuf, rxLen); rxState = 0; }
        break;
      default: rxState = 0;
    }
  }
}

// =====================================================
// AUTO-RECONNECT TO ROBOT
// =====================================================
void checkBtConnection()
{
  if (!SerialBT.connected())
  {
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry > 3000)
    {
      lastRetry = millis();
      Serial.println("[BT] Reconnecting to Robot...");
      SerialBT.connect(robotAddress);
    }
  }
}

// =====================================================
// OLED DISPLAY
// =====================================================
void updateOLED()
{
  if (millis() - lastOLED < 200) return;
  lastOLED = millis();

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);

  // Line 1: Bluetooth Connection Status
  oled.drawStr(0, 10, SerialBT.connected() ? "BT: CONNECTED" : "BT: CONNECTING...");

  // Line 2: Drive Mode (Exact alignment: LOW = LAND, HIGH = WATER)
  if (digitalRead(MODE_SWITCH) == LOW)
  {
    oled.drawStr(0, 21, "MODE: LAND");
  }
  else
  {
    oled.drawStr(0, 21, "MODE: WATER");
  }

  // Line 3: Distance Telemetry
  char d[24];
  snprintf(d, sizeof(d), distCM < 0 ? "Dist: -- cm" : "Dist: %d cm", distCM);
  oled.drawStr(0, 34, d);

  // Line 4: Push-to-Talk Status & RX Packet Count
  bool isTalking = (digitalRead(PTT_BUTTON_PIN) == LOW);
  bool isHearing = (millis() - lastAudioRecvTime < 500);

  char a[32];
  if (isTalking) {
    snprintf(a, sizeof(a), "TX:[TALKING] Peak:%d", ctrlMicPeak);
  } else if (isHearing) {
    snprintf(a, sizeof(a), "RX:%lu [HEARING]", audioRecv);
  } else {
    snprintf(a, sizeof(a), "RX:%lu [READY]", audioRecv);
  }
  oled.drawStr(0, 47, a);

  // Line 5: Emergency Status
  oled.drawStr(0, 60, digitalRead(JOY_MS) == LOW ? "!!! EMERGENCY STOP !!!" : "SYSTEM READY");

  oled.sendBuffer();
}

// =====================================================
// I2C SCANNER HELPER (Runs on boot)
// =====================================================
void scanI2C()
{
  Serial.println("\n--- Scanning I2C Bus on SDA=GPIO18, SCL=GPIO19 ---");
  byte count = 0;
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
    {
      Serial.printf(">>> Found I2C device at address: 0x%02X <<<\n", i);
      count++;
    }
  }
  if (count == 0)
  {
    Serial.println("WARNING: No I2C display detected! Please check OLED SDA(18), SCL(19), VCC(3.3V/5V), GND.");
  }
  Serial.println("--------------------------------------------------\n");
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n======= CONTROLLER ESP32 =======");

  btSendMutex = xSemaphoreCreateMutex();

  pinMode(JOY_MS, INPUT_PULLUP);
  pinMode(MODE_SWITCH, INPUT_PULLUP);
  pinMode(PTT_BUTTON_PIN, INPUT_PULLUP);  // 4-Pin Push-To-Talk Button

  calibrateJoystick();

  // Initialize I2C on GPIO 18 (SDA) and GPIO 19 (SCL)
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000); // 400kHz Fast I2C
  
  // Diagnostic scan for display
  scanI2C();

  // Initialize OLED
  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 20, "CONTROLLER ESP32");
  oled.drawStr(0, 35, "Starting audio & BT...");
  oled.sendBuffer();

  setupAudio();

  SerialBT.begin(CONTROLLER_NAME, true);
  Serial.println("BT init. Connecting...");
  bool conn = SerialBT.connect(robotAddress);
  Serial.println(conn ? "BT connected!" : "BT connect failed.");

  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 20, "CONTROLLER");
  oled.drawStr(0, 35, conn ? "BT: CONNECTED" : "BT: CONNECTING...");
  oled.drawStr(0, 50, "Voice Active");
  oled.sendBuffer();

  Serial.println("Setup done.\n");
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  checkBtConnection();

  receiveBT();

  // Process mic when PTT is pressed
  processMic();

  if (millis() - lastJoySend >= 20)
  {
    lastJoySend = millis();
    sendJoystick();
  }

  updateOLED();

  static unsigned long dbg = 0;
  if (millis() - dbg > 2000)
  {
    dbg = millis();
    Serial.printf("[CTRL] BT:%s | PTT:%s | MicPeak:%d | Sent:%lu | Recv:%lu\n",
      SerialBT.connected() ? "CONN" : "WAIT",
      digitalRead(PTT_BUTTON_PIN) == LOW ? "TALKING" : "IDLE",
      ctrlMicPeak, audioSent, audioRecv);
  }
  delay(1);
}