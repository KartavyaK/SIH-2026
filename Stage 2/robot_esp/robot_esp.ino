#include <Arduino.h>
#include "BluetoothSerial.h"
#include <driver/i2s_std.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <math.h>

// =====================================================
// BLUETOOTH & THREAD SAFETY
// =====================================================
BluetoothSerial SerialBT;
SemaphoreHandle_t btSendMutex = NULL;

#define BT_NAME "ROBOT_ESP32"

bool isBtConnected()
{
  return SerialBT.hasClient() || SerialBT.connected();
}

// =====================================================
// GPS MODULE (RESERVED) - GPIO 16 RX, GPIO 17 TX
// =====================================================

// =====================================================
// ULTRASONIC SENSOR
// =====================================================
#define TRIG_PIN 2
#define ECHO_PIN 34

// =====================================================
// LAND MOTORS (L298N)
// =====================================================
#define LAND_ENA 13
#define LAND_IN1 14
#define LAND_IN2 27
#define LAND_ENB 32
#define LAND_IN3 23
#define LAND_IN4 19

// =====================================================
// WATER MOTORS (L298N) - ENA side connected to Propeller
// =====================================================
#define WATER_ENA 18
#define WATER_IN1 5
#define WATER_IN2 4
#define WATER_ENB 15
#define WATER_IN3 12
#define WATER_IN4 21

#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

// =====================================================
// AUDIO HARDWARE (Robot: PAM8403 Speaker + INMP441 Mic)
//   SPEAKER: PAM8403 Analog Amp -> ESP32 Built-in DAC1 (GPIO 25)
//   MIC:     INMP441 I2S Mic     -> SCK=22, WS=26, SD=33
// =====================================================
#define DAC_PIN         25   // Built-in DAC1 -> Connect to PAM8403 L (and R)
#define I2S_MIC_SCK     22   // Connect to INMP441 SCK
#define I2S_MIC_WS      26   // Connect to INMP441 WS
#define I2S_MIC_SD      33   // Connect to INMP441 SD

#define AUDIO_SAMPLE_RATE    8000
#define AUDIO_CHUNK_SAMPLES  64

i2s_chan_handle_t rx_handle = NULL;
float hpfX = 0.0f, hpfY = 0.0f;

uint8_t smoothedSample = 128;

int robotMicPeak = 0;
unsigned long audioPktsSent = 0;
unsigned long audioPktsRecv = 0;

// =====================================================
// PACKET TYPES & MODES
// =====================================================
#define PKT_CMD   1
#define PKT_C2R   2
#define PKT_R2C   3
#define PKT_DIST  4

#define MODE_LAND  0
#define MODE_WATER 1

int   targetLeft = 0, targetRight = 0;
int   currentLeft = 0, currentRight = 0;
uint8_t mode    = MODE_LAND;
unsigned long lastCmdTime = 0;

uint8_t  rxBuf[512];
uint8_t  rxState = 0, rxType = 0;
uint16_t rxLen = 0, rxIdx = 0;

// =====================================================
// THREAD-SAFE SEND PACKET
// =====================================================
void sendPacket(uint8_t type, const uint8_t *data, uint16_t len)
{
  if (!isBtConnected() || len > 255) return;
  if (btSendMutex && xSemaphoreTake(btSendMutex, pdMS_TO_TICKS(15)) == pdTRUE)
  {
    uint8_t hdr[4] = { 0xAA, 0x55, type, (uint8_t)len };
    SerialBT.write(hdr, 4);
    if (len && data) SerialBT.write(data, len);
    xSemaphoreGive(btSendMutex);
  }
}

// =====================================================
// STARTUP SHORT BEEP (0.2s clean tone on PAM8403 DAC)
// =====================================================
void playStartupBeep()
{
  Serial.println("Robot: Playing startup beep on PAM8403 (GPIO 25)...");
  for (int i = 0; i < 200; i++)
  {
    for (int j = 0; j < 8; j++)
    {
      dacWrite(DAC_PIN, (j < 4) ? 210 : 45);
      delayMicroseconds(125);
    }
  }
  dacWrite(DAC_PIN, 128); // Neutral bias
  Serial.println("Robot: Startup beep complete.");
}

// =====================================================
// MOTOR CONTROL FUNCTIONS (True Bidirectional H-Bridge)
// =====================================================
void motor(int in1, int in2, int pwmPin, int speed)
{
  speed = constrain(speed, -255, 255);

  if (speed > 0)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmPin, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmPin, -speed);
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmPin, 0);
  }
}

void stopAllMotors()
{
  targetLeft = 0;
  targetRight = 0;
  currentLeft = 0;
  currentRight = 0;
  motor(LAND_IN1,  LAND_IN2,  LAND_ENA,  0);
  motor(LAND_IN3,  LAND_IN4,  LAND_ENB,  0);
  motor(WATER_IN1, WATER_IN2, WATER_ENA, 0);
  motor(WATER_IN3, WATER_IN4, WATER_ENB, 0);
}

// Soft-Start Current Slew Rate Limiter (Prevents sudden voltage drops)
void updateMotorSlew()
{
  static unsigned long lastSlew = 0;
  if (millis() - lastSlew < 15) return; // 15ms ramp interval
  lastSlew = millis();

  const int MAX_STEP = 35; // Maximum speed change per 15ms

  if (currentLeft < targetLeft) currentLeft = min(currentLeft + MAX_STEP, targetLeft);
  else if (currentLeft > targetLeft) currentLeft = max(currentLeft - MAX_STEP, targetLeft);

  if (currentRight < targetRight) currentRight = min(currentRight + MAX_STEP, targetRight);
  else if (currentRight > targetRight) currentRight = max(currentRight - MAX_STEP, targetRight);

  if (mode == MODE_LAND)
  {
    // LAND MODE: Only Land L298N active
    motor(LAND_IN1, LAND_IN2, LAND_ENA, currentLeft);
    motor(LAND_IN3, LAND_IN4, LAND_ENB, currentRight);

    // Water L298N completely stopped
    motor(WATER_IN1, WATER_IN2, WATER_ENA, 0);
    motor(WATER_IN3, WATER_IN4, WATER_ENB, 0);
  }
  else
  {
    // WATER MODE: Both Land wheels and Water Propeller active
    motor(LAND_IN1, LAND_IN2, LAND_ENA, currentLeft);
    motor(LAND_IN3, LAND_IN4, LAND_ENB, currentRight);

    int propellerSpeed = (currentLeft + currentRight) / 2;
    motor(WATER_IN1, WATER_IN2, WATER_ENA, propellerSpeed);

    motor(WATER_IN3, WATER_IN4, WATER_ENB, 0);
  }
}

void setupMotors()
{
  pinMode(LAND_IN1, OUTPUT);
  pinMode(LAND_IN2, OUTPUT);
  pinMode(LAND_IN3, OUTPUT);
  pinMode(LAND_IN4, OUTPUT);

  pinMode(WATER_IN1, OUTPUT);
  pinMode(WATER_IN2, OUTPUT);
  pinMode(WATER_IN3, OUTPUT);
  pinMode(WATER_IN4, OUTPUT);

  ledcAttach(LAND_ENA,  PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(LAND_ENB,  PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(WATER_ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(WATER_ENB, PWM_FREQ, PWM_RESOLUTION);

  stopAllMotors();
}

// =====================================================
// ULTRASONIC DISTANCE SENSOR
// =====================================================
void sendDistance()
{
  static unsigned long last = 0;
  if (millis() - last < 200) return;
  last = millis();

  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 25000);
  int16_t cm = dur ? (int16_t)(dur * 0.0343f / 2.0f) : -1;
  uint8_t d[2];
  memcpy(d, &cm, 2);
  sendPacket(PKT_DIST, d, 2);
}

// =====================================================
// I2S AUDIO SETUP
// =====================================================
void setupAudio()
{
  // 1. Built-in DAC on GPIO 25 for PAM8403 Speaker
  pinMode(DAC_PIN, OUTPUT);
  dacWrite(DAC_PIN, 128);

  playStartupBeep();

  // 2. I2S RX for INMP441 Mic (GPIO 22, 26, 33)
  i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan.dma_desc_num  = 6;
  chan.dma_frame_num = AUDIO_CHUNK_SAMPLES;
  chan.auto_clear    = true;

  esp_err_t ret = i2s_new_channel(&chan, NULL, &rx_handle);
  if (ret != ESP_OK) { Serial.printf("Robot mic i2s_new_channel fail: %x\n", ret); return; }

  i2s_std_config_t rx_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_MIC_SCK,
      .ws   = (gpio_num_t)I2S_MIC_WS,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)I2S_MIC_SD,
      .invert_flags = { false, false, false },
    },
  };
  ret = i2s_channel_init_std_mode(rx_handle, &rx_cfg);
  if (ret != ESP_OK) { Serial.printf("Robot mic RX init fail: %x\n", ret); return; }

  i2s_channel_enable(rx_handle);
  Serial.println("Robot Audio Ready (Speaker: PAM8403 DAC1 GPIO 25, Mic: SCK 22, WS 26, SD 33)");
}

// =====================================================
// PLAY AUDIO (From Controller PTT Mic to PAM8403 Speaker)
// =====================================================
void playAudio(const uint8_t *data, uint16_t len)
{
  if (!len) return;
  audioPktsRecv++;

  for (uint16_t i = 0; i < len; i++)
  {
    uint8_t target = data[i];
    smoothedSample = (uint8_t)(((int)smoothedSample * 1 + (int)target * 3) / 4);
    dacWrite(DAC_PIN, smoothedSample);
    delayMicroseconds(120);
  }
}

// =====================================================
// CAPTURE ROBOT MIC (CONTINUOUS STREAMING TO CONTROLLER)
// =====================================================
void processMic()
{
  if (!isBtConnected() || !rx_handle) return;

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

    // High Speech Gain Boost (20x)
    int32_t s = (int32_t)(y * 20.0f);
    s = constrain(s, -32768, 32767);

    // Noise Gate: silence room floor noise
    if (abs(s) < 300) s = 0;

    pcm[i] = (uint8_t)((s + 32768) >> 8);
    if (abs(s) > peak) peak = abs(s);
  }

  robotMicPeak = peak;
  audioPktsSent++;
  sendPacket(PKT_R2C, pcm, n);
}

// =====================================================
// PACKET RECEIVE
// =====================================================
void processPacket(uint8_t type, uint8_t *data, uint16_t len)
{
  if (type == PKT_CMD && len == 5)
  {
    mode = data[0];
    int16_t l16, r16;
    memcpy(&l16, &data[1], 2);
    memcpy(&r16, &data[3], 2);
    targetLeft  = constrain(l16, -255, 255);
    targetRight = constrain(r16, -255, 255);
    lastCmdTime = millis();
  }
  else if (type == PKT_C2R)
  {
    playAudio(data, len);
  }
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
// SETUP
// =====================================================
void setup()
{
  // Disable hardware brownout detector to prevent nuisance reboots from slight motor dips
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(500);
  Serial.println("\n======= ROBOT ESP32 (PAM8403 SPEAKER) =======");

  btSendMutex = xSemaphoreCreateMutex();
  SerialBT.begin(BT_NAME, false);
  Serial.print("BT MAC: "); Serial.println(SerialBT.getBtAddressString());

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  setupMotors();
  Serial.println("Motors initialized");

  setupAudio();

  Serial.println("Waiting for controller...\n");
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  receiveBT();
  sendDistance();
  processMic();

  // Failsafe: stop motors if no command received for 1000ms
  if (millis() - lastCmdTime > 1000)
  {
    targetLeft = 0;
    targetRight = 0;
  }

  // Smooth acceleration ramp
  updateMotorSlew();

  static unsigned long dbg = 0;
  if (millis() - dbg > 2000)
  {
    dbg = millis();
    Serial.printf("[ROBOT] BT:%s | Mode:%s | L:%d R:%d | MicPeak:%d\n",
      isBtConnected() ? "CONN" : "WAIT",
      (mode == MODE_LAND) ? "LAND" : "WATER",
      currentLeft, currentRight, robotMicPeak);
  }
  delay(1);
}