// ============================================================
//  Teensy Audio Shield — 4-band Notch + Delay + Autocut CLI
//  Board  : Teensy 3.x / 4.x
//  Shield : PJRC Audio Shield (SGTL5000)
//
//  CLI (115200 baud) — real units, no 0–100 abstraction:
//    f1freq          → get notch 1 frequency (Hz)
//    f1freq 800      → set notch 1 to 800 Hz
//    f1q             → get notch 1 Q
//    f1q 3.5         → set notch 1 Q to 3.5
//    f2freq / f2q    → notch 2
//    f3freq / f3q    → notch 3
//    f4freq / f4q    → notch 4
//    dlt             → get delay time (ms)
//    dlt 200         → set delay tap interval to 200 ms
//    list            → show all params
//    autocut         → run feedback calibration
//    help            → this message
//
//  Parameter ranges:
//    freq  : 20 – 20000 Hz
//    Q     : 0.1 – 20.0
//    delay : 10  – 1000 ms
// ============================================================

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// ---------- Audio graph -------------------------------------
//
//  i2s1 ──► biquad1 ──► delay1 ──► mixer1 ──► i2s2
//                                      ↑
//  i2s1 ──────────────────────────── (tap0, dry)
//
//  fft1024 taps from i2s1 (pre-filter, for autocut detection)

AudioInputI2S            i2s1;
AudioFilterBiquad        biquad1;
AudioEffectDelay         delay1;
AudioMixer4              mixer1;
AudioOutputI2S           i2s2;
AudioAnalyzeFFT1024      fft1024;

AudioConnection          patchCord1(i2s1,    0, biquad1,  0);
AudioConnection          patchCord2(biquad1, 0, delay1,   0);
AudioConnection          patchCord3(delay1,  0, mixer1,   0);   // dry (0 ms)
AudioConnection          patchCord4(delay1,  1, mixer1,   1);   // tap 1
AudioConnection          patchCord5(delay1,  2, mixer1,   2);   // tap 2
AudioConnection          patchCord6(delay1,  3, mixer1,   3);   // tap 3
AudioConnection          patchCord7(mixer1,  0, i2s2,     0);
AudioConnection          patchCord8(i2s1,    0, fft1024,  0);   // FFT from raw input

AudioControlSGTL5000     sgtl5000;

// ---------- Parameters (real units) -------------------------
struct NotchParam {
  float freq;   // Hz
  float q;
};

NotchParam notch[4] = {
  { 500.0f,  2.0f },
  { 1000.0f, 2.0f },
  { 2000.0f, 2.0f },
  { 4000.0f, 2.0f }
};

float delayMs    = 100.0f;   // ms, tap interval
int   inputLevel = 5;        // sgtl5000 lineInLevel 0–15

// ---------- Input buffer ------------------------------------
#define BUF_SIZE 64
char inputBuf[BUF_SIZE];
uint8_t bufIdx = 0;

// ============================================================
//  DSP apply
// ============================================================
void applyNotch(uint8_t band) {
  biquad1.setNotch(band, notch[band].freq, notch[band].q);
}

void applyAllNotches() {
  for (uint8_t i = 0; i < 4; i++) applyNotch(i);
}

void applyDelay() {
  delay1.delay(0, 0);               // tap 0 = dry (always 0 ms)
  delay1.delay(1, delayMs);
  delay1.delay(2, delayMs * 2.0f);
  delay1.delay(3, delayMs * 3.0f);
  delay1.disable(4);
  delay1.disable(5);
  delay1.disable(6);
  delay1.disable(7);

  mixer1.gain(0, 1.0f);
  mixer1.gain(1, 0.8f);
  mixer1.gain(2, 0.6f);
  mixer1.gain(3, 0.4f);
}

// ============================================================
//  Autocut calibration
// ============================================================

// FFT bin index → frequency in Hz
float binToHz(int bin) {
  return bin * (44100.0f / 1024.0f);
}

// Find the bin with the highest magnitude, excluding a guard zone
// around already-found peaks (guardBins wide on each side)
int findPeakBin(float* magnitudes, bool* excluded, int numBins) {
  int   peakBin = -1;
  float peakMag = 0.0f;
  for (int i = 2; i < numBins; i++) {   // skip DC (bin 0-1)
    if (!excluded[i] && magnitudes[i] > peakMag) {
      peakMag = magnitudes[i];
      peakBin = i;
    }
  }
  return peakBin;
}

void cmdAutocut() {
  const int   FFT_BINS      = 512;    // FFT1024 gives 512 usable bins
  const int   GUARD_BINS    = 8;      // ~344 Hz exclusion zone around each found peak
  const float RAMP_STEP_MS  = 200;    // ms between gain steps
  const int   STABLE_READS  = 6;      // FFT reads to average at each gain level
  const int   TARGET_PEAKS  = 4;
  const float PEAK_THRESHOLD = 0.1f;  // minimum magnitude to count as feedback

  Serial.println(F("\n  [AUTOCUT] Starting calibration..."));
  Serial.println(F("  [AUTOCUT] Make sure microphone and speaker are active."));
  Serial.println(F("  [AUTOCUT] Ramping gain — stand by.\n"));

  // Save current state
  int savedLevel = inputLevel;

  // Start from a safe gain
  inputLevel = 3;
  sgtl5000.lineInLevel(inputLevel);
  delay(300);

  float   magnitudes[FFT_BINS] = {0};
  bool    excluded[FFT_BINS]   = {false};
  float   foundFreqs[TARGET_PEAKS];
  int     foundCount = 0;

  // Ramp gain until we find peaks or hit ceiling
  while (inputLevel <= 14 && foundCount < TARGET_PEAKS) {
    sgtl5000.lineInLevel(inputLevel);
    Serial.print(F("  [AUTOCUT] lineInLevel = "));
    Serial.println(inputLevel);
    delay(200);   // let gain settle

    // Average several FFT frames for stability
    float avgMag[FFT_BINS] = {0};
    int   validReads = 0;
    unsigned long t0 = millis();
    while (validReads < STABLE_READS) {
      if (fft1024.available()) {
        for (int i = 0; i < FFT_BINS; i++)
          avgMag[i] += fft1024.read(i);
        validReads++;
      }
      if (millis() - t0 > 5000) break;   // timeout safety
    }
    if (validReads > 0)
      for (int i = 0; i < FFT_BINS; i++) avgMag[i] /= validReads;

    // Copy into magnitudes array for peak search
    memcpy(magnitudes, avgMag, sizeof(avgMag));

    // Check if any bin crossed the threshold
    float maxMag = 0;
    for (int i = 2; i < FFT_BINS; i++)
      if (!excluded[i] && magnitudes[i] > maxMag) maxMag = magnitudes[i];

    if (maxMag < PEAK_THRESHOLD) {
      // No feedback yet — increase gain
      inputLevel++;
      continue;
    }

    // Extract peaks greedily
    while (foundCount < TARGET_PEAKS) {
      int peakBin = findPeakBin(magnitudes, excluded, FFT_BINS);
      if (peakBin < 0 || magnitudes[peakBin] < PEAK_THRESHOLD) break;

      float hz = binToHz(peakBin);
      foundFreqs[foundCount] = hz;

      Serial.print(F("  [AUTOCUT] Peak found: "));
      Serial.print((int)hz);
      Serial.print(F(" Hz  (mag="));
      Serial.print(magnitudes[peakBin], 3);
      Serial.println(F(")"));

      // Apply notch immediately so it doesn't mask other peaks
      notch[foundCount].freq = hz;
      notch[foundCount].q    = 5.0f;   // tight notch for feedback
      applyNotch(foundCount);
      foundCount++;

      // Exclude guard zone around this peak
      int lo = max(0, peakBin - GUARD_BINS);
      int hi = min(FFT_BINS - 1, peakBin + GUARD_BINS);
      for (int j = lo; j <= hi; j++) excluded[j] = true;

      delay(100);   // let notch settle before next FFT read
    }

    if (foundCount < TARGET_PEAKS) inputLevel++;
  }

  // Restore original gain
  inputLevel = savedLevel;
  sgtl5000.lineInLevel(inputLevel);

  // Report
  Serial.println();
  if (foundCount == 0) {
    Serial.println(F("  [AUTOCUT] No feedback peaks detected."));
    Serial.println(F("  [AUTOCUT] Try increasing input level first (lineInLevel)."));
  } else {
    Serial.print(F("  [AUTOCUT] Done — "));
    Serial.print(foundCount);
    Serial.println(F(" notch(es) applied:"));
    for (int i = 0; i < foundCount; i++) {
      Serial.print(F("    f")); Serial.print(i + 1);
      Serial.print(F("freq = ")); Serial.print((int)notch[i].freq);
      Serial.print(F(" Hz  Q = ")); Serial.println(notch[i].q, 1);
    }
    if (foundCount < TARGET_PEAKS) {
      Serial.println(F("  [AUTOCUT] Fewer than 4 peaks found — remaining notches unchanged."));
    }
  }
  Serial.println();
}

// ============================================================
//  CLI — parameter accessors by name
// ============================================================

// Returns true if name matches a known parameter, fills type/index
// type: 0=freq, 1=q, 2=delay
bool parsePotName(const char* name, uint8_t* type, uint8_t* band) {
  for (uint8_t i = 0; i < 4; i++) {
    char freqName[8], qName[8];
    snprintf(freqName, sizeof(freqName), "f%dfreq", i + 1);
    snprintf(qName,    sizeof(qName),    "f%dq",    i + 1);
    if (strcasecmp(name, freqName) == 0) { *type = 0; *band = i; return true; }
    if (strcasecmp(name, qName)    == 0) { *type = 1; *band = i; return true; }
  }
  if (strcasecmp(name, "dlt") == 0) { *type = 2; *band = 0; return true; }
  return false;
}

void printParam(uint8_t type, uint8_t band) {
  if (type == 0) {
    Serial.print(F("  f")); Serial.print(band + 1);
    Serial.print(F("freq = ")); Serial.print((int)notch[band].freq);
    Serial.println(F(" Hz"));
  } else if (type == 1) {
    Serial.print(F("  f")); Serial.print(band + 1);
    Serial.print(F("q = ")); Serial.println(notch[band].q, 2);
  } else {
    Serial.print(F("  dlt = ")); Serial.print((int)delayMs);
    Serial.println(F(" ms"));
  }
}

void setParam(uint8_t type, uint8_t band, float val) {
  if (type == 0) {
    if (val < 20 || val > 20000) {
      Serial.println(F("  [ERR] freq must be 20–20000 Hz")); return;
    }
    notch[band].freq = val;
    applyNotch(band);
    Serial.print(F("  OK  f")); Serial.print(band + 1);
    Serial.print(F("freq <- ")); Serial.print((int)val); Serial.println(F(" Hz"));
  } else if (type == 1) {
    if (val < 0.1f || val > 20.0f) {
      Serial.println(F("  [ERR] Q must be 0.1–20.0")); return;
    }
    notch[band].q = val;
    applyNotch(band);
    Serial.print(F("  OK  f")); Serial.print(band + 1);
    Serial.print(F("q <- ")); Serial.println(val, 2);
  } else {
    if (val < 10 || val > 1000) {
      Serial.println(F("  [ERR] delay must be 10–1000 ms")); return;
    }
    delayMs = val;
    applyDelay();
    Serial.print(F("  OK  dlt <- ")); Serial.print((int)val); Serial.println(F(" ms"));
  }
}

// ============================================================
//  Command dispatcher
// ============================================================
void processCommand(const char* raw) {
  while (*raw == ' ') raw++;
  if (strlen(raw) == 0) return;

  char buf[BUF_SIZE];
  strncpy(buf, raw, BUF_SIZE - 1);
  buf[BUF_SIZE - 1] = '\0';

  // First token = name or verb
  char* token = strtok(buf, " ");
  if (!token) return;

  // --- Built-in verbs
  if      (strcasecmp(token, "help")    == 0) { cmdHelp(); return; }
  else if (strcasecmp(token, "list")    == 0) { cmdList(); return; }
  else if (strcasecmp(token, "autocut") == 0) { cmdAutocut(); return; }

  // --- Parameter shortcut: name [value]
  uint8_t type, band;
  if (parsePotName(token, &type, &band)) {
    char* valueStr = strtok(NULL, " ");
    if (!valueStr) {
      // No value → GET
      printParam(type, band);
    } else {
      // Value present → SET
      float val = atof(valueStr);
      setParam(type, band, val);
    }
    return;
  }

  Serial.print(F("  [ERR] Unknown: ")); Serial.println(token);
  Serial.println(F("  Type 'help'"));
}

// ============================================================
void cmdList() {
  Serial.println(F("\n  Notch filters:"));
  Serial.println(F("  +--------+-----------+-------+"));
  Serial.println(F("  | Band   | Freq (Hz) |   Q   |"));
  Serial.println(F("  +--------+-----------+-------+"));
  for (uint8_t i = 0; i < 4; i++) {
    char row[48];
    snprintf(row, sizeof(row), "  | f%d      | %9d | %5.2f |",
             i + 1, (int)notch[i].freq, notch[i].q);
    Serial.println(row);
  }
  Serial.println(F("  +--------+-----------+-------+"));
  Serial.print(F("\n  Delay tap interval : ")); Serial.print((int)delayMs); Serial.println(F(" ms"));
  Serial.print(F("  Tap times          : 0 / "));
  Serial.print((int)delayMs); Serial.print(F(" / "));
  Serial.print((int)(delayMs * 2)); Serial.print(F(" / "));
  Serial.print((int)(delayMs * 3)); Serial.println(F(" ms"));
  Serial.print(F("  lineInLevel        : ")); Serial.println(inputLevel);
  Serial.println();
}

void cmdHelp() {
  Serial.println(F("\n  Commands:"));
  Serial.println(F("    <name>            — get current value"));
  Serial.println(F("    <name> <value>    — set value (real units)"));
  Serial.println(F("    list              — show all params"));
  Serial.println(F("    autocut           — auto feedback calibration"));
  Serial.println(F("    help              — this message"));
  Serial.println(F("\n  Parameters:"));
  Serial.println(F("    f1freq … f4freq   — notch frequency  20–20000 Hz"));
  Serial.println(F("    f1q    … f4q      — notch Q          0.1–20.0"));
  Serial.println(F("    dlt               — delay tap        10–1000 ms"));
  Serial.println();
}

// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  AudioMemory(1024);

  sgtl5000.enable();
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000.volume(0.8);
  sgtl5000.lineInLevel(inputLevel);
  sgtl5000.lineOutLevel(5);

  applyAllNotches();
  applyDelay();

  printBanner();
  printPrompt();
}

// ============================================================
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    Serial.print(c);

    if (c == '\r') continue;

    if (c == '\n') {
      inputBuf[bufIdx] = '\0';
      Serial.println();
      processCommand(inputBuf);
      bufIdx = 0;
      printPrompt();
    } else if (c == '\b' || c == 127) {
      if (bufIdx > 0) { bufIdx--; Serial.print(F(" \b")); }
    } else if (bufIdx < BUF_SIZE - 1) {
      inputBuf[bufIdx++] = c;
    }
  }
}

// ============================================================
void printBanner() {
  Serial.println(F("\r\n================================"));
  Serial.println(F("  4-band Notch + Delay + Autocut"));
  Serial.println(F("  f1freq 800  →  set to 800 Hz"));
  Serial.println(F("  f1freq      →  read value"));
  Serial.println(F("  autocut     →  calibrate"));
  Serial.println(F("================================"));
}

void printPrompt() { Serial.print(F("> ")); }
