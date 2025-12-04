/* 
  Controls a 2-DOF robotic ankle (sagittal + transverse) using two hobby servos.
  Produces smooth, human-like gait-ish motion inspired by the project's motion ranges:
   - Sagittal (dorsiflexion/plantarflexion): ~ ±45° (tweakable)
   - Transverse (inversion/eversion): ~ ±15° (tweakable)
  Based on the mechanical design & ranges in the project report. :contentReference[oaicite:1]{index=1}
*/

#include <Servo.h>

// -------------------- Hardware pins --------------------
const uint8_t PIN_SAGITTAL    = 3;   // servo driving sagittal axis (dorsiflexion/plantarflexion)
const uint8_t PIN_TRANSVERSE  = 5;   // servo driving transverse axis (inversion/eversion)

Servo servoSag, servoTrans;

// -------------------- Calibration & limits --------------------
// Offsets (servo center positions for neutral pose). Tune on your hardware.
int offsetSag = 90;      // neutral servo angle (degrees)
int offsetTrans = 90;

// Inversion flags (1 or -1) if servo mounting is flipped
int invSag = 1;
int invTrans = 1;

// Safety servo limits (degrees)
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;

int clampServo(int v){
  if (v < SERVO_MIN) return SERVO_MIN;
  if (v > SERVO_MAX) return SERVO_MAX;
  return v;
}

// -------------------- Motion parameters (tweakable) --------------------
float sagittalAmplitudeDeg = 35.0;    // peak from neutral (start conservative; can go up to ~45)
float transverseAmplitudeDeg = 12.0;  // peak from neutral (start conservative; can go up to ~15)

float gaitCycleSec = 1.0;   // duration of one gait cycle in seconds (0.6 - 1.2 typical walking)
unsigned long lastMillis = 0;

// human-like micro-correction parameters
float microCorrectionMagnitude = 1.2; // degrees
unsigned long microCorrectionInterval = 700; // ms
unsigned long lastMicro = 0;
float microSag = 0.0;
float microTrans = 0.0;

// phase shift: transverse slightly lags sagittal for natural motion
float phaseShift = 0.35; // fraction of cycle (0..1). 0.25-0.5 gives a natural lag

// control flags
bool runGait = true;

// interpolation (smoothing) - number of steps for move interpolation
const uint8_t INTERP_STEPS = 12;

// -------------------- Utility helpers --------------------
float clampf(float x, float a, float b){ return (x < a) ? a : (x > b) ? b : x; }
float degToServo(int offset, int inversion, float deg){ // map joint deg to servo command
  int s = round(offset + inversion * deg);
  return (float)clampServo(s);
}

// -------------------- Setup / Loop --------------------
void setup(){
  Serial.begin(115200);
  servoSag.attach(PIN_SAGITTAL);
  servoTrans.attach(PIN_TRANSVERSE);

  // go to neutral
  servoSag.write(offsetSag);
  servoTrans.write(offsetTrans);

  lastMillis = millis();
  lastMicro = millis();

  Serial.println("Ankle2DOF ready. Commands: 'p' pause/resume, 'fNN' set gait freq (Hz), 'aNN' set sagittal amp, 'tNN' set transverse amp");
}

void loop(){
  // handle serial commands without blocking
  handleSerial();

  if (!runGait) {
    delay(10);
    return;
  }

  unsigned long now = millis();
  float dt = (now - lastMillis) / 1000.0;
  lastMillis = now;

  // calculate phase (0..1) for gait cycle
  static float phase = 0.0;
  float cycleSec = gaitCycleSec;
  phase += dt / cycleSec;
  if (phase >= 1.0) phase -= 1.0;

  // basic sinusoidal human-like pattern
  // sagittal: primary walking waveform (sin)
  float sagittalJointDeg = sagittalAmplitudeDeg * sinf(2.0 * PI * phase); // -A..+A

  // transverse: smaller amplitude, phase-shifted to lag/lead
  float transversePhase = phase + phaseShift;
  if (transversePhase >= 1.0) transversePhase -= 1.0;
  float transverseJointDeg = transverseAmplitudeDeg * sinf(2.0 * PI * transversePhase);

  // micro-corrections every microCorrectionInterval ms to break perfect periodicity (more natural)
  if (now - lastMicro >= microCorrectionInterval){
    lastMicro = now;
    microSag = ((float)random(-100,100)/100.0f) * microCorrectionMagnitude;   // ±mag
    microTrans = ((float)random(-100,100)/100.0f) * (microCorrectionMagnitude * 0.6f);
  }
  // apply a small low-pass to micro corrections so they are smooth (simple linear decay)
  microSag *= 0.98;
  microTrans *= 0.98;

  sagittalJointDeg += microSag;
  transverseJointDeg += microTrans;

  // safety clamp to documented ranges (based on project): sagittal approx ±45, transverse approx ±15
  sagittalJointDeg = clampf(sagittalJointDeg, -50.0, 50.0);   // small margin
  transverseJointDeg = clampf(transverseJointDeg, -20.0, 20.0);

  // Convert to servo angles
  int servoCmdSag = (int)round(degToServo(offsetSag, invSag, sagittalJointDeg));
  int servoCmdTrans = (int)round(degToServo(offsetTrans, invTrans, transverseJointDeg));

  // Smooth interpolation: read current commanded by previous writes isn't easily readable from hobby servos,
  // so we simply step toward target for nicer motion
  static int lastCmdSag = offsetSag, lastCmdTrans = offsetTrans;
  for (uint8_t step = 1; step <= INTERP_STEPS; ++step){
    float t = (float)step / INTERP_STEPS;
    int sSag = round(lastCmdSag + t * (servoCmdSag - lastCmdSag));
    int sTrans = round(lastCmdTrans + t * (servoCmdTrans - lastCmdTrans));
    servoSag.write(sSag);
    servoTrans.write(sTrans);
    delay( (uint16_t)( (gaitCycleSec*1000.0) / (INTERP_STEPS*6.0) ) ); // small delay tuned for smoothness
  }
  lastCmdSag = servoCmdSag;
  lastCmdTrans = servoCmdTrans;
}

// -------------------- Serial control --------------------
void handleSerial(){
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  if (s == "p"){
    runGait = !runGait;
    Serial.print("runGait=");
    Serial.println(runGait ? "ON" : "OFF");
    return;
  }
  // set gait frequency in Hz, e.g. "f1.2"
  if (s.charAt(0) == 'f'){
    float hz = s.substring(1).toFloat();
    if (hz > 0.05){
      gaitCycleSec = 1.0 / hz;
      Serial.print("gait freq set to ");
      Serial.print(hz);
      Serial.print(" Hz (cycle ");
      Serial.print(gaitCycleSec,3);
      Serial.println(" s)");
    }
    return;
  }
  // set sagittal amplitude deg e.g. "a40"
  if (s.charAt(0) == 'a'){
    float a = s.substring(1).toFloat();
    sagittalAmplitudeDeg = clampf(a, 0.0, 50.0);
    Serial.print("sagittal amplitude set to ");
    Serial.println(sagittalAmplitudeDeg);
    return;
  }
  // set transverse amplitude e.g. "t12"
  if (s.charAt(0) == 't'){
    float a = s.substring(1).toFloat();
    transverseAmplitudeDeg = clampf(a, 0.0, 25.0);
    Serial.print("transverse amplitude set to ");
    Serial.println(transverseAmplitudeDeg);
    return;
  }

  Serial.println("Unknown cmd. Use 'p' pause/resume, 'fNN' freqHz, 'aNN' sagittal amp, 'tNN' transverse amp");
}
