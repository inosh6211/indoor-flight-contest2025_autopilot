const int fsrPin = A0; // FSR 406 の接続ピン
const float R1 = 10000.0; // 分圧用の固定抵抗（10kΩ）
const float Vcc = 5.0; // 電源電圧（5V）

void setup() {
  Serial.begin(9600);
}

void loop() {
  int adcValue = analogRead(fsrPin);
  float Vout = (adcValue / 1023.0) * Vcc; // ADC値を電圧に変換
  float Rfsr = (Vout == 0) ? 1e6 : (R1 * (Vcc - Vout) / Vout); // FSRの抵抗値を計算

  // 抵抗値を力（N）に変換
  float force = 2.482e6 * pow(Rfsr, -1.4);

  Serial.print("ADC: "); Serial.print(adcValue);
  Serial.print(", Vout: "); Serial.print(Vout, 3);
  Serial.print(", R_FSR: "); Serial.print(Rfsr, 3);
  Serial.print(", Force: "); Serial.print(force, 3);
  Serial.println(" N");

  delay(500);
}
