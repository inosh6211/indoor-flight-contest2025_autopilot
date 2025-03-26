const int led_pin = 13;//LEDを接続するピン
const int vol_pin = 0;//圧力センサ用アナログピン

void setup() {
// シリアルモニターに映す為。
  Serial.begin(9600);
  //led_pinを出力ピンに設定
  pinMode(led_pin,OUTPUT);
}
// void loopの{}で囲われた箇所は、電源オフまたはリセットボタンを押さない限り永久に繰り返されます
void loop() {
// アナログピンの入力値を読み込み。
  int sensorValue = analogRead(vol_pin);
  // 読み込んだ状態をシリアルモニターに表示する文。
  Serial.println(sensorValue);
  //見やすくするため少し遅延。
  delay(100);
 
  //センサーの値が500未満になったらLED点灯。そうでなければ消灯。
  //センサーによって感度が違うので500という数字はお好みで。
  if(sensorValue < 500){
    digitalWrite(led_pin,HIGH);
  }
  else{
    digitalWrite(led_pin,LOW);
    }
}
