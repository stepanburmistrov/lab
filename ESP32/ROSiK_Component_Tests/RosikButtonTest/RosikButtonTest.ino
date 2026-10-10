const uint8_t BUTTON_PIN = 25;
bool lastState = 1;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Start");
}

void loop() {
  bool state = digitalRead(BUTTON_PIN);
  
  if (state != lastState) {
    Serial.print("Кнопка: ");
    Serial.println(state == LOW ? "Pressed" : "Released");
    lastState = state;
  }

  delay(10);
}