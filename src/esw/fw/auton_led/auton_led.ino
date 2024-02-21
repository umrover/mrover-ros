int red_led = 12;
int green_led = 10;
int blue_led = 11;

void led_control(char led_ID) {
    if (led_ID == 'r') {
        digitalWrite(red_led, LOW);
        digitalWrite(green_led, HIGH);
        digitalWrite(blue_led, HIGH);
    } else if (led_ID == 'g') {
        digitalWrite(green_led, LOW);
        digitalWrite(blue_led, HIGH);
        digitalWrite(red_led, HIGH);
    }
    else if (led_ID == 'b') {
        digitalWrite(blue_led, LOW);
        digitalWrite(red_led, HIGH);
        digitalWrite(green_led, HIGH);
    }
    else if (led_ID == 'o') {
        digitalWrite(blue_led, HIGH);
        digitalWrite(red_led, HIGH);
        digitalWrite(green_led, HIGH);
    }
    else if (led_ID == 'w') {
        digitalWrite(blue_led, LOW);
        digitalWrite(red_led, LOW);
        digitalWrite(green_led, LOW);
    }
    else if (led_ID == "n") {
        digitalWrite(blue_led, HIGH);
        digitalWrite(red_led, LOW);
        digitalWrite(green_led, LOW);
    }
    else if (led_ID == "p") {
        digitalWrite(blue_led, LOW);
        digitalWrite(red_led, LOW);
        digitalWrite(green_led, HIGH);
    }
    else if (led_ID == "p") {
        digitalWrite(blue_led, LOW);
        digitalWrite(red_led, HIGH);
        digitalWrite(green_led, LOW);
    }
}

void setup() {
    // put your setup code here, to run once:
    pinMode(red_led, OUTPUT);   // red led
    pinMode(green_led, OUTPUT); // green led
    pinMode(blue_led, OUTPUT);  // blue led
    digitalWrite(blue_led, LOW);
    digitalWrite(red_led, LOW);
    digitalWrite(green_led, LOW);
    Serial.begin(115200);
}

void loop() {

    if (Serial.available()) {
        char input = Serial.read();
        led_control(input);
    }
}