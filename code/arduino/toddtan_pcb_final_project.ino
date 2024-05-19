#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTTStream.h>
RTTStream rtt;


//======/WiFi Logic=============================
const int RX = 0;
const int TX = 1;
//=================================================
//======/OLED Logic =============================
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SCL 21
#define SDA 20
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define NUMFLAKES     10 // Number of snowflakes in the animation example
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };
//=================================================
//======/Buzzer Logic =============================
const int buzzer = 4;
int buzzer_is_high = false;
unsigned long buzzer_last_time = millis();
unsigned long buzzer_current_time = millis();
//=================================================
//======/LEDs Logic =============================
const int led1 = 18;
const int led2 = 8;
int led_state = 0;
int led_is_high = false;
unsigned long led_last_time = millis();
unsigned long led_current_time = millis();
//=================================================
//======/Vibrator Logic =============================
const int vibrator = 7;
int should_buzz = true;
int vibrator_is_high = false;
unsigned long vibrator_last_time = millis();
unsigned long vibrator_current_time = millis();
//======/Thermistor Logic =========================
const int thermistor = 9;
float R1 = 2000;
int Vo;
float logR2, R2, T;
float T0 = 298.15; //Reference temperature in Kelvin
float B = 3420;
float c1 = 1.0/T0;
float c2 = 1.0/B;
float c3 = 0;
//=================================================
//======/Force Sensor Logic =========================
const int force_sensor = 15;
int force_current_time = millis();
int force_last_time = millis();
int prev_force_val = 0;
int current_force_val = 0;
bool has_tap = false;
bool has_release = false;
//=================================================
//======/Microphone Logic =========================
const int microphone = 19;
//=================================================
// the setup function runs once when you press reset or power the board
void setup() {
  //rtt.println("HCI PCB");
  SerialUSB.begin(9600);
  delay(1000);
  SerialUSB.println("Begin Setup");
  Serial1.begin(115200);
  delay(1000);
  pinMode(buzzer, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(vibrator, OUTPUT);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    SerialUSB.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2000);
  display.clearDisplay();
  display.drawPixel(10, 10, SSD1306_WHITE);
  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
  draw_normal_emotion();
  
  //===========================================================
  //rtt.write("Finished Setup\n");
}

// the loop function runs over and over again forever
void loop() {
  //test_leds();
  //test_wifi();
  
  //test_microphone();
  //test_thermistor();
  
  if(should_buzz == true){//idle time
    buzz();
  }
  show_led_state(led_state);
  get_force_signal();
  force_trigger_vibrator_and_oled();
  
}

void print_num(float num_float){
  int num = (int) num_float;
  int num_arr[5] = {0,0,0,0,0};
  int i = 4;
  while(num > 0){
    if(i == -1){
      rtt.write("WARNING, INTEGER GREATER THAN 5 DIGITS!\n");
      break;
    }
    int remainder = num % 10;
    num_arr[i] = remainder; 
    num = num / 10;
    i--;
  }
  for(int j = 0; j < 5; j++){
    int digit = num_arr[j];
    if(digit == 0){rtt.write("0");}
    else if(digit == 1){rtt.write("1");}
    else if(digit == 2){rtt.write("2");}
    else if(digit == 3){rtt.write("3");}
    else if(digit == 4){rtt.write("4");}
    else if(digit == 5){rtt.write("5");}
    else if(digit == 6){rtt.write("6");}
    else if(digit == 7){rtt.write("7");}
    else if(digit == 8){rtt.write("8");}
    else if(digit == 9){rtt.write("9");}
  }
  rtt.write("\n");
}

void show_led_state(int state){
  if(state == 0){
    blink_leds();
  }else if(state == 1){
    digitalWrite(led1, HIGH);
    digitalWrite(led2, LOW);
  }else if(state == 2){
    digitalWrite(led1, LOW);
    digitalWrite(led2, HIGH);
  }
}

void get_force_signal(){
  //no signal(pressure sensor not pressed)
  // No Signal: No force applied at all, no signal means nothing to do; continue the previous action
  // Single Tap: Force applied only once, if after 4 seconds there's not a second tap, perform action #1
  // Double Tap: Force applied twice, within 4 seconds there are 2 taps(High - Low - Hight)
  // Long Press: Force applied only once, but within 4 seconds, analogRead always High
  // Timing starts when the first tap occurs
  force_current_time = millis();
  current_force_val = analogRead(force_sensor);
  if(has_tap == false && current_force_val <= 200){
    //rtt.write("{\"force_signal\": \"NO_SIGNAL\"}");
  }
  //singal tap
  else if(has_tap == false && current_force_val > 200){
    has_tap = true;
    //start the 4 second window
    force_last_time = millis();
    prev_force_val = current_force_val;
  }
  else if(has_tap == true){
    if(force_current_time - force_last_time >= 2000){//check if 2s timeframe has passed
      if(has_release == true){//single tap
        rtt.write("{\"force_signal\": \"SINGLE_TAP\"}\n");
        led_state = 1;
      }else{//long press
        rtt.write("{\"force_signal\": \"LONG_PRESS\"}\n");
        led_state = 0;
        delay(2000);
      }
      //reset
      has_tap = false;
      has_release = false;
      prev_force_val = 0;
      force_last_time = millis();
    }
    else if(current_force_val <= 200){
      has_release = true;
    }
    else if(current_force_val > 200 && has_release == true){
      rtt.write("{\"force_signal\": \"DOUBLE_TAP\"}\n");
      led_state = 2;
      //reset
      has_tap = false;
      has_release = false;
      prev_force_val = 0;
      force_last_time = millis();
    }
  }
  
}
void force_trigger_vibrator_and_oled(){
  int sense_force = analogRead(force_sensor);
  //rtt.write("Force sensor reading: ");
  //print_num(float(sense_force));
  //rtt.write("\n");
  if(sense_force > 80){
    should_buzz = false;
    //rtt.write("turning on vibrator\n");
    digitalWrite(vibrator, HIGH);
    draw_pressure_emotion();
  }else{
    should_buzz=true;
    digitalWrite(vibrator, LOW);
    draw_normal_emotion();
  }
}

void test_oled(){
  testdrawline();      // Draw many lines
  /*
  testdrawstyles();    // Draw 'stylized' characters
  testscrolltext();    // Draw scrolling text
  testdrawrect();      // Draw rectangles (outlines)
  testfillrect();      // Draw rectangles (filled)
  testdrawcircle();    // Draw circles (outlines)
  testfillcircle();    // Draw circles (filled)
  testdrawroundrect(); // Draw rounded rectangles (outlines)
  testfillroundrect(); // Draw rounded rectangles (filled)
  testdrawtriangle();  // Draw triangles (outlines)
  testfilltriangle();  // Draw triangles (filled)
  testdrawchar();      // Draw characters of the default font
  testdrawstyles();    // Draw 'stylized' characters
  testdrawbitmap();    // Draw a small bitmap image
  */
  // Invert and restore display, pausing in-between
  
  display.invertDisplay(true);
  delay(1000);
  display.invertDisplay(false);
  delay(1000);
}
void test_wifi(){
  rtt.write("enter test_wifi()\n");
  Serial1.print("AT+CWMODE=3\r\n");
  while(Serial1.available()){
    byte inByte = Serial1.read();
    rtt.write(inByte);
  }
  delay(1000);
  Serial1.print("AT+CIPMUX=1\r\n");
  delay(1000);
  Serial1.print("AT+CWJAP=\"intro-robo\",\"turtlebot\"\r\n"); 
  delay(3000);
  rtt.write("Initial AT commands sent\n");
  Serial1.print("AT+CIPSTART=0,\"TCP\",\"35.94.176.147\",5050\r\n");
  rtt.write("CIPSTART\n");
  delay(1000);
  //HTTP/1.1\r\nHost: + domain \r\n Connection: close
  Serial1.print("AT+CIPSEND=0,30\r\n");
  rtt.write("AT+CIPSEND\n");
  delay(1000);
  Serial1.print("GET /data?data1=1 HTTP/1.1\r\n");
  rtt.write("GET\n");
  delay(1000);
  Serial1.print("AT+CIPCLOSE=0\r\n");
  Serial1.print("Close\n");
  delay(1000);
}
void test_microphone(){
  int val = analogRead(microphone);
  rtt.write("microphone val = ");
  print_num(float(val));
  rtt.write("\n");
}
void test_force_sensor(){
  int sense_force = analogRead(force_sensor);
  rtt.write("Force sensor reading: ");
  print_num(float(sense_force));
  rtt.write("\n");
  delay(200);
}

void buzz(){
  buzzer_current_time = millis();
  if(buzzer_current_time - buzzer_last_time > 50){
    if(buzzer_is_high == false){
      digitalWrite(buzzer,HIGH);
    }else{
      digitalWrite(buzzer, LOW);
    }
    buzzer_is_high = !buzzer_is_high;
    buzzer_last_time = buzzer_current_time;
  }
}

void test_thermistor(){
  int Vo = analogRead(thermistor);
  R2 = R1 * (1023.0 / (float) Vo - 1.0);
  logR2 = log(R2);
  T = (1.0/(c1 + c2*logR2 + c3*logR2*logR2*logR2));
  float Tc = T - 273.15;
  rtt.write("Temperature: "); 
  print_num(Tc);
  rtt.write(" Celcius\n");
}

void test_vibrator(){
  vibrator_current_time = millis();
  if(vibrator_current_time - vibrator_last_time > 500){
    if(vibrator_is_high == false){
      digitalWrite(vibrator, HIGH);
    }else{
      digitalWrite(vibrator,LOW);
    }
    vibrator_is_high = !vibrator_is_high;
    vibrator_last_time = vibrator_current_time;
  }
}
void blink_leds(){
  led_current_time = millis();
  if(led_current_time - led_last_time > 500){
    if(led_is_high == false){
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
    }else{
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
    }
    led_is_high = !led_is_high;
    led_last_time = led_current_time;
  }
}
//====================FROM OLED Sample Code
void testdrawline() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000); // Pause for 2 seconds
}

void testdrawrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testfillrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=3) {
    // The INVERSE color is used so rectangles alternate white/black
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testdrawcircle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=2) {
    display.drawCircle(display.width()/2, display.height()/2, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillcircle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=3) {
    // The INVERSE color is used so circles alternate white/black
    display.fillCircle(display.width() / 2, display.height() / 2, i, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn circle
    delay(1);
  }

  delay(2000);
}

void testdrawroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    // The INVERSE color is used so round-rects alternate white/black
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawtriangle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=5) {
    display.drawTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfilltriangle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=5) {
    // The INVERSE color is used so triangles alternate white/black
    display.fillTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}


void draw_normal_emotion(void) {
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(38,7);             // Start at top-left corner
  display.println(F("O V O"));
  display.display();
  delay(200);
}

void draw_pressure_emotion(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(13,7);
  display.println(F("~ O W O ~"));
  display.display();      // Show initial text
  delay(100);
  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(100);
  display.stopscroll();
  delay(100);
  display.startscrollleft(0x00, 0x0F);
  delay(100);
  display.stopscroll();
  delay(100);
}

void testdrawbitmap(void) {
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(1000);
}

#define XPOS   0 // Indexes into the 'icons' array in function below
#define YPOS   1
#define DELTAY 2

void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h) {
  int8_t f, icons[NUMFLAKES][3];

  // Initialize 'snowflake' positions
  for(f=0; f< NUMFLAKES; f++) {
    icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
    icons[f][YPOS]   = -LOGO_HEIGHT;
    icons[f][DELTAY] = random(1, 6);
    Serial.print(F("x: "));
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(F(" y: "));
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(F(" dy: "));
    Serial.println(icons[f][DELTAY], DEC);
  }

  for(;;) { // Loop forever...
    display.clearDisplay(); // Clear the display buffer

    // Draw each snowflake:
    for(f=0; f< NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, SSD1306_WHITE);
    }

    display.display(); // Show the display buffer on the screen
    delay(200);        // Pause for 1/10 second

    // Then update coordinates of each flake...
    for(f=0; f< NUMFLAKES; f++) {
      icons[f][YPOS] += icons[f][DELTAY];
      // If snowflake is off the bottom of the screen...
      if (icons[f][YPOS] >= display.height()) {
        // Reinitialize to a random position, just off the top
        icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
        icons[f][YPOS]   = -LOGO_HEIGHT;
        icons[f][DELTAY] = random(1, 6);
      }
    }
  }
}
