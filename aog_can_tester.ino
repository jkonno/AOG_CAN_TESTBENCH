// Small program for testing CAN based AOG setup
 
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

void readJ1939WAS(CAN_message_t &msg);
void readSASAIID(CAN_message_t &msg);
void readFromBus(CAN_message_t &msg);
void writeToBus(CAN_message_t &msg);
  
CAN_message_t flowCommand;
CAN_message_t wasMessage;

int16_t steeringPosition = 0; //from steering sensor
  
// Define WAS, flow, button message ID to match from bus structure
// This stuff works for following config: 
// Danfoss DST X510 WAS
// Danfoss PVEA-CI CANBUS valve (or any other CAN valve using standard ISOBUS flow commands)
// Danfoss SASA IID sensor
uint32_t wasMessageID = 0x018FF0B15;
uint32_t steerSwID = 0x018EFFF21;
uint32_t pressureMessageID = 0x018FF034E;
uint32_t SASAIIDMessageID = 0x0CFF104D;
uint32_t flowCommandID = 0x0CFE3022;
uint16_t oldSteerAngle = 0;
// CAN ends here

#include <SPI.h>
//#include <ST7735_t3.h> // Hardware-specific library
#include <ST7789_t3.h> // Hardware-specific library
//#include <ST7735_t3_font_Arial.h>
//#include <ST7735_t3_font_ArialBold.h>

#define TFT_RST    32   // chip reset
#define TFT_DC     9   // tells the display if you're sending data (D) or commands (C)   --> WR pin on TFT
#define TFT_MOSI   11  // Data out    (SPI standard)
#define TFT_SCLK   13  // Clock out   (SPI standard)
#define TFT_CS     10  // chip select (SPI standard)

int LCD_BL = 33;       // LCD back light control

ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
IntervalTimer timer;
uint8_t d=0;
bool stopfd = 0;
int askel = 0;

void canSniff20(const CAN_message_t &msg) { // global callback
  Serial.print("T4: ");
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print(" OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print(" BUS "); Serial.print(msg.bus);
  Serial.print(" LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" REMOTE: "); Serial.print(msg.flags.remote);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" IDHIT: "); Serial.print(msg.idhit);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}



void setup(void) {
  
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);  // Turn LCD backlight on
 
  Serial.begin(115200); delay(1000);
  Serial.println("Teensy 4.0 CAN FD test. www.skpang.co.uk Jan 2020");

  can3.begin();
  can3.setBaudRate(250000);
  can3.setMaxMB(16);
  can3.enableFIFO();
  can3.enableFIFOInterrupt();
  can3.onReceive(readFromBus);
  can3.mailboxStatus();
      
  Serial.println("CAN BUS Init OK!");
  
  tft.init(240, 240);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLUE);

  tft.setCursor(5,5);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLUE);
  tft.setTextSize(2);
  tft.println("AOG valve test");

  tft.setCursor(10,40);
  tft.println("Valve cmd: ");
  
  tft.setCursor(10 ,70);
  tft.println("SASA angle: ");

  tft.setCursor(10 ,100);
  tft.println("SASA speed: ");

  tft.setCursor(10 ,130);
  tft.println("WAS angle: ");
  
  tft.setCursor(10,212);
  tft.println("www.konno.fi");   
}

void loop() {

  //delay(50);

  //askel++;
  //  tft.setCursor(160 , 40);
  //tft.println(askel);
}

void readFromBus(CAN_message_t &msg){
  // WAS message arrives
  if (msg.id == wasMessageID) {
    readJ1939WAS(msg);
  } 
  // SASAIID sensor message arrives
  if (msg.id == SASAIIDMessageID) {
    readSASAIID(msg);
  }
  // Valve command message arrives
  if (msg.id == flowCommandID) {
    readFlowCmd(msg);
  }
}

void readJ1939WAS(CAN_message_t &msg)
{
    // Check for correct id
    if (msg.id & wasMessageID == wasMessageID){ 
      uint16_t temp = 0;
      temp = (uint16_t) msg.buf[0] << 8;
      temp |= (uint16_t) msg.buf[1];
      steeringPosition = (int)temp;
      // WAS angle
      tft.setCursor(160 ,130);
      tft.println(steeringPosition/10.0);
  }
}


void readSASAIID(CAN_message_t &msg) {
  // Check steering angle
  uint16_t tempAngle = 0;
  uint16_t deltaAngle = 0;
  tempAngle = (uint16_t) msg.buf[1] << 8;
  tempAngle |= (uint16_t) msg.buf[0];
  // Check steering angle velocity
  uint16_t tempVelocity = 0;
  tempVelocity = (uint16_t) msg.buf[3] << 8;
  tempVelocity |= (uint16_t) msg.buf[2];  
  // Steer velocity in dRPM (20480 is 300 RPM = 3000 dRPM, PVED-CLS limit is 5 dRPM)
  int16_t steerSpeed = abs((tempVelocity - 20480)*0.146484375);
  // Steering stationary below 5 dRPM
  if (steerSpeed < 5) {
    // Set the stationary angle point
    oldSteerAngle = tempAngle;
  } else {
    deltaAngle = abs(tempAngle-oldSteerAngle);
    if (deltaAngle > 2048) {
      deltaAngle = 4096 - deltaAngle; // Handle the 0-360 jump
    }
  }
  // SASA angle
  tft.setCursor(160 ,70);
  tft.println(deltaAngle/4096.0*360);
  // SASA speed
  tft.setCursor(160 ,100);
  tft.println(steerSpeed/10.0);
}

void readFlowCmd(CAN_message_t &msg)
{
  // Check direction
  if (msg.buf[2] == 0x01) {
    // RIGHT
    tft.fillTriangle(170, 160, 170, 200, 210, 180, ST77XX_RED);
    tft.fillTriangle(70, 160, 70, 200, 30, 180, ST77XX_BLUE);
    tft.fillRoundRect(100, 160, 40, 40, 5, ST77XX_BLUE);
  } else if (msg.buf[2] == 0x02) {
    // LEFT
    tft.fillTriangle(170, 160, 170, 200, 210, 180, ST77XX_BLUE);
    tft.fillTriangle(70, 160, 70, 200, 30, 180, ST77XX_GREEN);
    tft.fillRoundRect(100, 160, 40, 40, 5, ST77XX_BLUE);
  } else if (msg.buf[2] == 0x00) {
    // OFF
    tft.fillTriangle(170, 160, 170, 200, 210, 180, ST77XX_BLUE);
    tft.fillTriangle(70, 160, 70, 200, 30, 180, ST77XX_BLUE);
    tft.fillRoundRect(100, 160, 40, 40, 5, ST77XX_BLACK);    
  }
  
  uint8_t flowValue = 0;
  flowValue = msg.buf[0];
  tft.setCursor(160 , 40);
  tft.println(flowValue);
}