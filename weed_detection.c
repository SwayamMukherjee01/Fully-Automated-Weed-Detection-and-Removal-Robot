/*
 * OV7670 Basic Weed Detection for Arduino Mega
 * FULLY CORRECTED VERSION
 * 
 * IMPORTANT NOTES:
 * - Simple color-based detection (NOT true AI/ML)
 * - Low resolution due to memory constraints
 * - REQUIRES Arduino Mega (8KB RAM minimum)
 * - Serial Monitor MUST be set to 9600 baud
 * 
 * Hardware Connections:
 * OV7670 Module -> Arduino Mega
 * ================================
 * SIOC (SCL)  -> A5 (SCL) - USE 3.3V LEVEL SHIFTER!
 * SIOD (SDA)  -> A4 (SDA) - USE 3.3V LEVEL SHIFTER!
 * VSYNC       -> Pin 2
 * HREF        -> Pin 3
 * PCLK        -> Pin 21
 * XCLK        -> Pin 11 (PWM)
 * D7          -> Pin 22
 * D6          -> Pin 23
 * D5          -> Pin 24
 * D4          -> Pin 25
 * D3          -> Pin 26
 * D2          -> Pin 27
 * D1          -> Pin 28
 * D0          -> Pin 29
 * RESET       -> 3.3V
 * PWDN        -> GND
 * 3.3V        -> 3.3V (IMPORTANT!)
 * GND         -> GND
 */

#include <Wire.h>

// Camera I2C address
#define OV7670_I2C_ADDR 0x21

// Pin definitions
#define PIN_VSYNC 2
#define PIN_HREF 3
#define PIN_PCLK 21
#define PIN_XCLK 11

// Data pins D7 to D0
const uint8_t dataPins[8] = {22, 23, 24, 25, 26, 27, 28, 29};

// Image dimensions (QQVGA)
#define IMG_WIDTH 160
#define IMG_HEIGHT 120
#define TOTAL_PIXELS (IMG_WIDTH * IMG_HEIGHT)

// Memory optimization - sample every Nth pixel
#define SAMPLE_RATE 16
#define BUFFER_SIZE 1200

// Detection thresholds
#define GREEN_RATIO_THRESHOLD 1.2
#define MIN_GREEN_PIXELS 50
#define WEED_ALERT_THRESHOLD 0.3

// Frame buffer
uint16_t frameBuffer[BUFFER_SIZE];

// Camera register configuration
struct CameraReg {
  uint8_t reg;
  uint8_t val;
};

// OV7670 initialization registers
const CameraReg ov7670_config[] PROGMEM = {
  // Reset all registers
  {0x12, 0x80}, // COM7: Reset
  {0xFF, 0xFF}, // Delay marker
  
  // Image format
  {0x12, 0x14}, // COM7: QCIF, RGB
  {0x40, 0xd0}, // COM15: RGB565, full range
  {0x8C, 0x00}, // RGB444 disabled
  
  // Clock settings
  {0x11, 0x01}, // CLKRC: Use external clock
  {0x6B, 0x4A}, // PLL control
  
  // Hardware window
  {0x17, 0x13}, // HSTART
  {0x18, 0x01}, // HSTOP
  {0x32, 0x80}, // HREF
  {0x19, 0x02}, // VSTART
  {0x1A, 0x7A}, // VSTOP
  {0x03, 0x0A}, // VREF
  
  // Scaling for QQVGA
  {0x0C, 0x00}, // COM3: Enable scaling
  {0x3E, 0x00}, // COM14: No scaling PCLK
  {0x70, 0x3A}, // X_SCALING
  {0x71, 0x35}, // Y_SCALING
  {0x72, 0x11}, // DCW control
  {0x73, 0xF1}, // PCLK divider
  {0xA2, 0x02}, // PCLK delay
  
  // Color matrix
  {0x4F, 0x80}, // MTX1
  {0x50, 0x80}, // MTX2
  {0x51, 0x00}, // MTX3
  {0x52, 0x22}, // MTX4
  {0x53, 0x5E}, // MTX5
  {0x54, 0x80}, // MTX6
  {0x58, 0x9E}, // MTXS
  
  // Edge enhancement
  {0x41, 0x08}, // COM16
  {0x3F, 0x00}, // Edge enhancement
  {0x75, 0x05}, // Edge enhancement
  
  // Denoise
  {0x76, 0xE1}, // White pixel correction
  
  // Auto exposure/gain/white balance
  {0x13, 0xE0}, // COM8: AWB, AGC, AEC
  {0x00, 0x00}, // Gain
  {0x10, 0x00}, // AEC
  {0x0D, 0x40}, // COM4
  {0x14, 0x18}, // COM9: AGC ceiling
  {0x24, 0x95}, // AGC/AEC
  {0x25, 0x33}, // AGC/AEC
  {0x26, 0xE3}, // AGC/AEC
  {0x9F, 0x78}, // Histogram AEC
  {0xA0, 0x68}, // Histogram AEC
  {0xA1, 0x03}, // Reserved
  {0xA6, 0xD8}, // Histogram AEC
  {0xA7, 0xD8}, // Histogram AEC
  {0xA8, 0xF0}, // Histogram AEC
  {0xA9, 0x90}, // Histogram AEC
  {0xAA, 0x94}, // Histogram AEC
  {0x3A, 0x0C}, // TSLB
  
  // Other format
  {0x0E, 0x61}, // COM5
  {0x0F, 0x4B}, // COM6
  {0x16, 0x02}, // Reserved
  {0x1E, 0x00}, // MVFP: No mirror/vflip
  {0x21, 0x02}, // ADCCTR1
  {0x22, 0x91}, // ADCCTR2
  {0x29, 0x07}, // HSYNC rising edge delay
  {0x33, 0x0B}, // CHLF: Array current
  {0x35, 0x0B}, // Reserved
  {0x37, 0x1D}, // ADC control
  {0x38, 0x71}, // ADC control
  {0x39, 0x2A}, // ADC control
  {0x3C, 0x78}, // COM12
  {0x4D, 0x40}, // Reserved
  {0x4E, 0x20}, // Reserved
  {0x69, 0x00}, // GFIX: Fix gain
  {0x6B, 0x4A}, // PLL control
  {0x74, 0x10}, // REG74
  {0x8D, 0x4F}, // Reserved
  {0x8E, 0x00}, // Reserved
  {0x8F, 0x00}, // Reserved
  {0x90, 0x00}, // Reserved
  {0x91, 0x00}, // Reserved
  {0x96, 0x00}, // Reserved
  {0x9A, 0x00}, // Reserved
  {0xB0, 0x84}, // Reserved
  {0xB1, 0x0C}, // ABLC1
  {0xB2, 0x0E}, // Reserved
  {0xB3, 0x82}, // THL_ST
  {0xB8, 0x0A}, // Reserved
  
  // End of configuration
  {0xFF, 0xFF}
};

void setup() {
  // Initialize serial with reliable baud rate
  Serial.begin(9600);
  
  // Wait for serial port to connect (important for Leonardo/Micro)
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 3000)) {
    ; // Wait up to 3 seconds
  }
  
  // Small delay to let serial stabilize
  delay(200);
  
  // Clear any garbage in buffer
  while (Serial.available()) Serial.read();
  
  // Print header
  Serial.println(F("============================"));
  Serial.println(F("OV7670 Weed Detection"));
  Serial.println(F("Version 1.0 - Arduino Mega"));
  Serial.println(F("============================"));
  Serial.println();
  
  // Check if we have enough RAM
  int freeMemory = getFreeRAM();
  Serial.print(F("Available RAM: "));
  Serial.print(freeMemory);
  Serial.println(F(" bytes"));
  
  if (freeMemory < 3000) {
    Serial.println(F("ERROR: Insufficient memory!"));
    Serial.println(F("This code requires Arduino Mega"));
    Serial.println(F("(Arduino Uno has only 2KB RAM)"));
    while (1) {
      delay(1000);
    }
  }
  
  // Initialize pins
  initializePins();
  
  // Start I2C communication
  Wire.begin();
  Wire.setClock(100000); // 100kHz for OV7670
  delay(50);
  
  // Initialize camera
  Serial.println(F("Initializing OV7670 camera..."));
  Serial.println(F("Please wait..."));
  
  if (initializeCamera()) {
    Serial.println(F("SUCCESS: Camera ready!"));
    Serial.println();
  } else {
    Serial.println(F("FAILED: Cannot initialize camera"));
    Serial.println();
    Serial.println(F("Troubleshooting:"));
    Serial.println(F("1. Check all wire connections"));
    Serial.println(F("2. Verify 3.3V power supply"));
    Serial.println(F("3. Confirm level shifters on I2C"));
    Serial.println(F("4. Check camera module is seated"));
    Serial.println(F("5. Try pressing Arduino reset"));
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println(F("System is ready!"));
  Serial.println(F("Starting in 3 seconds..."));
  Serial.println();
  delay(3000);
}

void loop() {
  static unsigned long frameCount = 0;
  
  frameCount++;
  
  Serial.print(F("=== Frame "));
  Serial.print(frameCount);
  Serial.println(F(" ==="));
  
  // Capture frame from camera
  Serial.print(F("Capturing... "));
  bool captureSuccess = captureFrame();
  
  if (captureSuccess) {
    Serial.println(F("OK"));
    
    // Analyze the captured frame
    analyzeFrame();
  } else {
    Serial.println(F("FAILED"));
    Serial.println(F("Check camera connection"));
  }
  
  Serial.println();
  
  // Wait before next capture
  delay(2000);
}

// Initialize all pins
void initializePins() {
  Serial.println(F("Setting up pins..."));
  
  // Configure data pins as inputs
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(dataPins[i], INPUT);
  }
  
  // Configure control pins
  pinMode(PIN_VSYNC, INPUT);
  pinMode(PIN_HREF, INPUT);
  pinMode(PIN_PCLK, INPUT);
  pinMode(PIN_XCLK, OUTPUT);
  
  // Generate 8MHz clock on XCLK using Timer1
  // Timer1 CTC mode: 16MHz / 2 = 8MHz
  TCCR1A = _BV(COM1A0);           // Toggle OC1A on compare match
  TCCR1B = _BV(WGM12) | _BV(CS10); // CTC mode, no prescaler
  OCR1A = 0;                       // Divide by 2
  
  delay(100); // Let clock stabilize
  
  Serial.println(F("Pins configured"));
}

// Initialize camera with register configuration
bool initializeCamera() {
  // Test I2C connection
  Wire.beginTransmission(OV7670_I2C_ADDR);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print(F("I2C Error Code: "));
    Serial.println(error);
    return false;
  }
  
  Serial.println(F("I2C communication OK"));
  
  // Load configuration
  uint16_t regCount = 0;
  
  for (uint16_t i = 0; ; i++) {
    uint8_t reg = pgm_read_byte(&ov7670_config[i].reg);
    uint8_t val = pgm_read_byte(&ov7670_config[i].val);
    
    // Check for end marker
    if (i > 0 && reg == 0xFF && val == 0xFF) {
      break;
    }
    
    // Check for delay marker (after reset)
    if (reg == 0xFF && val == 0xFF && i == 1) {
      delay(300); // Wait after reset
      continue;
    }
    
    // Write register
    if (!writeCameraRegister(reg, val)) {
      Serial.print(F("Failed at register 0x"));
      Serial.println(reg, HEX);
      return false;
    }
    
    regCount++;
    delayMicroseconds(100);
  }
  
  Serial.print(F("Loaded "));
  Serial.print(regCount);
  Serial.println(F(" registers"));
  
  // Final delay
  delay(300);
  
  // Verify configuration
  uint8_t testReg = readCameraRegister(0x12);
  Serial.print(F("COM7 readback: 0x"));
  Serial.println(testReg, HEX);
  
  return true;
}

// Write single register
bool writeCameraRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

// Read single register
uint8_t readCameraRegister(uint8_t reg) {
  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom((uint8_t)OV7670_I2C_ADDR, (uint8_t)1);
  
  if (Wire.available()) {
    return Wire.read();
  }
  
  return 0xFF;
}

// Read byte from data pins
inline uint8_t readDataPins() {
  uint8_t data = 0;
  
  // Read D7 to D0 (MSB to LSB)
  if (digitalRead(dataPins[0])) data |= 0x80; // D7
  if (digitalRead(dataPins[1])) data |= 0x40; // D6
  if (digitalRead(dataPins[2])) data |= 0x20; // D5
  if (digitalRead(dataPins[3])) data |= 0x10; // D4
  if (digitalRead(dataPins[4])) data |= 0x08; // D3
  if (digitalRead(dataPins[5])) data |= 0x04; // D2
  if (digitalRead(dataPins[6])) data |= 0x02; // D1
  if (digitalRead(dataPins[7])) data |= 0x01; // D0
  
  return data;
}

// Capture frame from camera
bool captureFrame() {
  uint16_t pixelIndex = 0;
  uint32_t sampleCounter = 0;
  unsigned long timeout;
  
  // Wait for VSYNC high (start of frame)
  timeout = millis();
  while (!digitalRead(PIN_VSYNC)) {
    if (millis() - timeout > 1000) {
      return false;
    }
  }
  
  // Wait for VSYNC low (active frame)
  timeout = millis();
  while (digitalRead(PIN_VSYNC)) {
    if (millis() - timeout > 1000) {
      return false;
    }
  }
  
  // Capture pixels line by line
  for (uint16_t row = 0; row < IMG_HEIGHT; row++) {
    // Wait for HREF high (line active)
    timeout = millis();
    while (!digitalRead(PIN_HREF)) {
      if (millis() - timeout > 100) return false;
      if (digitalRead(PIN_VSYNC)) return false; // Frame ended
    }
    
    // Read pixels in this line
    for (uint16_t col = 0; col < IMG_WIDTH; col++) {
      // Wait for PCLK rising edge
      while (!digitalRead(PIN_PCLK));
      uint8_t highByte = readDataPins();
      while (digitalRead(PIN_PCLK));
      
      // Wait for PCLK rising edge
      while (!digitalRead(PIN_PCLK));
      uint8_t lowByte = readDataPins();
      while (digitalRead(PIN_PCLK));
      
      // Sample every Nth pixel to save memory
      if ((sampleCounter % SAMPLE_RATE == 0) && (pixelIndex < BUFFER_SIZE)) {
        frameBuffer[pixelIndex++] = ((uint16_t)highByte << 8) | lowByte;
      }
      
      sampleCounter++;
    }
    
    // Wait for HREF low (line end)
    while (digitalRead(PIN_HREF));
  }
  
  return (pixelIndex > 100); // Valid if we captured something
}

// Analyze captured frame
void analyzeFrame() {
  uint16_t greenCount = 0;
  uint16_t soilCount = 0;
  uint16_t otherCount = 0;
  uint32_t greenIntensitySum = 0;
  
  // Process each pixel
  for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
    uint16_t rgb565 = frameBuffer[i];
    
    // Convert RGB565 to RGB888
    uint8_t r = ((rgb565 >> 11) & 0x1F) * 255 / 31;
    uint8_t g = ((rgb565 >> 5) & 0x3F) * 255 / 63;
    uint8_t b = (rgb565 & 0x1F) * 255 / 31;
    
    // Classify pixel
    if (isGreen(r, g, b)) {
      greenCount++;
      greenIntensitySum += g;
    } else if (isSoil(r, g, b)) {
      soilCount++;
    } else {
      otherCount++;
    }
  }
  
  // Calculate statistics
  uint16_t totalPixels = BUFFER_SIZE;
  float greenPercent = (greenCount * 100.0) / totalPixels;
  float soilPercent = (soilCount * 100.0) / totalPixels;
  uint8_t avgGreenIntensity = (greenCount > 0) ? (greenIntensitySum / greenCount) : 0;
  
  // Print analysis
  Serial.println(F("--- Analysis ---"));
  Serial.print(F("Green: "));
  Serial.print(greenPercent, 1);
  Serial.println(F("%"));
  
  Serial.print(F("Soil: "));
  Serial.print(soilPercent, 1);
  Serial.println(F("%"));
  
  Serial.print(F("Other: "));
  Serial.print(((otherCount * 100.0) / totalPixels), 1);
  Serial.println(F("%"));
  
  // Make detection decision
  if (greenCount < MIN_GREEN_PIXELS) {
    Serial.println(F("Status: Bare soil"));
  } else {
    Serial.print(F("Green intensity: "));
    Serial.println(avgGreenIntensity);
    
    if (greenPercent > 70) {
      Serial.println(F("Status: Dense vegetation"));
    } else if (greenPercent > 40) {
      float nonGreen = 1.0 - (greenPercent / 100.0);
      if (nonGreen > WEED_ALERT_THRESHOLD) {
        Serial.println(F("* WEED DETECTED *"));
      } else {
        Serial.println(F("Status: Moderate coverage"));
      }
    } else {
      Serial.println(F("Status: Sparse vegetation"));
    }
    
    if (avgGreenIntensity < 80) {
      Serial.println(F("Note: Low chlorophyll"));
    }
  }
}

// Check if pixel is green vegetation
bool isGreen(uint8_t r, uint8_t g, uint8_t b) {
  if (g < 60) return false;
  if (g <= r || g <= b) return false;
  
  float avgRB = ((float)r + (float)b) / 2.0;
  float ratio = (float)g / (avgRB + 1.0);
  
  return (ratio >= GREEN_RATIO_THRESHOLD);
}

// Check if pixel is soil
bool isSoil(uint8_t r, uint8_t g, uint8_t b) {
  int diffRG = abs((int)r - (int)g);
  return (r > 50 && g > 40 && g < 150 && b < g && diffRG < 40);
}

// Get available RAM
int getFreeRAM() {
  extern int _heap_start, *_brkval;
  int v;
  return (int)&v - (_brkval == 0 ? (int)&heap_start : (int)_brkval);
}
