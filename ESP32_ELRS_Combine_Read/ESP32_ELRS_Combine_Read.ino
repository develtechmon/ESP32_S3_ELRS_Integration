/*
 * ELRS Channel Reader using ESP32 Hardware Serial
 * 
 * Hardware Connections:
 * - ELRS RX Pin (Data Out) -> ESP32 Pin 5 (PIN_RX)
 * - ELRS TX Pin (if available) -> ESP32 Pin 6 (PIN_TX)
 * - ELRS VCC -> 3.3V or 5V (depending on your ELRS module)
 * - ELRS GND -> GND
 * 
 * This uses ESP32's Serial1 (UART1) for ELRS communication
 * Serial (UART0) remains available for debugging via USB
 */

#include <HardwareSerial.h>

#define PIN_RX 5
#define PIN_TX 6
#define STATUS_LED 2  // ESP32 built-in LED

// Set up Serial1 for CRSF communication
HardwareSerial crsfSerial(1);

// CRSF Protocol Constants
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS 0x16
#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_CHANNEL_COUNT 16

// Channel data storage (CRSF range: 172-1811, converted to PWM: 1000-2000)
uint16_t channels[CRSF_CHANNEL_COUNT];
unsigned long lastValidFrame = 0;

// Frame processing variables
uint8_t frameBuffer[CRSF_FRAME_SIZE_MAX];
uint8_t frameIndex = 0;
bool frameStarted = false;
uint8_t expectedFrameLength = 0;

void setup() {
  // Initialize USB Serial for debugging
  Serial.begin(115200);
  Serial.println("ELRS Channel Reader - ESP32 Hardware Serial");
  
  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  
  // Initialize CRSF Serial on pins 5,6 at standard CRSF baud rate
  crsfSerial.begin(420000, SERIAL_8N1, PIN_RX, PIN_TX);
  
  // Initialize channels to center position
  for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
    channels[i] = 1500; // Center position in PWM microseconds
  }
  
  Serial.println("CRSF Serial initialized on pins 5(RX) and 6(TX)");
  Serial.println("Listening for CRSF frames...");
  
  // Startup blink sequence
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(200);
    digitalWrite(STATUS_LED, LOW);
    delay(200);
  }
}

void loop() {
  // Process incoming CRSF data
  if (crsfSerial.available()) {
    uint8_t incomingByte = crsfSerial.read();
    
    if (!frameStarted) {
      // Look for CRSF sync byte
      if (incomingByte == CRSF_SYNC_BYTE) {
        frameStarted = true;
        frameIndex = 0;
        frameBuffer[frameIndex++] = incomingByte;
      }
    } else if (frameIndex == 1) {
      // Second byte is payload length
      expectedFrameLength = incomingByte + 2; // +2 for sync and length bytes
      frameBuffer[frameIndex++] = incomingByte;
      
      // Sanity check frame length
      if (expectedFrameLength > CRSF_FRAME_SIZE_MAX) {
        frameStarted = false;
        frameIndex = 0;
      }
    } else {
      // Collect frame data
      frameBuffer[frameIndex++] = incomingByte;
      
      // Check if frame is complete
      if (frameIndex >= expectedFrameLength) {
        processCRSFFrame();
        frameStarted = false;
        frameIndex = 0;
      }
      
      // Prevent buffer overflow
      if (frameIndex >= CRSF_FRAME_SIZE_MAX) {
        frameStarted = false;
        frameIndex = 0;
      }
    }
  }
  
  // Status indication and real-time channel printing
  static unsigned long lastBlink = 0;
  static unsigned long lastPrint = 0;
  
  if (millis() - lastValidFrame < 1000) {
    // Fast blink when receiving valid data
    if (millis() - lastBlink > 100) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      lastBlink = millis();
    }
    
    // Print all 16 channels in real-time - every 10ms for smooth updates
    if (millis() - lastPrint > 10) {
      // Clear screen and print all channels in a compact format
      Serial.print("\r");
      for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
        Serial.print(channels[i]);
        if (i < CRSF_CHANNEL_COUNT - 1) Serial.print("|");
      }
      Serial.println("    "); // Clear any leftover characters
      
      lastPrint = millis();
    }
  } else {
    // Slow blink when no data
    if (millis() - lastBlink > 1000) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      Serial.println("\nNo CRSF frames - check connection");
      lastBlink = millis();
    }
  }
  
  // Your application code here
  processChannelData();
}

void processCRSFFrame() {
  // Verify minimum frame length
  if (frameIndex < 4) return;
  
  // Check frame type (3rd byte should be RC channels)
  if (frameBuffer[2] != CRSF_FRAMETYPE_RC_CHANNELS) {
    return; // Skip non-channel frames
  }
  
  // Verify frame length for RC channels (should be 26 bytes total)
  if (frameIndex != 26) {
    return;
  }
  
  // Calculate and verify CRC
  uint8_t crc = calculateCRC(&frameBuffer[2], frameIndex - 3);
  if (crc != frameBuffer[frameIndex - 1]) {
    Serial.println("CRC mismatch - frame corrupted");
    return;
  }
  
  // Decode channel data
  decodeChannels(&frameBuffer[3]);
  lastValidFrame = millis();
}

void decodeChannels(uint8_t* data) {
  // CRSF channels are packed as 11-bit values (0-2047, typically 172-1811)
  // Think of it like unpacking a suitcase where everything is squeezed together
  
  // Extract 11-bit values using bit manipulation
  uint16_t rawChannels[16];
  
  rawChannels[0]  = ((data[0]    | data[1] << 8))                 & 0x07FF;
  rawChannels[1]  = ((data[1]>>3 | data[2] << 5))                 & 0x07FF;
  rawChannels[2]  = ((data[2]>>6 | data[3] << 2 | data[4] << 10)) & 0x07FF;
  rawChannels[3]  = ((data[4]>>1 | data[5] << 7))                 & 0x07FF;
  rawChannels[4]  = ((data[5]>>4 | data[6] << 4))                 & 0x07FF;
  rawChannels[5]  = ((data[6]>>7 | data[7] << 1 | data[8] << 9))  & 0x07FF;
  rawChannels[6]  = ((data[8]>>2 | data[9] << 6))                 & 0x07FF;
  rawChannels[7]  = ((data[9]>>5 | data[10] << 3))                & 0x07FF;
  rawChannels[8]  = ((data[11]   | data[12] << 8))                & 0x07FF;
  rawChannels[9]  = ((data[12]>>3| data[13] << 5))                & 0x07FF;
  rawChannels[10] = ((data[13]>>6| data[14] << 2 | data[15] << 10)) & 0x07FF;
  rawChannels[11] = ((data[15]>>1| data[16] << 7))                & 0x07FF;
  rawChannels[12] = ((data[16]>>4| data[17] << 4))                & 0x07FF;
  rawChannels[13] = ((data[17]>>7| data[18] << 1 | data[19] << 9)) & 0x07FF;
  rawChannels[14] = ((data[19]>>2| data[20] << 6))                & 0x07FF;
  rawChannels[15] = ((data[20]>>5| data[21] << 3))                & 0x07FF;
  
  // Convert from CRSF range (172-1811) to standard PWM range (1000-2000)
  for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
    channels[i] = map(rawChannels[i], 172, 1811, 1000, 2000);
    channels[i] = constrain(channels[i], 1000, 2000);
  }
}

uint8_t calculateCRC(uint8_t* data, uint8_t length) {
  // CRSF CRC8 calculation
  uint8_t crc = 0;
  for (uint8_t i = 0; i < length; i++) {
    crc = crc ^ data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

void processChannelData() {
  // This is where you use your channel data
  // channels[0] = Channel 1 (typically throttle) - 1000 to 2000 microseconds
  // channels[1] = Channel 2 (typically roll)
  // channels[2] = Channel 3 (typically pitch)  
  // channels[3] = Channel 4 (typically yaw)
  
  // Example: Control servos
  // myServo.writeMicroseconds(channels[0]);
  
  // Example: Map to analog output
  // int pwmValue = map(channels[0], 1000, 2000, 0, 255);
  // analogWrite(LED_BUILTIN, pwmValue);
  
  // Example: Detect switch positions
  // if (channels[4] > 1700) {
  //   // Switch high
  // } else if (channels[4] < 1300) {
  //   // Switch low  
  // }
}

// Helper functions
uint16_t getChannel(uint8_t channelNum) {
  if (channelNum < 1 || channelNum > 16) return 1500;
  return channels[channelNum - 1];
}

bool isReceiverConnected() {
  return (millis() - lastValidFrame < 1000);
}