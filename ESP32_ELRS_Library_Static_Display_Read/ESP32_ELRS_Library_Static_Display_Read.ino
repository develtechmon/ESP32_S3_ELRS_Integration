/*
  ================================================================================
  Project     : ELRS Channel Reader Test
  File        : ELRS_Channel_Test.ino
  Author      : Testing ELRS integration
  Date        : 2025-07-25
  Description : Standalone ELRS channel reader using flight controller style
  ================================================================================
*/

// ===== DEBUG CONTROL =====
#define ENABLE_DEBUG 1

#if ENABLE_DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(format, ...) Serial.printf(format, __VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(format, ...)
#endif

#include "ELRS_CRSF.h"
#include <Adafruit_NeoPixel.h>

// ===== HARDWARE DEFINITIONS =====
#define LED_PIN 48  
#define NUM_LEDS 1
#define NUM_CHANNELS 8         
#define CHANNEL_MIN 1000       
#define CHANNEL_MAX 2000       

// ===== ELRS RECEIVER SETUP =====
#define ELRS_RX_PIN 5
#define ELRS_TX_PIN 6
ELRS_CRSF elrs(ELRS_RX_PIN, ELRS_TX_PIN);

// ===== RECEIVER DATA (FLIGHT CONTROLLER STYLE) =====
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
int channelValues[NUM_CHANNELS];

// ===== LED SETUP =====
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== TIMING VARIABLES =====
uint32_t LoopTimer;
const float t = 0.004; // 250Hz loop timing (same as flight controller)

// ===== ELRS RECEIVER FUNCTION (FLIGHT CONTROLLER STYLE) =====
void read_receiver(int *channelValues) {
  // Update ELRS data (non-blocking)
  elrs.update();
  
  if (elrs.isConnected()) {
    // Read all 8 channels from ELRS and populate the ReceiverValue array
    for (int i = 0; i < NUM_CHANNELS; i++) {
      int channelValue = elrs.getChannel(i + 1); // ELRS channels are 1-based
      
      // Apply safety limits
      if (channelValue < CHANNEL_MIN) channelValue = CHANNEL_MIN;
      else if (channelValue > CHANNEL_MAX) channelValue = CHANNEL_MAX;
      
      // Update both arrays for compatibility
      ReceiverValue[i] = channelValue;
      channelValues[i] = channelValue;
    }
    
    // Status LED: Green when connected and receiving
    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate > 100) {
      strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green
      strip.show();
      lastLedUpdate = millis();
    }
    
  } else {
    // ELRS not connected - set failsafe values
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (i == 2) { // Throttle channel (usually channel 3)
        ReceiverValue[i] = CHANNEL_MIN; // Throttle to minimum
        channelValues[i] = CHANNEL_MIN;
      } else {
        ReceiverValue[i] = 1500; // Center position for other channels
        channelValues[i] = 1500;
      }
    }
    
    // Status LED: Red when disconnected
    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate > 500) {
      static bool ledState = false;
      ledState = !ledState;
      if (ledState) {
        strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red
      } else {
        strip.setPixelColor(0, strip.Color(0, 0, 0)); // Off
      }
      strip.show();
      lastLedUpdate = millis();
    }
    
    #if ENABLE_DEBUG
    static unsigned long lastFailsafeWarning = 0;
    if (millis() - lastFailsafeWarning > 5000) { // Warn every 5 seconds
      DEBUG_PRINTLN("ELRS FAILSAFE - No signal");
      lastFailsafeWarning = millis();
    }
    #endif
  }
}

// ===== SETUP FUNCTION =====
void setup() {
  Serial.begin(115200);
  
  // Initialize LED
  strip.begin();
  strip.show();
  pinMode(2, OUTPUT);
  
  // Startup LED sequence
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue
    strip.show();
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();
    delay(100);
  }
  
  // ===== INITIALIZE ELRS =====
  DEBUG_PRINTLN("=== ELRS Channel Reader Test ===");
  DEBUG_PRINTLN("Initializing ELRS...");
  elrs.enableDebug(ENABLE_DEBUG);
  
  if (!elrs.begin()) {
    DEBUG_PRINTLN("ELRS initialization failed!");
    // Flash red LED to indicate error
    for (int i = 0; i < 10; i++) {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red
      strip.show();
      delay(100);
      strip.setPixelColor(0, strip.Color(0, 0, 0));
      strip.show();
      delay(100);
    }
  } else {
    DEBUG_PRINTLN("ELRS initialized successfully");
    // Green flash to confirm ELRS is ready
    strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green
    strip.show();
    delay(500);
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();
  }

  // Wait for ELRS connection before proceeding
  DEBUG_PRINTLN("Waiting for ELRS connection...");
  DEBUG_PRINTLN("Turn on your transmitter and check binding...");
  
  unsigned long elrsWaitStart = millis();
  while (!elrs.isConnected() && (millis() - elrsWaitStart < 15000)) { // 15 second timeout
    elrs.update();
    // Blink blue while waiting
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 250) {
      static bool blinkState = false;
      blinkState = !blinkState;
      if (blinkState) {
        strip.setPixelColor(0, strip.Color(0, 0, 255)); // Blue
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        strip.setPixelColor(0, strip.Color(0, 0, 0)); // Off
        digitalWrite(LED_BUILTIN, LOW);
      }
      strip.show();
      lastBlink = millis();
    }
    delay(10);
  }
  
  if (elrs.isConnected()) {
    DEBUG_PRINTLN("ELRS connected successfully!");
    DEBUG_PRINTF("Frame rate: %d Hz\n", elrs.getFrameRate());
    strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green
    strip.show();
    delay(1000);
  } else {
    DEBUG_PRINTLN("ELRS connection timeout - continuing anyway");
    DEBUG_PRINTLN("Check transmitter binding and receiver wiring");
    strip.setPixelColor(0, strip.Color(255, 255, 0)); // Yellow warning
    strip.show();
    delay(2000);
  }
  
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  DEBUG_PRINTLN("=== Channel Mapping ===");
  DEBUG_PRINTLN("CH1: Roll    | CH2: Pitch   | CH3: Throttle | CH4: Yaw");
  DEBUG_PRINTLN("CH5: AUX1    | CH6: AUX2    | CH7: AUX3     | CH8: AUX4");
  DEBUG_PRINTLN("===============================================");
  DEBUG_PRINTLN("Move your sticks and switches to test...");
  DEBUG_PRINTLN();

  // Initialize loop timer
  LoopTimer = micros();
}

// ===== MAIN LOOP (FLIGHT CONTROLLER STYLE) =====
void loop() {
  // Read receiver data (same function call as flight controller)
  read_receiver(channelValues);
  
  // ===== DISPLAY CHANNEL DATA (FIXED SCREEN) =====
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) { // Update every 100ms for smooth display
    
    // Clear screen and move cursor to top
    Serial.print("\033[2J");      // Clear entire screen
    Serial.print("\033[H");       // Move cursor to home position (0,0)
    
    // Header
    Serial.println("=====================================");
    Serial.println("      ELRS CHANNEL READER TEST      ");
    Serial.println("=====================================");
    Serial.println();
    
    if (elrs.isConnected()) {
      // Connection status
      Serial.printf("Status: CONNECTED | Frame Rate: %3d Hz\n", elrs.getFrameRate());
      Serial.println();
      
      // Channel data in organized format
      Serial.println("┌─────────────────────────────────────┐");
      Serial.println("│            CHANNEL DATA             │");
      Serial.println("├─────────────────────────────────────┤");
      
      // Primary flight channels
      Serial.printf("│ Roll (CH1):     %4d  [%7s]   │\n", 
                   channelValues[0], 
                   channelValues[0] < 1400 ? "LEFT" : channelValues[0] > 1600 ? "RIGHT" : "CENTER");
      
      Serial.printf("│ Pitch (CH2):    %4d  [%7s]   │\n", 
                   channelValues[1], 
                   channelValues[1] < 1400 ? "DOWN" : channelValues[1] > 1600 ? "UP" : "CENTER");
      
      Serial.printf("│ Throttle (CH3): %4d  [%7s]   │\n", 
                   channelValues[2], 
                   channelValues[2] < 1200 ? "LOW" : channelValues[2] > 1800 ? "HIGH" : "MID");
      
      Serial.printf("│ Yaw (CH4):      %4d  [%7s]   │\n", 
                   channelValues[3], 
                   channelValues[3] < 1400 ? "LEFT" : channelValues[3] > 1600 ? "RIGHT" : "CENTER");
      
      Serial.println("├─────────────────────────────────────┤");
      
      // AUX channels
      Serial.printf("│ AUX1 (CH5):     %4d  [%7s]   │\n", 
                   channelValues[4], 
                   channelValues[4] < 1300 ? "LOW" : channelValues[4] > 1700 ? "HIGH" : "MID");
      
      Serial.printf("│ AUX2 (CH6):     %4d  [%7s]   │\n", 
                   channelValues[5], 
                   channelValues[5] < 1300 ? "LOW" : channelValues[5] > 1700 ? "HIGH" : "MID");
      
      Serial.printf("│ AUX3 (CH7):     %4d  [%7s]   │\n", 
                   channelValues[6], 
                   channelValues[6] < 1300 ? "LOW" : channelValues[6] > 1700 ? "HIGH" : "MID");
      
      Serial.printf("│ AUX4 (CH8):     %4d  [%7s]   │\n", 
                   channelValues[7], 
                   channelValues[7] < 1300 ? "LOW" : channelValues[7] > 1700 ? "HIGH" : "MID");
      
      Serial.println("└─────────────────────────────────────┘");
      Serial.println();
      
      // Flight controller style calculations
      float DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
      float DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
      float DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);
      
      Serial.println("┌─────────────────────────────────────┐");
      Serial.println("│        FLIGHT CALCULATIONS         │");
      Serial.println("├─────────────────────────────────────┤");
      Serial.printf("│ Desired Roll:   %+6.1f degrees    │\n", DesiredAngleRoll);
      Serial.printf("│ Desired Pitch:  %+6.1f degrees    │\n", DesiredAnglePitch);
      Serial.printf("│ Desired Yaw:    %+6.1f deg/s      │\n", DesiredRateYaw);
      Serial.println("└─────────────────────────────────────┘");
      Serial.println();
      
      // Switch states
      bool altitudeHoldSwitch = (ReceiverValue[4] > 1700);
      bool positionHoldSwitch = (ReceiverValue[5] > 1700);
      bool armingThrottleLow = (ReceiverValue[2] < 1050);
      bool armingYawRight = (ReceiverValue[3] > 1900);
      
      Serial.println("┌─────────────────────────────────────┐");
      Serial.println("│          FLIGHT MODES              │");
      Serial.println("├─────────────────────────────────────┤");
      Serial.printf("│ Altitude Hold:  %s              │\n", altitudeHoldSwitch ? "ACTIVE  " : "INACTIVE");
      Serial.printf("│ Position Hold:  %s              │\n", positionHoldSwitch ? "ACTIVE  " : "INACTIVE");
      Serial.printf("│ Throttle Low:   %s              │\n", armingThrottleLow ? "YES     " : "NO      ");
      Serial.printf("│ Yaw Right:      %s              │\n", armingYawRight ? "YES     " : "NO      ");
      Serial.println("└─────────────────────────────────────┘");
      
    } else {
      // Disconnected status
      Serial.println("Status: DISCONNECTED");
      Serial.println();
      Serial.println("┌─────────────────────────────────────┐");
      Serial.println("│              FAILSAFE               │");
      Serial.println("├─────────────────────────────────────┤");
      Serial.println("│ • Check transmitter power           │");
      Serial.println("│ • Verify receiver binding           │");
      Serial.println("│ • Check wiring connections          │");
      Serial.println("│ • RX Pin 5, TX Pin 6                │");
      Serial.println("└─────────────────────────────────────┘");
    }
    
    Serial.println();
    Serial.println("Move sticks and switches to test...");
    Serial.println("Press Ctrl+C to exit");
    
    lastPrint = millis();
  }
  
  // ===== FLIGHT CONTROLLER STYLE ACCESS EXAMPLES =====
  // You can access channels exactly like in your flight controller:
  
  // Direct array access (flight controller style)
  int roll_input = ReceiverValue[0];      // Channel 1 (Roll)
  int pitch_input = ReceiverValue[1];     // Channel 2 (Pitch)  
  int throttle_input = ReceiverValue[2];  // Channel 3 (Throttle)
  int yaw_input = ReceiverValue[3];       // Channel 4 (Yaw)
  int aux1_switch = ReceiverValue[4];     // Channel 5 (AUX1)
  int aux2_switch = ReceiverValue[5];     // Channel 6 (AUX2)
  
  // Or use the channelValues array
  // Both arrays contain identical data
  
  // Calculate desired angles (same as flight controller)
  float DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
  float DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  float DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);
  
  // Detect switch positions (same logic as flight controller)
  bool altitudeHoldSwitch = (ReceiverValue[4] > 1700); // Channel 5 high
  bool positionHoldSwitch = (ReceiverValue[5] > 1700); // Channel 6 high
  
  // Check for arming conditions (same as flight controller)
  bool throttleLow = (ReceiverValue[2] < 1050);
  bool yawRight = (ReceiverValue[3] > 1900);
  bool yawLeft = (ReceiverValue[3] < 1100);
  
  // ===== MAINTAIN FLIGHT CONTROLLER LOOP TIMING =====
  // This maintains exact same timing as your flight controller (250Hz)
  while (micros() - LoopTimer < (t * 1000000));
  LoopTimer = micros();
}