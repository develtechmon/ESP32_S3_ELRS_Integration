/*
 * ELRS_CRSF.h - A clean API for reading ELRS/CRSF data on ESP32
 * 
 * Usage:
 *   ELRS_CRSF elrs(rxPin, txPin);
 *   elrs.begin();
 *   
 *   In loop():
 *   elrs.update();
 *   if (elrs.isConnected()) {
 *     uint16_t throttle = elrs.getChannel(1);
 *   }
 */

#ifndef ELRS_CRSF_H
#define ELRS_CRSF_H

#include <Arduino.h>
#include <HardwareSerial.h>

class ELRS_CRSF {
private:
    // Hardware configuration
    uint8_t _rxPin;
    uint8_t _txPin;
    uint8_t _uartNum;
    HardwareSerial* _serial;
    
    // CRSF Protocol Constants
    static const uint8_t CRSF_SYNC_BYTE = 0xC8;
    static const uint8_t CRSF_FRAMETYPE_RC_CHANNELS = 0x16;
    static const uint8_t CRSF_FRAME_SIZE_MAX = 64;
    static const uint8_t CRSF_CHANNEL_COUNT = 16;
    
    // Data storage
    uint16_t _channels[CRSF_CHANNEL_COUNT];
    unsigned long _lastValidFrame;
    
    // Frame processing
    uint8_t _frameBuffer[CRSF_FRAME_SIZE_MAX];
    uint8_t _frameIndex;
    bool _frameStarted;
    uint8_t _expectedFrameLength;
    
    // Internal methods
    void processCRSFFrame();
    void decodeChannels(uint8_t* data);
    uint8_t calculateCRC(uint8_t* data, uint8_t length);
    
public:
    // Constructor
    ELRS_CRSF(uint8_t rxPin, uint8_t txPin, uint8_t uartNum = 1);
    
    // Core API methods
    bool begin(uint32_t baudRate = 420000);
    void update();
    void end();
    
    // Data access methods
    uint16_t getChannel(uint8_t channelNum);           // Get channel 1-16 in PWM (1000-2000)
    uint16_t getRawChannel(uint8_t channelNum);        // Get raw CRSF value (172-1811)
    int16_t getChannelPercent(uint8_t channelNum);     // Get channel as percentage (-100 to +100)
    void getAllChannels(uint16_t* channelArray);       // Copy all channels to array
    
    // Status methods
    bool isConnected();                                // Check if receiving valid data
    unsigned long getLastFrameTime();                  // Time of last valid frame
    uint16_t getFrameRate();                          // Frames per second
    
    // Debugging methods
    void printAllChannels();                          // Print all channels to Serial
    void printChannelLine();                          // Print channels in one line
    void enableDebug(bool enable = true);             // Enable debug output
    
private:
    bool _debugEnabled;
    unsigned long _frameCount;
    unsigned long _lastFrameRateCalc;
    uint16_t _currentFrameRate;
};

// Implementation
ELRS_CRSF::ELRS_CRSF(uint8_t rxPin, uint8_t txPin, uint8_t uartNum) {
    _rxPin = rxPin;
    _txPin = txPin;
    _uartNum = uartNum;
    _serial = nullptr;
    _lastValidFrame = 0;
    _frameIndex = 0;
    _frameStarted = false;
    _expectedFrameLength = 0;
    _debugEnabled = false;
    _frameCount = 0;
    _lastFrameRateCalc = 0;
    _currentFrameRate = 0;
    
    // Initialize channels to center position
    for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
        _channels[i] = 1500;
    }
}

bool ELRS_CRSF::begin(uint32_t baudRate) {
    // Create hardware serial instance
    _serial = new HardwareSerial(_uartNum);
    
    if (!_serial) {
        if (_debugEnabled) Serial.println("ELRS_CRSF: Failed to create serial instance");
        return false;
    }
    
    // Initialize serial communication
    _serial->begin(baudRate, SERIAL_8N1, _rxPin, _txPin);
    
    if (_debugEnabled) {
        Serial.print("ELRS_CRSF: Initialized on UART");
        Serial.print(_uartNum);
        Serial.print(" - RX:");
        Serial.print(_rxPin);
        Serial.print(" TX:");
        Serial.print(_txPin);
        Serial.print(" Baud:");
        Serial.println(baudRate);
    }
    
    return true;
}

void ELRS_CRSF::update() {
    if (!_serial) return;
    
    // Process incoming data
    while (_serial->available()) {
        uint8_t incomingByte = _serial->read();
        
        if (!_frameStarted) {
            // Look for sync byte
            if (incomingByte == CRSF_SYNC_BYTE) {
                _frameStarted = true;
                _frameIndex = 0;
                _frameBuffer[_frameIndex++] = incomingByte;
            }
        } else if (_frameIndex == 1) {
            // Second byte is payload length
            _expectedFrameLength = incomingByte + 2;
            _frameBuffer[_frameIndex++] = incomingByte;
            
            // Sanity check
            if (_expectedFrameLength > CRSF_FRAME_SIZE_MAX) {
                _frameStarted = false;
                _frameIndex = 0;
            }
        } else {
            // Collect frame data
            _frameBuffer[_frameIndex++] = incomingByte;
            
            // Check if frame is complete
            if (_frameIndex >= _expectedFrameLength) {
                processCRSFFrame();
                _frameStarted = false;
                _frameIndex = 0;
            }
            
            // Prevent buffer overflow
            if (_frameIndex >= CRSF_FRAME_SIZE_MAX) {
                _frameStarted = false;
                _frameIndex = 0;
            }
        }
    }
    
    // Update frame rate calculation
    if (millis() - _lastFrameRateCalc >= 1000) {
        _currentFrameRate = _frameCount;
        _frameCount = 0;
        _lastFrameRateCalc = millis();
    }
}

void ELRS_CRSF::end() {
    if (_serial) {
        _serial->end();
        delete _serial;
        _serial = nullptr;
    }
}

uint16_t ELRS_CRSF::getChannel(uint8_t channelNum) {
    if (channelNum < 1 || channelNum > CRSF_CHANNEL_COUNT) return 1500;
    if (!isConnected()) return 1500;
    return _channels[channelNum - 1];
}

uint16_t ELRS_CRSF::getRawChannel(uint8_t channelNum) {
    if (channelNum < 1 || channelNum > CRSF_CHANNEL_COUNT) return 992;
    if (!isConnected()) return 992;
    // Convert back to raw CRSF range
    return map(_channels[channelNum - 1], 1000, 2000, 172, 1811);
}

int16_t ELRS_CRSF::getChannelPercent(uint8_t channelNum) {
    if (channelNum < 1 || channelNum > CRSF_CHANNEL_COUNT) return 0;
    if (!isConnected()) return 0;
    return map(_channels[channelNum - 1], 1000, 2000, -100, 100);
}

void ELRS_CRSF::getAllChannels(uint16_t* channelArray) {
    for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
        channelArray[i] = _channels[i];
    }
}

bool ELRS_CRSF::isConnected() {
    return (millis() - _lastValidFrame < 1000);
}

unsigned long ELRS_CRSF::getLastFrameTime() {
    return _lastValidFrame;
}

uint16_t ELRS_CRSF::getFrameRate() {
    return _currentFrameRate;
}

void ELRS_CRSF::printAllChannels() {
    Serial.println("=== ELRS Channels ===");
    for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
        Serial.print("CH");
        if (i + 1 < 10) Serial.print(" ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(_channels[i]);
        Serial.print("us");
        
        if ((i + 1) % 4 == 0) {
            Serial.println();
        } else {
            Serial.print("\t");
        }
    }
    if (CRSF_CHANNEL_COUNT % 4 != 0) Serial.println();
    Serial.print("FPS: ");
    Serial.print(_currentFrameRate);
    Serial.print(" | Connected: ");
    Serial.println(isConnected() ? "YES" : "NO");
    Serial.println("====================");
}

void ELRS_CRSF::printChannelLine() {
    for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
        Serial.print(_channels[i]);
        if (i < CRSF_CHANNEL_COUNT - 1) Serial.print("|");
    }
    Serial.print(" FPS:");
    Serial.print(_currentFrameRate);
    Serial.println();
}

void ELRS_CRSF::enableDebug(bool enable) {
    _debugEnabled = enable;
}

void ELRS_CRSF::processCRSFFrame() {
    if (_frameIndex < 4) return;
    
    // Check frame type
    if (_frameBuffer[2] != CRSF_FRAMETYPE_RC_CHANNELS) {
        return;
    }
    
    // Verify frame length
    if (_frameIndex != 26) {
        if (_debugEnabled) {
            Serial.print("Wrong frame length: ");
            Serial.println(_frameIndex);
        }
        return;
    }
    
    // Verify CRC
    uint8_t crc = calculateCRC(&_frameBuffer[2], _frameIndex - 3);
    if (crc != _frameBuffer[_frameIndex - 1]) {
        if (_debugEnabled) Serial.println("CRC mismatch");
        return;
    }
    
    // Valid frame - decode channels
    decodeChannels(&_frameBuffer[3]);
    _lastValidFrame = millis();
    _frameCount++;
}

void ELRS_CRSF::decodeChannels(uint8_t* data) {
    uint16_t rawChannels[16];
    
    // Unpack 11-bit channels
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
    rawChannels[11] = ((data[15]>>1| data[16] << 7))                 & 0x07FF;
    rawChannels[12] = ((data[16]>>4| data[17] << 4))                 & 0x07FF;
    rawChannels[13] = ((data[17]>>7| data[18] << 1 | data[19] << 9)) & 0x07FF;
    rawChannels[14] = ((data[19]>>2| data[20] << 6))                 & 0x07FF;
    rawChannels[15] = ((data[20]>>5| data[21] << 3))                 & 0x07FF;
    
    // Convert to PWM range
    for (int i = 0; i < CRSF_CHANNEL_COUNT; i++) {
        _channels[i] = map(rawChannels[i], 172, 1811, 1000, 2000);
        _channels[i] = constrain(_channels[i], 1000, 2000);
    }
}

uint8_t ELRS_CRSF::calculateCRC(uint8_t* data, uint8_t length) {
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

#endif // ELRS_CRSF_H

/*
 * Example Usage in your main sketch:
 * 
 * #include "ELRS_CRSF.h"
 * 
 * ELRS_CRSF elrs(5, 6);  // RX pin 5, TX pin 6
 * 
 * void setup() {
 *   Serial.begin(115200);
 *   elrs.enableDebug(true);
 *   elrs.begin();
 * }
 * 
 * void loop() {
 *   elrs.update();
 *   
 *   if (elrs.isConnected()) {
 *     uint16_t throttle = elrs.getChannel(1);
 *     int16_t roll_percent = elrs.getChannelPercent(2);
 *     
 *     // Or print all channels
 *     static unsigned long lastPrint = 0;
 *     if (millis() - lastPrint > 100) {
 *       elrs.printChannelLine();
 *       lastPrint = millis();
 *     }
 *   }
 * }
 */