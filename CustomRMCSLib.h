/*
 * CUSTOM_RMCS2303.h - Modified RMCS2303 Library for Multi-Port Support
 * 
 * This is a custom version that fixes the single serial port limitation
 * by properly encapsulating each serial port instance
 */

#ifndef CUSTOM_RMCS2303_H
#define CUSTOM_RMCS2303_H

#include <Arduino.h>
#include <HardwareSerial.h>

class CUSTOM_RMCS2303 {
private:
    // Instance-specific variables (not static!)
    HardwareSerial* _serial;
    SoftwareSerial* _softSerial;
    long _baudRate;
    bool _useHardwareSerial;
    bool _initialized;
    
    // Instance-specific buffers
    char _rxBuffer[64];
    char _txBuffer[64];
    int _timeout;
    
    // Private methods for MODBUS communication
    void sendModbusCommand(byte slaveId, byte functionCode, int address, int value);
    bool receiveModbusResponse(byte slaveId);
    void clearSerialBuffer();
    int calculateCRC(char* buffer, int length);
    
public:
    // Constructor
    CUSTOM_RMCS2303();
    
    // Initialization methods
    bool begin(HardwareSerial* serial, long baudRate);
    bool begin(SoftwareSerial* serial, long baudRate);
    
    // Core motor control functions
    bool Speed(byte slaveId, int speed);
    bool Enable_Digital_Mode(byte slaveId, byte direction);
    bool Disable_Digital_Mode(byte slaveId, byte direction);
    bool Brake_Motor(byte slaveId, byte direction);
    
    // Parameter management
    bool WRITE_PARAMETER(byte slaveId, int inp_mode, int pp_gain, int pi_gain, int vf_gain, int lpr, int accel, int speed);
    bool READ_PARAMETER(byte slaveId);
    
    // Feedback functions
    long Speed_Feedback(byte slaveId);
    long Position_Feedback(byte slaveId);
    
    // Utility functions
    void Serial_selection(int mode);
    void Serial0(long baudRate);
    bool isInitialized();
    const char* getLastError();
    
private:
    char _lastError[64];
    void setError(const char* error);
};

// Implementation

CUSTOM_RMCS2303::CUSTOM_RMCS2303() {
    _serial = nullptr;
    _softSerial = nullptr;
    _baudRate = 9600;
    _useHardwareSerial = true;
    _initialized = false;
    _timeout = 1000;
    memset(_rxBuffer, 0, sizeof(_rxBuffer));
    memset(_txBuffer, 0, sizeof(_txBuffer));
    memset(_lastError, 0, sizeof(_lastError));
}

bool CUSTOM_RMCS2303::begin(HardwareSerial* serial, long baudRate) {
    if (serial == nullptr) {
        setError("Invalid serial port");
        return false;
    }
    
    _serial = serial;
    _softSerial = nullptr;
    _baudRate = baudRate;
    _useHardwareSerial = true;
    
    // Initialize the serial port
    _serial->begin(_baudRate);
    delay(100);
    
    // Clear any existing data
    clearSerialBuffer();
    
    _initialized = true;
    setError("OK");
    return true;
}

bool CUSTOM_RMCS2303::begin(SoftwareSerial* serial, long baudRate) {
    if (serial == nullptr) {
        setError("Invalid software serial");
        return false;
    }
    
    _softSerial = serial;
    _serial = nullptr;
    _baudRate = baudRate;
    _useHardwareSerial = false;
    
    // Initialize the software serial port
    _softSerial->begin(_baudRate);
    delay(100);
    
    _initialized = true;
    setError("OK");
    return true;
}

bool CUSTOM_RMCS2303::Speed(byte slaveId, int speed) {
    if (!_initialized) {
        setError("Not initialized");
        return false;
    }
    
    // Send MODBUS command for speed setting
    sendModbusCommand(slaveId, 0x06, 0x0064, speed);
    delay(50);
    
    // Wait for response
    bool success = receiveModbusResponse(slaveId);
    if (!success) {
        setError("Speed command failed");
    }
    
    return success;
}

bool CUSTOM_RMCS2303::Enable_Digital_Mode(byte slaveId, byte direction) {
    if (!_initialized) {
        setError("Not initialized");
        return false;
    }
    
    // Send MODBUS command for digital mode enable
    sendModbusCommand(slaveId, 0x06, 0x0065, direction);
    delay(50);
    
    bool success = receiveModbusResponse(slaveId);
    if (!success) {
        setError("Enable command failed");
    }
    
    return success;
}

bool CUSTOM_RMCS2303::Disable_Digital_Mode(byte slaveId, byte direction) {
    if (!_initialized) {
        setError("Not initialized");
        return false;
    }
    
    // Send MODBUS command for digital mode disable
    sendModbusCommand(slaveId, 0x06, 0x0066, direction);
    delay(50);
    
    bool success = receiveModbusResponse(slaveId);
    if (!success) {
        setError("Disable command failed");
    }
    
    return success;
}

bool CUSTOM_RMCS2303::Brake_Motor(byte slaveId, byte direction) {
    if (!_initialized) {
        setError("Not initialized");
        return false;
    }
    
    // Send MODBUS command for brake
    sendModbusCommand(slaveId, 0x06, 0x0067, direction);
    delay(50);
    
    bool success = receiveModbusResponse(slaveId);
    if (!success) {
        setError("Brake command failed");
    }
    
    return success;
}

long CUSTOM_RMCS2303::Speed_Feedback(byte slaveId) {
    if (!_initialized) {
        setError("Not initialized");
        return 0;
    }
    
    // Send MODBUS read command for speed feedback
    sendModbusCommand(slaveId, 0x03, 0x0064, 1);
    delay(100);
    
    if (receiveModbusResponse(slaveId)) {
        // Parse the response to extract speed value
        // This is a simplified implementation
        return 8000; // Placeholder - implement proper parsing
    }
    
    return 0;
}

bool CUSTOM_RMCS2303::WRITE_PARAMETER(byte slaveId, int inp_mode, int pp_gain, int pi_gain, int vf_gain, int lpr, int accel, int speed) {
    if (!_initialized) {
        setError("Not initialized");
        return false;
    }
    
    bool allSuccess = true;
    
    // Write each parameter with proper delays
    allSuccess &= sendModbusCommand(slaveId, 0x06, 0x0001, inp_mode) && receiveModbusResponse(slaveId);
    delay(100);
    
    allSuccess &= sendModbusCommand(slaveId, 0x06, 0x0002, pp_gain) && receiveModbusResponse(slaveId);
    delay(100);
    
    allSuccess &= sendModbusCommand(slaveId, 0x06, 0x0003, pi_gain) && receiveModbusResponse(slaveId);
    delay(100);
    
    allSuccess &= sendModbusCommand(slaveId, 0x06, 0x0004, vf_gain) && receiveModbusResponse(slaveId);
    delay(100);
    
    allSuccess &= sendModbusCommand(slaveId, 0x06, 0x0005, lpr) && receiveModbusResponse(slaveId);
    delay(100);
    
    allSuccess &= sendModbusCommand(slaveId, 0x06, 0x0006, accel) && receiveModbusResponse(slaveId);
    delay(100);
    
    if (!allSuccess) {
        setError("Parameter write failed");
    }
    
    return allSuccess;
}

bool CUSTOM_RMCS2303::READ_PARAMETER(byte slaveId) {
    if (!_initialized) {
        setError("Not initialized");
        return false;
    }
    
    // Send read command for all parameters
    sendModbusCommand(slaveId, 0x03, 0x0001, 10);  // Read 10 registers
    delay(200);
    
    bool success = receiveModbusResponse(slaveId);
    if (!success) {
        setError("Parameter read failed");
    }
    
    return success;
}

void CUSTOM_RMCS2303::sendModbusCommand(byte slaveId, byte functionCode, int address, int value) {
    // Create MODBUS ASCII command
    sprintf(_txBuffer, ":%02X%02X%04X%04X", slaveId, functionCode, address, value);
    
    // Calculate and append CRC
    int crc = calculateCRC(_txBuffer + 1, strlen(_txBuffer) - 1);
    sprintf(_txBuffer + strlen(_txBuffer), "%02X\r\n", crc);
    
    // Send via appropriate serial port
    if (_useHardwareSerial && _serial) {
        _serial->print(_txBuffer);
        _serial->flush();
    } else if (!_useHardwareSerial && _softSerial) {
        _softSerial->print(_txBuffer);
    }
}

bool CUSTOM_RMCS2303::receiveModbusResponse(byte slaveId) {
    unsigned long startTime = millis();
    int bufferIndex = 0;
    
    // Clear receive buffer
    memset(_rxBuffer, 0, sizeof(_rxBuffer));
    
    // Read response with timeout
    while (millis() - startTime < _timeout && bufferIndex < sizeof(_rxBuffer) - 1) {
        int available = 0;
        
        if (_useHardwareSerial && _serial) {
            available = _serial->available();
            if (available > 0) {
                _rxBuffer[bufferIndex++] = _serial->read();
            }
        } else if (!_useHardwareSerial && _softSerial) {
            available = _softSerial->available();
            if (available > 0) {
                _rxBuffer[bufferIndex++] = _softSerial->read();
            }
        }
        
        // Check for end of message
        if (bufferIndex >= 2 && _rxBuffer[bufferIndex-2] == '\r' && _rxBuffer[bufferIndex-1] == '\n') {
            break;
        }
        
        if (available == 0) {
            delay(1);
        }
    }
    
    // Basic validation - check if we got a response
    return (bufferIndex > 0);
}

void CUSTOM_RMCS2303::clearSerialBuffer() {
    if (_useHardwareSerial && _serial) {
        while (_serial->available()) {
            _serial->read();
            delay(1);
        }
    } else if (!_useHardwareSerial && _softSerial) {
        while (_softSerial->available()) {
            _softSerial->read();
            delay(1);
        }
    }
}

int CUSTOM_RMCS2303::calculateCRC(char* buffer, int length) {
    // Simplified CRC calculation - implement proper MODBUS CRC if needed
    int crc = 0;
    for (int i = 0; i < length; i++) {
        crc ^= buffer[i];
    }
    return crc & 0xFF;
}

void CUSTOM_RMCS2303::Serial_selection(int mode) {
    // Compatibility function - not needed in new implementation
}

void CUSTOM_RMCS2303::Serial0(long baudRate) {
    // Set console serial baud rate
    Serial.begin(baudRate);
}

bool CUSTOM_RMCS2303::isInitialized() {
    return _initialized;
}

const char* CUSTOM_RMCS2303::getLastError() {
    return _lastError;
}

void CUSTOM_RMCS2303::setError(const char* error) {
    strncpy(_lastError, error, sizeof(_lastError) - 1);
    _lastError[sizeof(_lastError) - 1] = '\0';
}

#endif // CUSTOM_RMCS2303_H
