#include <Wire.h>

// I2C Configuration
#define I2C_SDA 21
#define I2C_SCL 22

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("\n\nESP32 I2C Scanner");
  Serial.println("==================");
  
  // Initialize I2C with specific pins
  Wire.begin(I2C_SDA, I2C_SCL);
  
  delay(1000);
}

void loop() {
  scanI2CDevices();
  delay(5000); // Scan every 5 seconds
}

void scanI2CDevices() {
  byte error, address;
  int devicesFound = 0;
  
  Serial.println("\nScanning I2C bus...");
  Serial.println("   | 0 1 2 3 4 5 6 7 8 9 A B C D E F");
  Serial.println("-------------------------------------");
  
  for(address = 1; address < 127; address++) {
    // Print row header every 16 addresses
    if (address % 16 == 1) {
      Serial.print((address / 16) * 16, HEX);
      Serial.print(" | ");
    }
    
    // Try to communicate with the device
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print(address, HEX);
      Serial.print(" ");
      devicesFound++;
      
      // Print device info if known
      printDeviceInfo(address);
    } else if (error == 4) {
      Serial.print("XX "); // Other error
    } else {
      Serial.print("-- "); // No device
    }
    
    // New line every 16 addresses
    if (address % 16 == 0) {
      Serial.println();
    }
  }
  
  Serial.println("\n-------------------------------------");
  Serial.print("Devices found: ");
  Serial.println(devicesFound);
  
  if (devicesFound == 0) {
    Serial.println("No I2C devices found!");
    Serial.println("Check wiring: SDA->GPIO"+String(I2C_SDA)+", SCL->GPIO"+String(I2C_SCL));
  }
}

void printDeviceInfo(byte address) {
  Serial.println();
  Serial.print("  Device at 0x");
  Serial.print(address, HEX);
  Serial.print(" (");
  Serial.print(address);
  Serial.print(") - ");
  
  // Common I2C device addresses
  switch(address) {
    case 0x08: Serial.println("Unknown/Generic"); break;
    case 0x0A: Serial.println("Unknown/Generic"); break;
    case 0x0C: Serial.println("Unknown/Generic"); break;
    case 0x0E: Serial.println("Unknown/Generic"); break;
    case 0x18: Serial.println("BMP180/BMP280"); break;
    case 0x19: Serial.println("LIS3DH/LSM303"); break;
    case 0x1D: Serial.println("ADXL345"); break;
    case 0x1E: Serial.println("HMC5883L"); break;
    case 0x20: Serial.println("MCP23017/MCP23008"); break;
    case 0x21: Serial.println("MCP23017/MCP23008"); break;
    case 0x27: Serial.println("LCD PCF8574"); break;
    case 0x38: Serial.println("FT6206 Touch"); break;
    case 0x39: Serial.println("TSL2561"); break;
    case 0x3C: Serial.println("OLED SSD1306"); break;
    case 0x3D: Serial.println("OLED SSD1306"); break;
    case 0x40: Serial.println("SHT21/HTU21D"); break;
    case 0x48: Serial.println("ADS1115/PCF8591"); break;
    case 0x50: Serial.println("EEPROM AT24C32"); break;
    case 0x53: Serial.println("ADXL345"); break;
    case 0x68: Serial.println("DS3231/MPU6050"); break;
    case 0x76: Serial.println("BMP280/BME280"); break;
    case 0x77: Serial.println("BMP180/BME280"); break;
    default: Serial.println("Unknown device"); break;
  }
}

// Alternative simple scanner function
void quickScan() {
  byte error, address;
  int devicesFound = 0;
  
  Serial.println("\nQuick I2C Scan:");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Found device at: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devicesFound++;
    }
  }
  
  if (devicesFound == 0) {
    Serial.println("No I2C devices found");
  }
}
