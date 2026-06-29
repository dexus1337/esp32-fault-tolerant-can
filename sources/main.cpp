#include <Arduino.h>
#include <SPI.h>

// --- Pin Definitions (Adjust to match your PCB layout) ---
#define MCP_INT_PIN   10  // Top signal pin on header -> GPIO 10
#define MCP_SCK_PIN   11  // -> GPIO 11
#define MCP_MOSI_PIN  12  // -> GPIO 12
#define MCP_MISO_PIN  13  // -> GPIO 13
#define MCP_CS_PIN    14  // Bottom signal pin on header -> GPIO 14
// 5V connects to ESP32 5V pin
// GND connects to ESP32 GND pin

// --- MCP2515 Direct Register Commands ---
#define CMD_READ_RX_BUF0  0x90
#define CMD_RX_STATUS     0xB0
#define CMD_BIT_MODIFY    0x05
#define REG_CANINTF       0x2C

// FreeRTOS Task and Queue Handles
TaskHandle_t CanTaskHandle = NULL;
QueueHandle_t CanRxQueue = NULL;

// Simple structure to pass data out of the critical task quickly
struct CanMessage 
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
};

// 1. Extreme Low-Latency ISR
void IRAM_ATTR mcp2515_InterruptHandler() 
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Unblock the high-priority handling task immediately
    vTaskNotifyGiveFromISR(CanTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) 
    {
        portYIELD_FROM_ISR();
    }
}

// 2. High-Priority Dedicated Processing Task
void CanProcessingTask(void *pvParameters) 
{
    CanMessage msg;
    
    // Initialize SPI Settings for ESP32-S3 
    // MCP2515 supports up to 10 MHz SPI clock max
    SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE0);

    while (1) 
    {
        // Wait indefinitely for a notification from the ISR (Zero CPU polling overhead)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Access SPI bus exclusively 
        SPI.beginTransaction(spiSettings);
        digitalWrite(MCP_CS_PIN, LOW);
        
        // Example: Read RX Buffer 0 shortcut instruction
        SPI.transfer(CMD_READ_RX_BUF0);
        uint8_t sidh = SPI.transfer(0x00);
        uint8_t sidl = SPI.transfer(0x00);
        uint8_t eid8 = SPI.transfer(0x00);
        uint8_t eid0 = SPI.transfer(0x00);
        uint8_t dlc  = SPI.transfer(0x00);
        
        msg.dlc = dlc & 0x0F;
        if (msg.dlc > 8) msg.dlc = 8;
        
        for (int i = 0; i < 8; i++) 
        {
            msg.data[i] = SPI.transfer(0x00);
        }
        
        digitalWrite(MCP_CS_PIN, HIGH);
        
        if (sidl & 0x08) // Extended ID
        {
            msg.id = ((uint32_t)sidh << 21) | 
                     (((uint32_t)sidl & 0xE0) << 13) | 
                     (((uint32_t)sidl & 0x03) << 16) | 
                     ((uint32_t)eid8 << 8) | 
                     (uint32_t)eid0;
        }
        else // Standard ID
        {
            msg.id = ((uint32_t)sidh << 3) | (sidl >> 5);
        }

        // Clear RX0IF in CANINTF register to enable next interrupt
        digitalWrite(MCP_CS_PIN, LOW);
        SPI.transfer(CMD_BIT_MODIFY);
        SPI.transfer(REG_CANINTF);
        SPI.transfer(0x01); // Mask: RX0IF (Bit 0)
        SPI.transfer(0x00); // Value: 0
        digitalWrite(MCP_CS_PIN, HIGH);
        
        SPI.endTransaction();

        // Send message to a larger ring-buffer queue for processing/logging 
        xQueueSendToBack(CanRxQueue, &msg, 0);
    }
}

void setup() 
{
    Serial.begin(115200);
    
    pinMode(MCP_CS_PIN, OUTPUT);
    digitalWrite(MCP_CS_PIN, HIGH);
    
    pinMode(MCP_INT_PIN, INPUT_PULLUP);
    
    // Create a deep thread-safe queue to ensure no dropped messages during high bursts
    CanRxQueue = xQueueCreate(64, sizeof(CanMessage));
    
    // Initialize Hardware SPI with custom pins mapping to the ascending header order
    SPI.begin(MCP_SCK_PIN, MCP_MISO_PIN, MCP_MOSI_PIN, MCP_CS_PIN); 

    // --- MCP2515 Register Addresses & SPI Commands ---
    const uint8_t CMD_RESET       = 0xC0;
    const uint8_t CMD_WRITE       = 0x02;
    
    const uint8_t REG_CNF3        = 0x28;
    const uint8_t REG_CNF2        = 0x29;
    const uint8_t REG_CNF1        = 0x2A;
    const uint8_t REG_CANCTRL     = 0x0F;
    const uint8_t REG_RXB0CTRL    = 0x60;
    
    SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE0);
    SPI.beginTransaction(spiSettings);

    // 1. Reset the MCP2515 to enter Configuration Mode automatically
    digitalWrite(MCP_CS_PIN, LOW);
    SPI.transfer(CMD_RESET);
    digitalWrite(MCP_CS_PIN, HIGH);
    delay(10); // Give the oscillator time to stabilize

    // 2. Write Configuration Registers for 83.333 kbps @ 8 MHz
    digitalWrite(MCP_CS_PIN, LOW);
    SPI.transfer(CMD_WRITE);
    SPI.transfer(REG_CNF3);
    SPI.transfer(0x07); // CNF3: PhaseSeg2 = 8 TQ
    SPI.transfer(0xBE); // CNF2: BTLMODE = 1, PropSeg = 7 TQ, PhaseSeg1 = 8 TQ
    SPI.transfer(0x01); // CNF1: BRP = 1 (gives 24 TQ total per bit)
    digitalWrite(MCP_CS_PIN, HIGH);

    // 3. Open RX Buffer 0 to accept all messages (Disable masks/filters)
    digitalWrite(MCP_CS_PIN, LOW);
    SPI.transfer(CMD_WRITE);
    SPI.transfer(REG_RXB0CTRL);
    SPI.transfer(0x60); 
    digitalWrite(MCP_CS_PIN, HIGH);

    // 4. Switch to Normal Mode
    digitalWrite(MCP_CS_PIN, LOW);
    SPI.transfer(CMD_WRITE);
    SPI.transfer(REG_CANCTRL);
    SPI.transfer(0x00); // CANCTRL: Normal Mode, CLKOUT disabled
    digitalWrite(MCP_CS_PIN, HIGH);

    SPI.endTransaction();

    // Create the dedicated CAN handling task on Core 1 with maximum priority
    xTaskCreatePinnedToCore
    (
        CanProcessingTask,
        "CAN_Processor",
        4096,
        NULL,
        configMAX_PRIORITIES - 1, 
        &CanTaskHandle,
        1 
    );

    // Attach the hardware interrupt to the falling edge of the INT line
    attachInterrupt(digitalPinToInterrupt(MCP_INT_PIN), mcp2515_InterruptHandler, FALLING);
}

void loop() 
{
    CanMessage currentMsg;
    // Process the data down here safely without slowing down the critical hardware SPI loop
    if (xQueueReceive(CanRxQueue, &currentMsg, portMAX_DELAY)) 
    {
        Serial.printf("CAN MSG BEGIN - ID %x : DATA", currentMsg.id);
        for (int i = 0; i < currentMsg.dlc; i++) 
        {
            Serial.printf(" %02x", currentMsg.data[i]);
        }
        Serial.println(" - END");
    }
}