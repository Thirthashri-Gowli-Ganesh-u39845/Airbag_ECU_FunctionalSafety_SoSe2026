#include <due_can.h>

// =====================================================
// ECU CONFIGURATION
// =====================================================

#define ECU_ID      1
#define DATA_ID     0x100
#define REMOTE_ID   0x200

// =====================================================
// INPUTS
// =====================================================

#define PIN_ACCEL      A0
#define PIN_GYRO       A1
#define PIN_PRESSURE   A2
#define PIN_BABY       11

// =====================================================
// OUTPUTS
// =====================================================

#define PIN_AIRBAG1    8
#define PIN_AIRBAG2    9
#define PIN_WARNING    10
#define PIN_STATUS     13

// =====================================================
// THRESHOLDS
// =====================================================

#define THRESH_ACCEL   30.0
#define THRESH_GYRO    120.0
#define THRESH_PRESS   200.0

#define ADC_MAX        4095.0

// =====================================================
// TIMERS
// =====================================================

unsigned long lastPrint = 0;
unsigned long lastHB = 0;
unsigned long lastRemoteHB = 0;
unsigned long lastBlink = 0;

// =====================================================
// FAULTS
// =====================================================

bool faultCPU = false;
bool faultRAM = false;
bool faultADC = false;
bool faultCOMM = false;
bool systemFault = false;

// =====================================================
// REMOTE ECU STATUS
// =====================================================

bool remoteCrash = false;
bool remoteFault = false;

// =====================================================
// SELF TESTS
// =====================================================

bool cpuTest()
{
    volatile uint32_t a = 100;
    volatile uint32_t b = 200;
    return ((a + b) == 300);
}

bool ramTest()
{
    uint32_t test = 0xAAAAAAAA;
    return (test == 0xAAAAAAAA);
}

bool adcTest()
{
    int v = analogRead(A0);
    return (v >= 0 && v <= 4095);
}

// =====================================================
// SEND HEARTBEAT
// =====================================================



// =====================================================
// SEND SENSOR DATA
// =====================================================

void sendData(
    uint16_t a0,
    uint16_t a1,
    uint16_t a2,
    bool crash,
    bool baby,
    bool fault)
{
    CAN_FRAME frame;

    frame.id = DATA_ID;
    frame.length = 8;

    frame.data.bytes[0] = a0 & 0xFF;
    frame.data.bytes[1] = a0 >> 8;

    frame.data.bytes[2] = a1 & 0xFF;
    frame.data.bytes[3] = a1 >> 8;

    frame.data.bytes[4] = a2 & 0xFF;
    frame.data.bytes[5] = a2 >> 8;

    uint8_t status = 0;

    if(crash) status |= (1 << 0);
    if(baby)  status |= (1 << 1);
    if(fault) status |= (1 << 2);

    frame.data.bytes[6] = status;
    frame.data.bytes[7] = ECU_ID;

    Can0.sendFrame(frame);
}

// =====================================================
// RECEIVE CAN
// =====================================================

void receiveCAN()
{
    CAN_FRAME incoming;

    while (Can0.rx_avail())
    {
        Can0.read(incoming);

        Serial.print("RX ID = 0x");
        Serial.println(incoming.id, HEX);

        if(incoming.id == REMOTE_ID)
        {
            lastRemoteHB = millis();

            uint8_t status =
                incoming.data.bytes[6];

            remoteCrash =
                (status & 0x01);

            remoteFault =
                (status & 0x04);
        }
    }
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    analogReadResolution(12);

    pinMode(PIN_BABY, INPUT);

    pinMode(PIN_AIRBAG1, OUTPUT);
    pinMode(PIN_AIRBAG2, OUTPUT);
    pinMode(PIN_WARNING, OUTPUT);
    pinMode(PIN_STATUS, OUTPUT);

    Can0.begin(CAN_BPS_500K);
    Can0.watchFor();

    faultCPU = !cpuTest();
    faultRAM = !ramTest();
    faultADC = !adcTest();

    lastRemoteHB = millis();

    Serial.println();
    Serial.println("================================");
    Serial.print(" ECU ");
    Serial.println(ECU_ID);
    Serial.println(" STARTED");
    Serial.println("================================");
}

void loop()
{
    receiveCAN();

    // -----------------------------------------
    // HEARTBEAT TIMEOUT
    // -----------------------------------------

    if(millis() - lastRemoteHB > 2000)
        faultCOMM = true;
    else
        faultCOMM = false;

    // -----------------------------------------
    // READ SENSORS
    // -----------------------------------------

    uint16_t rawA0 = analogRead(A0);
    uint16_t rawA1 = analogRead(A1);
    uint16_t rawA2 = analogRead(A2);

    bool babySeat =
        (digitalRead(PIN_BABY) == LOW);

    float accel =
        (rawA0 / ADC_MAX) * 100.0;

    float gyro =
        (rawA1 / ADC_MAX) * 360.0;

    float press =
        (rawA2 / ADC_MAX) * 500.0;

    bool localCrash =
        (accel > THRESH_ACCEL) ||
        (gyro > THRESH_GYRO) ||
        (press > THRESH_PRESS);

    systemFault =
        faultCPU ||
        faultRAM ||
        faultADC ||
        faultCOMM;

    // -----------------------------------------
    // LOCAL VOTE ONLY
    // AND GATE DOES FINAL 2oo2
    // -----------------------------------------

    bool voteDeploy =
        localCrash &&
        remoteCrash &&
        !systemFault &&
        !remoteFault;

    digitalWrite(
        PIN_AIRBAG1,
        voteDeploy
    );

    digitalWrite(
        PIN_AIRBAG2,
        voteDeploy &&
        !babySeat
    );

    digitalWrite(
        PIN_WARNING,
        systemFault
    );

    // -----------------------------------------
    // HEARTBEAT TX
    // -----------------------------------------

    if(millis() - lastHB > 500)
    {
        sendData(
            rawA0,
            rawA1,
            rawA2,
            localCrash,
            babySeat,
            systemFault
        );

        lastHB = millis();
    }

    // -----------------------------------------
    // STATUS LED
    // -----------------------------------------

    if(millis() - lastBlink > 500)
    {
        digitalWrite(
            PIN_STATUS,
            !digitalRead(PIN_STATUS)
        );

        lastBlink = millis();
    }

    // -----------------------------------------
    // SERIAL OUTPUT
    // -----------------------------------------

    if(millis() - lastPrint > 1000)
    {
        lastPrint = millis();

        Serial.println();
        Serial.println("================================");

        Serial.print("ECU=");
        Serial.println(ECU_ID);

        Serial.print("A0=");
        Serial.println(rawA0);

        Serial.print("A1=");
        Serial.println(rawA1);

        Serial.print("A2=");
        Serial.println(rawA2);

        Serial.print("Baby=");
        Serial.println(babySeat);

        Serial.print("Local Crash=");
        Serial.println(localCrash);

        Serial.print("Remote Crash=");
        Serial.println(remoteCrash);

        Serial.print("Comm Fault=");
        Serial.println(faultCOMM);

        Serial.print("System Fault=");
        Serial.println(systemFault);

        Serial.print("Vote Deploy=");
        Serial.println(voteDeploy);

        Serial.println("================================");
    }
}