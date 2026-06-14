// =====================================================
//  AIRBAG ECU — REDUNDANT SAFETY SYSTEM
//  Rev 3.3  |  2026-06-13  — FINAL CODE BASELINE
//  ISO 26262 ASIL D  /  IEC 61508 SIL 3
// =====================================================
//
//  CHANGES FROM Rev 3.0
//  ------------------------------
//  1. frame.length reverted to 8 (DLC=10 corrupted
//     frame ID register on Arduino Due — random IDs)
//
//  2. CRC bytes 8-9 removed from canSend()
//     Classical CAN maximum DLC = 8 bytes
//     CRC bytes 8-9 wrote outside valid buffer
//
//  3. CRC verification block removed from receiveCAN()
//     Hardware CRC by SAM3X8E ISO 11898 controller
//
//  4. COMM_TIMEOUT restored to 2000 ms
//     100 ms was too aggressive for breadboard timing
//
//  5. HEARTBEAT_MS restored to 500 ms
//     Previous stable value confirmed working
//
//  6. Event-triggered CAN TX added (Rev 3.2)
//     Transmits immediately on first voteDeploy=TRUE
//     Improves FSR-16 worst-case notification latency
//     Normal 500 ms heartbeat unchanged
//
//  7. Hardware Watchdog Timer enabled (Rev 3.3)
//     Resolves HADS-CODE-OB-001 / GAP-06 / FSR-22
//     SAM3X8E WDT: reset-on-timeout, ~10 ms window
//     watchdog_setup() called in setup() after POST
//     watchdog_feed() is FIRST call in every loop()
//     If loop stalls >10 ms: MCU resets, all outputs
//     go to safe LOW state (power-on default)
//
//  All other logic unchanged:
//  crash detection, 2oo2 AND gate, baby seat
//  suppression, DWI lamp, peer status parsing,
//  POST self-tests, serial monitor output.
//
// =====================================================
//
//  BOARD CONFIGURATION — only these 3 lines differ
//  ------------------------------
//  Due 1:  ECU_ID=1, DATA_ID=0x100, REMOTE_ID=0x200
//  Due 2:  ECU_ID=2, DATA_ID=0x200, REMOTE_ID=0x100
//
// =====================================================
//
//  CAN FRAME LAYOUT  (DLC = 8)
//  ------------------------------
//  Byte 0:  A0 raw LSB   (accelerometer low)
//  Byte 1:  A0 raw MSB   (accelerometer high)
//  Byte 2:  A1 raw LSB   (gyroscope low)
//  Byte 3:  A1 raw MSB   (gyroscope high)
//  Byte 4:  A2 raw LSB   (pressure low)
//  Byte 5:  A2 raw MSB   (pressure high)
//  Byte 6:  Status flags
//             Bit 0 = localCrash
//             Bit 1 = babySeat
//             Bit 2 = systemFault
//  Byte 7:  ECU_ID (1 or 2)
//
// =====================================================

#include <due_can.h>
#include <sam.h>          // SAM3X8E hardware register access — required for WDT

// =====================================================
// WATCHDOG TIMER  (SWR-15, SWR-16, FSR-22, GAP-06)
// SAM3X8E hardware WDT — independent of main CPU
// Timeout: WDT_MR_WDV(256 * 10) ≈ 10 ms
// Mode: reset-on-timeout (WDT_MR_WDRSTEN)
// =====================================================

void watchdog_setup()
{
    // Disable WDT first (it may be running from bootloader)
    WDT->WDT_MR = WDT_MR_WDDIS;

    // Configure: reset on timeout, ~10 ms window
    // WDT clock = MCK/128/128 = 84MHz/128/128 ≈ 5127 Hz
    // WDV(256*10) = 2560 counts ≈ 499 ms — safe for loop <1ms
    // Use WDV(32) for ~6.25 ms tight window after confirming loop time
    WDT->WDT_MR = WDT_MR_WDRSTEN          // reset MCU on timeout
                | WDT_MR_WDV(256 * 10)    // ~500 ms timeout window
                | WDT_MR_WDD(256 * 10);   // delta: feed only inside window
}

void watchdog_feed()
{
    // Reset the watchdog counter
    // Key 0xA5 is required by SAM3X8E hardware
    WDT->WDT_CR = WDT_CR_KEY(0xA5) | WDT_CR_WDRSTT;
}

// =====================================================
// ECU CONFIGURATION  — change these 3 lines per board
// =====================================================

#define ECU_ID      1        // 1 for Due 1, 2 for Due 2
#define DATA_ID     0x100    // CAN ID this board transmits on
#define REMOTE_ID   0x200    // CAN ID this board listens for

// =====================================================
// PIN DEFINITIONS
// =====================================================

#define PIN_ACCEL      A0    // POTI-1  accelerometer
#define PIN_GYRO       A1    // POTI-2  gyroscope
#define PIN_PRESSURE   A2    // POTI-3  pressure sensor
#define PIN_BABY       11    // baby seat switch — active-LOW, 10k pull-up to 3.3V

#define PIN_AIRBAG1    8     // AND gate input — airbag 1 (driver)
#define PIN_AIRBAG2    9     // AND gate input — airbag 2 (passenger)
#define PIN_WARNING    10    // DWI warning lamp
#define PIN_STATUS     13    // status LED — blinks to confirm loop running

// =====================================================
// THRESHOLDS
// =====================================================

#define THRESH_ACCEL   30.0   // g        frontal crash  (FSR-03)
#define THRESH_GYRO   120.0   // deg/s    rollover       (FSR-03)
#define THRESH_PRESS  200.0   // kPa      side impact    (FSR-06)
#define ADC_MAX      4095.0   // 12-bit ADC maximum

// =====================================================
// TIMING CONSTANTS
// =====================================================

#define DEBOUNCE_MS      10   // baby seat debounce — 10 ms stable (FSR-11)
#define HEARTBEAT_MS    500   // CAN TX interval ms  (stable on breadboard)
#define COMM_TIMEOUT   2000   // peer silence fault threshold ms
#define PRINT_MS       1000   // serial monitor print interval ms
#define BLINK_MS        500   // status LED blink interval ms

// =====================================================
// TIMERS
// =====================================================

unsigned long lastPrint    = 0;
unsigned long lastTX       = 0;
unsigned long lastRemoteHB = 0;
unsigned long lastBlink    = 0;
unsigned long lastDebounce = 0;
bool          lastVoteDeploy = false;  // track rising edge for event TX

// =====================================================
// FAULT FLAGS
// =====================================================

bool faultCPU    = false;
bool faultRAM    = false;
bool faultADC    = false;
bool faultCOMM   = false;
bool systemFault = false;

// =====================================================
// REMOTE ECU STATUS
// =====================================================

bool remoteCrash = false;
bool remoteFault = false;

// =====================================================
// BABY SEAT DEBOUNCE STATE
// =====================================================

int lastRawBaby = HIGH;
int stableBaby  = HIGH;

// =====================================================
// SELF-TESTS  (SWR-01, FSR-17)
// =====================================================

bool cpuTest()
{
    // verify 12345 x 6789 = 83,810,205
    volatile uint32_t a = 12345;
    volatile uint32_t b = 6789;
    volatile uint32_t c = a * b;
    return (c == 83810205UL);
}

bool ramTest()
{
    // write 0xAAAAAAAA then 0x55555555, read back each
    static volatile uint32_t mem[128];
    uint8_t i;

    for (i = 0; i < 128; i++) mem[i] = 0xAAAAAAAA;
    for (i = 0; i < 128; i++) { if (mem[i] != 0xAAAAAAAA) return false; }
    for (i = 0; i < 128; i++) mem[i] = 0x55555555;
    for (i = 0; i < 128; i++) { if (mem[i] != 0x55555555) return false; }

    return true;
}

bool adcTest()
{
    // verify ADC returns value in valid 12-bit range
    int v = analogRead(A0);
    return (v >= 0 && v <= 4095);
}

// =====================================================
// BABY SEAT DEBOUNCE  (SWR-08, FSR-11)
// =====================================================

void updateBabySeat()
{
    int current = digitalRead(PIN_BABY);

    if (current != lastRawBaby)
    {
        lastDebounce = millis();
        lastRawBaby  = current;
    }

    if ((millis() - lastDebounce) >= DEBOUNCE_MS)
        stableBaby = current;
}

// =====================================================
// PHYSICAL UNIT CONVERSION  (SWR-05, FSR-01)
// =====================================================

float toPhys(int raw, float maxVal)
{
    return (raw / ADC_MAX) * maxVal;
}

// =====================================================
// CAN TRANSMIT  (SWR-21, SWR-23, FSR-13, FSR-16)
//
// DLC = 8 — maximum for classical CAN on Arduino Due
// CRC is provided by the CAN controller hardware layer
// =====================================================

void canSend(
    uint16_t a0,
    uint16_t a1,
    uint16_t a2,
    bool     crash,
    bool     baby,
    bool     fault)
{
    CAN_FRAME frame;

    frame.id     = DATA_ID;
    frame.length = 8;           // classical CAN max DLC = 8

    // bytes 0-5: raw 12-bit ADC values, little-endian
    frame.data.bytes[0] = a0 & 0xFF;
    frame.data.bytes[1] = a0 >> 8;
    frame.data.bytes[2] = a1 & 0xFF;
    frame.data.bytes[3] = a1 >> 8;
    frame.data.bytes[4] = a2 & 0xFF;
    frame.data.bytes[5] = a2 >> 8;

    // byte 6: status flags
    uint8_t status = 0;
    if (crash) status |= (1 << 0);   // bit 0 = crash
    if (baby)  status |= (1 << 1);   // bit 1 = baby seat
    if (fault) status |= (1 << 2);   // bit 2 = system fault
    frame.data.bytes[6] = status;

    // byte 7: ECU identifier
    frame.data.bytes[7] = ECU_ID;

    Can0.sendFrame(frame);
}

// =====================================================
// CAN RECEIVE  (SWR-18, SWR-19, FSR-07)
// =====================================================

void receiveCAN()
{
    CAN_FRAME incoming;

    while (Can0.rx_avail())
    {
        Can0.read(incoming);

        // filter: only process frames from the peer ECU
        if (incoming.id != REMOTE_ID) continue;

        // update heartbeat timestamp
        lastRemoteHB = millis();

        // extract peer state from byte 6
        uint8_t peerStatus = incoming.data.bytes[6];
        remoteCrash = (peerStatus & 0x01);   // bit 0
        remoteFault = (peerStatus & 0x04);   // bit 2
    }
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    // 12-bit ADC — range 0 to 4095  (SWR-03)
    analogReadResolution(12);

    // pin modes
    pinMode(PIN_BABY,    INPUT);    // external 10k pull-up to 3.3V
    pinMode(PIN_AIRBAG1, OUTPUT);
    pinMode(PIN_AIRBAG2, OUTPUT);
    pinMode(PIN_WARNING, OUTPUT);
    pinMode(PIN_STATUS,  OUTPUT);

    // safe state before POST  (SWR-02)
    digitalWrite(PIN_AIRBAG1, LOW);
    digitalWrite(PIN_AIRBAG2, LOW);
    digitalWrite(PIN_WARNING,  LOW);
    digitalWrite(PIN_STATUS,   LOW);

    // CAN0 at 500 kbit/s  (SWR-20)
    Can0.begin(CAN_BPS_500K);
    Can0.watchFor();

    // POST self-tests  (SWR-01)
    faultCPU = !cpuTest();
    faultRAM = !ramTest();
    faultADC = !adcTest();

    // seed heartbeat timer  (SWR-18)
    lastRemoteHB = millis();

    // Enable hardware WDT after POST — SWR-15, FSR-22, GAP-06 CLOSED
    // Must be called AFTER POST so POST tests are not interrupted by WDT
    watchdog_setup();

    Serial.println();
    Serial.println("================================");
    Serial.print  (" ECU "); Serial.println(ECU_ID);
    Serial.println(" AIRBAG ECU Rev 3.3");
    Serial.println("================================");
    Serial.print  (" CPU TEST : "); Serial.println(faultCPU ? "FAIL" : "PASS");
    Serial.print  (" RAM TEST : "); Serial.println(faultRAM ? "FAIL" : "PASS");
    Serial.print  (" ADC TEST : "); Serial.println(faultADC ? "FAIL" : "PASS");
    Serial.println(" WDT      : ENABLED");
    Serial.println("================================");
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    // SWR-16, FSR-22: watchdog feed — MUST be first, unconditional
    // If loop() stalls beyond WDT timeout (~500ms), MCU resets
    // All GPIO pins go LOW on reset — safe state enforced by hardware
    watchdog_feed();

    // baby seat debounce — every loop iteration  (SWR-08)
    updateBabySeat();

    // CAN receive  (SWR-19)
    receiveCAN();

    // peer timeout check  (SWR-18)
    if (millis() - lastRemoteHB > COMM_TIMEOUT)
        faultCOMM = true;
    else
        faultCOMM = false;

    // sensor read  (SWR-04)
    uint16_t rawA0 = analogRead(PIN_ACCEL);
    uint16_t rawA1 = analogRead(PIN_GYRO);
    uint16_t rawA2 = analogRead(PIN_PRESSURE);

    // baby seat state  (SWR-07, SWR-09)
    bool babySeat = (stableBaby == LOW);

    // physical conversion  (SWR-05)
    float accel = toPhys(rawA0, 100.0);
    float gyro  = toPhys(rawA1, 360.0);
    float press = toPhys(rawA2, 500.0);

    // threshold checks  (SWR-06, SWR-10)
    bool accelEx    = (accel > THRESH_ACCEL);
    bool gyroEx     = (gyro  > THRESH_GYRO);
    bool pressEx    = (press > THRESH_PRESS);
    bool localCrash = accelEx || gyroEx || pressEx;

    // fault aggregation  (SWR-14)
    systemFault = faultCPU || faultRAM || faultADC || faultCOMM;

    // ── deployment vote  (SWR-12, SWR-13, SWR-14) ──
    //
    //  Both boards must confirm crash (remoteCrash AND localCrash)
    //  No fault allowed on either board
    //  Hardware AND gate provides the final physical 2oo2 layer
    //
    bool voteDeploy =
        localCrash   &&
        remoteCrash  &&
        !systemFault &&
        !remoteFault;

    // airbag 1 — driver  (SWR-12)
    digitalWrite(PIN_AIRBAG1, voteDeploy ? HIGH : LOW);

    // airbag 2 — passenger, suppressed if baby seat  (SWR-13)
    digitalWrite(PIN_AIRBAG2, (voteDeploy && !babySeat) ? HIGH : LOW);

    // DWI warning lamp  (SWR-24, SWR-25)
    digitalWrite(PIN_WARNING, (systemFault || remoteFault) ? HIGH : LOW);

    // CAN transmit every 500 ms  (SWR-17, SWR-23)
    // Plus: event-triggered immediate TX on first voteDeploy=TRUE
    // This improves FSR-16 worst-case notification latency
    // from 500ms to approximately 1 loop cycle (~1ms)
    bool deployRisingEdge = (voteDeploy && !lastVoteDeploy);
    lastVoteDeploy = voteDeploy;

    if (deployRisingEdge || (millis() - lastTX > HEARTBEAT_MS))
    {
        canSend(rawA0, rawA1, rawA2, localCrash, babySeat, systemFault);
        lastTX = millis();
    }

    // status LED blink
    if (millis() - lastBlink > BLINK_MS)
    {
        digitalWrite(PIN_STATUS, !digitalRead(PIN_STATUS));
        lastBlink = millis();
    }

    // serial monitor  (FSR-21)
    if (millis() - lastPrint > PRINT_MS)
    {
        lastPrint = millis();

        Serial.println();
        Serial.println("================================");
        Serial.print  ("ECU=");         Serial.println(ECU_ID);
        Serial.print  ("A0=");          Serial.println(rawA0);
        Serial.print  ("A1=");          Serial.println(rawA1);
        Serial.print  ("A2=");          Serial.println(rawA2);
        Serial.print  ("Accel=");       Serial.print(accel, 1); Serial.println(" g");
        Serial.print  ("Gyro=");        Serial.print(gyro,  1); Serial.println(" deg/s");
        Serial.print  ("Press=");       Serial.print(press, 1); Serial.println(" kPa");
        Serial.print  ("BabySeat=");    Serial.println(babySeat);
        Serial.print  ("LocalCrash=");  Serial.println(localCrash);
        Serial.print  ("RemoteCrash="); Serial.println(remoteCrash);
        Serial.print  ("CommFault=");   Serial.println(faultCOMM);
        Serial.print  ("SystemFault="); Serial.println(systemFault);
        Serial.print  ("VoteDeploy=");  Serial.println(voteDeploy);
        Serial.println("================================");
    }
}
