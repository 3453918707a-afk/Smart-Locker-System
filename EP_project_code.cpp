#include "mbed.h"
#include "platform/mbed_thread.h"
#include <cstdio>

// Fall detection
// SPI1 Interface Pin Definitions
#define SPI1_MOSI A6     // Master Out Slave In pin
#define SPI1_MISO A5     // Master In Slave Out pin
#define SPI1_SCLK A4     // Serial Clock pin
#define SPI1_CS D12      // Chip Select pin

// Mathematical Constant
#define PI 3.14          // Value of Pi for angle calculation

// SPI Interface Setup for Accelerometer or Other SPI Device
SPI spi(SPI1_MOSI, SPI1_MISO, SPI1_SCLK);  // Initialize SPI communication
DigitalOut cs(SPI1_CS, 1);                 // Initialize Chip Select (active low)

// Data Buffers and Variables
char buffer[6];          // Buffer to store raw SPI data
int16_t data[3];         // Array to store converted sensor data (e.g., x, y, z axes)
float x, y, z;           // Acceleration values on x, y, z axes
int acceleration;        // Overall acceleration magnitude
int angle;               // Calculated angle (e.g., tilt)

// Temperature Sensor Setup (TMP102 over I2C)
#define I2CSDA D0        // I2C Data pin
#define I2CSCL D1        // I2C Clock pin
I2C TMP102(I2CSDA, I2CSCL); // Initialize I2C interface for TMP102 sensor

const int TMP102Address = 0x90;         // I2C address of TMP102 temperature sensor
char Config_Reg_TMP102[3];              // Buffer to hold configuration register values
char Temp_Reg[2];                       // Buffer to hold temperature register values
float Temperature;                      // Variable to store the temperature in Celsius

// LED Indicators
DigitalOut LR_LowTemperature(D11);      // LED to indicate low temperature
DigitalOut LR_HighTemperature(D9);      // LED to indicate high temperature
DigitalOut LR_Fall(D6);                 // LED to indicate a fall detection event

// ADXL345 initialization
void ADXL345_SPI_Initialise() {
    // Configure SPI frequency and data format
    spi.frequency(2 * 1000 * 1000);  // Set SPI speed to 2 MHz
    spi.format(8, 3);                // 8-bit frame, SPI mode 3 (CPOL=1, CPHA=1)

    // Write to DATA_FORMAT register (0x31) to set range and format
    cs = 0;
    spi.write(0x31);    // Address of DATA_FORMAT register
    spi.write(0x03);    // ±16g range, right justified
    cs = 1;

    // Write to POWER_CTL register (0x2D) to enable measurement mode
    char reg_power_ctl = 0x2D;
    char value_measure = 0x08;

    cs = 0;
    spi.write(reg_power_ctl);  // Address of POWER_CTL register
    spi.write(value_measure);  // Set measurement mode
    cs = 1;
}


void ADXL345_axis() {
    // Initialize the ADXL345 sensor via SPI
    ADXL345_SPI_Initialise();

    // Wait for sensor to stabilize
    thread_sleep_for(200);

    // Begin SPI communication to read acceleration data starting from register 0x32
    cs = 0;
    uint8_t read_command = 0x80 | 0x40 | 0x32;  // Multi-byte read from 0x32
    spi.write(read_command);

    // Read 6 bytes of acceleration data into buffer
    for (int idx = 0; idx < 6; idx++) {
        buffer[idx] = spi.write(0x00);  // Send dummy byte to receive data
    }
    cs = 1;

    // Combine high and low bytes for each axis
    int16_t x_raw = (buffer[1] << 8) | buffer[0];
    int16_t y_raw = (buffer[3] << 8) | buffer[2];
    int16_t z_raw = (buffer[5] << 8) | buffer[4];

    // Convert raw values to physical units (scaling factors are based on sensor config)
    x = x_raw * 0.5f;
    y = y_raw * 0.5f;
    z = z_raw * 0.005f;

    // Print the calculated acceleration values
    printf("x=%.2fG, y=%.2fG, z=%.2fG\n", x, y, z);
}


void Calculate_Acceleration() {
    ADXL345_axis();
    acceleration = (fabs(x) > 0.5 || fabs(y) > 0.5 || fabs(z) > 0.5) ? 1 : 0;// Set acceleration flag based on the condition
}

void Calculate_Angle() {
    // Update acceleration data from the sensor
    ADXL345_axis();

    // Small value added to denominator to avoid division by zero
    const float epsilon = 0.0001f;

    // Calculate angles in degrees for each axis relative to the others
    float denominator_x = sqrt((y * y) + (z * z) + epsilon);
    float denominator_y = sqrt((x * x) + (z * z) + epsilon);
    float denominator_z = sqrt((x * x) + (y * y) + epsilon);

    float angle_x = atan(x / denominator_x) * (180.0f / PI);
    float angle_y = atan(y / denominator_y) * (180.0f / PI);
    float angle_z = atan(z / denominator_z) * (180.0f / PI);

    // Threshold for significant tilt angle
    const float angle_threshold = 45.0f;

    // Determine if any axis angle exceeds the threshold
    bool x_exceeds = fabs(angle_x) > angle_threshold;
    bool y_exceeds = fabs(angle_y) > angle_threshold;
    bool z_exceeds = fabs(angle_z) > angle_threshold;

    if (x_exceeds || y_exceeds || z_exceeds) {
        angle = 1;
    } else {
        angle = 0;
    }
}


void Fall() {
    // Update acceleration and angle status from sensor data
    Calculate_Acceleration();
    Calculate_Angle();

    // Determine fall condition based on both acceleration and angle flags
    if (acceleration == 1 && angle == 1) {
        LR_Fall = 1;
    } else {
        LR_Fall = 0;
    }
}


void ConfigureTMP102() {
    // Prepare configuration bytes for TMP102 sensor
    Config_Reg_TMP102[0] = 0x01;  // Configuration register address
    Config_Reg_TMP102[1] = 0x60;  // Configuration high byte
    Config_Reg_TMP102[2] = 0xA0;  // Configuration low byte

    // Write 3 bytes to TMP102 configuration register (address shifted left by 1 for write)
    TMP102.write(TMP102Address << 1, Config_Reg_TMP102, 3);

    // Set pointer register to temperature register (0x00) before reading temperature
    Config_Reg_TMP102[0] = 0x00;
    TMP102.write(TMP102Address, Config_Reg_TMP102, 1);
}


void ReadTemperature() {
    // Initialize TMP102 sensor configuration
    ConfigureTMP102();
    
    // Wait for sensor to stabilize
    thread_sleep_for(1000);
    
    // Read 2 bytes from temperature register of TMP102
    TMP102.read(TMP102Address, Temp_Reg, 2);
    
    // Combine the two bytes into a 12-bit signed temperature value
    int16_t rawTemp = ((Temp_Reg[0] << 8) | Temp_Reg[1]) >> 4;
    
    // Sign extend if negative (12-bit two's complement)
    if (rawTemp & 0x800) {
        rawTemp |= 0xF000;
    }
    
    // Convert raw value to Celsius degrees (each bit = 0.0625°C)
    Temperature = rawTemp * 0.0625f;
    
    // Additional read from TMP102 (possibly to reset or confirm)
    TMP102.read((TMP102Address << 1) | 0x01, Temp_Reg, 2);
    
    // Output temperature to console
    printf("Temperature = %.2f\n\r", Temperature);
}


void Temperature_Monitoring() {
    // Pause for 1 second before reading temperature
    thread_sleep_for(1000);

    // Update current temperature reading
    ReadTemperature();

    // If temperature exceeds 29°C, trigger high temperature warning LEDs
    if (Temperature > 29.0f) {
        LR_LowTemperature = 0;  // Ensure low temp indicator is off

        // Blink high temp indicator 5 times with 500ms intervals
        for (int i = 0; i < 5; ++i) {
            LR_HighTemperature = 1;    // Turn on high temp LED
            thread_sleep_for(500);
            LR_HighTemperature = 0;    // Turn off high temp LED
            thread_sleep_for(500);
        }
    }
    // If temperature is 28°C or below, activate low temperature warning LEDs
    else if (Temperature <= 28.0f) {
        LR_HighTemperature = 0;  // Make sure high temp LED is off

        // Blink low temp indicator 5 times with 500ms intervals
        for (int i = 0; i < 5; ++i) {
            LR_LowTemperature = 1;    // Turn on low temp LED
            thread_sleep_for(500);
            LR_LowTemperature = 0;    // Turn off low temp LED
            thread_sleep_for(500);
        }
    }
    // For temperatures between 28 and 29, turn off both LEDs
    else {
        LR_LowTemperature = 0;
        LR_HighTemperature = 0;
    }
}


#define BTN1 A1
#define BTN2 A0
#define LED_Unlock D5   // The LED that lights up when the unlocking is successful
#define LED_Error  D13  // The LED that flashes when there is an input error or a lock
// Set the two buttons as interrupt inputs and enable the pull-up resistor
InterruptIn button1(BTN1, PullUp);
InterruptIn button2(BTN2, PullUp);

DigitalOut unlockLED(LED_Unlock);
DigitalOut errorLED(LED_Error);

//Event queues and threads (to avoid blocking operations in interrupts)
EventQueue queue;             // Create an event queue
Thread queue_thread;          // This queue schedules events in independent threads

//Password System Settings
const int PASSWORD_LENGTH = 3;                    // Password length
char correct_password[PASSWORD_LENGTH] = {'1', '2', '1'}; // Correct password
char user_input[PASSWORD_LENGTH];                 // User input cache
char last_input[PASSWORD_LENGTH + 1] = {};        // Used to record the last input (including log printing)
int input_index = 0;                              // The number of digits currently entered
int attempt_counter = 0;                          // Total attempt counter
int failed_attempts = 0;                          // Number of consecutive failures
#define MAX_ATTEMPTS 3                            // Maximum limit of failure times


bool is_authenticated = false;                    // Whether it been unlocked

bool detection_active = false;                    // Whether to enable the monitoring function (true after unlocking)

//Utility Function: Reset Input State
void reset_input() {
    input_index = 0;
    for (int i = 0; i < PASSWORD_LENGTH; i++) {
        user_input[i] = 0;
        last_input[i] = 0;
    }
}

// Core function: Verify whether the password entered by the user is correct
void check_password() {
    attempt_counter++;   
    bool match = true;   

    // Compare the user input and the correct password bit by bit
    for (int i = 0; i < PASSWORD_LENGTH; i++) {
        if (user_input[i] != correct_password[i]) {
            match = false;
            break;
        }
    }

    last_input[PASSWORD_LENGTH] = '\0';  // Add a string terminator to facilitate log printing

    if (match) {
        // The password matching was successful
        is_authenticated = true;
        detection_active = true;   // Activate the system function
        unlockLED = 1;             // Light up the unlock LED
        errorLED = 0;

        queue.call([=] {
            printf("[Log] Attempt %d: Input = %s - Success\n", attempt_counter, last_input);
            printf("Successful unlock! The system will begin monitoring...\n");
        });

    } else {
        // Password incorrect
        failed_attempts++;
        unlockLED = 0;
        errorLED = 1;
        ThisThread::sleep_for(300ms); // Brief red light prompt
        errorLED = 0;

        // Log output failure information
        queue.call([=] {
            printf("[Log] Attempt %d: Input = %s - Failure (%d/%d)\n",
                   attempt_counter, last_input, failed_attempts, MAX_ATTEMPTS);
        });


    }

    reset_input(); // Clear the input cache after each verification
}

// Handle input from button BTN1 (corresponds to digit '1')
void handle_btn1() {
    // Exit early if user is already authenticated
    if (is_authenticated) return;

    // Process input only if there's room left
    bool can_input = (input_index < PASSWORD_LENGTH);
    if (can_input) {
        char digit = '1';
        user_input[input_index] = digit;
        last_input[input_index] = digit;
        input_index += 1;

        // Automatically check password once required length is reached
        bool input_complete = (input_index == PASSWORD_LENGTH);
        if (input_complete) {
            check_password();
        }
    }
}


// Handle input from button BTN2 (represents digit '2')
void handle_btn2() {
    // Skip input handling if already authenticated
    if (is_authenticated) return;

    // Allow input only if maximum length not yet reached
    if (input_index >= PASSWORD_LENGTH) return;

    char digit = '2';  // Define the digit to input
    user_input[input_index] = digit;
    last_input[input_index] = digit;
    input_index += 1;

    // Trigger password check automatically when input is complete
    if (input_index >= PASSWORD_LENGTH) {
        check_password();
    }
}


void isr_btn1() { queue.call(handle_btn1); }  // Call the processing function in the main thread
void isr_btn2() { queue.call(handle_btn2); }



void EnableShutdownMode() {
    // Prepare the command to enable shutdown mode on TMP102
    char shutdown_command[3] = {0x01, 0x61, 0xA0};  

    // Send the shutdown command to the TMP102 via I2C
    TMP102.write(TMP102Address << 1, shutdown_command, 3);

    // Inform the user that the device has entered shutdown mode
    printf("Device is now in Shutdown Mode. Please measure current with ammeter.\n");
}


int main() {

    printf("The system is locked, please enter code: BTN1-BTN2-BTN1\n");

    queue_thread.start(callback(&queue, &EventQueue::dispatch_forever)); // Start the event queue thread

    ThisThread::sleep_for(1000ms); // Delay ensures the completion of hardware initialization

    // Set the button interrupt response (triggered by the falling edge)
    button1.fall(&isr_btn1);
    button2.fall(&isr_btn2);

    reset_input();  // Initialize the input cache

    while (true) {
        if (detection_active) {
        printf("[Monitoring] The system is monitoring...\n");
        thread_sleep_for(200);
        Fall();
        Temperature_Monitoring();// If unlocked, perform the system's monitoring task (placeholder here)
        ThisThread::sleep_for(500ms);
        } else {
            EnableShutdownMode();
            ThisThread::sleep_for(300000ms);
        }
    }
}