#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "eecs388_lib.h"

#define BUF_SIZE        (80)

#define SERVO_PULSE_MAX 2400    /* 2400 us */
#define SERVO_PULSE_MIN 544     /* 544 us */
#define SERVO_PERIOD    20000   /* 20000 us (20ms) */
#define SERVO_MIN_US    544
#define SERVO_MAX_US    2400
#define SERVO_PERIOD_US 20000

#define PWM0_BASE      0x10015000
#define PWM_CFG        (*(volatile uint32_t *)(PWM0_BASE + 0x00))
#define PWM_COUNT      (*(volatile uint32_t *)(PWM0_BASE + 0x08))
#define PWM_CMP0       (*(volatile uint32_t *)(PWM0_BASE + 0x20))
#define PWM_CMP1       (*(volatile uint32_t *)(PWM0_BASE + 0x24))

//Array of function points for interrupts and exceptions
void (*interrupt_handler[MAX_INTERRUPTS])();
void (*exception_handler[MAX_INTERRUPTS])();
volatile int intr_count = 0;

int val = 0;        // On/Off value for braking LED
int last_led_state = 0;

void timer_handler() {
    intr_count++;       // Increment the interrupt counter variable
    set_cycles(get_cycles() + 3277);       // Set the mtimecmpr register to correct value to generate an interrupt after 100ms (3277 cycles in 100 ms)
}

int auto_brake(int devid) {
    // Task1 & 2: 
    // Your code here (Use Lab 02 - Lab 04 for reference)
    // Use the directions given in the project document
    uint16_t dist = 0;      // LIDAR distance data is 16 bits.
    int led_state = 0;      // Stores LED state: 1 for green, 2 for yellow, 3 for red, 4 for flashing red

    while (1) {     // Runs while the first two bytes are not 'Y'
        uint8_t b = ser_read(devid);        // Gets first byte
        if (b == 'Y') {     // Checks if first byte is 'Y'
            if (ser_read(devid) == 'Y') {       // If second byte is also 'Y', the loop breaks and the function is continued
                break;
            }
        }
    }

    uint8_t dist_L = ser_read(devid);
    uint8_t dist_H = ser_read(devid);
    uint8_t stren_L = ser_read(devid);
    uint8_t stren_H = ser_read(devid);       // Reads the reamaining 7 bytes from the transmission
    uint8_t rsvd = ser_read(devid);
    uint8_t quality = ser_read(devid);
    uint8_t received_chk_sum = ser_read(devid);

    // Confirm the checksum
    uint8_t checksum = (0x59 + 0x59 + dist_L + dist_H + stren_L + stren_H + rsvd + quality);        // Calculates checksum
    checksum = checksum & 0xFF;     // Gets the last 8 bits of the checksum
    if (checksum != received_chk_sum) {     // If calculated checksum does not equal the received checksum, the transmission was incorrect
        return last_led_state;      // Returns the last led state to prevent LED from stopping
    }

    dist = (dist_H << 8) | dist_L;      // Calculates the distance
    printf("Distance: %d cm\n", dist);          // Display distance

    if (dist > 200) {       // Safe distance, the Green LED turns on
        led_state = 1;
        gpio_write(RED_LED, 0);
        gpio_write(GREEN_LED, 1);
    }

    else if (200 >= dist && dist > 100) {      // Close distance, the Yellow LED turns on (Red and Green LEDs)
        led_state = 2;
        gpio_write(RED_LED, 1);
        gpio_write(GREEN_LED, 1);
    }

    else if (100 >= dist && dist > 60) {        // Very close distance, the Red LED turns on
        led_state = 3;
        gpio_write(GREEN_LED, 0);
        gpio_write(RED_LED, 1);
    }

    else if (60 >= dist) {      // Too close, the Red LED flashes
        led_state = 4;
        gpio_write(GREEN_LED, 0);
    }

    last_led_state = led_state;     // Updates last_led_state
    return led_state;       // Returns led_state to main where the LEDs are controlled
}

int read_from_pi(int devid) {
    // Task-3: 
    // You code goes here (Use Lab 09-option1 for reference)
    // After performing Task-2 at dnn.py code, modify this part to read angle values from Raspberry Pi.
    char string_array[BUF_SIZE];        // Creates string_array to store read from the Pi

    if (ser_isready(devid)) {       // Checks if the Pi is ready to send
        int bytes_read = ser_readline(devid, BUF_SIZE, string_array);       // Stores the result of ser_readline in bytes_read
        if (bytes_read > 0) {       // If bytes_read > 0, an angle was received 
            int angle = 0;
            sscanf(string_array, "%d", &angle);     // Extracts the angle from string_array
            return angle;       // Returns the angle to main
        }
    }
    return 0;       // Returns 0 if the Pi is not ready to send
}

static uint32_t angle_to_us(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    return (SERVO_MIN_US + (angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180);
}

void pwm_init_servo() {
    // Route PWM0_1 to GPIO 19
    gpio_mode(PIN_19, PWM);

    // Reset counter
    PWM_COUNT = 0;

    // Set PWM clock to 1 MHz (1 tick = 1 us)
    // Bit 0 = enable, Bit 3 = scale (divide by 1)
    //PWM_CFG = (1 << 0);
    PWM_CFG = (3 << 0) | (1 << 9) | (1 << 12);

    // Set period to 20 ms
    PWM_CMP0 = SERVO_PERIOD_US;

    // Start with neutral pulse
    PWM_CMP1 = 1500;
}

void servo_set_angle(int angle) {
    PWM_CMP1 = angle_to_us(angle);
}

void steering(int gpio, int pos) {
    // Task-4: 
    // Your code goes here (Use Lab 05 for reference)
    // Check the project document to understand the task
    servo_set_angle(pos);
    /*
    uint16_t pulse_width = 544 + ((1856 * pos) / 180);       // Calculates the pulse width

    gpio_write(gpio, ON);
    delay_usec(pulse_width);     // Duty cycle
    
    gpio_write(gpio, OFF);
    delay_usec((SERVO_PERIOD - pulse_width));       // Remainder of the PWM period
    */
}

int main() {
    // initialize UART channels
    ser_setup(0); // uart0
    ser_setup(1); // uart1
    int pi_to_hifive = 1; //The connection with Pi uses uart 1
    int lidar_to_hifive = 0; //the lidar uses uart 0

    // Interrupt setup
    interrupt_handler[MIE_MTIE_BIT] = timer_handler;        // install timer interrupt handler
    register_trap_handler(handle_trap);         // write handle_trap address to mtvec
    enable_timer_interrupt();         // enable timer interrupt
    enable_interrupt();         // enable global interrupt
    set_cycles(get_cycles() + 40000);           // cause timer interrupt for some time in future 
    int prev_intr_count = intr_count;           // Previous interrupt count
    
    pwm_init_servo();       // PWM setup

    printf("\nUsing UART %d for Pi -> HiFive", pi_to_hifive);
    printf("\nUsing UART %d for Lidar -> HiFive", lidar_to_hifive);
    
    //Initializing PINs
    //gpio_mode(PIN_19, OUTPUT);
    gpio_mode(RED_LED, OUTPUT);
    gpio_mode(BLUE_LED, OUTPUT);
    gpio_mode(GREEN_LED, OUTPUT);

    printf("Setup completed.\n");
    printf("Begin the main loop.\n");

    while (1) {
        /*
        disable_interrupt();
        int led_state = auto_brake(lidar_to_hifive);        // Measuring distance using lidar and braking
        if (led_state == 4) {
            if (prev_intr_count != intr_count) {        // Checks for new braking LED interrupt
                val ^= 1;       // Toggle for LED ON/OFF
                gpio_write(RED_LED, val);       // Turns Red LED ON or OFF
                prev_intr_count = intr_count;       // Save off the interrupt count
            }
        }
        enable_interrupt();
        */
        int angle = read_from_pi(pi_to_hifive);     // Getting turn direction from pi
        if (angle != 0) {
            printf("angle=%d\n", angle);
        }

        int gpio = PIN_19;
        for (int i = 0; i < 10; i++){
            // Here, we set the angle to 180 if the prediction from the DNN is a positive angle
            // and 0 if the prediction is a negative angle.
            // This is so that it is easier to see the movement of the servo.
            // You are welcome to pass the angle values directly to the steering function.
            // If the servo function is written correctly, it should still work,
            // only the movements of the servo will be more subtle
            if (angle > 0) {
                steering(gpio, 180);
            }
            else {
                steering(gpio,0);
            }
            // Uncomment the line below to see the actual angles on the servo.
            // Remember to comment out the if-else statement above!
            // steering(gpio, angle);
        }
    }
    return 0;
}
