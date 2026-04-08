#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "eecs388_lib.h"

#define ONE_HUND        (100)

//Array of function points for interrupts and exceptions
void (*interrupt_handler[MAX_INTERRUPTS])();
void (*exception_handler[MAX_INTERRUPTS])();

volatile int intr_count = 0;


void timer_handler() {
    intr_count++;
    set_cycles(get_cycles()+ 3277);       // 3277 cycles in 100 ms
    /* Task 2.3 Increment the interrupt counter variable*/
    /* Task 2.3 Set the mtimecmpr register to a correct value to 
       generate an interrupt after 100ms */
}

int auto_brake(int devid) {
    // Task1 & 2: 
    // Your code here (Use Lab 02 - Lab 04 for reference)
    // Use the directions given in the project document

    uint16_t dist = 0;      // LIDAR distance data is 16 bits.
    int led_state = 0;      // Stores LED state: 1 for green, 2 for yellow, 3 for red, 4 for flashing red

    if ('Y' == ser_read(devid) && 'Y' == ser_read(devid)) {
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

        if (checksum == received_chk_sum) {     // If calculated checksum equals the received checksum, the transmission was correct
            dist = (dist_H << 8) | dist_L;      // Calculates the distance

            if (dist > 200) {       // Safe distance, the Green LED turns on
                gpio_write(RED_LED, 0);       // Red LED OFF
                gpio_write(GREEN_LED, 1);       // Green LED ON
                led_state = 1;
            }

            else if (200 >= dist && dist > 100) {      // Close distance, the Yellow LED turns on (Red and Green LEDs)
                gpio_write(RED_LED, 1);        // Red LED ON
                gpio_write(GREEN_LED, 1);        // Green LED ON
                led_state = 2;
            }

            else if (100 >= dist && dist > 60) {        // Very close distance, the Red LED turns on
                gpio_write(GREEN_LED, 0);        // Green LED OFF
                gpio_write(RED_LED, 1);        // Red LED ON
                led_state = 3;
            }

            else if (60 >= dist) {      // Too close, the Red LED flashes
                gpio_write(GREEN_LED, 0);        // Green LED OFF
                gpio_write(RED_LED, 0);      // Red LED ON
                led_state = 4;
            }
            // Display distance
            printf("Distance: %d cm\n", dist);

            return led_state;
        }
    }
    return 0;
}

int read_from_pi(int devid) {
    // Task-3: 
    // You code goes here (Use Lab 09-option1 for reference)
    // After performing Task-2 at dnn.py code, modify this part to read angle values from Raspberry Pi.
    uint16_t angle = ser_read(devid);
    printf("%d", angle);
    return angle;
}

void steering(int gpio, int pos) {
    // Task-4: 
    // Your code goes here (Use Lab 05 for reference)
    // Check the project document to understand the task
}


int main() {
    // initialize UART channels
    ser_setup(0); // uart0
    ser_setup(1); // uart1
    int pi_to_hifive = 1; //The connection with Pi uses uart 1
    int lidar_to_hifive = 0; //the lidar uses uart 0
    /*
    // Interrupt setup
    interrupt_handler[MIE_MTIE_BIT] = timer_handler;        // install timer interrupt handler
    register_trap_handler(handle_trap());         // write handle_trap address to mtvec
    enable_timer_interrupt();         // enable timer interrupt
    enable_interrupt();         // enable global interrupt
    set_cycles(get_cycles() + 40000);           // cause timer interrupt for some time in future 

    int prev_intr_count = intr_count;           // Previous LED interrupt count
    */
    int val = 0;        // On/Off value for braking LED

    printf("\nUsing UART %d for Pi -> HiFive", pi_to_hifive);
    printf("\nUsing UART %d for Lidar -> HiFive", lidar_to_hifive);
    
    //Initializing PINs
    gpio_mode(PIN_19, OUTPUT);
    gpio_mode(RED_LED, OUTPUT);
    gpio_mode(BLUE_LED, OUTPUT);
    gpio_mode(GREEN_LED, OUTPUT);

    printf("Setup completed.\n");
    printf("Begin the main loop.\n");

    while (1) {
        //disable_interrupt();
        /*
        int led_state = auto_brake(lidar_to_hifive);        // Measuring distance using lidar and braking
        
        if (prev_intr_count != intr_count) {        // Checks for new braking LED interrupt
            if (led_state == 4) {       // Checks for flashing red state
                val ^= 1;       // Toggle for LED ON/OFF
                gpio_write(RED_LED, val);       // Turns Red LED ON or OFF
                delay(ONE_HUND);
            }
            //prev_intr_count = intr_count;       // Save off the interrupt count
        }
        */
        

        printf("Beginning to read angle");
        int angle = read_from_pi(pi_to_hifive);     // Getting turn direction from pi
        printf("\nangle=%d", angle);
        printf("Done reading angle");

        /*
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
        */
        //enable_interrupt();
    }
    return 0;
}
