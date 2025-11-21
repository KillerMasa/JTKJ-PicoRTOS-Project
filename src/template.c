
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include "morse.c"
#define RED_LED_PIN           14
#define LED1                  RED_LED_PIN
#define RGB_LED_G             19

#include "tkjhat/sdk.h"
  

// Default stack size for the tasks. It can be reduced to 1024 if task is not using lot of memory.
#define DEFAULT_STACK_SIZE 2048 

#define INPUT_BUFFER_SIZE 256

// Creating a structure to deal with IMU data and naming it imuData
struct imu_data {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float t;
} imuData;

// Creating a buffer for the last three marks. 
char last_marks[3] = {0, 0 , 0};

// Creating a text buffer for debugging messages
char text_buffer[INPUT_BUFFER_SIZE];

// Creating a buffer for storing marks.
char mark_buffer[INPUT_BUFFER_SIZE];

// The final message after converting
char decoded_text[INPUT_BUFFER_SIZE];

int morse_line_index = 0;


// Creating states for the state machine and making it start at IDLE state
enum state {IDLE=0, READ_IMU, READ_TAG, UPDATE_DATA};
enum state programState = IDLE;
enum state previousState = IDLE;

// Creating task and function prototypes
static void imu_task(void *arg);
static void lcd_task(void *arg);
static void btn_fxn(uint gpio, uint32_t eventMask);
static void idle_task(void *arg);
static void serial_task(void *pvParameters);
void update_lines_to_buffer(char* buffer, char* line_buffer);
void update_buffer(char *buffer, char new_mark);
void update_last_marks(char *buffer, char new_mark);
void update_last_marks_client(char *buffer, char new_mark);
void check_message_end_client(char *buffer);
void detect_moves(char *buffer, struct imu_data *data);
void init_led(void);
void toggle_led(void);

void update_lines_to_buffer(char* buffer, char* line_buffer){
    int start_point = 0;
    char ch[2] = {0, 0};
    // Finding the end of the current buffer
    while (start_point < INPUT_BUFFER_SIZE - 1 && buffer[start_point] != '\0') start_point++;

    // Appending the new line to the end of buffer
    for (int i= 0; line_buffer[i] != '\0' && i<INPUT_BUFFER_SIZE - 1 && start_point < INPUT_BUFFER_SIZE - 1; i++) {
        ch[0] = line_buffer[i];
        buffer[start_point] = ch[0];
        start_point++;
    }
    // Adding a new line character and null terminator at the end
    buffer[start_point + 1] = '\0';

}

// Creating a function to update the last three marks buffer for the client input.
void update_last_marks_client(char *buffer, char new_mark) {
    
    for (int i=0; i<2; i++) {
        buffer[i] = buffer[i+1];
    }
    
}

// Creating function to check if the message ending condition is met for the client input.
void check_message_end_client(char *buffer) {
    if (buffer[1] == ' ' && buffer[2] == ' ') {
        puts("Detected word break from client, changing state to UPDATE_DATA\n");
        previousState = READ_TAG;
        vTaskDelay(pdMS_TO_TICKS(50));
        programState = UPDATE_DATA;
        
    }
}

// Creating a function to update the last three marks buffer. Then checks if the message ending condition is met.
void update_last_marks(char *buffer, char new_mark) {
    
    for (int i=0; i<2; i++) {
        buffer[i] = buffer[i+1];
    }
    buffer[2] = new_mark;
    printf("Last three marks: %s\n", buffer);
    if (new_mark == '\n' && buffer[0] == ' ' && buffer[1] == ' ') {
        puts("Detected word break, changing state to UPDATE_DATA\n");
        previousState = READ_IMU;
        vTaskDelay(pdMS_TO_TICKS(50));
        programState = UPDATE_DATA;
        
    } 
}

// Creating a function to update the morsecode buffer. 
void update_buffer(char *buffer, char new_mark) {
    
    for (int i=0; i<=INPUT_BUFFER_SIZE - 1; i++) {
        if (buffer[i] == '\0') {
            buffer[i] = new_mark;
            break;
        }

    }
    update_last_marks(last_marks, new_mark);
    
}

// Creating a function to detect physical movements that creates morsecode marks. After that changing state to UPDATE_DATA to print the new mark to lcd.
void detect_moves(char *buffer, struct imu_data *data) {
    if (data->gx > 100) {
        update_buffer(buffer, '.');
        puts("Detected .");
        toggle_led();
        sleep_ms(400);
        toggle_led();
        sleep_ms(150);
    }
    else if (data->gx < -100) {
        update_buffer(buffer, '-');
        puts("Detected -");
        toggle_led();
        sleep_ms(800);
        toggle_led();
        sleep_ms(150);
        
    }
    else if (data->gy > 100) {
        update_buffer(buffer, ' ');
        puts("Detected space");
        
    }
    else if (data->gy < -100) {
        update_buffer(buffer, '\n');
        puts("Detected new line");
        
    }
    vTaskDelay(pdMS_TO_TICKS(50));

}

// Creating a button interrupt function to change the state machine state when button2 is pressed.
static void btn_fxn(uint gpio, uint32_t eventMask) {
    programState = (programState + 1) % 3;  // With button press, we can change to IDLE, READ_IMU AND READ_TAG states
    sprintf(text_buffer, "Button pressed, changing state to %d\n", programState);
    puts(text_buffer); 
}



//Creating a task for reading data from IMU sensor. Based on the data, it calls the detect_moves to add morsecode marks to the buffer.
static void imu_task(void *pvParameters) {
    (void)pvParameters;
    // Creating a pointer to imu_data structure
    struct imu_data *data = &imuData;

    // Data collection starts from the infinite loop which only operates when the state machine is at READ_IMU state.
    while (1)
    {
        if (programState == READ_IMU) {
            if (ICM42670_read_sensor_data(&data->ax, &data->ay, &data->az, &data->gx, 
                &data->gy, &data->gz, &data->t) == 0) {
                
                // printf below if for debugging imu data. Uncomment if needed.

                //printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f| Temp: %2.2f°C\n", 
                    //imuData.ax, imuData.ay, imuData.az, imuData.gx, imuData.gy, imuData.gz, imuData.t);

                detect_moves(mark_buffer, data);
                
            } else {
                puts("Failed to read imu data\n");
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
    }
}

//Creating a task for displaying morsecode in LCD
static void lcd_task(void *arg) {
    (void)arg;
        //First in the infinite loop, which only works when the state machine is at UPDATE_DATA state, 
        // it clears the lcd display. Then it writes the morsecode buffer to the lcd. After that
        // it waits a bit and changes the state to READ_IMU to continue creating the message.
        while(1){
            if (programState == UPDATE_DATA) {
                // Testing if previous state was READ_IMU to decode the message
                //if (previousState == READ_IMU) {
                    morsebuffer_to_text(mark_buffer, decoded_text);
                for (int i = 0; i < 3; i++) {
                    toggle_led();
                    sleep_ms(300);
                    toggle_led();
                    sleep_ms(120);
    }
                    sprintf(text_buffer, "Decoded message: %s\n", decoded_text);
                    puts(text_buffer);
                //}
        
                clear_display();
                // Initialize coordinates for text pringting
                int16_t x = 0;
                int16_t y = 0;
                // Iniliazing a char array to store one character a time
                char ch[2] = {0, 0};
                // Looping through the mark_buffer and printing one character at a time
                for (int i = 0; i < INPUT_BUFFER_SIZE; i++) {
                    
                    ch[0] = mark_buffer[i];
                    // Testing if the end of the buffer is reached
                    if (ch[0] == '\0') {
                        puts("Message was sent successfully to LCD!\n");
                        break;
                    }
                    // Testing if character is newline
                    if (ch[0] == '\n') {
                        x = 0;
                        y += 10; // Move to next line
                    } 
                    // Testing if one line is full (21 marks), if yes move to next line
                    else if (x >= 126){
                        x = 0;
                        y += 10;
                        write_text_xy(x, y, ch);
                        x += 6; // Move to next character position
                    }else {
                        write_text_xy(x, y, ch);
                        x += 6; // Move to next character position
                    }
                }
                
                
                vTaskDelay(pdMS_TO_TICKS(300)); 
                // Clear the mark buffer after displaying
                mark_buffer[0] = '\0'; 
                
                programState = previousState; // Change back to previous state (READ_IMU or READ_TAG)
                vTaskDelay(pdMS_TO_TICKS(100));
            }

        }
}

//Creating a task for waiting. It applies when the state machine is at IDLE state.
static void idle_task(void *arg) {
    (void)arg;
    while (1) {
        if (programState == IDLE) {
           vTaskDelay(pdMS_TO_TICKS(500));
        }
        
    }
}

//Creating a task for receiving messages from the serial-client. It only works when the state machine is at READ_TAG state.
static void serial_task(void *arg) {
    //This task is for receiving data from serial-client. It's mainly from the example but modified a bit to make it fit better. It reads what 
    //serial-client sends and then prints it to the serial monitor adding a "CLIENT: " to the start of the message to make it easier to see where
    // the data is coming from.
    (void)arg;
    
    char line[INPUT_BUFFER_SIZE];
    size_t index = 0;
    
    while (1){
        if (programState == READ_TAG){
            int c = getchar_timeout_us(0);
            if (c != PICO_ERROR_TIMEOUT){
                if (c == '\r') {
                    continue;
                }
                if (c == '\n'){
                    // terminate and process the collected line
                    line[index] = '\0'; 
                    printf("__CLIENT:\"%s\"__\n", line); //Print as debug in the output
                    update_lines_to_buffer(mark_buffer, line);
                    check_message_end_client(last_marks);
                    update_last_marks_client(last_marks, c);
                    index = 0;
                    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for new message

                }
                else if(index < INPUT_BUFFER_SIZE - 1){
                    update_last_marks_client(last_marks, c);
                    line[index++] = (char)c;
                }
                else { //Overflow: print and restart the buffer with the new character. 
                    line[INPUT_BUFFER_SIZE - 1] = '\0';
                    printf("__CLIENT:\"%s\"__\n", line); //Print as debug in the output
                    update_lines_to_buffer(mark_buffer, line);
                    update_last_marks_client(last_marks, c);
                    index = 0; 
                    line[index++] = (char)c; 
                }
            }
            else {
                vTaskDelay(pdMS_TO_TICKS(100)); // Wait for new message
            }
        }
    }
}


int main() {
    // First setting up the stdio and waiting for usb connection.
    stdio_init_all();
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    puts("USB connected.");
    // Then iniliazing the hat sdk.
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.
    // Setting up the led
    init_led();
    // Setting up the lcd
    init_display();
    // Setting up the IMU sensor. 
    if (init_ICM42670() == 0) {
        puts("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            puts("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    } else {
        puts("Failed to initialize ICM-42670P.\n");
    }

    // Initializing button2 and making an interruption when pressed to change the state. FYI the button1 seems not to work on our device.
    init_button2();
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    TaskHandle_t IMUTask = NULL, LCDTask = NULL, IDLETask = NULL, SerialTask = NULL;
    // Creating the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(imu_task, 
                "IMU",
                DEFAULT_STACK_SIZE,
                NULL,
                2, 
                &IMUTask);
    
    if(result != pdPASS) {
        puts("IMU Task creation failed");
        return 0;
    }
    result = xTaskCreate(lcd_task, 
                "LCD",
                DEFAULT_STACK_SIZE,
                NULL,
                2, 
                &LCDTask);
    
    if(result != pdPASS) {
        puts("LCD Task creation failed");
        return 0;
    }
    result = xTaskCreate(idle_task, 
                "IDLE",
                1024,
                NULL,
                2, 
                &IDLETask);
    
    if(result != pdPASS) {
        puts("IDLETask creation failed");
        return 0;
    }
    result = xTaskCreate(serial_task, 
                "Serial",
                DEFAULT_STACK_SIZE,
                NULL,
                2, 
                &SerialTask);
    
    if(result != pdPASS) {
        puts("IDLETask creation failed");
        return 0;
    }


    
    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

