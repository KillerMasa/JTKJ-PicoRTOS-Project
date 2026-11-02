// Joona Myllymäki, Leevi Astikainen, Ursa Muranen
// Pisteiden jaottelu 2,2,2
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

#define DEFAULT_STACK_SIZE 2048
#define INPUT_BUFFER_SIZE 256

// Rakenne kiihtyvyysanturin arvoille
struct imu_data {
    float ax, ay, az;
    float gx, gy, gz;
    float t;
} imuData;

// FreeRTOS-jono merkkien siirtoon IMU/nappi-taskilta sarjaporttitaskille
static QueueHandle_t char_queue = NULL;

// Taskien ja funktioiden esittelyt löytyy tästä
static void imu_task(void *pvParameters);
static void serial_task(void *pvParameters);
static void btn_fxn(uint gpio, uint32_t eventMask);

// Lippumuuttujat napinpainalluksille
static volatile bool btn1_pressed = false; // Merkin vahvistus (. tai -)
static volatile bool btn2_pressed = false; // Välilyönti / erotin

// Keskeytyskäsittelijä molemmille napeille
static void btn_fxn(uint gpio, uint32_t eventMask) {
    (void)eventMask;
    if (gpio == BUTTON1) {
        btn1_pressed = true;
    } else if (gpio == BUTTON2) {
        btn2_pressed = true;
    }
}

/*
 * Täällä luetaan laitteen asento ja tarkistetaan napinpainallukset.
 * - Asento: pöydällä = '.', pystyssä = '-'
 * - Nappi 1: lähettää valitun merkin
 * - Nappi 2: lähettää välilyönnin
 */
static void imu_task(void *pvParameters) {
    (void)pvParameters;
    struct imu_data *data = &imuData;
    char current_symbol = '.';
    const char space_symbol = ' ';

    while (1) {
        // Luetaan anturidata
        if (ICM42670_read_sensor_data(&data->ax, &data->ay, &data->az,
                                      &data->gx, &data->gy, &data->gz, &data->t) == 0) {
            
            // Tunnistetaan asento kiihtyvyyden suunnan mukaan
            if (data->az > 0.7f) {
                // Laite makaa vaakatasossa ja napin painalluksella lisätään piste jonoon
                current_symbol = '.';
            } else if (data->ay > 0.5f || data->ax > 0.5f) {
                // Laite nostettu pystyyn tai kallistettu ja nappia painamalla lisätään viiva jonoon
                current_symbol = '-';
            }

            // Nappi 1 painettu -> viedään nykyinen morse-merkki jonoon
            if (btn1_pressed) {
                btn1_pressed = false;
                xQueueSend(char_queue, &current_symbol, 0);
            }

            // Nappi 2 painettu -> viedään välilyönti jonoon
            if (btn2_pressed) {
                btn2_pressed = false;
                xQueueSend(char_queue, &space_symbol, 0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/*
 * Täällä vastaanotetaan merkit jonosta puskuriin.
 * Kun tulee 2 peräkkäistä välilyöntiä, lähetetään koko viesti terminaaliin 
 * samalle riville ja perään rivinvaihto jotta uuden viestin kirjoittaminen onnistuu.
 */
static void serial_task(void *pvParameters) {
    (void)pvParameters;
    char received_mark;
    char message_buffer[INPUT_BUFFER_SIZE];
    int buf_idx = 0;
    int space_count = 0;

    while (1) {
        if (xQueueReceive(char_queue, &received_mark, portMAX_DELAY) == pdTRUE) {
            
            // Tallennetaan merkki puskuriin jos tilaa on riittävästi
            if (buf_idx < INPUT_BUFFER_SIZE - 2) {
                message_buffer[buf_idx++] = received_mark;
            }

            // Lasketaan peräkkäiset välilyönnit
            if (received_mark == ' ') {
                space_count++;
            } else {
                space_count = 0;
            }

            // Kun tulee kaksi välilyöntiä peräkkäin, päätetään viesti
            if (space_count >= 2) {
                message_buffer[buf_idx++] = '\n';
                message_buffer[buf_idx] = '\0';

                // Tulostetaan koko viesti sarjaporttiin kerralla
                printf("%s", message_buffer);
                fflush(stdout);

                // Nollataan puskuri ja laskuri seuraavaa viestiä varten
                buf_idx = 0;
                space_count = 0;
            }
        }
    }
}

int main(void) {
    // USB-sarjaviestinnän alustus
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }
    
    // Alustetaan hat_sdk
    init_hat_sdk();
    sleep_ms(200);

    // Alustetaan IMU-anturi
    if (init_ICM42670() == 0) {
        ICM42670_start_with_default_values();
    }

    // Alustetaan molemmat napit
    init_button1();
    init_button2();

    // Asetetaan keskeytykset kummallekin napille
    gpio_set_irq_enabled_with_callback(BUTTON1, 0x8, true, &btn_fxn);
    gpio_set_irq_enabled(BUTTON2, 0x8, true);

    // Luodaan jono merkkien välittämistä varten
    char_queue = xQueueCreate(32, sizeof(char));

    // Luodaan tarpeelliset FreeRTOS-taskit
    xTaskCreate(imu_task, "IMUTask", DEFAULT_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(serial_task, "SerialTask", DEFAULT_STACK_SIZE, NULL, 2, NULL);

    // Käynnistetään ajastin
    vTaskStartScheduler();

    while (1) {
        // Tänne ei koskaan päädytä
    }

    return 0;
}