#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

// Configurações do Wi-Fi
#define WIFI_SSID "SUA_REDE_WIFI"
#define WIFI_PASSWORD "SUA_SENHA_WIFI"

// Configurações do servidor BitDogLab
#define BITDOGLAB_HOST "api.bitdoglab.com"
#define BITDOGLAB_PORT 80
#define API_KEY "SUA_CHAVE_API"

// Configurações do I2C para o acelerômetro MPU6050
#define I2C_PORT i2c0
#define MPU6050_ADDR 0x68
#define ACCEL_XOUT_H 0x3B

// Configurações do ADC para o sensor de temperatura
#define TEMP_SENSOR_PIN 26

// Função para inicializar o Wi-Fi
bool init_wifi() {
    if (cyw43_arch_init()) {
        printf("Erro ao inicializar o Wi-Fi\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Erro ao conectar ao Wi-Fi\n");
        return false;
    }
    printf("Conectado ao Wi-Fi\n");
    return true;
}

// Função para inicializar o I2C
void init_i2c() {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);
}

// Função para ler o acelerômetro MPU6050
void read_accel(int16_t *accelX, int16_t *accelY, int16_t *accelZ) {
    uint8_t buffer[6];
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &ACCEL_XOUT_H, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);
    *accelX = (buffer[0] << 8) | buffer[1];
    *accelY = (buffer[2] << 8) | buffer[3];
    *accelZ = (buffer[4] << 8) | buffer[5];
}

// Função para ler a temperatura
float read_temperature() {
    adc_select_input(0);
    uint16_t raw_value = adc_read();
    float voltage = raw_value * 3.3f / 4096.0f;
    float temperature = (voltage - 0.5f) * 100.0f; // Conversão para LM35
    return temperature;
}

// Função para enviar dados ao BitDogLab via HTTP
err_t send_to_bitdoglab(float temperature, bool fall_detected) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        printf("Erro ao criar PCB\n");
        return ERR_MEM;
    }

    ip_addr_t server_ip;
    if (!ipaddr_aton(BITDOGLAB_HOST, &server_ip)) {
        printf("Erro ao converter endereço IP\n");
        return ERR_ARG;
    }

    err_t err = tcp_connect(pcb, &server_ip, BITDOGLAB_PORT, NULL);
    if (err != ERR_OK) {
        printf("Erro ao conectar ao servidor\n");
        return err;
    }

    // Preparar os dados JSON
    char json_data[100];
    snprintf(json_data, sizeof(json_data), "{\"temperatura\": %.2f, \"queda\": %s}",
             temperature, fall_detected ? "true" : "false");

    // Montar a requisição HTTP
    char request[200];
    int request_len = snprintf(request, sizeof(request),
                               "POST /data HTTP/1.1\r\n"
                               "Host: %s\r\n"
                               "Content-Type: application/json\r\n"
                               "Content-Length: %d\r\n"
                               "Authorization: Bearer %s\r\n"
                               "\r\n"
                               "%s",
                               BITDOGLAB_HOST, strlen(json_data), API_KEY, json_data);

    // Enviar a requisição
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, request_len, PBUF_RAM);
    if (!p) {
        printf("Erro ao alocar pbuf\n");
        return ERR_MEM;
    }
    memcpy(p->payload, request, request_len);
    err = tcp_write(pcb, p->payload, p->len, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        printf("Erro ao enviar dados\n");
        pbuf_free(p);
        return err;
    }
    tcp_output(pcb);
    pbuf_free(p);

    // Fechar a conexão
    tcp_close(pcb);
    return ERR_OK;
}

// Função principal
int main() {
    stdio_init_all();

    // Inicializar ADC para o sensor de temperatura
    adc_init();
    adc_gpio_init(TEMP_SENSOR_PIN);

    // Inicializar I2C para o acelerômetro
    init_i2c();

    // Inicializar Wi-Fi
    if (!init_wifi()) {
        return 1;
    }

    // Loop principal
    while (true) {
        // Ler dados dos sensores
        int16_t accelX, accelY, accelZ;
        read_accel(&accelX, &accelY, &accelZ);
        float temperature = read_temperature();

        // Detectar quedas (exemplo simples)
        bool fall_detected = (abs(accelZ) > 15000); // Ajuste o limite conforme necessário

        // Enviar dados ao BitDogLab
        err_t err = send_to_bitdoglab(temperature, fall_detected);
        if (err != ERR_OK) {
            printf("Erro ao enviar dados: %d\n", err);
        } else {
            printf("Dados enviados com sucesso!\n");
        }

        // Esperar 5 segundos antes da próxima leitura
        sleep_ms(5000);
    }

    return 0;
}
