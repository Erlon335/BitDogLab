#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ssd1306.h"
#include "font.h"
#include "ws2818b.pio.h"

// Definições para o Conversor.c
#define AZUL 11
#define VERDE 12
#define VERMELHO 13
#define VERTICALY 26
#define HORIZONTALX 27
#define SELEC 22
#define PWM_WRAP 4095
#define PWM_CLK_DIV 30.52f
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define ENDERECO_I2C 0x3C

// Definições para o neopixel_pio.c
#define LED_COUNT 25
#define LED_PIN 7

// Definições para o Buzzer
#define BUZZER_PIN 21
#define BUZZER_FREQ 1000 // Frequência do buzzer em Hz
#define BUZZER_DURATION_MS 100 // Duração do toque em milissegundos

// Estrutura para o NeoPixel
struct pixel_t {
    uint8_t G, R, B;
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;

// Variáveis globais
bool cor = 0;
bool controle_leds = true;
absolute_time_t last_interrupt_time = 0;
ssd1306_t ssd;
int captura = 0;
npLED_t leds[LED_COUNT];
PIO np_pio;
uint sm;

// Funções do Conversor.c
void pwm_init_gpio(uint gpio, uint wrap, float clkdiv) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, wrap);
    pwm_config_set_clkdiv(&config, clkdiv);
    pwm_init(slice_num, &config, true);
}

int16_t mapeamento_Y(uint16_t valor_y) {
    int16_t valor_mapeado;
    if (valor_y < 2047) {
        valor_mapeado = 2047 - valor_y;
    } else if (valor_y > 2047) {
        valor_mapeado = valor_y - 2047;
    } else {
        valor_mapeado = 0;
    }
    return valor_mapeado;
}

int16_t mapeamento_x(uint16_t valor_x) {
    int16_t valor_mapeado_x;
    if (valor_x < 2047) {
        valor_mapeado_x = 2047 - valor_x;
    } else if (valor_x > 2047) {
        valor_mapeado_x = valor_x - 2047;
    } else {
        valor_mapeado_x = 0;
    }
    return valor_mapeado_x;
}

void display() {
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);

    switch (captura) {
        case 0:
            ssd1306_rect(&ssd, 3, 3, 122, 60, true, false);
            break;
        case 1:
            ssd1306_rect(&ssd, 3, 3, 122, 58, true, false);
            ssd1306_rect(&ssd, 5, 5, 118, 54, true, false);
            break;
    }
}

void gpio_callback(uint gpio, uint32_t events) {
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(last_interrupt_time, now) < 250000) return;
    last_interrupt_time = now;

    if (gpio == SELEC) {
        cor = !cor;
        gpio_put(VERDE, cor);
        captura = (captura + 1) % 2;
        display();
    }
}

// Funções do neopixel_pio.c
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;
    sm = pio_claim_unused_sm(np_pio, false);
    if (sm < 0) {
        np_pio = pio1;
        sm = pio_claim_unused_sm(np_pio, true);
    }
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);

    for (uint i = 0; i < LED_COUNT; ++i) {
        leds[i].R = 0;
        leds[i].G = 0;
        leds[i].B = 0;
    }
}

void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b) {
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}

void npClear() {
    for (uint i = 0; i < LED_COUNT; ++i)
        npSetLED(i, 0, 0, 0);
}

void npWrite() {
    for (uint i = 0; i < LED_COUNT; ++i) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100);
}

// Função para mapear a posição (x, y) da matriz para o índice do LED
int getIndex(int x, int y) {
    // Para uma matriz 5x5, percorrendo da esquerda para a direita, linha por linha
    // Ajuste para corrigir a orientação de cabeça para baixo
    return (4 - y) * 5 + x;
}

// Função para tocar o buzzer
void tocarBuzzer(uint gpio, uint frequencia, uint duracao_ms) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 125000000 / frequencia); // Configura a frequência
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(gpio, 62500); // 50% de duty cycle
    sleep_ms(duracao_ms); // Mantém o som por um tempo
    pwm_set_gpio_level(gpio, 0); // Desliga o buzzer
}

// Função principal
int main() {
    stdio_init_all();
    sleep_ms(2000);

    // Inicializações do Conversor.c
    pwm_init_gpio(AZUL, PWM_WRAP, PWM_CLK_DIV);
    pwm_init_gpio(VERMELHO, PWM_WRAP, PWM_CLK_DIV);

    adc_init();
    adc_gpio_init(VERTICALY);
    adc_gpio_init(HORIZONTALX);

    gpio_init(VERDE);
    gpio_set_dir(VERDE, GPIO_OUT);

    gpio_init(SELEC);
    gpio_set_dir(SELEC, GPIO_IN);
    gpio_pull_up(SELEC);

    gpio_set_irq_enabled_with_callback(SELEC, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_init(&ssd, 128, 64, false, ENDERECO_I2C, I2C_PORT);

    // Inicializações do neopixel_pio.c
    npInit(LED_PIN);
    npClear();

    const int square_size = 8;
    int centro_x = (128 - square_size) / 2;
    int centro_y = (64 - square_size) / 2;

    // Matriz de cores fixa
    int matriz[5][5][3] = {
        {{0, 0, 0}, {255, 255, 0}, {0, 0, 0}, {255, 255, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 255, 0}, {0, 255, 0}, {0, 255, 0}, {0, 0, 0}},
        {{0, 255, 0}, {255, 0, 0}, {0, 255, 0}, {255, 0, 0}, {0, 255, 0}},
        {{0, 0, 0}, {0, 255, 0}, {0, 255, 0}, {0, 255, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 255, 0}, {0, 0, 0}, {0, 0, 0}}
    };

    while (true) {
        // Leitura do joystick
        adc_select_input(0);
        uint16_t valor_y = adc_read();
        adc_select_input(1);
        uint16_t valor_x = adc_read();

        if (controle_leds) {
            pwm_set_gpio_level(AZUL, mapeamento_Y(valor_y));
            pwm_set_gpio_level(VERMELHO, mapeamento_x(valor_x));
        } else {
            pwm_set_gpio_level(AZUL, 0);
            pwm_set_gpio_level(VERMELHO, 0);
        }

        // Atualização do display OLED
        display();
        int pos_x = centro_x + ((2048 - (int)valor_x) * centro_x) / 2048;
        int pos_y = centro_y + ((2048 - (int)valor_y) * centro_y) / 2048;
        ssd1306_rect(&ssd, pos_y, pos_x, square_size, square_size, true, true);
        ssd1306_send_data(&ssd);

        // Aplica a intensidade à matriz de cores fixa
        uint8_t intensidade_r = (valor_x >> 4); // Intensidade do vermelho
        uint8_t intensidade_g = (valor_y >> 4); // Intensidade do verde
        uint8_t intensidade_b = ((valor_x + valor_y) >> 5); // Intensidade do azul

        for (int linha = 0; linha < 5; linha++) {
            for (int coluna = 0; coluna < 5; coluna++) {
                int posicao = getIndex(coluna, linha); // Ajuste para orientação horizontal correta
                uint8_t r = (matriz[linha][coluna][0] * intensidade_r) >> 8;
                uint8_t g = (matriz[linha][coluna][1] * intensidade_g) >> 8;
                uint8_t b = (matriz[linha][coluna][2] * intensidade_b) >> 8;
                npSetLED(posicao, r, g, b);
            }
        }
        npWrite();

        // Tocar o buzzer quando o joystick é movido
        if (valor_x > 2048 + 500 || valor_x < 2048 - 500 || valor_y > 2048 + 500 || valor_y < 2048 - 500) {
            tocarBuzzer(BUZZER_PIN, BUZZER_FREQ, BUZZER_DURATION_MS);
        }

        sleep_ms(10); // Ajuste o tempo de atualização conforme necessário
    }
}