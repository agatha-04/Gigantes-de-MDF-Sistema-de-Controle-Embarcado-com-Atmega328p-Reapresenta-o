/*
  car_main_final.c
  ATmega328P, F_CPU = 16MHz
  Integra PWM (OC0A/OC0B) para EN pins, ADC LDR, HC-05 serial, e lógica de "vidas".
  Wiring (conforme sua descrição):
  - LEDs: PB0, PB1, PB2
  - Laser: PD7
  - LDR: ADC4 (A4)
  - L293D:
      Enable1/2 -> PD5 (OC0B)
      Enable3/4 -> PD6 (OC0A)
      1A -> PC0
      2A -> PC1
      3A -> PC2
      4A -> PC3
  - HC-05:
      TX (HC-05) -> PD0 (RXD)
      RX (HC-05) -> PD1 (TXD)
*/
 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
 
/* ---------- CONFIGURÁVEIS ---------- */
#define ADC_THRESHOLD 700           // ajuste: valor ADC (0-1023) que conta como "luz forte"
#define DEBOUNCE_MS 50              // tempo de debounce da leitura do LDR
#define LASER_BLINK_INTERVAL_MS 1000UL
#define ROTATE_TIME_MS 700UL        // tempo aproximado para girar ~180° (ajuste conforme seu chassi)
#define PAUSE_AFTER_ROTATE_MS 5000UL // 5 segundos parado
/* ----------------------------------- */
 
/* Pinos (macros) */
#define LED_PORT PORTB
#define LED_DDR  DDRB
#define LED_PIN0 PB0
#define LED_PIN1 PB1
#define LED_PIN2 PB2
 
#define LASER_PORT PORTD
#define LASER_DDR  DDRD
#define LASER_PIN PD7
 
/* Motor control on PORTC */
#define MOT_PORT PORTC
#define MOT_DDR  DDRC
#define M1_IN1 PC0
#define M1_IN2 PC1
#define M2_IN1 PC2
#define M2_IN2 PC3
 
/* Enables on PD5 PD6 (OC0B, OC0A) */
#define ENABLE_PORT PORTD
#define ENABLE_DDR  DDRD
#define EN12 PD5  // OC0B
#define EN34 PD6  // OC0A
 
/* USART baud */
#define BAUD 9600
#define UBRR_VAL ((F_CPU/16/BAUD)-1)
 
/* estado */
volatile uint32_t millis_count = 0; // incrementado por timer0 overflow ~1ms
uint8_t lives = 3;
bool system_locked = false; // true quando vidas = 0 (aguarda reset)
bool reaction_active = false;
 
/* helper: devolve ms */
uint32_t millis(void) {
    uint32_t m;
    uint8_t sreg = SREG;
    cli();
    m = millis_count;
    SREG = sreg;
    return m;
}
 
/* Timer0 overflow ISR: cria um incrementador de ms aproximado */
ISR(TIMER0_OVF_vect) {
    // Timer0 em Fast PWM 8-bit -> overflow ~1.024 ms com prescaler 64
    // contabilizamos cada overflow como 1 ms (pequena imprecisão aceitável)
    static uint8_t accum = 0;
    (void)accum;
    millis_count++;
}
 
/* Inicializa Timer0 para PWM e contagem (Fast PWM, prescaler 64).
   Configura também OC0A/OC0B para saída PWM. */
void timer0_pwm_millis_init(void) {
    // PD6 = OC0A, PD5 = OC0B -> definir como outputs
    DDRD |= (1<<EN12) | (1<<EN34);
 
    // Fast PWM (WGM00=1, WGM01=1), non-inverting for OC0A/OC0B (COM0A1=1, COM0B1=1)
    TCCR0A = (1<<WGM01) | (1<<WGM00) | (1<<COM0A1) | (1<<COM0B1);
    // Prescaler 64
    TCCR0B = (1<<CS01) | (1<<CS00);
 
    // Enable overflow interrupt for millis
    TIMSK0 |= (1<<TOIE0);
 
    // Inicialmente zero duty
    OCR0A = 0;
    OCR0B = 0;
}
 
/* Ajusta PWM (0..255) para ambas enables */
void motors_pwm(uint8_t val) {
    OCR0A = val; // PD6 -> EN34
    OCR0B = val; // PD5 -> EN12
}
 
/* Inicializa USART (9600, 8N1) */
void usart_init(void) {
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR_VAL & 0xFF);
    UCSR0A = 0;
    UCSR0B = (1<<RXEN0) | (1<<TXEN0); // enable RX/TX
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8-bit
}
 
/* enviar um byte */
void uart_send(uint8_t b) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = b;
}
 
/* checar se há dado disponível e ler (retorna -1 se nada) */
int uart_read_nonblocking(void) {
    if (UCSR0A & (1<<RXC0)) {
        return UDR0;
    }
    return -1;
}
 
/* ADC init e leitura */
void adc_init(void) {
    // AVcc ref, ADC prescaler 128 (125 kHz), ADC enable
    ADMUX = (1<<REFS0); // AVcc as reference, ADC0 selected by default
    ADCSRA = (1<<ADEN) | (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}
 
/* Read ADC channel (0..7) */
uint16_t adc_read(uint8_t ch) {
    ch &= 0x07;
    ADMUX = (ADMUX & 0xF8) | ch;
    _delay_us(5);
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC));
    return ADC;
}
 
/* Funções motor - usando PWM em enables (OC0A/OC0B) e entradas em PORTC */
void motors_stop(void) {
    // coloca entradas em 0 e duty 0
    MOT_PORT &= ~((1<<M1_IN1)|(1<<M1_IN2)|(1<<M2_IN1)|(1<<M2_IN2));
    motors_pwm(0);
}
 
void motors_forward(void) {
    // M1 forward: IN1=1 IN2=0 ; M2 forward: IN1=1 IN2=0
    MOT_PORT |= (1<<M1_IN1) | (1<<M2_IN1);
    MOT_PORT &= ~((1<<M1_IN2) | (1<<M2_IN2));
    motors_pwm(255);
}
 
void motors_back(void) {
    MOT_PORT |= (1<<M1_IN2) | (1<<M2_IN2);
    MOT_PORT &= ~((1<<M1_IN1) | (1<<M2_IN1));
    motors_pwm(255);
}
 
void motors_turn_left(void) {
    // left turn: left motor backward, right motor forward (spin-turn left)
    MOT_PORT |= (1<<M1_IN2) | (1<<M2_IN1);
    MOT_PORT &= ~((1<<M1_IN1) | (1<<M2_IN2));
    motors_pwm(255);
}
 
void motors_turn_right(void) {
    // right turn: left forward, right backward (spin-turn right)
    MOT_PORT |= (1<<M1_IN1) | (1<<M2_IN2);
    MOT_PORT &= ~((1<<M1_IN2) | (1<<M2_IN1));
    motors_pwm(255);
}
 
/* LED helpers */
void leds_all_on(void) {
    LED_PORT |= (1<<LED_PIN0) | (1<<LED_PIN1) | (1<<LED_PIN2);
}
void led_off_index(uint8_t idx) {
    // idx 0 -> PB0, 1 -> PB1, 2 -> PB2
    if (idx == 0) LED_PORT &= ~(1<<LED_PIN0);
    else if (idx == 1) LED_PORT &= ~(1<<LED_PIN1);
    else if (idx == 2) LED_PORT &= ~(1<<LED_PIN2);
}
 
/* rotate 180: gira por ROTATE_TIME_MS (time-based)
   Opção A: spin turn (left forward, right backward) */
void rotate_180_and_back(void) {
    // gira 180 (spin: left forward, right backward)
    // left = M1, right = M2
    // For spin-left: left forward? For option A we use left forward + right back.
    // According to your choice: "A - Motor da esquerda para frente + motor da direita para trás"
    // We'll use that to rotate, then pause, then invert to return.
    // ROTATE -> left forward, right backward
    // RETURN -> left backward, right forward
    // Start rotate
    // left forward: M1_IN1=1, M1_IN2=0
    // right backward: M2_IN1=0, M2_IN2=1
 
    // rotate
    MOT_PORT |=  (1<<M1_IN1);
    MOT_PORT &= ~((1<<M1_IN2) | (1<<M2_IN1));
    MOT_PORT |=  (1<<M2_IN2);
    motors_pwm(255);
 
    uint32_t t0 = millis();
    while (millis() - t0 < ROTATE_TIME_MS) { /* aguarda */ }
 
    motors_stop();
 
    // pausa
    t0 = millis();
    while (millis() - t0 < PAUSE_AFTER_ROTATE_MS) { /* aguarda */ }
 
    // volta (inverte)
    MOT_PORT |=  (1<<M1_IN2);
    MOT_PORT &= ~((1<<M1_IN1) | (1<<M2_IN2));
    MOT_PORT |=  (1<<M2_IN1);
    motors_pwm(255);
 
    t0 = millis();
    while (millis() - t0 < ROTATE_TIME_MS) { /* aguarda */ }
 
    motors_stop();
}
 
/* Função que realiza reação ao LDR ser atingido uma vez:
   - apaga 1 LED
   - desliga laser (por segurança)
   - faz sequência de girar 180, pausar 5s, voltar 180
*/
void do_reaction_once(void) {
    if (lives == 0) return;
    reaction_active = true;
 
    // Apaga um LED: usa ordem PB0 -> PB1 -> PB2
    uint8_t idx_to_turn_off = 3 - lives; // se lives=3 -> idx=0; lives=2 -> idx=1; lives=1 -> idx=2
    led_off_index(idx_to_turn_off);
 
    // Laser off (manter desligado durante reação)
    LASER_PORT &= ~(1<<LASER_PIN);
 
    // Sequência: girar 180, pausar, voltar
    rotate_180_and_back();
 
    // decrementa vidas
    if (lives > 0) lives--;
    if (lives == 0) {
        system_locked = true;
        motors_stop();
        // laser já desligado
    } else {
        // após reação, laser voltará a piscar no loop principal
    }
 
    reaction_active = false;
}
 
/* Processa comando recebido por bluetooth */
void process_command(char c) {
    if (reaction_active) return; // ignorar comandos durante reação
    if (system_locked) {
        if (c == 'X' || c == 'x') {
            // reset system
            lives = 3;
            leds_all_on();
            system_locked = false;
            // laser pode voltar a piscar
        }
        return;
    }
 
    switch (c) {
        case 'F': case 'f': motors_forward(); break;
        case 'B': case 'b': motors_back(); break;
        case 'L': case 'l': motors_turn_left(); break;
        case 'R': case 'r': motors_turn_right(); break;
        case 'S': case 's': motors_stop(); break;
        case 'X': case 'x': // reset from BT
            lives = 3;
            leds_all_on();
            system_locked = false;
            break;
        default:
            // ignorar
            break;
    }
}
 
/* main */
int main(void) {
    /* portas */
    LED_DDR |= (1<<LED_PIN0)|(1<<LED_PIN1)|(1<<LED_PIN2);
    // start LEDs off; depois ligamos
    LED_PORT &= ~((1<<LED_PIN0)|(1<<LED_PIN1)|(1<<LED_PIN2));
 
    LASER_DDR |= (1<<LASER_PIN);
    LASER_PORT &= ~(1<<LASER_PIN);
 
    // motor input pins on PORTC
    MOT_DDR |= (1<<M1_IN1)|(1<<M1_IN2)|(1<<M2_IN1)|(1<<M2_IN2);
 
    // initialize motors outputs low
    MOT_PORT &= ~((1<<M1_IN1)|(1<<M1_IN2)|(1<<M2_IN1)|(1<<M2_IN2));
 
    // Inicializa ADC, USART, Timer0 (PWM+millis)
    adc_init();
    usart_init();
    timer0_pwm_millis_init();
 
    sei(); // enable interrupts
 
    // estado inicial
    leds_all_on();
    system_locked = false;
    lives = 3;
 
    uint32_t last_laser_toggle = millis();
    bool laser_state = true;
    LASER_PORT |= (1<<LASER_PIN); // laser ligado inicialmente
    last_laser_toggle = millis();
 
    uint32_t last_ldr_sample = 0;
    const uint16_t threshold = ADC_THRESHOLD;
 
    for (;;) {
        // 1) piscar laser (quando sistema não bloqueado e não em reação)
        if (!system_locked && !reaction_active) {
            uint32_t now = millis();
            if (now - last_laser_toggle >= LASER_BLINK_INTERVAL_MS) {
                laser_state = !laser_state;
                if (laser_state) LASER_PORT |= (1<<LASER_PIN);
                else LASER_PORT &= ~(1<<LASER_PIN);
                last_laser_toggle = now;
            }
        } else {
            // se sistema bloqueado ou em reação, manter laser desligado
            LASER_PORT &= ~(1<<LASER_PIN);
        }
 
        // 2) ler UART não-bloqueante
        int r = uart_read_nonblocking();
        if (r >= 0) {
            process_command((char)r);
        }
 
        // 3) LDR sampling (não enquanto bloqueado)
        if (!system_locked && !reaction_active) {
            uint32_t now = millis();
            if (now - last_ldr_sample >= DEBOUNCE_MS) {
                last_ldr_sample = now;
                // ADC4 = channel 4
                uint16_t val = adc_read(4);
                if (val >= threshold) {
                    // luz forte detectada -> reação
                    do_reaction_once();
                }
            }
        }
 
        // loop leve
    }
 
    return 0;
}