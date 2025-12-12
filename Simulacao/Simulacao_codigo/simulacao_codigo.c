#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

// ====== DEFINIÇÕES ======
#define LDR_CHANNEL 4      // ADC4 = A4
#define BUTTON_PIN  PD4
#define LASER_PIN   PD7

#define LED1 PB0
#define LED2 PB1
#define LED3 PB2

// Motores L293D
#define EN12 PD5
#define EN34 PD6
#define M1A PC0
#define M1B PC1
#define M2A PC2
#define M2B PC3

// ====== VARIÁVEIS ======
uint8_t vidas = 3;
uint8_t laser_state = 0;

// ===============================
//       Leitura do ADC (LDR)
// ===============================
uint16_t read_adc(uint8_t ch)
{
    ADMUX = (1 << REFS0) | (ch & 0x07);   // AVcc + canal
    ADCSRA |= (1 << ADSC);               // inicia conversão
    while (ADCSRA & (1 << ADSC));        // espera terminar
    return ADC;
}

// ===============================
//     Controle do Laser (pisca)
// ===============================
void laser_blink()
{
    laser_state ^= 1;
    if (laser_state)
        PORTD |= (1 << LASER_PIN);
    else
        PORTD &= ~(1 << LASER_PIN);
}

// ===============================
//       Controle dos motores
// ===============================
void motores_parar() {
    PORTD &= ~((1 << EN12) | (1 << EN34));
}

void motores_frente() {
    PORTC = (1 << M1A) | (1 << M2A);
    PORTD |= (1 << EN12) | (1 << EN34);
}

void motores_direita() {
    PORTC = (1 << M1A) | (1 << M2B);
    PORTD |= (1 << EN12) | (1 << EN34);
}

void motores_esquerda() {
    PORTC = (1 << M1B) | (1 << M2A);
    PORTD |= (1 << EN12) | (1 << EN34);
}

void motores_re() {
    PORTC = (1 << M1B) | (1 << M2B);
    PORTD |= (1 << EN12) | (1 << EN34);
}

void motores_giro180() {
    PORTC = (1 << M1A) | (1 << M2B);
    PORTD |= (1 << EN12) | (1 << EN34);
    _delay_ms(850);   // ajuste fino conforme motor real
    motores_parar();
}

// ===============================
//          MAIN
// ===============================
int main(void)
{
    // LEDs
    DDRB |= (1 << LED1) | (1 << LED2) | (1 << LED3);

    // Laser
    DDRD |= (1 << LASER_PIN);

    // Motores
    DDRD |= (1 << EN12) | (1 << EN34);
    DDRC |= (1 << M1A) | (1 << M1B) | (1 << M2A) | (1 << M2B);

    // Botão PD4
    DDRD &= ~(1 << BUTTON_PIN);
    PORTD |= (1 << BUTTON_PIN); // pull-up

    // ADC config
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Prescaler 64

    // LEDs começam acesos
    PORTB |= (1 << LED1) | (1 << LED2) | (1 << LED3);

    uint16_t ldr;
    uint16_t threshold = 300;   // mais sensível!

    while (1)
    {
        // ========================
        //  Vida acabou → aguarda botão
        // ========================
        if (vidas == 0) 
        {
            motores_parar();
            PORTD &= ~(1 << LASER_PIN);

            while (PIND & (1 << BUTTON_PIN)); // espera reset

            vidas = 3;
            PORTB |= (1 << LED1) | (1 << LED2) | (1 << LED3);
        }

        // Pisca laser a cada 500ms
        laser_blink();

        // Faz o ciclo dos motores com checagem do LDR entre passos
        for (uint8_t etapa = 0; etapa < 4; etapa++)
        {
            if (etapa == 0) motores_frente();
            if (etapa == 1) motores_direita();
            if (etapa == 2) motores_esquerda();
            if (etapa == 3) motores_re();

            // 2s por etapa, dividido em 4 blocos de 500ms
            for (uint8_t i = 0; i < 4; i++)
            {
                // Pisca laser
                laser_blink();

                // Leitura do LDR
                ldr = read_adc(LDR_CHANNEL);

                if (ldr > threshold)
                {
                    vidas--;

                    if (vidas == 2) PORTB &= ~(1 << LED3);
                    else if (vidas == 1) PORTB &= ~(1 << LED2);
                    else if (vidas == 0) PORTB &= ~(1 << LED1);

                    PORTD &= ~(1 << LASER_PIN);

                    motores_giro180();
                    motores_parar();

                    for (uint8_t p = 0; p < 5; p++)
                        _delay_ms(1000);

                    goto continuar_loop;
                }

                _delay_ms(500);
            }
        }

        continuar_loop:
        continue;
    }
}
