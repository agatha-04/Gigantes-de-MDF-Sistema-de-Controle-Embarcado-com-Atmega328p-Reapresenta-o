# Gigantes-de-MDF-Sistema-de-Controle-Embarcado-com-Atmega328p-Reapresenta-o
Firmware, Eletrônica e Documentação Técnica do projeto final da disciplina de Programação de Hardware - 2025.

## Descrição Geral
Esse projeto implementa o sistema eletrônico e programação em C ao robô do projeto final da disciplina Programação de Hardware (2025).
Ele deve:
* Se mover de acordo com os comandos enviados pelo celular via bluetooth (módulo HC-05);
* Controlar os motores por via PWM;
* Detectar impactos do laser inimigo através de um LDR;
* Disparar sua própria arma laser de 1 em 1 segundo;
* Registrar danos (causados pelo laser inimigo) através de LEDs que representam as vidas;
* Reagir ao ser atingido (giro de 180º, perda de uma vida, pausa de 5 segundos);
* Parar o sistema inteiro após a perda das três vidas, reiniciando após apertar o botão de reset.

## Objetivos
* Aplicar conceitos de programação de hardware usando o microcontrolador ATmega328p.
* Projetar um circuito completo de acionamento de motores.
* Implementar controle PWM para movimentação.
* Implementar timers para disparo do laser e rotina de 1 segundo.
* Programar rotinas de interrupção para o botão reset das vidas.
* Criar documentação com o Doxygen.
* Disponilizar todo o código em repositório GitHub.

## Componentes Eletrônicos Usados no Projeto
* 1 microcontrolador do tipo ATmega328p.
* 1 diodo laser de 5V/5mW.
* 1 LDR de 20 milímetros.
* 3 LEDs azuis 10 milímetros.
* 1 CI de ponte H L293d.
* 2 motores DC com faixa de operação de 3V-6V.
* 1 módulo bluetooth HC-05 (para comunicação do controle no celular com o atmega que controla o carrinho).
* Regulador de tensão de 5V (LM7805).
* Regulador de tensão dd 6V (LM7806).
* 2 baterias de 9V.

## Como Testar
**1.** Gravar o arquivo main.ino.hex no respectivo microcontrolador.
**2.** Conectar as baterias ao robô.
**3.** Testar as respectivas funções:
     * Giro no eixo;
     * PWM dos motores;
     * Disparo do laser:
     * Resposta ao laser inimigo;
     * LEDs de vida;
     * Botão de reset.

## Autores
* **Autor(a):** Agatha Nicole Marques Fávaro - **RA:** 231335
* **Autor(a):** Mykaella Scarllet de Lemos Santana - **RA:** 233508
* **Autor(a):** Rafael Ribeiro de Lima - **RA:** 226326
* Universidade Santa Cecília - Engenharia da Computação
* **Disciplina:** Programação de Hardware - 2/2025
* **Professor(a):** Sérgio Schina de Andrade
