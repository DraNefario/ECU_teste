// chip.c - Simulador de Motor Toyota 4+1
#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct {
  pin_t pin_ne;
  pin_t pin_g;
  uint32_t tooth_counter; // Contador de 0 a 3
  timer_t timer_id;
} chip_state_t;

void chip_timer_callback(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;

  // Lógica: 4 dentes no distribuidor = 4 eventos por ciclo de 720 graus
  // NE sempre pulsa
  pin_write(chip->pin_ne, HIGH);

  // G pulsa apenas no primeiro dente (Sincronia Cilindro 1)
  if (chip->tooth_counter == 0) {
    pin_write(chip->pin_g, HIGH);
  }

  // Simula a largura do pulso (dwell do sensor) - 1ms
  // Nota: O timer do Wokwi não bloqueia, mas para simplicidade aqui usaremos
  // um timer one-shot interno ou apenas desligaremos no proximo ciclo se fosse mais complexo.
  // Como não temos delay() aqui, vamos fazer o pulso descer rapidinho na simulação? 
  // Na verdade, precisamos agendar o desligamento. 
  // Para simplificar: vamos manter alto e desligar na próxima chamada? Não, isso cria onda quadrada 50%.
  // Vamos assumir onda quadrada 50% para simplificar a visualização no Logic Analyzer.
  
  // -- ESTADO LOGICO --
  // Para fazer um pulso curto, o ideal seria outro timer, mas vamos fazer 
  // uma onda quadrada simples: HIGH agora, LOW depois.
  // Vamos simplificar: O chip emite pulsos curtos (1ms seria ideal).
  // Como a API C do Wokwi é evento-baseada, vamos deixar HIGH e usar um timer auxiliar?
  // Não, vamos simplificar: O pino fica HIGH agora, e eu vou assumir que o usuário
  // configura o ESP32 para ler borda de subida (RISING).
  // Logo em seguida (na simulação é instantaneo se nao houver delay) baixamos? Não.
  
  // HACK PARA WOKWI SIMPLES:
  // Vamos inverter o estado no proximo tick? Não.
  // Vamos apenas setar HIGH, e agendar um timer para setar LOW em 1ms.
}

// Timer para desligar os pinos (criar o pulso curto)
void pin_reset_callback(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  pin_write(chip->pin_ne, LOW);
  pin_write(chip->pin_g, LOW);
}

// Timer principal de Rotação
void engine_cycle_callback(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;

  // 1. Sobe os sinais
  pin_write(chip->pin_ne, HIGH);
  if (chip->tooth_counter == 0) {
    pin_write(chip->pin_g, HIGH);
  }

  // 2. Incrementa contador (0, 1, 2, 3)
  chip->tooth_counter = (chip->tooth_counter + 1) % 4;

  // 3. Agenda o desligamento do pulso em 2ms (simulando largura do dente)
  timer_t reset_timer = timer_init(&(timer_config_t){
    .callback = pin_reset_callback,
    .user_data = chip,
  });
  timer_start(reset_timer, 2000, false); // 2000 micros = 2ms
}

void chip_init() {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  chip->pin_ne = pin_init("NE", OUTPUT);
  chip->pin_g = pin_init("G", OUTPUT);
  chip->tooth_counter = 0;

  const timer_config_t timer_cfg = {
    .callback = engine_cycle_callback,
    .user_data = chip,
  };
  chip->timer_id = timer_init(&timer_cfg);

  // CONFIGURAÇÃO DE RPM
  // 1000 RPM = 16.6 Hz virabrequim = 8.3 Hz comando
  // 4 eventos por volta do comando = 33.3 Hz freq de eventos
  // Periodo = 1/33.3 = 30ms = 30000 micros
  timer_start(chip->timer_id, 30000, true); // Dispara a cada 30ms

  printf("Simulador 7A-FE Iniciado - 1000 RPM Fixo\n");
}