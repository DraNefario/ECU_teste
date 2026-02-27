#include <Arduino.h>

// --- PINOUT ---
#define PIN_NE 4  // Sinal de Rotação (4 pulsos/ciclo)
#define PIN_G  5  // Sinal de Fase (1 pulso/ciclo)

// --- VARIÁVEIS VOLÁTEIS (Usadas dentro da interrupção) ---
volatile unsigned long lastNeTime = 0;
volatile unsigned long currentRpmPeriod = 0;
volatile int cylinderCounter = -1; // -1 significa "não sincronizado"
volatile bool syncState = false;   // True quando recebemos o primeiro G
volatile unsigned long pulseCountTotal = 0;

// --- INTERRUPÇÕES (IRAM_ATTR para rodar na RAM rápida) ---

// Dispara a cada pulso G (1 vez a cada 720 graus)
void IRAM_ATTR onPulseG() {
  // O sinal G diz: "O próximo pulso NE é o Cilindro 1 (ou o atual é)"
  // No 4A-FE/7A-FE, G geralmente acontece junto ou pouco antes do NE #1.
  // Vamos resetar nosso contador lógico.
  cylinderCounter = 0; 
  syncState = true;
}

// Dispara a cada pulso NE (4 vezes a cada 720 graus)
void IRAM_ATTR onPulseNE() {
  unsigned long now = micros();
  
  // Cálculo de período (RPM)
  if (lastNeTime > 0) {
    currentRpmPeriod = now - lastNeTime;
  }
  lastNeTime = now;
  pulseCountTotal++;

  // Lógica de Sequência de Injeção
  if (syncState) {
    // Se estamos sincronizados, incrementamos o contador de cilindros (0 a 3)
    // 0 = Cilindro 1 (Combustão)
    // 1 = Cilindro 3
    // 2 = Cilindro 4
    // 3 = Cilindro 2
    // Nota: O incremento é feito DEPOIS de processar o evento atual, ou antes?
    // Se G reseta para 0, este pulso é o 0. O próximo será 1.
    // Vamos incrementar apenas se não acabamos de resetar (evitar conflito de corrida G e NE)
    // Uma forma segura é incrementar, e se passar de 3, volta a 0.
    // O sinal G serve para CORRIGIR se sair de sincronia.
    
    // Simples incremento circular:
    cylinderCounter++;
    if (cylinderCounter > 3) cylinderCounter = 0;
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_NE, INPUT);
  pinMode(PIN_G, INPUT);

  // Configura interrupções na borda de SUBIDA
  attachInterrupt(digitalPinToInterrupt(PIN_G), onPulseG, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_NE), onPulseNE, RISING);

  Serial.println("--- ECU 7A-FE START ---");
  Serial.println("Aguardando sincronia do distribuidor...");
}

void loop() {
  static unsigned long lastPrint = 0;
  
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    
    // Copia segura das variáveis voláteis
    noInterrupts();
    unsigned long period = currentRpmPeriod;
    int currentCyl = cylinderCounter;
    bool isSynced = syncState;
    interrupts();

    // Cálculo de RPM para exibição
    // 4 pulsos NE = 720 graus virabrequim (2 voltas)
    // 2 pulsos NE = 1 volta virabrequim
    // RPM = (1.000.000 us / periodo_us) * (60s) / (2 pulsos/volta)
    // Simplificando: 30.000.000 / periodo
    
    long rpm = 0;
    if (period > 0) rpm = 30000000 / period;

    Serial.print("Status: ");
    Serial.print(isSynced ? "[SYNC OK]" : "[NO SYNC]");
    Serial.print(" | RPM: ");
    Serial.print(rpm);
    Serial.print(" | Prox. Cilindro (Firing): ");
    
    // Mapeamento da ordem de ignição 1-3-4-2
    // Se o contador é 0 (TDC #1), estamos queimando o 1.
    if (isSynced) {
      switch(currentCyl) {
        case 1: Serial.print("1"); break; // Acabou de passar o pulso 0, estamos no ciclo do 1? 
        // Ajuste fino: O contador incrementa no pulso. 
        // Pulso 0 (G+NE) -> Cilindro 1 TDC Comp.
        // Pulso 1 -> Cilindro 3 TDC Comp.
        // Pulso 2 -> Cilindro 4 TDC Comp.
        // Pulso 3 -> Cilindro 2 TDC Comp.
        case 0: Serial.print("Cil #1"); break; // 0 acabou de acontecer
        case 1: Serial.print("Cil #3"); break;
        case 2: Serial.print("Cil #4"); break;
        case 3: Serial.print("Cil #2"); break;
      }
    } else {
      Serial.print("?");
    }
    
    Serial.println();
  }
}