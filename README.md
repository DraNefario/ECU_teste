# ECU_teste

Arquitetura modular para ECU standalone no ESP32-S3:

- `core/` lógica pura da ECU (testável com `g++`)
- `hal/` abstração de hardware ESP32/Arduino
- `app/` integração de ISR/tasks/setup/loop
- `tests/` testes unitários da camada core
- `tools/` scripts de amalgamação para gerar sketch único do Wokwi
- `wokwi/sketch.ino` arquivo único gerado automaticamente

## Amalgamação para Wokwi

### Soluções disponíveis

1. `tools/make_wokwi_sketch_manual.py`
   - **Ponto forte:** ordem 100% determinística e explícita.
   - **Limitação:** manutenção manual quando surgem novos arquivos.

2. `tools/make_wokwi_sketch_parse.py`
   - **Ponto forte:** usa parsing de `#include "..."` + ordenação topológica.
   - **Limitação:** depende da qualidade dos includes para ordenar tudo.

3. `tools/make_wokwi_sketch_heuristic.py`
   - **Ponto forte:** fallback robusto mesmo sem grafo de includes perfeito.
   - **Limitação:** ordenação inferida (menos previsível que manual).

### Versão final consolidada (recomendada)

`tools/make_wokwi_sketch.py` combina os melhores pontos:

- ordem manual preferencial para os módulos críticos
- resolução de dependências por includes locais com topological sort
- fallback heurístico para casos remanescentes/ciclos
- saída idempotente em `wokwi/sketch.ino`
- sem `tests/` no sketch gerado
- `#include <Arduino.h>` apenas uma vez
- comentários de início/fim de módulo com origem do arquivo

## Comandos

```bash
# recomendado
python tools/make_wokwi_sketch.py

# alternativas independentes
python tools/make_wokwi_sketch_manual.py
python tools/make_wokwi_sketch_parse.py
python tools/make_wokwi_sketch_heuristic.py
```

## Simulação no Wokwi

1. Gere `wokwi/sketch.ino` com o script recomendado.
2. Copie o conteúdo para o projeto Wokwi.
3. Conexão do potenciômetro TPS:
   - `SIG -> GPIO34`
   - `VCC -> 3V3`
   - `GND -> GND`
4. Execute e monitore o Serial.
