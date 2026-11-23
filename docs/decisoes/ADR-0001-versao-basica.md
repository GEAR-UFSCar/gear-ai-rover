# ADR-0001: manter uma versão básica ao lado da v21

**Data:** 2026-09-17  **Estado:** aceita

## Contexto

A v21 (`src/gear_ai_rover_v21.py`) juntou DQN, odometria visual, Kalman, A*, mapa persistente, curiosidade (ICM) e ONNX. Os módulos competiam entre si e o bug real de comportamento (via o alvo e continuava girando) vinha da busca contínua: o servo varria enquanto tentava confirmar o alvo, então a confirmação nunca fechava.

## Decisão

Reescrever do zero uma versão mínima (`src/gear_ai_rover_basico.py`): busca por ângulos fixos com a câmera parada, confirmação por quadros seguidos, trava de identidade do alvo e desvio só pelo ultrassônico. Recursos da v21 só voltam um de cada vez, quando fizerem falta medida.

## Consequências

- O comportamento fica previsível e fácil de depurar.
- A v21 continua no repositório como referência e banco de ideias.
