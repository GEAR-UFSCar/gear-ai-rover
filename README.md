# GEAR AI Rover

> Rover autônomo com visão computacional: um carrinho educacional que procura, reconhece e vai até objetos usando YOLO11, câmera ESP32-S3 e ultrassônico.

**Trilha:** Projetos  **Responsável:** @mchqueiroz
**Estado:** em andamento

[![CI](https://github.com/GEAR-UFSCar/gear-ai-rover/actions/workflows/ci.yml/badge.svg)](https://github.com/GEAR-UFSCar/gear-ai-rover/actions/workflows/ci.yml)

## Contexto

Estamos transformando o kit LAFVIN Smart Robot Car with Camera num rover que recebe uma missão ("encontre uma garrafa") e decide sozinho onde olhar, para onde ir e quando parar. A visão e a decisão rodam no computador; o carrinho só executa comandos de motor e servo e devolve leituras dos sensores. Visão completa, regras e fases do projeto: [`docs/contexto-mestre.md`](docs/contexto-mestre.md).

## Arquitetura

```
 Computador (Python)                        Rover
┌───────────────────────────┐   Wi-Fi    ┌──────────────────┐  serial  ┌────────────────────┐
│ YOLO11 → busca → alinha   │◄──MJPEG────│  ESP32-S3-CAM    │◄────────►│ Arduino UNO        │
│ → aproxima → para         │───TCP/JSON►│  (AP 192.168.4.1)│          │ motores, servo,    │
│ GUI Tkinter + log         │            └──────────────────┘          │ ultrassônico, LED  │
└───────────────────────────┘                                          └────────────────────┘
```

## Estado atual

| Programa | O que faz | Situação |
|---|---|---|
| `src/gear_ai_rover_basico.py` | Busca por ângulos fixos, confirma o alvo, alinha, aproxima reduzindo a velocidade e para; desvio pelo ultrassônico | **Versão recomendada** |
| `src/gear_ai_rover_v21.py` | Tudo da básica + DQN, odometria visual, Kalman, A*, mapa persistente, curiosidade, ONNX | Experimental ([por quê](docs/decisoes/ADR-0001-versao-basica.md)) |

Funciona: controle por TCP, stream da câmera, detecção YOLO, busca e aproximação.
Falta: mapeamento confiável, planejamento de caminho e missões com vários objetivos (fases seguintes do contexto mestre).

## Requisitos

- Python 3.12 e Wi-Fi para conectar à rede do rover
- Hardware: [`docs/bom.md`](docs/bom.md)
- Para gravar firmware: [arduino-cli](https://arduino.github.io/arduino-cli/) ou Arduino IDE 2

## Instalação e execução

```bash
git clone https://github.com/GEAR-UFSCar/gear-ai-rover.git
cd gear-ai-rover
make venv      # cria .venv e instala requirements.txt
make modelo    # baixa src/yolo11n.pt
```

1. Ligue o rover e conecte o computador à rede Wi-Fi `LAFVIN-...` criada pela ESP32 (sem senha).
2. Rode `make basico` (ou `make v21`).
3. Na janela, conecte, escolha a classe-alvo (padrão: `bottle`) e inicie.

Os pesos já treinados do DQN da v21 (`rover_dqn.pt`) estão na página de [Releases](https://github.com/GEAR-UFSCar/gear-ai-rover/releases); coloque-os em `src/`.

## Firmware

| Placa | Pasta | Como gravar |
|---|---|---|
| Arduino UNO | `firmware/arduino-uno/SmartRobotCar_Main_V1` | `arduino-cli compile --profile uno` e `arduino-cli upload --profile uno -p <porta>` (o `sketch.yaml` fixa FastLED 3.5.0 + Servo 1.3.0) |
| ESP32-S3-CAM | `firmware/esp32s3-cam/ESP32S3_Cam_Main_V1` | Arduino IDE com *ESP32S3 Dev Module*, Flash 8 MB, *8M with spiffs*, QIO 80 MHz, OPI PSRAM; ou grave o `.bin` das Releases |

O firmware do UNO é o do fabricante com modificações do GEAR; o da ESP32 é o original. Para ver exatamente o que o GEAR mudou, abra o commit `feat(firmware)` no histórico (o commit anterior é o código original do fabricante).

> **Atenção:** o firmware modificado não bloqueia mais o avanço quando há obstáculo à frente; o desvio é responsabilidade do programa em Python. Sem o Python conectado, o rover anda para onde for mandado. A rede Wi-Fi da ESP32 é aberta (padrão do fabricante).

### Protocolo (computador → ESP32, TCP porta 100)

| Comando | Significado |
|---|---|
| `{"N":102,"D1":<1-9>,"D2":<vel>}` | Motor: direção e velocidade |
| `{"N":5,"D1":1,"D2":<ângulo>}` | Servo em ângulo absoluto |
| `{"N":21}` | Pede leitura do ultrassônico |
| `{"N":100}` | Parada explícita (limpa modo e LEDs) |
| `{Heartbeat}` | Mantém a conexão viva (texto cru, não JSON) |

Stream da câmera: `http://192.168.4.1:81/stream`.

## Testes

```bash
make test       # verifica se todos os programas compilam
make firmware   # compila o firmware do UNO
```

O CI roda os dois a cada push.

## Estrutura

```
src/          programas do rover (Python)
firmware/     Arduino UNO (modificado) e ESP32-S3-CAM (original do fabricante)
tests/        testes automatizados
docs/         contexto mestre, decisões (ADR), lista de materiais
```

## Segurança

- Risco físico baixo: rodas e servo em movimento e bateria de lítio do kit. Teste no chão, longe de escadas, e desligue a bateria ao gravar firmware.
- Sem dados pessoais. A câmera só transmite na rede local do rover.

## Licença e autoria

Código do GEAR sob [MIT](LICENSE); as partes do fabricante seguem os termos dele. Desenvolvido pelo GEAR — Grupo de Estudo e Aprendizagem em Robótica (UFSCar).

## Contato

Abra uma *issue* neste repositório.
