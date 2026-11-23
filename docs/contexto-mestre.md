# GEAR AUTONOMOUS ROVER
## Contexto Mestre do Projeto

> Este documento é o contexto principal do projeto.
> Leia este arquivo antes de modificar qualquer código.
> O objetivo não é simplesmente fazer o carro andar, mas desenvolver progressivamente um sistema de robótica autônoma capaz de perceber, mapear, planejar, agir, aprender e cumprir objetivos em ambientes desconhecidos.

---

# 1. VISÃO DO PROJETO

O projeto é desenvolvido pela GEAR — Grupo de Estudo e Aprendizagem em Robótica.

Estamos transformando um carro robótico educacional em um **rover autônomo experimental**.

O objetivo de longo prazo é criar um sistema capaz de operar com o mínimo possível de intervenção humana.

O rover deve evoluir de:

    carro controlado
        ↓
    carro com sensores
        ↓
    carro que desvia de obstáculos
        ↓
    carro que entende o ambiente
        ↓
    carro que constrói memória espacial
        ↓
    carro que planeja caminhos
        ↓
    carro que explora autonomamente
        ↓
    agente autônomo orientado por objetivos

A prioridade absoluta é produzir **comportamento autônomo real, robusto e mensurável**, e não simplesmente adicionar técnicas de IA ao código.

---

# 2. OBJETIVO FINAL

O objetivo final é que o rover consiga receber uma missão de alto nível, por exemplo:

    "Explore o ambiente e encontre uma garrafa."

e seja capaz de decidir sozinho:

    onde olhar
    para onde andar
    quando parar
    quando virar
    quando investigar
    quando evitar uma região
    quando considerar um caminho ruim
    quando voltar
    quando procurar uma nova região
    quando utilizar a câmera
    quando utilizar o ultrassônico
    quando realizar uma varredura
    quando replanejar
    quando considerar a missão concluída

O humano deve fornecer principalmente:

    OBJETIVO

e não:

    SEQUÊNCIA DE MOVIMENTOS.

---

# 3. HARDWARE ATUAL

O sistema possui aproximadamente a seguinte arquitetura física:

    ESP32-S3-CAM
          │
          │ Wi-Fi / vídeo
          ↓
       COMPUTADOR
          │
          │ comandos / dados
          ↓
       ARDUINO
          │
       ┌──┴───────────┐
       ↓              ↓
    MOTORES         SERVO
                       │
                       ↓
                    CÂMERA /
                 SENSOR DIRECIONAL

Existe também um sensor ultrassônico utilizado para percepção de obstáculos.

---

# 4. ESP32-S3-CAM

A ESP32-S3-CAM é utilizada principalmente para:

- câmera;
- streaming de vídeo;
- comunicação sem fio.

Informações utilizadas anteriormente:

IP:

    192.168.4.1

Stream:

    http://192.168.4.1:81/stream

TCP:

    192.168.4.1:100

IMPORTANTE:

O Python deve utilizar a câmera do ESP32-S3-CAM.

NÃO utilizar automaticamente:

- webcam do notebook;
- câmera padrão do OpenCV;
- `/dev/video0`;
- índice `0` da câmera;

sem confirmação explícita.

Se houver configuração de câmera, ela deve apontar para o stream do ESP32.

---

# 5. ARDUINO

O Arduino é responsável pelo controle de baixo nível.

Ele deve permanecer responsável principalmente por:

- motores;
- direção;
- servo;
- comandos físicos;
- comunicação com o computador.

Não transferir lógica complexa para o Arduino sem necessidade.

O computador Python deve permanecer responsável pelo raciocínio de alto nível sempre que possível.

---

# 6. SENSOR ULTRASSÔNICO

O ultrassônico é atualmente uma das principais fontes de percepção de obstáculos.

Ele permite estimar:

    distância até obstáculo

Porém, o ultrassônico sozinho não fornece compreensão completa do ambiente.

Ele deve ser tratado como uma fonte de informação dentro de uma arquitetura de percepção maior.

Não assumir que uma única leitura representa perfeitamente a realidade.

Considerar:

- ruído;
- leituras inválidas;
- obstáculos inclinados;
- superfícies que refletem mal;
- distância mínima/máxima;
- mudanças abruptas;
- necessidade de filtragem.

---

# 7. SERVO

O servo movimenta a câmera/sistema de sensoriamento.

IMPORTANTE:

O servo não deve ficar movimentando constantemente.

A câmera já possui uma amplitude considerável.

O objetivo é utilizar o servo como **sensoriamento ativo**.

Exemplo:

    rover detecta situação de decisão
             ↓
        pequena varredura
             ↓
       esquerda / centro / direita
             ↓
       comparar informações
             ↓
       escolher direção

O movimento deve ser:

- suave;
- lento;
- limitado;
- necessário;
- temporizado;
- previsível.

Evitar:

- oscilações;
- comandos repetitivos;
- movimentações muito rápidas;
- mudança de posição a cada ciclo;
- servo sendo utilizado sem necessidade.

---

# 8. PROBLEMA CENTRAL

O principal problema identificado até agora NÃO é a potência dos motores.

O problema principal é:

# NAVEGAÇÃO INEFICIENTE.

Um algoritmo simples tende a fazer:

    andar
      ↓
    obstáculo
      ↓
    virar
      ↓
    andar
      ↓
    obstáculo
      ↓
    virar

Isso pode causar:

- loops;
- revisitação;
- movimentos redundantes;
- exploração ruim;
- becos sem saída;
- indecisão;
- comportamento oscilatório;
- retorno para regiões ruins;
- ficar preso;
- escolha ruim entre caminhos.

Portanto:

# NÃO tentar resolver o problema simplesmente adicionando mais regras de desvio.

Precisamos criar uma arquitetura de autonomia.

---

# 9. PRINCÍPIO DE AUTONOMIA

O rover deve funcionar aproximadamente através deste ciclo:

    PERCEBER
       ↓
    ESTIMAR ESTADO
       ↓
    ATUALIZAR MEMÓRIA
       ↓
    CONSTRUIR / ATUALIZAR MAPA
       ↓
    DEFINIR OBJETIVO LOCAL
       ↓
    PLANEJAR
       ↓
    AGIR
       ↓
    OBSERVAR RESULTADO
       ↓
    AVALIAR PROGRESSO
       ↓
    APRENDER / ATUALIZAR MEMÓRIA
       ↓
    NOVO CICLO

Esse ciclo é mais importante que qualquer algoritmo específico.

---

# 10. ARQUITETURA DE SOFTWARE DESEJADA

A arquitetura ideal deve ser modular.

Não criar um único arquivo gigante contendo toda a inteligência.

Idealmente:

    perception/
        camera.py
        ultrasonic.py
        sensor_fusion.py

    localization/
        odometry.py
        pose_estimator.py

    mapping/
        grid_map.py
        memory.py

    planning/
        path_planner.py
        exploration.py
        goal_planner.py

    control/
        motion_controller.py
        servo_controller.py

    behavior/
        state_machine.py
        recovery.py

    vision/
        object_detection.py
        tracking.py

    learning/
        reinforcement_learning.py
        models.py

    communication/
        esp32.py
        arduino.py

    ui/
        dashboard.py

    main.py

A estrutura real do projeto deve ser preservada se já existir uma arquitetura funcional.

NÃO reorganizar tudo sem necessidade.

---

# 11. PERCEPÇÃO

A percepção deve combinar diferentes fontes de informação.

Possíveis fontes:

    ultrassônico
    câmera
    odometria
    orientação
    histórico de movimentos

O sistema deve tentar responder:

    O que está ao redor?
    A que distância?
    Onde estou?
    Para onde estou olhando?
    O que mudou?
    Existe uma região nova?
    Existe um obstáculo?
    Existe um objeto relevante?

---

# 12. FUSÃO SENSORIAL

Quando houver sensores diferentes fornecendo informações relacionadas, considerar fusão sensorial.

Por exemplo:

    ultrassônico
          +
    odometria
          +
    visão
          ↓
    estimativa de estado

Um filtro de Kalman pode ser utilizado quando houver justificativa técnica.

Não implementar Kalman simplesmente por ser uma técnica conhecida.

Primeiro identificar:

- qual estado queremos estimar;
- quais sensores fornecem observações;
- quais são os modelos de ruído;
- qual ganho real teremos.

---

# 13. LOCALIZAÇÃO

O rover precisa estimar sua posição.

Uma primeira abordagem pode utilizar:

- odometria;
- direção;
- distância percorrida;
- histórico de movimentos;
- sensores.

Posteriormente pode-se estudar:

- visual odometry;
- SLAM;
- landmarks;
- outras técnicas de localização.

A precisão real do hardware deve ser considerada.

Não assumir que a posição estimada é perfeita.

---

# 14. MAPA

Uma das principais evoluções desejadas é um mapa do ambiente.

Uma representação inicial pode ser um occupancy grid.

Exemplo:

    ???????????????
    ??????..???????
    ??????..???????
    ?????..###?????
    ?????.....?????
    ?????R....?????
    ???????????????

Onde:

    ? = desconhecido
    . = livre/conhecido
    # = obstáculo
    R = rover

O mapa deve ser atualizado conforme novas informações chegam.

---

# 15. MEMÓRIA ESPACIAL

O rover precisa lembrar:

- onde esteve;
- onde não esteve;
- onde encontrou obstáculos;
- onde ficou preso;
- quais caminhos foram ruins;
- quais caminhos funcionaram;
- regiões já exploradas;
- regiões ainda desconhecidas;
- direção de chegada;
- direção de saída;
- histórico recente de posições.

Isso é fundamental para resolver o problema atual.

---

# 16. EVITAR REVISITAÇÃO

Se duas opções forem semelhantes:

    caminho A = já explorado
    caminho B = desconhecido

e a missão for exploração, normalmente preferir:

    caminho B

Regiões problemáticas devem receber penalização.

A memória deve evitar padrões como:

    A → B → C → B → A → B → C

O objetivo é produzir algo mais próximo de:

    A → B → C → D → E → F

quando o ambiente permitir.

---

# 17. DETECÇÃO DE LOOP

O sistema deve detectar comportamento repetitivo.

Possíveis sinais:

- mesma posição visitada várias vezes;
- mesma sequência de comandos;
- retorno recorrente;
- pouco deslocamento;
- baixa descoberta de novas células;
- alternância esquerda/direita;
- movimentos de vai-e-volta.

Exemplo:

    FRENTE
    DIREITA
    FRENTE
    ESQUERDA
    FRENTE
    DIREITA
    FRENTE
    ESQUERDA

Se isso estiver acontecendo repetidamente, o rover deve considerar:

    POSSÍVEL LOOP

e mudar de estratégia.

---

# 18. DETECÇÃO DE "ESTOU PRESO"

O rover deve saber quando está parado ou praticamente parado.

Indicadores possíveis:

    pouco deslocamento
    +
    motores ativos
    +
    sensores sem mudança significativa
    +
    repetição de comandos

Nesse caso:

    STOP
      ↓
    RECOVERY
      ↓
    RECUAR
      ↓
    NOVA OBSERVAÇÃO
      ↓
    MARCAR REGIÃO / CAMINHO
      ↓
    REPLANEJAR

---

# 19. RECOVERY

A recuperação deve ser inteligente.

Nunca repetir indefinidamente a mesma ação que falhou.

Exemplo:

    tentativa 1
       ↓
    falhou
       ↓
    marcar estratégia como ruim
       ↓
    tentativa 2 diferente
       ↓
    falhou
       ↓
    aumentar penalização
       ↓
    escolher outra rota

O sistema deve ter memória de falhas.

---

# 20. EXPLORAÇÃO

Quando não houver um destino específico, o rover deve explorar.

O objetivo não deve ser simplesmente:

    "ande para frente."

Deve ser:

    "maximizar descoberta de espaço útil minimizando risco e movimento redundante."

Uma função conceitual de pontuação pode considerar:

    novidade
    + segurança
    + acessibilidade
    + distância
    + progresso
    - revisitação
    - risco
    - histórico de falha

---

# 21. FRONTIERS

Para exploração, estudar o conceito de:

# Frontier-based exploration

Uma frontier é aproximadamente uma fronteira entre:

    região conhecida

e

    região desconhecida.

O rover deve procurar regiões onde exista potencial de descoberta.

Em vez de andar aleatoriamente, ele pode escolher:

    "Qual fronteira é mais promissora?"

Isso pode melhorar drasticamente a eficiência da exploração.

---

# 22. PLANEJAMENTO

O sistema deve separar:

    percepção

de:

    planejamento

de:

    controle.

O planejador deve responder:

> Qual caminho é melhor considerando o mapa, objetivo, obstáculos, memória e risco?

Algoritmos possíveis:

- BFS;
- Dijkstra;
- A*;
- D*;
- frontier exploration;
- planejamento baseado em custo.

Começar simples.

A complexidade deve ser adicionada somente quando necessária.

---

# 23. FUNÇÃO DE CUSTO

O caminho escolhido não deve ser baseado apenas em distância.

Um custo futuro pode considerar:

    distância
    + risco
    + proximidade de obstáculos
    + revisitação
    + regiões problemáticas
    + incerteza
    + dificuldade de manobra

Assim:

    caminho curto e perigoso

pode perder para:

    caminho ligeiramente maior e muito mais seguro.

---

# 24. VISÃO COMPUTACIONAL

A câmera deve complementar os sensores de distância.

O ultrassônico pode responder:

    "há alguma coisa?"

A visão pode responder:

    "o que é?"

e:

    "onde está?"

Possíveis capacidades:

- detecção de objetos;
- classificação;
- tracking;
- reconhecimento de alvo;
- estimativa de direção;
- percepção de obstáculos;
- identificação de regiões.

---

# 25. OBJETOS-ALVO

A interface deve permitir definir objetos que o rover deve procurar.

Exemplo:

    OBJETO-ALVO:
    garrafa

O rover deve então:

    explorar
       ↓
    procurar objeto
       ↓
    detectar candidato
       ↓
    verificar
       ↓
    aproximar
       ↓
    confirmar
       ↓
    objetivo concluído

No futuro, múltiplos objetos podem ser permitidos.

---

# 26. BUSCA ORIENTADA A OBJETIVO

A exploração muda quando existe uma missão.

Sem objetivo:

    maximizar exploração.

Com objetivo:

    maximizar probabilidade de encontrar o alvo.

Isso deve alterar o comportamento do planejador.

Por exemplo, se o objeto provavelmente está em determinada região:

    priorizar essa região.

---

# 27. REINFORCEMENT LEARNING

RL pode ser utilizado, mas NÃO deve controlar tudo inicialmente.

A arquitetura preferida é híbrida.

Algoritmos clássicos:

    percepção
    mapa
    localização
    planejamento
    segurança

ML/RL:

    otimização
    decisão de alto nível
    adaptação
    escolha de estratégias

O RL pode aprender:

    qual direção tende a ser melhor;
    qual estratégia de recuperação funciona;
    quais regiões são promissoras;
    como otimizar exploração;
    como escolher entre estratégias.

---

# 28. ESTADO DO RL

Possíveis variáveis:

    posição
    orientação
    distâncias
    obstáculos
    regiões visitadas
    objetivo
    progresso
    histórico recente
    nível de incerteza

---

# 29. AÇÕES DO RL

Exemplos:

    frente
    esquerda
    direita
    ré
    parar
    realizar scan
    escolher frontier
    replanejar

Sempre manter uma camada de segurança independente.

O RL nunca deve conseguir ignorar limites físicos importantes.

---

# 30. RECOMPENSA

Exemplo conceitual:

    + progresso
    + exploração nova
    + aproximação do objetivo
    + segurança
    + conclusão da missão

    - colisão
    - ficar preso
    - loop
    - revisitação excessiva
    - movimento inútil
    - caminho perigoso

A função de recompensa deve ser cuidadosamente projetada.

---

# 31. SAFETY LAYER

Independentemente da inteligência utilizada, deve existir uma camada de segurança.

Arquitetura:

    PLANEJADOR / IA
            ↓
       SAFETY CHECK
            ↓
       CONTROLLER
            ↓
          MOTORES

A safety layer pode impedir:

- colisões;
- velocidade excessiva;
- comandos impossíveis;
- movimentos perigosos;
- ações incompatíveis com sensores.

Se houver perda de comunicação:

    parar motores.

---

# 32. MÁQUINA DE ESTADOS

O rover deve possuir estados explícitos.

Exemplo:

    IDLE
      ↓
    EXPLORING
      ↓
    MOVING
      ↓
    OBSTACLE_DETECTED
      ↓
    SCANNING
      ↓
    PLANNING
      ↓
    MOVING

Possíveis estados adicionais:

    SEARCHING_TARGET
    TRACKING_TARGET
    RECOVERY
    STUCK
    RETURNING_HOME
    GOAL_REACHED
    COMMUNICATION_LOST
    EMERGENCY_STOP

Isso evita lógica caótica espalhada pelo programa.

---

# 33. CONTROLE

A inteligência decide:

    "quero ir para lá."

O controlador decide:

    "como mover os motores para isso."

Não misturar planejamento com PWM/motor control.

A camada de controle deve cuidar de:

- velocidade;
- direção;
- aceleração;
- frenagem;
- curvas;
- servo;
- limites.

---

# 34. MOVIMENTO SUAVE

Evitar:

    parar completamente
    virar bruscamente
    acelerar
    parar
    virar novamente

quando não for necessário.

Preferir:

    controle progressivo;
    pequenas correções;
    curvas suaves;
    velocidades adaptativas.

A velocidade pode diminuir quando:

    obstáculo está próximo;
    incerteza é alta;
    curva é apertada;
    região é desconhecida.

---

# 35. VELOCIDADE ADAPTATIVA

Conceito desejado:

    ambiente aberto
        ↓
    velocidade maior

    ambiente complexo
        ↓
    velocidade menor

    obstáculo próximo
        ↓
    velocidade muito menor

    alta incerteza
        ↓
    observar antes de avançar

Isso aumenta segurança e qualidade de percepção.

---

# 36. SENSORIAMENTO ATIVO

O rover não deve simplesmente receber sensores passivamente.

Ele pode decidir:

> "Preciso de mais informação."

Então:

    reduzir/parar
        ↓
    mover servo
        ↓
    observar
        ↓
    comparar
        ↓
    decidir

Isso é particularmente importante porque o campo de visão/sensoriamento é limitado.

---

# 37. DASHBOARD

A interface Python deve idealmente mostrar:

    vídeo da câmera
    distância ultrassônica
    mapa
    posição estimada
    orientação
    estado
    objetivo
    caminho planejado
    regiões visitadas
    eventos
    logs
    decisões
    status de comunicação

Também deve permitir:

    iniciar autonomia
    parar
    selecionar objetivo
    definir objetos
    controlar manualmente
    visualizar mapa
    limpar mapa
    alterar parâmetros seguros

---

# 38. MODO MANUAL

Sempre manter um modo manual.

O operador deve conseguir assumir controle.

Isso permite:

- testes;
- diagnóstico;
- recuperação;
- calibração;
- demonstrações.

A autonomia deve poder ser interrompida imediatamente.

---

# 39. LOGGING

O sistema deve registrar decisões importantes.

Exemplo:

    [12:30:01] obstacle detected: 32 cm
    [12:30:02] scanning
    [12:30:03] left: 120 cm
    [12:30:03] center: 42 cm
    [12:30:04] right: 85 cm
    [12:30:04] selecting RIGHT
    [12:30:08] progress detected
    [12:30:10] new area discovered

Também registrar:

- loops;
- recovery;
- falhas;
- perda de comunicação;
- decisões do planner;
- detecção de objetos;
- mudanças de estado.

---

# 40. TESTES

Toda alteração relevante deve ser testada.

Não assumir que o código funciona apenas porque compila.

Testar:

### Sensores

- valores normais;
- valores inválidos;
- ruído;
- ausência de leitura.

### Servo

- limites;
- velocidade;
- estabilidade;
- comandos repetidos.

### Motores

- frente;
- ré;
- esquerda;
- direita;
- parada.

### Comunicação

- conexão;
- desconexão;
- reconexão;
- mensagens inválidas.

### Navegação

- espaço aberto;
- corredor;
- obstáculo frontal;
- obstáculo lateral;
- beco sem saída;
- ambiente com loops;
- ambiente desconhecido.

---

# 41. MÉTRICAS

Não avaliar "inteligência" apenas visualmente.

Registrar:

    distância percorrida
    área explorada
    tempo
    número de colisões
    número de loops
    número de recuperações
    regiões revisitadas
    taxa de descoberta
    sucesso da missão
    distância até objetivo
    eficiência do caminho

Uma métrica especialmente importante:

    ÁREA NOVA DESCOBERTA / DISTÂNCIA PERCORRIDA

Quanto maior, melhor a eficiência de exploração.

---

# 42. EXPERIMENTAÇÃO

Criar ambientes controlados.

Por exemplo:

### Ambiente A

Poucos obstáculos.

### Ambiente B

Corredores.

### Ambiente C

Becos sem saída.

### Ambiente D

Múltiplas rotas.

### Ambiente E

Obstáculos distribuídos aleatoriamente.

### Ambiente F

Ambiente nunca visto anteriormente.

O objetivo é testar generalização.

---

# 43. SIMULAÇÃO

Antes de testar alterações perigosas no rover físico, considerar criar um ambiente simulado.

A simulação pode permitir:

    mapa
    obstáculos
    rover
    sensores simulados
    ruído
    objetivos

Isso permite testar algoritmos de:

- exploração;
- planejamento;
- RL;
- recuperação.

Depois transferir para hardware real.

---

# 44. SIM-TO-REAL

Se RL for utilizado, considerar:

    simulação
       ↓
    treinamento
       ↓
    testes em ambientes diferentes
       ↓
    transferência
       ↓
    rover real

Nunca assumir que uma política treinada em simulação funcionará perfeitamente no hardware.

Considerar:

- ruído;
- latência;
- erro de odometria;
- diferença de motores;
- iluminação;
- obstáculos reais.

---

# 45. MACHINE LEARNING

Não adicionar ML sem necessidade.

Para cada componente, perguntar:

    Existe um problema que um algoritmo clássico não resolve bem?

Se não:

    usar método clássico.

Se sim:

    avaliar ML.

Priorizar métodos simples antes de modelos complexos.

---

# 46. HIERARQUIA DE INTELIGÊNCIA

A arquitetura ideal pode ser:

    MISSÃO
       ↓
    OBJETIVO
       ↓
    PLANEJAMENTO GLOBAL
       ↓
    PLANEJAMENTO LOCAL
       ↓
    COMPORTAMENTO
       ↓
    CONTROLE
       ↓
    ATUADORES

Paralelamente:

    SENSORES
       ↓
    PERCEPÇÃO
       ↓
    ESTADO
       ↓
    MAPA / MEMÓRIA

E:

    EXPERIÊNCIA
       ↓
    APRENDIZADO
       ↓
    MELHORIA DAS DECISÕES

---

# 47. PRINCÍPIO DE "WORLD MODEL"

O objetivo de longo prazo é que o rover mantenha uma representação interna do ambiente.

Ele deve conseguir representar aproximadamente:

    onde estou
    onde estive
    onde existem obstáculos
    onde existe espaço livre
    onde ainda não sei o que existe
    onde tive problemas
    onde quero chegar
    quais caminhos são melhores

Isso é muito mais importante que simplesmente adicionar uma rede neural.

---

# 48. INCERTEZA

O sistema não deve fingir saber algo que não sabe.

Exemplo:

    sensor não confiável
        ↓
    aumentar incerteza

    região nunca observada
        ↓
    desconhecido

    localização pouco confiável
        ↓
    reduzir agressividade

A incerteza pode influenciar:

- velocidade;
- necessidade de scan;
- escolha de rota;
- necessidade de exploração.

---

# 49. DECISÃO

Uma decisão deve considerar múltiplos fatores.

Exemplo:

    SCORE =
        segurança
        + novidade
        + progresso
        + proximidade do objetivo
        + acessibilidade
        - distância
        - revisitação
        - risco
        - histórico de falhas

Os pesos devem ser configuráveis.

Não deixar dezenas de números mágicos espalhados pelo código.

---

# 50. CONFIGURAÇÃO

Parâmetros importantes devem ficar centralizados.

Exemplo:

    MAX_SPEED
    MIN_OBSTACLE_DISTANCE
    SERVO_MOVE_INTERVAL
    SCAN_ANGLE
    STUCK_TIMEOUT
    LOOP_THRESHOLD
    MAP_RESOLUTION
    REVISIT_PENALTY
    OBSTACLE_COST
    SAFETY_MARGIN

Evitar números mágicos espalhados pelo projeto.

---

# 51. COMPATIBILIDADE

Antes de alterar qualquer protocolo entre:

    Python ↔ ESP32
    Python ↔ Arduino
    ESP32 ↔ câmera

verificar o código existente.

Nunca inventar:

- IP;
- porta;
- formato de mensagem;
- comando;
- pino;
- protocolo.

Se algo não estiver claro, investigar o projeto.

---

# 52. REGRA SOBRE HARDWARE

Nunca assumir que o hardware funciona.

Se um comportamento físico parecer estranho:

1. verificar software;
2. verificar comunicação;
3. verificar configuração;
4. verificar alimentação;
5. verificar conexões;
6. verificar sensores;
7. testar componente isoladamente.

Não mascarar um problema físico com software complexo.

---

# 53. REGRA SOBRE ALTERAÇÕES

Antes de modificar:

    entender.

Depois:

    planejar.

Depois:

    implementar.

Depois:

    testar.

Depois:

    avaliar.

Não substituir o sistema inteiro por uma solução nova apenas porque ela parece mais elegante.

Preservar funcionalidades já comprovadamente funcionando.

---

# 54. CONTROLE DE VERSÃO

Sempre que possível:

    git status
    git diff
    git log

Antes de alterações grandes:

    criar commit.

Depois:

    registrar mudança.

Evitar perder versões funcionais.

---

# 55. NÃO CRIAR "CÓDIGO MONSTRO"

Um problema já observado foi a tendência de gerar versões cada vez maiores do programa.

Isso deve ser evitado.

Não fazer:

    v1
    v2
    v3
    ...
    v20
    v30
    v50

com centenas de funcionalidades acumuladas sem arquitetura.

Preferir:

    módulos
    interfaces
    classes
    testes
    documentação
    separação de responsabilidades.

---

# 56. DESENVOLVIMENTO INCREMENTAL

Ordem recomendada:

## Fase 1 — Hardware

Garantir:

- comunicação;
- motores;
- servo;
- ultrassônico;
- câmera.

## Fase 2 — Controle

Garantir movimento confiável.

## Fase 3 — Percepção

Garantir leituras confiáveis.

## Fase 4 — Navegação básica

Desviar de obstáculos.

## Fase 5 — Detecção de estado

Detectar:

- parado;
- preso;
- loop.

## Fase 6 — Memória

Guardar experiências.

## Fase 7 — Mapa

Criar representação espacial.

## Fase 8 — Planejamento

A* / Dijkstra / métodos semelhantes.

## Fase 9 — Exploração

Frontier-based exploration.

## Fase 10 — Visão

Detecção de objetos.

## Fase 11 — Objetivos

Busca e navegação orientada a missão.

## Fase 12 — Fusão sensorial

Melhorar estimativas.

## Fase 13 — Aprendizado

RL/ML.

## Fase 14 — Autonomia avançada

Combinar todos os componentes.

---

# 57. PRIORIDADES

Quando houver várias melhorias possíveis, priorizar:

1. Segurança
2. Confiabilidade
3. Percepção
4. Localização
5. Memória
6. Planejamento
7. Exploração
8. Visão
9. Otimização
10. Aprendizado

Não inverter essa ordem apenas para adicionar "IA".

---

# 58. DEFINIÇÃO DE SUCESSO

O rover será considerado mais autônomo quando conseguir:

- operar sem comandos humanos contínuos;
- evitar obstáculos;
- tomar decisões;
- lembrar experiências;
- evitar loops;
- detectar quando está preso;
- recuperar-se;
- construir mapa;
- escolher regiões novas;
- planejar caminhos;
- utilizar a câmera de maneira inteligente;
- procurar objetos;
- cumprir objetivos;
- retornar quando necessário;
- lidar com ambientes que não conhece previamente.

---

# 59. VISÃO DE LONGO PRAZO

O projeto pode evoluir para um agente robótico capaz de:

    perceber
       ↓
    compreender
       ↓
    localizar
       ↓
    mapear
       ↓
    planejar
       ↓
    agir
       ↓
    observar resultado
       ↓
    aprender
       ↓
    adaptar

O objetivo não é construir simplesmente um "carro com IA".

O objetivo é construir um **sistema robótico autônomo experimental**, no qual percepção, memória, planejamento, controle e aprendizado trabalham juntos.

---

# 60. REGRAS PARA O AGENTE DE DESENVOLVIMENTO

Ao trabalhar neste projeto:

### REGRA 1
Leia este documento antes de modificar o código.

### REGRA 2
Investigue o código existente antes de propor alterações.

### REGRA 3
Não assuma que uma implementação está correta apenas porque parece correta.

### REGRA 4
Não reescreva arquivos inteiros quando uma alteração localizada for suficiente.

### REGRA 5
Não remova funcionalidades existentes sem justificar.

### REGRA 6
Não alterar hardware/protocolo sem verificar evidências.

### REGRA 7
Sempre considerar casos extremos.

### REGRA 8
Sempre que possível, testar automaticamente.

### REGRA 9
Quando um teste falhar, investigar a causa antes de criar outra solução.

### REGRA 10
Nunca esconder erros com `try/except` genérico.

### REGRA 11
Não adicionar IA apenas para aumentar a complexidade.

### REGRA 12
Preferir arquitetura híbrida e modular.

### REGRA 13
Manter logs suficientes para reconstruir decisões.

### REGRA 14
Medir melhorias quantitativamente.

### REGRA 15
Priorizar comportamento real do rover sobre aparência do código.

---

# 61. PROTOCOLO DE TRABALHO DO CLAUDE

Para tarefas grandes, seguir:

    1. INSPECIONAR
    2. ENTENDER
    3. IDENTIFICAR PROBLEMA
    4. FORMULAR HIPÓTESES
    5. TESTAR HIPÓTESES
    6. PLANEJAR
    7. IMPLEMENTAR
    8. TESTAR
    9. CRITICAR
    10. CORRIGIR
    11. DOCUMENTAR

Não pular diretamente para:

    "reescrever o código."

---

# 62. QUANDO RECEBER UM BUG

Não fazer imediatamente:

    "Aqui está o código corrigido."

Primeiro determinar:

    O que aconteceu?
    O que deveria acontecer?
    Onde ocorre?
    Qual componente é responsável?
    É software?
    É comunicação?
    É hardware?
    É sensor?
    É timing?
    É estado?
    É lógica?

Depois corrigir.

---

# 63. QUANDO RECEBER UM NOVO RECURSO

Primeiro avaliar:

    Por que precisamos disso?

Depois:

    Onde essa funcionalidade pertence na arquitetura?

Depois:

    Quais módulos serão afetados?

Depois:

    Como testar?

Somente então implementar.

---

# 64. QUANDO EXISTIREM VÁRIAS SOLUÇÕES

Comparar:

- simplicidade;
- robustez;
- desempenho;
- manutenção;
- consumo computacional;
- compatibilidade;
- facilidade de teste;
- escalabilidade;
- adequação ao hardware.

Não escolher automaticamente a solução mais sofisticada.

---

# 65. PESQUISA EXTERNA

Quando houver dúvida sobre uma técnica importante, pesquisar literatura/documentação atual antes de implementar.

Priorizar:

- documentação oficial;
- artigos científicos;
- repositórios reconhecidos;
- projetos open source relevantes;
- exemplos comprovados em robótica.

Não inventar referências.

Se uma solução já existir e puder ser adaptada, avaliar antes de reinventar.

---

# 66. PRINCÍPIO FINAL

O rover deve seguir uma filosofia:

    NÃO:
    "Tenho uma regra para cada situação."

    SIM:
    "Tenho percepção suficiente para entender a situação,
     memória suficiente para aprender com o passado,
     planejamento suficiente para escolher uma ação,
     controle suficiente para executá-la,
     e mecanismos de segurança suficientes para impedir
     comportamentos perigosos."

Esse é o objetivo central do GEAR Autonomous Rover.