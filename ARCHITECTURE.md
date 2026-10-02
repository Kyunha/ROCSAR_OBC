# Arquitetura do Sistema ROCSAR

## 1. Visão Geral do Sistema
O **ROCSAR** é um sistema embarcado de apontamento autónomo e compensação de movimento para antenas a bordo de uma plataforma aérea/gôndola. O objetivo principal é manter as antenas orientadas continuamente para uma direção alvo (*Target Heading*), compensando as rotações e turbulências da plataforma em tempo real.

---

## 2. Requisitos do Sistema

### 2.1. Requisitos Funcionais (RF)
* **RF01 - Apontamento Contínuo:** O sistema deve calcular a posição dos servos a 50 Hz para compensar a rotação da gôndola.
* **RF02 - Receção de Comandos:** A Ground Station (GS) deve conseguir enviar novos alvos de orientação e ordens de controlo para a OBC.
* **RF03 - Transmissão de Telemetria:** A OBC deve enviar dados do estado do sistema (posição das antenas, orientação da plataforma, temperaturas, voltagens e GNSS) para a GS em tempo real.
* **RF04 - Leitura de Posição (BNO055):** A Raspberry Pi Pico deve ler a bússola/IMU via I2C para determinar a orientação atual.
* **RF05 - Controlo de Servos ST3215:** A Pico deve acionar dois servos ST3215 acoplados a gearboxes de redução 5:1.
* **RF06 - Módulos de Aquecimento:** O sistema deve permitir ligar/desligar dois aquecedores (GPIOs 4 e 5) para operação em baixas temperaturas.
* **RF07 - Ingestão de Dados GNSS:** A OBC deve ler as coordenadas UBX de 3 receptores GNSS expostos em portas UDP locais e integrá-las na telemetria.
* **RF08 - Gestão do SDR Ettus B210:** A OBC deve disparar e gerir a recolha de dados de Radar e guardar os resultados no SSD.
* **RF09 - Download de Ficheiros:** O operador na GS deve poder transferir ficheiros de logs e dados do SSD via HTTP.

### 2.2. Requisitos Não-Funcionais (RNF)
* **RNF01 - Tempo Real e Determinismo:** O loop de controlo de apontamento na Pico deve correr rigorosamente a 50 Hz (período de 20 ms).
* **RNF02 - Limitação Dinâmica de Largura de Banda:** A comunicação Ethernet entre a GS e a OBC deve ser passível de limitação de débito em tempo real usando ferramentas do SO (`tc`).
* **RNF03 - Autonomia do Subsistema de Controlo:** Se a comunicação entre a OBC e a Pico falhar, a Pico deve continuar o loop de controlo autónomo usando a última direção recebida.
* **RNF04 - Modularidade:** O código deve ser dividido em módulos independentes (comunicação, controlo, sensores e atuadores).

---

## 3. Arquitetura de Hardware

A infraestrutura física do ROCSAR está dividida em três níveis:

+-----------------------------------------------------------------------------------+
|                               GROUND STATION (GS)                                 |
|  - Laptop do Operador (GUI de monitorização e controlo)                           |
+-----------------------------------------------------------------------------------+
|
Ethernet (Com limitação de banda)
|
+-----------------------------------------------------------------------------------+
|                             OBC (Raspberry Pi 4B)                                 |
|  - SSD USB (Armazenamento de dados, logs e imagens)                               |
|  - SDR Ettus B210 USB (Radar - Transmissão e Receção)                             |
|  - Câmara USB (Fotos das antenas)                                                 |
|  - 3x Receptores GNSS USB (Dados UBX a 57600 baud -> UDP local)                   |
+-----------------------------------------------------------------------------------+
|
Cabo USB (Serial ASCII)
|
+-----------------------------------------------------------------------------------+
|                           RASPBERRY PI PICO (Firmware)                            |
|  - BNO055 (IMU/Bússola via I2C nos pinos SDA 16 / SCL 17)                          |
|  - 2x Servos ST3215 (UART Half-Duplex via GPIO 0 e 1 + Circuito de Adaptação)     |
|  - 2x Módulos de Aquecimento (GPIO 4 e GPIO 5)                                    |
+-----------------------------------------------------------------------------------+


---

## 4. Arquitetura de Software e Comunicação

### 4.1. Comunicação Ground Station <-> OBC (Ethernet)
1. **Comandos de Controlo (ZeroMQ ROUTER/DEALER - Porta 5555):**
   * Canal bidirecional e assíncrono para o operador enviar ordens (ex: alterar alvo, ligar aquecedores, mudar limite de banda).
   * Formato: JSON ou Protocol Buffers para máxima eficiência de transmissão.
2. **Telemetria Downlink (ZeroMQ PUB/SUB - Porta 5556):**
   * Canal de transmissão unidirrecional da OBC para a GS a uma taxa típica de 5 Hz.
   * Publica pacotes contendo dados de orientação, estado dos servos, coordenadas GNSS e temperatura do processador.
3. **Download de Ficheiros (Servidor HTTP Leve - Porta 8080):**
   * Um servidor HTTP embutido em Python (`http.server`) expõe a diretoria de dados no SSD, permitindo descargas diretas via navegador ou `curl`.

### 4.2. Controlo Dinâmico de Banda (Traffic Control)
* A OBC utiliza a ferramenta `tc` (*Traffic Control*) do Linux para aplicar regras TBF (*Token Bucket Filter*) na interface Ethernet (`eth0`).
* Permite alterar o limite de débito (ex: 115 kbps) em pleno voo via comando expedido pela Ground Station.

### 4.3. Comunicação OBC <-> Raspberry Pi Pico (Serial USB)
* **Baudrate:** 115200 bps.
* **Formato:** Protocolo textual baseado em linhas ASCII.
* **Comandos Principais enviados do OBC para a Pico:**
  * `TARGET <graus>`: Define a nova direção absoluta no espaço.
  * `HEAT1 <1|0>` / `HEAT2 <1|0>`: Liga ou desliga os módulos de aquecimento.
  * `STATUS`: Solicita uma linha de telemetria atualizada.

---

## 5. Decisões de Design (Stack Tecnológica)

| Componente | Tecnologia Escolhida | Justificação |
| :--- | :--- | :--- |
| **Linguagem na Pico** | C++ (Arduino Framework / RP2040) | Garante execução determinística sem latências de garbage collector. |
| **Linguagem na OBC** | Python 3 | Facilidade na integração de bibliotecas de rede, suporte ao SDR e suporte nativo a threads/subprocessos. |
| **Rede (Comandos/Telemetria)** | ZeroMQ | Leve, de alta performance e sem os *overheads* de conexões HTTP convencionais. |
| **Serialização de Dados** | Protobuf / JSON / ASCII | ASCII na ligação Serial para fácil depuração; JSON/Protobuf na rede para compactação de dados. |
| **Gestão de Tráfego** | Linux `tc` | Método nativo do kernel Linux para controlo preciso e fiável de taxa de transferência na Ethernet. |
| **Download de Ficheiros** | HTTP Leve | Simples, robusto e compatível com retoma e descarregamento via ferramentas padrão de mercado. |

---

## 6. Tolerância a Falhas e Segurança (*Fail-safe*)

1. **Perda de Comunicação GS <-> OBC:**
   * A OBC continua a registar dados no SSD e a executar as tarefas agendadas autonomamente.
2. **Perda de Comunicação OBC <-> Pico:**
   * A Pico **não bloqueia**. Ela mantém o seu loop de controlo autónomo a 50 Hz, orientando as antenas para a última direção válida (`globalTargetHeading`) com base nas leituras da sua própria IMU (BNO055).
3. **Filtragem de Dados e Suavização:**
   * A orientação lida da BNO055 passa por um filtro de média móvel exponencial (EMA) para eliminar ruído.
   * Aplica-se uma zona morta (*deadband*) nos servos para evitar desgaste mecânico desnecessário com microajustes inferiores a ~0.17°.
