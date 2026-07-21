# Onboarding Driverless — Carrinho + Telemetria

Sistema completo do desafio de onboarding da divisão Driverless: um carrinho que
identifica duas caixas, calcula o ponto médio entre elas, dirige até lá e
"estaciona" — com telemetria ao vivo num dashboard web e um kill switch de
emergência com trava a nível de firmware.

Este documento assume que você **nunca rodou nada disso antes**. Siga na ordem.

---

## 1. Hardware e pré-requisitos de software

### Hardware usado
- Jetson AGX Xavier (o "cérebro" do carrinho)
- Câmera ZED 2i, conectada via USB 3.0 na Jetson
- Arduino, conectado via USB na Jetson
- Ponte H (driver de motor), servo de direção, motor de tração com bateria de
  força **separada** (o USB nunca alimenta o motor, só manda o sinal)
- Duas caixas idênticas pra tarefa principal (a extensão com cones coloridos
  azul/laranja é só bônus, não implementada no código entregue — ver o PDF do
  onboarding)

### Onde cada coisa roda

| Máquina | SO / ambiente | ROS2 |
|---|---|---|
| Jetson AGX Xavier (o carrinho) | JetPack, **já com ROS2 Humble instalado nativamente** | Humble |
| Notebook (dashboard) | Qualquer SO com Python 3.9+ | não usa ROS2 |
| PC de desenvolvimento (opcional) | Ubuntu 24.04 — sem pacote oficial de Humble | Humble, só dentro de um container Docker |

> **Docker NÃO roda na Jetson.** O `Dockerfile` deste repositório existe só
> porque quem desenvolve pode estar num PC com Ubuntu 24.04 (sem pacote
> oficial do ROS2 Humble) e precisar de um ambiente Humble local pra
> escrever/testar código antes de levar pra oficina. **A Jetson já tem o
> ROS2 Humble instalado nativamente** — o deploy real nela é simplesmente
> copiar os arquivos (`perception.py`, `mapping.py`, `controle.py`,
> `telemetry.py`, `best.pt`) pro workspace ROS2 que já existe lá, e rodar
> `colcon build` direto na máquina. Se você só mexe direto na Jetson, pode
> ignorar o Dockerfile inteiramente.

### Instalando as dependências

**Na Jetson:**
```bash
pip3 install -r requirements-jetson.txt
```
A `pyzed` (bindings Python da ZED SDK) não vem desse arquivo — ela é
instalada junto com a ZED SDK nativa da Stereolabs, específica pra
Jetson/L4T. Siga a documentação oficial:
https://github.com/stereolabs/zed-python-api

**No notebook (dashboard):**
```bash
cd dashboard/
pip install -r requirements.txt
```
Esse lado não precisa de Docker nem de ROS2 — `telemetry_server.py` é um
script Python comum, que só fala UDP/TCP/WebSocket com a Jetson.

**Firmware do Arduino:** `v3.ino`, via Arduino IDE (biblioteca `Servo.h`, já
vem por padrão). Baudrate: **115200**.

---

## 2. Estrutura do repositório

```
onboarding_pkg/            -> pacote ROS2 (percepção, mapeamento, controle, telemetria)
  onboarding_pkg/
    perception.py          -> lê a ZED, roda a YOLO, publica coordenadas das 2 caixas
    mapping.py             -> calcula o ponto médio entre as 2 caixas (waypoint)
    controle.py            -> pure pursuit, fala com o Arduino, publica telemetria, recebe kill switch
    telemetry.py           -> ponte entre ROS2 e a rede (UDP telemetria + TCP kill switch)
  best.pt                  -> modelo YOLO treinado (classe única: "caixa")
  setup.py, package.xml    -> configuração do pacote ROS2

dashboard/                 -> roda no notebook, FORA do ROS2
  telemetry_server.py      -> servidor Flask + Socket.IO (recebe UDP, expõe WebSocket)
  templates/index.html     -> página do dashboard (card de status, kill switch, gráficos)
  requirements.txt         -> dependências Python do dashboard
  simulate_telemetry.py    -> simula a Jetson mandando telemetria via UDP (sem hardware)
  simulate_kill_receiver.py -> simula a Jetson recebendo o kill switch via TCP (sem hardware)

v3.ino                     -> firmware do Arduino (protocolo serial + trava do kill switch)
requirements-jetson.txt    -> dependências Python do lado da Jetson
Dockerfile                 -> opcional, só para dev/teste local (ver seção 1)
README.md
DECISOES.md                -> histórico de decisões técnicas, leia se tiver dúvida "por que foi feito assim"
```

As pastas `build/`, `install/` e `log/` que o `colcon build` gera dentro do
workspace ROS2 são automáticas — nunca edite nada dentro delas manualmente, e
nunca as copie de uma máquina pra outra.

---

## 2.1 Protocolo Arduino ↔ controle_node

Vale entender esse protocolo antes de mexer em qualquer coisa, porque tanto o
`controle.py` quanto o `v3.ino` dependem dele. São sempre 5 caracteres ASCII,
sem separador:

| Posição | Significado |
|---|---|
| `[0:2]` | Ângulo de direção, em hexadecimal (0–180, ex: `5A` = 90°) |
| `[2]` | Estado: `0`=parar (rampa suave), `1`=frente, `2`=ré, `8`=**reset do kill switch**, `9`=**kill switch** (corta e trava o motor), outro=freio brusco |
| `[3:5]` | Potência do motor, em hexadecimal (0–100%) |

Exemplo: `"5A114"` = ângulo 90°, frente, potência `0x14` = 20%.
Exemplo de kill: `"5A900"` = estado `9`, corta e trava o motor.

Baudrate: **115200** — precisa bater entre `controle.py` (parâmetro
`baudrate`) e o `Serial.begin()` do `.ino`.

Os estados `8`/`9` diferenciam esse protocolo de versões anteriores do
firmware: o `9` ativa uma trava (`killed = true`) que **ignora** qualquer
comando de frente/ré subsequente até um `8` explícito chegar (ver
`DECISOES.md`).

---

## 3. Como as peças se conectam (arquitetura)

```
                        JETSON (ROS2 Humble, nativo)                    NOTEBOOK
┌───────────────────────────────────────────────────────┐   ┌────────────────────────────┐
│  perception_node                                       │   │                            │
│  (câmera ZED 2i -> YOLO -> coordenadas das 2 caixas)   │   │  dashboard/                │
│         │ tópico ROS2 "coordinates"                    │   │  telemetry_server.py       │
│         v                                              │   │  (Flask + Socket.IO)       │
│  mapping_node                                           │   │            ^               │
│  (calcula o ponto médio -> waypoint)                    │   │            │ UDP :5006     │
│         │ tópico ROS2 "waypoint"                        │   │            │ (telemetria)  │
│         v                                              │   │            │               │
│  controle_node                                          │   │            │ TCP :5007     │
│  (pure pursuit + serial p/ Arduino)          <───────── │───│────────────┘ (kill switch) │
│         │ tópico ROS2 "car_telemetry" (JSON)            │   │                            │
│         │ tópico ROS2 "kill_switch" (Bool)              │   └────────────────────────────┘
│         v                                              │
│  telemetry_node                                         │
│  (ponte ROS2 <-> rede: UDP pra fora, TCP pro kill)      │
│         │ USB serial (5 bytes, protocolo hex)           │
│         v                                              │
│      Arduino (v3.ino)                                   │
│      -> servo de direção + motor (com trava de kill)    │
└───────────────────────────────────────────────────────┘
```

Pontos que valem destacar:
- **4 nós ROS2** (pacote `onboarding_pkg`) rodam todos **na Jetson**, porque é
  ela que tem a câmera ZED e o Arduino fisicamente plugados.
- `controle_node` só cuida de dirigir o carro e falar com o Arduino; quem fala
  com a rede (UDP/TCP) é um nó **separado**, o `telemetry_node`. Os dois só se
  comunicam por tópicos ROS2 internos (`car_telemetry` e `kill_switch`) — o
  porquê dessa separação está em `DECISOES.md`.
- O **dashboard** (Flask + página web) roda **no notebook**, só pra
  visualização e para mandar o kill switch — não depende de ROS2.
- `controle_node` é o único nó que abre a conexão serial com o Arduino.

### Fluxo de telemetria e kill switch

```
Telemetria (dados):
  controle_node  -> publica ROS2 "car_telemetry" (JSON)
      │
      ▼
  telemetry_node  -> assina "car_telemetry", reenvia via UDP :5006
      │
      ▼
  telemetry_server.py  -> recebe UDP, retransmite via WebSocket (Socket.IO)
      │
      ▼
  Dashboard Web  -> exibe status do motor, potência, direção, mapa local

Kill Switch:
  Dashboard Web  -> operador aciona o botão de kill switch
      │ WebSocket
      ▼
  telemetry_server.py  -> abre conexão TCP e manda "KILL" (ou "RESET")
      │ TCP :5007
      ▼
  telemetry_node  -> recebe, publica ROS2 "kill_switch" (Bool)
      │
      ▼
  controle_node  -> manda o comando de corte (estado 9) pro Arduino, que trava o motor
```

---

## 4. Configuração de rede

Jetson e notebook precisam se enxergar na mesma rede pra telemetria e kill
switch funcionarem (**o controle do carro em si, via USB/serial, não depende
disso** — só a telemetria e o kill switch dependem de rede).

1. Coloque as duas máquinas na mesma rede Wi-Fi (a do roteador da oficina).
2. Descubra o IP da Jetson: `hostname -I` ou `ip addr show`. Anote (ex: `192.168.1.50`).
3. Descubra o IP do notebook do mesmo jeito (ou `ipconfig` no Windows).
4. Configure cada IP no lugar certo:

| Parâmetro/variável | Onde | Roda em | Deve apontar pra |
|---|---|---|---|
| `dashboard_ip` (ROS2 param) | `telemetry_node` | Jetson | IP do **notebook** |
| `JETSON_IP` | topo de `dashboard/telemetry_server.py` | Notebook | IP da **Jetson** |

Cada variável aponta pra **máquina oposta** de onde ela roda — fácil de
confundir, revise com calma.

```bash
# Na Jetson, ao subir o nó de telemetria:
ros2 run onboarding_pkg telemetry_node --ros-args -p dashboard_ip:=<IP_DO_NOTEBOOK>
```
```python
# Em dashboard/telemetry_server.py:
JETSON_IP = "<IP_DA_JETSON>"
```

**Portas usadas** (verifique se o firewall não bloqueia):

| Canal | Protocolo | Porta | Sentido |
|---|---|---|---|
| Telemetria (dados) | UDP | 5006 | Jetson → notebook |
| Kill switch | TCP | 5007 | notebook → Jetson |
| Dashboard web | HTTP | 8080 | navegador → notebook (localhost) |

Redes com DHCP podem reatribuir um IP diferente a cada reconexão — refaça os
passos acima sempre que o dashboard parar de receber telemetria do nada, antes
de suspeitar de qualquer outra coisa.

---

## 5. Colocando tudo pra rodar

### 5.1 — Compilar o pacote ROS2 na Jetson

```bash
mkdir -p ~/ros2_ws/src
cp -r onboarding_pkg ~/ros2_ws/src/
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 5.2 — Upload do firmware no Arduino

Pelo Arduino IDE: abra `v3.ino`, selecione a placa e a porta corretas
(geralmente `/dev/ttyACM0`), clique em "Verificar" e depois em "Carregar".

### 5.3 — Ordem de inicialização

Um nó ROS2 que assina um tópico só recebe mensagens publicadas **depois** que
ele já existe — mensagens anteriores se perdem para ele. O mesmo vale pro lado
da rede: se o backend do dashboard ainda não estiver escutando, os primeiros
pacotes de telemetria UDP também somem (inofensivo, mas gera ruído nos logs).
Por isso, suba tudo na ordem "quem assina primeiro":

1. **Arduino** já com `v3.ino` gravado, ligado.
2. **Notebook** — sobe o dashboard primeiro:
   ```bash
   cd dashboard/
   python telemetry_server.py
   ```
3. **Jetson**, nessa ordem:
   ```bash
   source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

   ros2 run onboarding_pkg telemetry_node --ros-args -p dashboard_ip:=<IP_DO_NOTEBOOK>
   ros2 run onboarding_pkg controle_node --ros-args -p serial_port:=/dev/ttyACM0
   ros2 run onboarding_pkg mapping_node
   ros2 run onboarding_pkg perception_node
   ```
   (`telemetry_node` primeiro porque publica `kill_switch`, que o
   `controle_node` assina; `controle_node` antes de `mapping_node` porque
   assina `waypoint`; `perception_node` por último, já que é o único que só
   publica e nunca assina nada.)
4. **Navegador**: `http://localhost:8080` no notebook.

**Sempre com as rodas fora do chão / carrinho apoiado até confirmar que tudo
está se comportando corretamente.**

---

## 6. Segurança

- **Watchdog de waypoint**: se o `controle_node` ficar mais de 1 segundo
  (`waypoint_timeout`) sem receber um waypoint novo (câmera perdeu as caixas
  de vista, `mapping_node` travou, etc.), o carro **para sozinho**.
- **Parada automática por chegada**: o carro para sozinho ao chegar a 30cm do
  waypoint (`arrival_radius`) — é o que implementa o "estacionar" pedido na
  tarefa.
- **Kill switch com trava de firmware**: o botão do dashboard manda um
  comando que o **próprio Arduino** trava internamente (estado `9`) — o motor
  fica cortado mesmo que cheguem comandos de frente/ré logo em seguida, e só
  volta com um reset explícito (estado `8`, botão "Resetar" no dashboard).
  Isso é mais forte que só zerar a potência via software.
- Teste qualquer mudança em `controle.py` ou no firmware sempre com as rodas
  fora do chão primeiro. Recompile (`colcon build`) e regrave o firmware
  sempre que houver mudança.

---

## 7. Testando sem o carro físico

Útil pra desenvolver fora da oficina, sem Jetson/ZED/Arduino.

**Só o dashboard (sem ROS2):**
```bash
cd dashboard/

# Terminal 1: o backend de verdade
python telemetry_server.py

# Terminal 2: simula a Jetson mandando telemetria via UDP
python simulate_telemetry.py

# Terminal 3 (opcional): simula a Jetson respondendo ao kill switch via TCP
python simulate_kill_receiver.py
```
Com `JETSON_IP = "127.0.0.1"` em `telemetry_server.py` (só pra esse teste
local), abra `http://localhost:8080` — os cards e gráficos devem se mexer
sozinhos.

**A pipeline ROS2 inteira (percepção simulada):** com `mapping_node` e
`controle_node` rodando (sem Arduino conectado, o `controle_node` só loga
erro na escrita da serial, sem travar), simule 2 caixas detectadas:
```bash
ros2 topic pub /coordinates std_msgs/msg/Float32MultiArray "{data: [1.5, 0.5, 1.5, -0.5]}" --once
```
Ou repetidamente:
```bash
ros2 topic pub /coordinates std_msgs/msg/Float32MultiArray "{data: [1.5, 0.5, 1.5, -0.5]}" -r 5
```
Pra testar só o `controle_node` isoladamente, publique direto em `/waypoint`:
```bash
ros2 topic pub /waypoint std_msgs/msg/Float32MultiArray "{data: [1.5, 0.2]}"
```
E confira a telemetria calculada com `ros2 topic echo /car_telemetry`.

**O kill switch sem navegador** (lembrando que ele usa TCP, não UDP):
```bash
python3 -c "
import socket
with socket.create_connection(('127.0.0.1', 5007), timeout=2) as s:
    s.sendall(b'KILL')
    print(s.recv(64))
"
```
Troque `b'KILL'` por `b'RESET'` pra destravar de novo.

---

## 8. Resolvendo problemas comuns

| Sintoma | Causa provável | Solução |
|---|---|---|
| `Não foi possível abrir a serial em /dev/ttyACM0` | Porta serial errada | Rode `ls /dev/tty*` antes/depois de plugar o Arduino; passe via `--ros-args -p serial_port:=/dev/ttyACM1` |
| Dashboard não recebe telemetria (`⚠️ Sem sinal da Jetson`) | IP errado, ou firewall bloqueando UDP 5006 | Confirme `dashboard_ip` do `telemetry_node` = IP real do notebook; teste com `nc -u -l 5006` no notebook |
| Kill switch dá "não consegui falar com a Jetson" | `JETSON_IP` errado em `telemetry_server.py`, ou `telemetry_node` não está rodando | Confirme o IP da Jetson e que `ros2 run onboarding_pkg telemetry_node` está de pé |
| Carro não anda mesmo com telemetria OK | Kill switch travado (LED do Arduino aceso) | Aperte "Resetar" no dashboard, ou `ros2 topic pub /kill_switch std_msgs/msg/Bool "{data: false}"` |
| `ModuleNotFoundError: flask_socketio` | Faltou instalar as dependências do dashboard | `pip install -r dashboard/requirements.txt` |
| `pip install ultralytics` lento/travando na Jetson | Compilando dependências pesadas (torch) do zero em ARM | Considere um wheel de torch pré-compilado pra Jetson (ver `requirements-jetson.txt`) |
| Container Docker não builda | Isso só afeta o PC de dev opcional, não a Jetson | Confirme que não está tentando usar o Dockerfile na própria Jetson — lá é tudo nativo |

---

## 9. Por trás das escolhas técnicas

O `DECISOES.md` documenta, entre outras coisas:
- por que `controle_node` é o único dono da conexão serial;
- por que a telemetria vem do último comando enviado pelo próprio
  `controle_node`, e não de uma leitura real de sensores no Arduino;
- por que o kill switch vai por TCP (porta separada) e a telemetria por UDP;
- por que o kill switch trava no firmware, e não só numa flag em Python.

Vale ler antes de mudar qualquer coisa relacionada a rede ou ao protocolo do
Arduino.
