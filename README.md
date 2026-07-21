# Onboarding Driverless — Carrinho + Telemetria

Sistema completo: percepção (ZED + YOLO) → mapeamento (waypoint) → controle
(Pure Pursuit + serial p/ Arduino) → telemetria (dashboard web com kill switch).

Este documento assume que você **nunca rodou nada disso antes**. Siga na ordem.

---

## 1. Pré-requisitos

### Lado da Jetson (dentro do container Docker)
- Docker instalado na Jetson.
- O `Dockerfile` já fornecido builda `osrf/ros:humble-desktop` (Ubuntu 22.04),
  então funciona mesmo com o host em Ubuntu 24.04 — **é obrigatório rodar
  tudo do lado da Jetson dentro desse container**, nunca direto no host.
- Dependências Python **dentro do container** (não vêm com a imagem base):
  ```bash
  pip3 install pyserial ultralytics opencv-python
  ```
  (a `pyzed` vem do SDK da Stereolabs, que precisa ser instalado seguindo a
  documentação oficial: https://github.com/stereolabs/zed-python-api — não
  dá pra instalar via pip comum, precisa do SDK da ZED já rodando no host/Jetson).
- Testado com: ROS2 Humble, Python 3.10 (o que vem no Ubuntu 22.04 da imagem
  `osrf/ros:humble-desktop`).

### Lado do notebook (dashboard)
- Python 3.9+ (qualquer 3.x recente serve).
- Instale as dependências do dashboard:
  ```bash
  cd dashboard/
  pip install -r requirements.txt
  ```
- **Não precisa de Docker nem de ROS2 no notebook** — o backend do
  dashboard (`telemetry_server.py`) é um script Python comum, que só fala
  UDP/TCP/WebSocket com a Jetson.

### Arduino
- Firmware `v3.ino`, subido via Arduino IDE (biblioteca `Servo.h`, já vem
  por padrão no IDE). Baud rate: **115200**.

---

## 2. Configuração de rede

O carro e o notebook precisam estar na **mesma rede Wi-Fi do roteador da
oficina**.

### Como descobrir o IP da Jetson
Dentro do container (ou no host da Jetson), rode:
```bash
hostname -I
```
ou
```bash
ip addr show wlan0
```
Anote o IP (algo como `192.168.1.50`).

### Onde configurar esse IP no projeto
1. **No `telemetry_server.py`** (roda no notebook), edite a constante no topo do arquivo:
   ```python
   JETSON_IP = "192.168.1.50"  # <-- IP real da Jetson
   ```
2. **No `telemetry_node`** (roda na Jetson), o parâmetro `dashboard_ip`
   precisa apontar para o IP do **notebook** (não da Jetson!). Descubra o
   IP do notebook do mesmo jeito (`hostname -I` no notebook, ou
   `ipconfig` no Windows) e passe como parâmetro ao rodar o nó:
   ```bash
   ros2 run onboarding_pkg telemetry_node --ros-args -p dashboard_ip:=192.168.1.42
   ```
   (troque `192.168.1.42` pelo IP do notebook).

Resumindo a direção dos IPs:
- Jetson → sabe o IP do **notebook** (`dashboard_ip`), pra mandar telemetria via UDP.
- Notebook → sabe o IP da **Jetson** (`JETSON_IP` no `telemetry_server.py`), pra mandar o kill switch via TCP.

### Portas usadas (verifique se o firewall não bloqueia)
| Canal              | Protocolo | Porta | Sentido            |
|--------------------|-----------|-------|---------------------|
| Telemetria (dados) | UDP       | 5006  | Jetson → notebook   |
| Kill switch        | TCP       | 5007  | notebook → Jetson   |
| Dashboard web       | TCP (HTTP)| 8080  | navegador → notebook (localhost) |

---

## 3. Ordem de inicialização

**Sempre nessa ordem.** Se inverter, o kill switch pode não conseguir
conectar na primeira tentativa (o servidor TCP da Jetson precisa estar de
pé antes do notebook tentar falar com ele), e a telemetria só aparece
depois que o `telemetry_node` sobe.

1. **Arduino**: já deve estar ligado e com o `v3.ino` gravado antes de tudo.

2. **Jetson (dentro do container Docker)** — suba os 4 nós, cada um em um
   terminal (ou use um `launch file`, se preferir automatizar depois):
   ```bash
   # dentro do container, com o workspace já compilado (colcon build) e sourced
   ros2 run onboarding_pkg perception_node
   ros2 run onboarding_pkg mapping_node
   ros2 run onboarding_pkg controle_node --ros-args -p serial_port:=/dev/ttyACM0
   ros2 run onboarding_pkg telemetry_node --ros-args -p dashboard_ip:=<IP_DO_NOTEBOOK>
   ```

3. **Notebook**: só depois que a Jetson já está de pé, suba o backend:
   ```bash
   cd dashboard/
   python telemetry_server.py
   ```

4. **Navegador**: acesse `http://localhost:8080` no notebook que está
   rodando o `telemetry_server.py`.

Por que essa ordem importa: o kill switch usa TCP, que exige que o
servidor (na Jetson) já esteja escutando quando o cliente (notebook) tenta
conectar — se você clicar no kill switch antes do `telemetry_node` estar
de pé, vai aparecer erro de conexão recusada no dashboard (inofensivo, só
tente de novo depois que a Jetson estiver rodando).

---

## 4. Como testar sem o carro

Você não precisa da Jetson, do Arduino nem da ZED pra desenvolver o
dashboard. Dois scripts simulam os dois lados:

```bash
cd dashboard/

# Terminal 1: o backend de verdade
python telemetry_server.py

# Terminal 2: simula a Jetson mandando telemetria via UDP
python simulate_telemetry.py

# Terminal 3 (opcional): simula a Jetson respondendo ao kill switch via TCP
python simulate_kill_receiver.py
```

Com `JETSON_IP = "127.0.0.1"` no `telemetry_server.py` (só para esse teste
local), abra `http://localhost:8080` — os cards e gráficos devem se mexer
com dados sintéticos, e o botão de kill switch deve conseguir "falar" com
o `simulate_kill_receiver.py`.

Para testar só os nós de ROS2 (perception/mapping/controle) sem o carro
físico, publique valores fake no tópico `waypoint` manualmente:
```bash
ros2 topic pub /waypoint std_msgs/msg/Float32MultiArray "{data: [1.5, 0.2]}"
```
Isso já é suficiente pra ver o `controle_node` calculando steering e
publicando telemetria em `car_telemetry` (verifique com
`ros2 topic echo /car_telemetry`), mesmo sem Arduino conectado (o nó loga
erro na escrita da serial, mas continua rodando).

---

## 5. Troubleshooting básico

| Sintoma | Causa provável | Solução |
|---|---|---|
| `Não foi possível abrir a serial em /dev/ttyACM0` | Porta serial errada, ou Arduino em outra porta | Rode `ls /dev/tty*` com o Arduino conectado antes/depois pra ver qual porta aparece; passe via `--ros-args -p serial_port:=/dev/ttyACM1` |
| Dashboard não recebe telemetria (`⚠️ Sem sinal da Jetson`) | IP errado, ou firewall bloqueando UDP 5006 | Confirme o `dashboard_ip` do `telemetry_node` = IP real do notebook; teste com `nc -u -l 5006` no notebook enquanto manda algo da Jetson |
| Kill switch dá "não consegui falar com a Jetson" | `JETSON_IP` errado no `telemetry_server.py`, ou `telemetry_node` não está rodando | Confirme o IP da Jetson (`hostname -I` na Jetson) e que `ros2 run onboarding_pkg telemetry_node` está de pé |
| Carro não anda mesmo com telemetria OK | Kill switch travado (LED do Arduino aceso) | Aperte "Resetar" no dashboard, ou envie manualmente `ros2 topic pub /kill_switch std_msgs/msg/Bool "{data: false}"` |
| Container Docker não builda | Falta de espaço, ou proxy da rede da oficina bloqueando `apt-get`/`pip` | Testar `docker build` fora da rede da oficina primeiro, se possível |
| `ModuleNotFoundError: flask_socketio` | Esqueceu de instalar as dependências do dashboard | `pip install -r dashboard/requirements.txt` |

---

## 6. Estrutura do repositório

```
onboarding_pkg/           # pacote ROS2 (roda dentro do container, na Jetson)
    perception.py
    mapping.py
    controle.py
    telemetry.py
setup.py
Dockerfile                 # ROS2 Humble (Ubuntu 22.04) via container

dashboard/                 # roda no notebook, FORA do container/ROS2
    telemetry_server.py
    templates/index.html
    requirements.txt
    simulate_telemetry.py
    simulate_kill_receiver.py

v3.ino                      # firmware do Arduino
README.md
DECISOES.md
```
