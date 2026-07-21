# Decisões Técnicas e Trade-offs — Telemetria

## 1. Protocolo de telemetria: UDP (dados) vs TCP (kill switch)

**Escolhido:** UDP para o fluxo contínuo de telemetria (`car_telemetry` →
`telemetry_node` → dashboard); TCP em **porta separada** só para o kill switch.

**Alternativas consideradas:** TCP para tudo; MQTT para tudo.

**Motivo:**
- Telemetria é publicada a 10 Hz e cada pacote sozinho já é útil — perder
  um pacote ocasional não é grave, o próximo chega 100ms depois. UDP evita
  o overhead de handshake/retransmissão do TCP, o que importa pra manter a
  "latência perceptivelmente baixa" exigida.
- O kill switch é o oposto: é um comando único, crítico, que **não pode se
  perder**. Por isso ele vai por TCP, que garante entrega (ou pelo menos
  avisa se falhou, via exceção de conexão), mesmo que isso custe uns
  milissegundos a mais de handshake — aceitável porque não é um comando de
  alta frequência.
- MQTT foi descartado por exigir um broker rodando (mais uma peça de
  infraestrutura pra manter de pé na oficina) sem ganho relevante pra esse
  escopo (só 2 peers: Jetson e notebook, sem necessidade de pub/sub
  multi-assinante do broker).

**Custo futuro:** se um dia precisarmos de múltiplos dashboards ou
múltiplos carros ao mesmo tempo, UDP ponto-a-ponto não escala bem (não tem
broadcast/multicast configurado) — nesse caso, migrar pra MQTT ou um
relay central faria mais sentido.

---

## 2. Separação de canais (dados vs kill switch em portas diferentes)

**Escolhido:** UDP:5006 para telemetria, TCP:5007 para kill switch, em
sockets/processos completamente independentes dentro do `telemetry_node`.

**Motivo:** garante que um pico de tráfego de telemetria (ou uma fila de
pacotes UDP se acumulando) nunca atrase ou bloqueie o comando de corte.
Se fosse tudo no mesmo canal, um problema no fluxo de dados poderia, na
pior hipótese, atrasar o kill switch — inaceitável pra um comando de
segurança.

**Custo:** duas portas pra configurar/lembrar em vez de uma, e dois
sockets pra gerenciar no `telemetry_node` (mitigado rodando o servidor TCP
numa thread separada, sem impacto no loop principal do rclpy).

---

## 3. Ponte backend → browser: WebSocket (Flask-SocketIO) vs polling HTTP

**Escolhido:** WebSocket via Flask-SocketIO.

**Motivo:** o backend recebe telemetria via UDP a 10Hz; com WebSocket, ele
**empurra** (`emit`) cada pacote pro navegador assim que chega, sem esperar
o navegador perguntar. Com polling HTTP, o navegador teria que ficar
perguntando "tem novidade?" em algum intervalo fixo, o que ou gasta
requisições à toa (intervalo curto) ou introduz atraso perceptível
(intervalo longo) — direto contra o requisito de "latência
perceptivelmente baixa".

**Custo:** WebSocket exige uma lib a mais (`flask-socketio`) e um pouco
mais de cuidado com CORS/portas do que um endpoint REST simples.

---

## 4. Framework de frontend: HTML puro + Chart.js vs React vs Plotly Dash

**Escolhido:** HTML/CSS/JS puro numa única página (`index.html`), com
Chart.js via CDN.

**Motivo:** zero pipeline de build (sem `npm install`, sem bundler) — o
Flask só serve o arquivo direto, o que é ideal pro escopo do onboarding
(poucos widgets: card de status, botão de kill switch, dois gráficos) e
reduz a curva de aprendizado pra quem tá vendo o projeto pela primeira
vez (é só abrir o `.html` e ler de cima a baixo).

**Custo futuro:** se o dashboard crescer bastante (múltiplos carros,
várias telas, estado complexo compartilhado entre componentes), React
facilitaria organizar isso — mas seria over-engineering pro MVP atual.

---

## 5. `async_mode='threading'` no Flask-SocketIO em vez de `eventlet`/`gevent`

**Escolhido:** `async_mode='threading'` (padrão do Python, sem lib extra).

**Motivo:** eventlet/gevent dão mais performance sob alta concorrência,
mas exigem mais uma dependência (com histórico de incompatibilidades com
versões novas do Python) só pra suportar um punhado de conexões
simultâneas — não é o gargalo desse projeto. Threading padrão é suficiente
e simplifica o `requirements.txt`.

**Custo:** não escala bem pra centenas de conexões simultâneas (não é o
nosso caso: só o dashboard local da equipe).

---

## 6. Origem dos dados de telemetria: estado interno do `controle_node` vs leitura real de sensores no Arduino

**Escolhido:** a telemetria (`motor_on`, `power_pct`, `steering_deg`) vem
do **último comando que o próprio `controle_node` mandou pro Arduino**, e
não de uma leitura de volta de sensores (velocidade/aceleração) no
firmware.

**Alternativa considerada:** Arduino lê sensores físicos (encoder de
velocidade, acelerômetro) e manda esses dados de volta por Serial, que um
nó ROS2 leria e publicaria.

**Motivo:** o requisito obrigatório do Card de Dados é só "motor
ligado/desligado" — informação que o `controle_node` já sabe com certeza
absoluta, porque é ele quem decide e envia o comando. Ler de volta do
Arduino exigiria: (1) hardware de sensor que não estava no escopo
combinado do onboarding, (2) modificar o firmware pra fazer
`Serial.print()` intercalado com a leitura de comandos, arriscando
interferir no protocolo de 5 bytes já testado e validado em bancada.
Preferimos não mexer numa peça que já funciona (`mapping` e o protocolo
serial já testados) só pra ganhar um dado que o requisito obrigatório não
pede.

**Custo futuro:** o dashboard mostra "o que o software mandou fazer", não
necessariamente "o que o carro está fazendo de verdade" (ex: se o motor
travar fisicamente, o dashboard ainda mostraria "ligado"). Pra
telemetria de competição de verdade, leitura real de sensores seria
obrigatória — deixamos indicado no `v3.ino` como próximo passo natural
(um `readSpeedSensor()`/`readAccelSensor()` publicando por uma linha de
Serial separada, sem interferir nos 5 bytes de comando).

---

## 7. Kill switch como trava (latch) em vez de "parada única"

**Escolhido:** o estado `9` (kill) no Arduino ativa uma flag `killed=true`
que **persiste** e ignora qualquer comando de frente/ré até um `8`
(reset) explícito chegar — diferente do estado `0` (parada normal, que
não trava nada).

**Motivo:** um kill switch de segurança não pode ser "desfeito" sem querer
por um comando de navegação que chegue logo em seguida (ex: o
`controle_node` mandando `frente` de novo porque recebeu um `waypoint`
novo). A trava garante que o corte só é desfeito por uma ação explícita
(o botão "Resetar" no dashboard).

**Custo:** ponto único de falha por software — se a Jetson travar,
o kill switch por software não funciona mais (o carro já estará parado
por causa do `waypoint_timeout` no `controle_node`, mas isso é uma
segurança diferente, não o kill switch em si). Para uso em pista de
verdade, um kill switch físico (botão que corta a alimentação do motor
diretamente, sem depender de nenhum software) continua sendo obrigatório
como camada adicional — o kill switch por software aqui é uma
funcionalidade de conveniência/monitoramento, não substitui a trava física.
