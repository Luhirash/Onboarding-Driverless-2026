/*
  ================================================================
  Carrinho Driverless - Firmware de controle (v3)
  ================================================================

  Baseado no v2.ino, com o KILL SWITCH da Telemetria adicionado.

  PROTOCOLO SERIAL (baud 115200) - continua sendo SEMPRE 5 caracteres:

    [0] [1]   [2]    [3] [4]
     ^^^^^     ^       ^^^^^
   steering  estado   potência
   (hex, 2   (1 char) (hex, 2
   dígitos)           dígitos)

  Estados possíveis (comandos[2] - '0'):
      '0' -> parar (rampa suave de PWM até 0)
      '1' -> frente
      '2' -> ré
      '8' -> RESET do kill switch (destrava o motor)
      '9' -> KILL SWITCH (corta o motor e TRAVA -- ignora frente/ré/parada
             suave até chegar um '8' explícito)
      outro valor -> freio brusco (PWM = 0 imediato)

  O KILL SWITCH é diferente do estado '0' (parar): o '0' é uma parada normal
  de navegação (o carro pode voltar a andar no próximo comando '1'/'2').
  Já o '9' é uma trava de segurança: uma vez killed=true, o motor fica em
  PWM=0 e SÓ volta a aceitar comandos de frente/ré depois de um '8' (reset)
  explícito, mesmo que continuem chegando comandos '1'/'2' nesse meio tempo.
  Isso é proposital -- ver DECISOES.md.

  Exemplo de mensagem: "5A119" -> steering=0x5A(90), estado=1 (frente),
  potência=0x19(25%). Exemplo de kill: "5A900" -> estado=9, corta e trava.
  ================================================================
*/

#include <Servo.h>

#define steeringPin 4
#define RPWM 5
#define LPWM 6
#define REN 7
#define LEN 8
#define LED 13

char comandos[5];
int steeringAngle, state, thrustPower;
int lastPwm = 0;
bool killed = false; // trava de segurança do kill switch

Servo steering;

void setup() {
  steering.attach(steeringPin);
  steering.write(90); // posição neutra (reto)

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  digitalWrite(LED, LOW);

  Serial.begin(115200);
}

int charToHexa(char input) {
  // Retorna o valor hexadecimal (0-15) correspondente a um char ('0'-'9', 'A'-'F').
  if (input >= 48 && input <= 57) {
    return input - 48;         // '0'-'9'
  } else if (input >= 65 && input <= 70) {
    return input - 55;         // 'A'-'F'
  } else {
    return -1;                 // char inválido
  }
}

void loop() {
  if (Serial.available() >= 5) {

    for (int k = 0; k < 5; k++) {
      comandos[k] = Serial.read();
    }

    int steerUp   = charToHexa(comandos[0]);
    int steerDown = charToHexa(comandos[1]);
    steeringAngle = (16 * steerUp) + steerDown;
    steeringAngle = constrain(steeringAngle, 0, 180); // segurança extra

    state = comandos[2] - 48;

    // ---------------- KILL SWITCH ----------------
    if (state == 9) {
      killed = true;
    }

    if (killed) {
      // Motor sempre cortado enquanto killed==true, não importa o que
      // mais chegue no campo de estado (exceto o reset '8').
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 0);
      lastPwm = 0;
      steering.write(steeringAngle); // direção ainda responde, só o motor é travado

      if (state == 8) {
        killed = false;
        digitalWrite(LED, LOW);
      } else {
        digitalWrite(LED, HIGH); // LED aceso = carro travado pelo kill switch
      }

      return; // não processa frente/ré/parada normal enquanto killed
    }
    // -----------------------------------------------

    if (state == 1 || state == 2) {
      int powerUp   = charToHexa(comandos[3]);
      int powerDown = charToHexa(comandos[4]);

      thrustPower = (16 * powerUp) + powerDown; // 0-100 (%)
      thrustPower = constrain(thrustPower, 0, 100);
      thrustPower = map(thrustPower, 0, 100, 0, 255);

      if (state == 1) { // Frente
        analogWrite(RPWM, thrustPower);
        analogWrite(LPWM, 0);
        lastPwm = thrustPower;

      } else { // state == 2 -> Ré
        analogWrite(RPWM, 0);
        analogWrite(LPWM, thrustPower);
        lastPwm = thrustPower;
      }

      steering.write(steeringAngle);

    } else if (state == 0) { // Parada suave (rampa de PWM até 0)
      for (int k = lastPwm; k >= 0; k--) {
        analogWrite(RPWM, k);
        delay(5);
      }
      lastPwm = 0;
      steering.write(steeringAngle);

    } else { // estado desconhecido (ex: 8 fora do contexto de kill) -> freio brusco
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 0);
      lastPwm = 0;
      steering.write(steeringAngle);
    }

  } else if (Serial.available() > 0) { // Mensagem incompleta chegou
    digitalWrite(LED, HIGH);
    while (Serial.available()) {
      Serial.read();
    }
    delay(50);
    digitalWrite(LED, killed ? HIGH : LOW);
  }
}
