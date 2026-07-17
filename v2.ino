/*
  ================================================================
  Carrinho Driverless - Firmware de controle (v2)
  ================================================================

  Baseado no v1.ino original, com o bug de compilação da linha do
  map() corrigido e comentários explicando o protocolo serial.

  PROTOCOLO SERIAL (baud 115200):
  --------------------------------
  O Arduino espera receber SEMPRE 5 caracteres ASCII por comando:

    [0] [1]   [2]    [3] [4]
     ^^^^^     ^       ^^^^^
   steering  estado   potência
   (hex, 2   (1 char) (hex, 2
   dígitos)           dígitos)

  - steeringAngle (0-180): ângulo do servo, escrito hexadecimal
      em 2 caracteres. Ex: 90  -> "5A"   (0x5A = 90)
                            0  -> "00"
                          180  -> "B4"

  - estado (1 caractere, dígito ASCII):
      '0' -> parar (rampa suave de PWM até 0)
      '1' -> frente
      '2' -> ré
      outro valor -> freio brusco (PWM = 0 imediato)

  - thrustPower (0-100, "%"): potência desejada, em hexadecimal,
      2 caracteres. Ex: 25%  -> "19"  (0x19 = 25)
                        100%  -> "64"  (0x64 = 100)
      Esse valor é internamente mapeado de 0-100 para 0-255 (PWM).

  Exemplo de mensagem completa: "5A119"
      -> steering = 0x5A = 90 graus
      -> estado   = 1 (frente)
      -> potência = 0x19 = 25%  -> mapeado para PWM ~63

  Do lado do Python (nó de controle), a mensagem é montada assim:
      msg = f"{steering_byte:02X}{state}{power_byte:02X}"
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

    if (state != 0) {
      int powerUp   = charToHexa(comandos[3]);
      int powerDown = charToHexa(comandos[4]);

      thrustPower = (16 * powerUp) + powerDown; // 0-100 (%)
      thrustPower = constrain(thrustPower, 0, 100);
      thrustPower = map(thrustPower, 0, 100, 0, 255); // <-- bug do v1 corrigido aqui

      if (state == 1) { // Frente
        analogWrite(RPWM, thrustPower);
        analogWrite(LPWM, 0);
        lastPwm = thrustPower;

      } else if (state == 2) { // Ré
        analogWrite(RPWM, 0);
        analogWrite(LPWM, thrustPower);
        lastPwm = thrustPower;

      } else { // Qualquer outro estado -> freio brusco
        analogWrite(RPWM, 0);
        analogWrite(LPWM, 0);
      }

      steering.write(steeringAngle);

    } else { // Estado = 0 -> parada suave (rampa de PWM até 0)
      for (int k = lastPwm; k >= 0; k--) {
        analogWrite(RPWM, k);
        delay(5);
      }
      lastPwm = 0;
      steering.write(steeringAngle);
    }

  } else if (Serial.available() > 0) { // Mensagem incompleta chegou
    digitalWrite(LED, HIGH);
    while (Serial.available()) {
      Serial.read();
    }
    delay(50);
    digitalWrite(LED, LOW);
  }
}
