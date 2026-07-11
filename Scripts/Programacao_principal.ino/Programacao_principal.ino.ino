/*
 * ======================================================
 * Projeto  : Robô para a Olimpiada Brasileira de Robotica
 * Autor    : Lucca Pupo Borin
 * Data     : 02/07/2026
 * ======================================================
 * Hardware:
 *   Plataforma  : RoboCore Vespa (ESP32)
 *   Sensores    : MÓDULO  com 5 x TCRT5000, sendo o central à frente e os demais alinhados (infravermelho, leitura digital)
 *   Motor driver: DRV8837 (via biblioteca VespaMotors)
 *   IMU         : MPU-6050 (somente giroscópio), via multiplexador I2C (canal 1)
 *
 * Convenções de nomenclatura utilizadas:
 *   #define / pinos  → SNAKE_CASE_MAIUSCULO
 *   Tipos (enum)     → PascalCase
 *   Valores de enum  → SNAKE_MAIUSCULO  (sem prefixo de categoria)
 *   Funções          → camelCase  (padrão Arduino)
 *   Variáveis globais → camelCase
 *   Booleanos        → prefixo is/tem para leitura natural
 *   Variáveis locais → camelCase simples
 *
 * Lógica geral:
 *   Os sensores retornam HIGH quando detectam a linha (superfície
 *   escura) e LOW quando estão sobre o fundo claro. O robô ajusta
 *   sua direção com base em quais sensores estão ativos.
 *
 *   Sensores de ponta (PE / PD): indicam que o robô saiu muito
 *   da linha → executa curva de 90°.
 *   Sensores de centro (CE / CD): indicam desvio leve → correção
 *   suave de trajetória.
 *   Sensor central (CM): referência de alinhamento nas curvas de 90°.
 *
 *   O giroscópio (MPU-6050) fornece o ângulo acumulado (em graus)
 *   nos eixos X, Y e Z, obtido pela integração da velocidade
 *   angular no tempo. A troca de canal no multiplexador I2C
 *   (TCA9548A, endereço 0x70) é feita pela função selectChannel(),
 *   escrevendo direto no registrador de controle, sem biblioteca.
 * ======================================================
 */

// ======================================================
// BIBLIOTECAS
// ======================================================
#include <RoboCore_Vespa.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ======================================================
// OBJETOS
// ======================================================
VespaMotors motors;
Adafruit_MPU6050 SensorGiro;

// ======================================================
// PINOS — SNAKE_CASE totalmente maiúsculo
//
// Mapa dos sensores (visão de cima do robô):
//
//   Esquerda ←————————————→ Direita
//   [PIN_SENSOR_PE] [PIN_SENSOR_CE] [PIN_SENSOR_CM] [PIN_SENSOR_CD] [PIN_SENSOR_PD]
//        L2               L1               M               R1              R2
//
// PE = Ponta Esquerda   CE = Centro Esquerda   CM = Centro Meio
// CD = Centro Direita   PD = Ponta Direita
// ======================================================
#define PIN_SENSOR_PE 17  // Ponta esquerda  (L2) -- TX2
#define PIN_SENSOR_CE 16  // Centro esquerda (L1) -- RX2
#define PIN_SENSOR_CM 18  // Centro meio     (M)  -- SCK
#define PIN_SENSOR_CD 5   // Centro direita  (R1) -- SS
#define PIN_SENSOR_PD 23  // Ponta direita   (R2) -- MOSI

// ======================================================
// I2C — Canal do multiplexador onde está o giroscópio
// ======================================================
#define I2C_CANAL_GIROSCOPIO 1  // Canal do MUX (TCA9548A, 0x70) onde o MPU-6050 está ligado

// ======================================================
// ENUM: Direcao
// ======================================================
enum Direcao {
  FRENTE,    // Ambos os motores para frente
  TRAS,      // Ambos os motores para trás
  DIREITA,   // Giro no próprio eixo para a direita
  ESQUERDA,  // Giro no próprio eixo para a esquerda
  PARAR      // Para os motores
};

// ======================================================
// ENUM: PerfilVelocidade
// Faixa válida: 0 a 100.
// Abaixo de ~30 o motor pode não vencer o atrito estático.
// ======================================================
enum PerfilVelocidade {
  VEL_DEFAULT = 80,
  VEL_BASE = 65,    // Velocidade padrão em linha reta
  VEL_CURVA = 75,   // Ajustada para manter a linha na curva
  VEL_SUBIDA = 75,  // Aumentada para vencer a gravidade
  VEL_DESCIDA = 55  // Reduzida para não perder o controle
};

// ======================================================
// ENUM: Desafio
// ======================================================
enum Desafio {
  INTERSECAO_SEM_MARCACAO,  // Interseções sem marcaçoes
  NOVENTA_GRAUS_ESQUERDA,   // Curva de 90 graus para esquerda
  NOVENTA_GRAUS_DIREITA,    // Curva de 90 graus para direita
  CURVA_LEVE_ESQUERDA,      // Curva leve/correção para esquerda
  CURVA_LEVE_DIREITA,       // Curva leve/correção para direita
  NENHUM                    // Andar para frente (não detectou nada)
};
// ======================================================
// VARIÁVEIS GLOBAIS — camelCase
// Booleanos com prefixo "is" para leitura natural nos ifs
// ======================================================
bool isSensorPE;                // Ponta esquerda  ativo = robô saiu muito à esq.
bool isSensorCE;                // Centro esquerda ativo = desvio leve à esq.
bool isSensorCM;                // Centro meio     ativo = robô centralizado
bool isSensorCD;                // Centro direita  ativo = desvio leve à dir.
bool isSensorPD;                // Ponta direita   ativo = robô saiu muito à dir.
Desafio desafioAtual = NENHUM;  // Variavel que define o desafio que o robô esta enfrentando

// ======================================================
// VARIÁVEIS GLOBAIS — MPU-6050 (somente giroscópio, ângulo em graus)
// ======================================================
float x = 0.0;  // Ângulo acumulado no eixo X (graus)
float y = 0.0;  // Ângulo acumulado no eixo Y (graus)
float z = 0.0;  // Ângulo acumulado no eixo Z (graus)

unsigned long tempoAnterior = 0;  // Usado para integrar °/s -> graus


// ======================================================
// PROTÓTIPOS DAS FUNÇÕES — camelCase, verbos no infinitivo
// ======================================================
void lerSensores();
void detectarDesafio();
void seguirLinha();
void mover(Direcao direcao, PerfilVelocidade velocidade, int tempo);
void selectChannel(uint8_t channel);

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  selectChannel(I2C_CANAL_GIROSCOPIO);

  SensorGiro.begin();
  tempoAnterior = millis();
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  lerSensores();
  detectarDesafio();
  seguirLinha();
  lerSensores();  // Atualiza sensores após os movimentos de seguirLinha()
}

// ======================================================
// IMPLEMENTAÇÕES
// ======================================================

/*
 * selectChannel(channel)
 * -------------------------------------------------------
 * O QUE FAZ : Seleciona o canal ativo no multiplexador I2C
 *             (TCA9548A, endereço 0x70), escrevendo direto
 *             no seu registrador de controle.
 *
 * PARÂMETROS:
 *   channel → número do canal (0 a 7)
 * -------------------------------------------------------
 */
void selectChannel(uint8_t channel) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

/*
 * lerSensores()
 * -------------------------------------------------------
 * O QUE FAZ : Lê os 5 sensores infravermelhos e armazena
 *             o resultado nas variáveis globais isSensor_.
 *             Também troca para o canal do giroscópio no
 *             multiplexador I2C (selectChannel) e lê o
 *             MPU-6050, integrando a velocidade angular no
 *             tempo para obter o ângulo acumulado (em graus)
 *             nos eixos X, Y, Z.
 *             Também imprime os valores no Monitor Serial
 *             para facilitar a depuração.
 *
 * QUANDO É CHAMADA:
 *   • No início e no fim de cada ciclo do loop()
 *   • Dentro de mover(), continuamente durante o
 *     movimento — garante que isSensorCM (e os demais)
 *     estejam sempre atualizados, mesmo enquanto o robô
 *     ainda está se movendo
 * -------------------------------------------------------
 */
void lerSensores() {
  isSensorPE = digitalRead(PIN_SENSOR_PE);
  isSensorCE = digitalRead(PIN_SENSOR_CE);
  isSensorCM = digitalRead(PIN_SENSOR_CM);
  isSensorCD = digitalRead(PIN_SENSOR_CD);
  isSensorPD = digitalRead(PIN_SENSOR_PD);

  // -------- SELECT CHANNEL: giroscópio --------
  selectChannel(I2C_CANAL_GIROSCOPIO);

  sensors_event_t a, g, temp;
  SensorGiro.getEvent(&a, &g, &temp);

  // Calcula delta de tempo desde a última leitura
  unsigned long agora = millis();
  float dt = (agora - tempoAnterior) / 1000.0;  // tempo em segundos
  tempoAnterior = agora;

  // -------- Converte para graus/s e remove o bias calibrado --------
  float 
    velX = (g.gyro.x * 57.2958) - offsetX;
  float velY = (g.gyro.y * 57.2958) - offsetY;
  float velZ = (g.gyro.z * 57.2958) - offsetZ;

  // -------- Zona-morta: ignora ruído pequeno (robô "parado") --------
  if (fabs(velX) < GIRO_DEADBAND) velX = 0;
  if (fabs(velY) < GIRO_DEADBAND) velY = 0;
  if (fabs(velZ) < GIRO_DEADBAND) velZ = 0;

  // Integra velocidade angular para obter ângulo em graus
  x += velX * dt;
  y += velY * dt;
  z += velZ * dt;

  // -------- Arredonda para número inteiro (sem casas decimais) --------
  x = roundf(x);
  y = roundf(y);
  z = roundf(z);

  // -------- DEBUG: mostra no Monitor Serial qual desafio foi detectado --------
  /**/
  Serial.print("PE: ");
  Serial.print(isSensorPE);
  Serial.print(" | CE: ");
  Serial.print(isSensorCE);
  Serial.print(" | CM: ");
  Serial.print(isSensorCM);
  Serial.print(" | CD: ");
  Serial.print(isSensorCD);
  Serial.print(" | PD: ");
  Serial.print(isSensorPD);
  Serial.print(" | X: ");
  Serial.print(x);
  Serial.print(" | Y: ");
  Serial.print(y);
  Serial.print(" | Z: ");
  Serial.println(z);
}

/*
 * mover(direcao, velocidade, tempo)
 * -------------------------------------------------------
 * O QUE FAZ : Move o robô por um tempo determinado e para.
 *
 * PARÂMETROS:
 *   direcao   → para onde ir        (enum Direcao)
 *                 FRENTE   — avança em linha reta
 *                 TRAS     — recua em linha reta
 *                 DIREITA  — gira no próprio eixo à direita
 *                 ESQUERDA — gira no próprio eixo à esquerda
 *                 PARAR    — para imediatamente
 *
 *   velocidade → quão rápido ir     (enum PerfilVelocidade)
 *                 VEL_BASE    — velocidade normal
 *                 VEL_CURVA   — velocidade para curvas
 *                 VEL_SUBIDA  — velocidade para subidas
 *                 VEL_DESCIDA — velocidade para descidas
 *
 *   tempo     → quanto tempo durar  (milissegundos)
 *                 Ex: 500 = meio segundo
 *
 * NÃO-BLOQUEANTE: em vez de delay(), usa millis() para
 * cronometrar o movimento. Isso permite que lerSensores()
 * continue sendo chamada durante o próprio movimento,
 * mantendo as leituras sempre atualizadas — importante
 * para os laços while() das curvas de 90° em seguirLinha().
 * -------------------------------------------------------
 */
void mover(Direcao direcao, PerfilVelocidade velocidade, int tempo) {
  lerSensores();
  int spd = (int)velocidade;  // converte o enum para int
  if (spd < 0) spd = 0;       // limite inferior: mínimo 0
  if (spd > 100) spd = 100;   // limite superior: máximo 100

  switch (direcao) {
    case FRENTE:
      motors.forward(spd);
      break;

    case TRAS:
      motors.backward(spd);
      break;

    case DIREITA:
      // Motor esquerdo avança + direito recua = giro à direita
      motors.setSpeedLeft(spd);
      motors.setSpeedRight(-spd);
      break;

    case ESQUERDA:
      // Motor direito avança + esquerdo recua = giro à esquerda
      motors.setSpeedLeft(-spd);
      motors.setSpeedRight(spd);
      break;

    case PARAR:
    default:
      motors.stop();
      break;
  }

  // -------- ESPERA NÃO-BLOQUEANTE (substitui delay(tempo)) --------
  // Em vez de travar a CPU, fica em loop lendo os sensores
  // continuamente até o tempo definido se esgotar.
  unsigned long tempoInicio = millis();
  while ((unsigned long)(millis() - tempoInicio) < (unsigned long)tempo) {
    lerSensores();
  }

  motors.stop();

  // Atualiza os sensores após o movimento para que o próximo
  // passo de seguirLinha() — ou o while() das curvas —
  // tome decisões com dados frescos
  lerSensores();
}

/*
 * detectarDesafio()
 * -------------------------------------------------------
 * O QUE FAZ : Analisa os sensores e determina qual
 *             situação o robô está enfrentando,
 *             armazenando o resultado em desafioAtual.
 *
 * PRIORIDADE DAS DECISÕES:
 *   1. isSensorPE ativo → curva de 90° à esquerda
 *
 *   2. isSensorPD ativo → curva de 90° à direita
 *
 *   3. isSensorCE ativo → correção suave à esquerda
 *
 *   4. isSensorCD ativo → correção suave à direita
 *
 *   5. Nenhum dos casos acima
 *      → segue em frente (NENHUM)
 *
 * POR QUE ESTA FUNÇÃO EXISTE:
 *   A decisão sobre qual movimento executar fica
 *   separada da execução do movimento em si.
 *   Assim, seguirLinha() apenas executa a ação
 *   correspondente usando um switch, deixando o
 *   código mais organizado e facilitando a inclusão
 *   de novos desafios no futuro.
 * -------------------------------------------------------
 */
void detectarDesafio() {
  if (isSensorPE || isSensorCE || isSensorCM || isSensorCD || isSensorPD) {  // -------- SENSORES VENDO PRETO EM QUALQUER LUGAR --------
    if (isSensorPE && isSensorPD && isSensorCM) {
      // -------- INTERSEÇÃO DUAS LINHAS SEM COR --------
      desafioAtual = INTERSECAO_SEM_MARCACAO;
    } else if (isSensorPE && !isSensorPD) {
      // -------- CURVA DE 90° PARA A ESQUERDA --------
      desafioAtual = NOVENTA_GRAUS_ESQUERDA;

    } else if (isSensorPD && !isSensorPE) {
      // -------- CURVA DE 90° PARA A DIREITA --------
      desafioAtual = NOVENTA_GRAUS_DIREITA;

    } else if (isSensorCE) {
      // -------- CORREÇÃO SUAVE PARA A ESQUERDA --------
      desafioAtual = CURVA_LEVE_ESQUERDA;

    } else if (isSensorCD) {
      // -------- CORREÇÃO SUAVE PARA A DIREITA --------
      desafioAtual = CURVA_LEVE_DIREITA;

    } else {
      // -------- LINHA RETA / NENHUM SENSOR ATIVO --------
      desafioAtual = NENHUM;
    }
  }

  // -------- DEBUG: mostra no Monitor Serial qual desafio foi detectado --------
  /*
  Serial.print("Desafio detectado: ");
  switch (desafioAtual) {
    case INTERSECAO_SEM_MARCACAO:
      Serial.println("INTERSECAO_SEM_MARCACAO");
      break;
    case NOVENTA_GRAUS_ESQUERDA:
      Serial.println("NOVENTA_GRAUS_ESQUERDA");
      break;
    case NOVENTA_GRAUS_DIREITA:
      Serial.println("NOVENTA_GRAUS_DIREITA");
      break;
    case CURVA_LEVE_ESQUERDA:
      Serial.println("CURVA_LEVE_ESQUERDA");
      break;
    case CURVA_LEVE_DIREITA:
      Serial.println("CURVA_LEVE_DIREITA");
      break;
    case NENHUM:
    default:
      Serial.println("NENHUM (linha reta)");
      break;
  }
  */
}


/*
 * seguirLinha()
 * -------------------------------------------------------
 * O QUE FAZ : Executa a ação correspondente ao desafio
 *             armazenado em desafioAtual.
 *
 * FUNCIONAMENTO:
 *   detectarDesafio() define qual situação o robô está
 *   enfrentando. Em seguida, esta função utiliza um
 *   switch para executar o movimento adequado.
 *
 * VANTAGEM:
 *   Separar a tomada de decisão da execução torna o
 *   código mais organizado, facilita a manutenção e
 *   permite adicionar novos desafios sem alterar a
 *   estrutura principal da função.
 * -------------------------------------------------------
 */
void seguirLinha() {

  switch (desafioAtual) {

    case INTERSECAO_SEM_MARCACAO:
      // -------- INTERSEÇÃO DUAS LINHAS SEM COR --------
      mover(PARAR, VEL_BASE, 500);
      mover(FRENTE, VEL_BASE, 425);
      break;

    case NOVENTA_GRAUS_ESQUERDA:
      // -------- CURVA DE 90° PARA A ESQUERDA --------
      mover(PARAR, VEL_BASE, 200);
      mover(FRENTE, VEL_BASE, 300);

      if (isSensorCM && isSensorPE && isSensorCE) {
        // -------- INTERSEÇÃO (uma ou duas linhas sem cor) --------
        mover(PARAR, VEL_BASE, 500);
        mover(FRENTE, VEL_BASE, 125);
      } else {
        // -------- CURVA 90° "PURA" --------
        mover(PARAR, VEL_BASE, 2000);
        mover(FRENTE, VEL_BASE, 50);
        mover(PARAR, VEL_BASE, 200);

        while (!isSensorCM) {
          mover(ESQUERDA, VEL_CURVA, 3);
        }

        mover(ESQUERDA, VEL_CURVA, 150);
      }

      break;

    case NOVENTA_GRAUS_DIREITA:
      // -------- CURVA DE 90° PARA A DIREITA --------
      mover(PARAR, VEL_BASE, 200);
      mover(FRENTE, VEL_BASE, 300);

      if (isSensorCM && isSensorPD && isSensorCD) {
        // -------- INTERSEÇÃO (uma ou duas linhas sem cor) --------
        mover(PARAR, VEL_BASE, 500);
        mover(FRENTE, VEL_BASE, 125);
      } else {
        // -------- CURVA 90° "PURA" --------
        mover(PARAR, VEL_BASE, 2000);
        mover(FRENTE, VEL_BASE, 50);
        mover(PARAR, VEL_BASE, 200);

        while (!isSensorCM) {
          mover(DIREITA, VEL_CURVA, 3);
        }

        mover(DIREITA, VEL_CURVA, 150);
      }

      break;

    case CURVA_LEVE_ESQUERDA:
      // -------- CORREÇÃO SUAVE PARA A ESQUERDA --------
      mover(ESQUERDA, VEL_CURVA, 125);
      mover(FRENTE, VEL_BASE, 50);
      break;

    case CURVA_LEVE_DIREITA:
      // -------- CORREÇÃO SUAVE PARA A DIREITA --------
      mover(DIREITA, VEL_CURVA, 125);
      mover(FRENTE, VEL_BASE, 50);
      break;

    case NENHUM:
    default:
      // -------- LINHA RETA / NENHUM SENSOR ATIVO --------
      mover(FRENTE, VEL_DEFAULT, 5);
      break;
  }
}