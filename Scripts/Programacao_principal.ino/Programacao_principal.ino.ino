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
 *   Distância   : VL53L0X x2 (sensores a laser), via multiplexador I2C
 *   Cor         : TCS34725 x2 (esquerda e direita), via multiplexador I2C (canais 3 e 4)
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
 *   O sensor de distância (VL53L0X) fornece a distância (em mm)
 *   até o obstáculo mais próximo à frente do robô. A troca de
 *   canal no multiplexador I2C (TCA9548A, endereço 0x70) é feita
 *   pela função selectChannel(), escrevendo direto no registrador
 *   de controle, sem biblioteca.
 * ======================================================
 */

// ======================================================
// BIBLIOTECAS
// ======================================================
#include <RoboCore_Vespa.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>

// ======================================================
// OBJETOS
// ======================================================
VespaMotors motors;
Adafruit_VL53L0X distanciaC = Adafruit_VL53L0X();
Adafruit_VL53L0X distanciaL = Adafruit_VL53L0X();
// Tempo de integração mínimo (2.4ms) para não travar o loop de
// seguirLinha() — leitura de cor rápida, ao custo de menor precisão.
Adafruit_TCS34725 corEsquerda = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 corDireita = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

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
// I2C — Canal do multiplexador onde está o sensor de distância
// ======================================================
#define I2C_CANAL_DISTANCIA_C 0
#define I2C_CANAL_DISTANCIA_L 2   // Canal do MUX (TCA9548A, 0x70) onde o VL53L0X está ligado
#define I2C_CANAL_COR_ESQUERDA 3  // Canal do MUX (TCA9548A, 0x70) onde o TCS34725 esquerdo está ligado
#define I2C_CANAL_COR_DIREITA 4   // Canal do MUX (TCA9548A, 0x70) onde o TCS34725 direito está ligado

// ======================================================
// AGENDAMENTO DAS LEITURAS I2C
// Os sensores de linha são lidos SEMPRE. As leituras I2C
// são espaçadas para não travar o seguimento da linha.
// ======================================================
#define INTERVALO_DISTANCIA_MS 12  // alterna C/L; cada VL atualiza ~a cada 24 ms
#define INTERVALO_COR_MS       12  // alterna E/D; cada TCS atualiza ~a cada 24 ms


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
  VEL_DEFAULT = 85,
  VEL_BASE = 70,    // Velocidade padrão em linha reta
  VEL_CURVA = 75,   // Ajustada para manter a linha na curva
  VEL_SUBIDA = 75,  // Aumentada para vencer a gravidade
  VEL_DESCIDA = 55  // Reduzida para não perder o controle
};

// ======================================================
// ENUM: Desafio
// ======================================================
enum Desafio {
  FIM_DA_PISTA,
  VERDE_DIREITA,
  VERDE_ESQUERDA,
  OBSTACULO,                // Obstaculo
  INTERSECAO_SEM_MARCACAO,  // Interseções sem marcaçoes
  NOVENTA_GRAUS_ESQUERDA,   // Curva de 90 graus para esquerda
  NOVENTA_GRAUS_DIREITA,    // Curva de 90 graus para direita
  CURVA_LEVE_ESQUERDA,      // Curva leve/correção para esquerda
  CURVA_LEVE_DIREITA,       // Curva leve/correção para direita
  NENHUM                    // Andar para frente (não detectou nada)
};
// ======================================================
// ENUM: Direcao
// ======================================================
enum Cor {
  VERDE,
  VERMELHO,
  SEM_COR
};
// ======================================================
// VARIÁVEIS GLOBAIS — camelCase
// Booleanos com prefixo "is" para leitura natural nos ifs
// ======================================================
bool isSensorPE;  // Ponta esquerda  ativo = robô saiu muito à esq.
bool isSensorCE;  // Centro esquerda ativo = desvio leve à esq.
bool isSensorCM;  // Centro meio     ativo = robô centralizado
bool isSensorCD;  // Centro direita  ativo = desvio leve à dir.
bool isSensorPD;  // Ponta direita   ativo = robô saiu muito à dir.
int intDistanciaC;
int intDistanciaL;                                                // Última distância lida pelo VL53L0X (em cm)
uint16_t corEsquerdaR, corEsquerdaG, corEsquerdaB, corEsquerdaC;  // Última leitura RGB do TCS34725 esquerdo
uint16_t corDireitaR, corDireitaG, corDireitaB, corDireitaC;      // Última leitura RGB do TCS34725 direito
Desafio desafioAtual = NENHUM;                                    // Variavel que define o desafio que o robô esta enfrentando


// ======================================================
// PROTÓTIPOS DAS FUNÇÕES — camelCase, verbos no infinitivo
// ======================================================
void lerSensores();
void lerSensoresI2C();
void detectarDesafio();
void seguirLinha();
void mover(Direcao direcao, PerfilVelocidade velocidade, int tempo);
void selectChannel(uint8_t channel);

// ======================================================
// SETUP
// ======================================================
void setup() {
  // -------- DEBUG: liga o Monitor Serial -------
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // Fast Mode: 400 kHz é o limite prático/recomendado aqui.
#if defined(ESP32)
  Wire.setTimeOut(2);     // evita travamento longo do loop caso algum dispositivo I2C falhe
#endif

  // -------- Inicializa o sensor de distância no canal 0 do MUX --------
  selectChannel(I2C_CANAL_DISTANCIA_C);
  if (!distanciaC.begin()) {
    Serial.println("ERRO: VL53L0X C nao iniciou");
  } else {
    distanciaC.startRangeContinuous(20);
  }
  selectChannel(I2C_CANAL_DISTANCIA_L);
  if (!distanciaL.begin()) {
    Serial.println("ERRO: VL53L0X L nao iniciou");
  } else {
    distanciaL.startRangeContinuous(20);
  }

  // -------- Inicializa os sensores de cor (canais 3 e 4 do MUX) --------
  selectChannel(I2C_CANAL_COR_ESQUERDA);
  if (!corEsquerda.begin()) {
    Serial.println("ERRO: TCS34725 esquerdo nao iniciou");
  }
  selectChannel(I2C_CANAL_COR_DIREITA);
  if (!corDireita.begin()) {
    Serial.println("ERRO: TCS34725 direito nao iniciou");
  }

  // Primeira leitura de linha + I2C para começar com valores válidos.
  lerSensores();
  selectChannel(I2C_CANAL_COR_ESQUERDA);
  corEsquerda.getRawData(&corEsquerdaR, &corEsquerdaG, &corEsquerdaB, &corEsquerdaC);
  selectChannel(I2C_CANAL_COR_DIREITA);
  corDireita.getRawData(&corDireitaR, &corDireitaG, &corDireitaB, &corDireitaC);
  // As distâncias serão preenchidas pelo modo contínuo no loop.
  intDistanciaC = 999;
  intDistanciaL = 999;
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  lerSensores();
  detectarDesafio();
  seguirLinha();
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
// Guarda o canal atualmente selecionado no MUX, para não escrever
// de novo no registrador quando o canal pedido já está ativo.
uint8_t canalAtualMUX = 0xFF;

// Troca de canal mínima: só escreve no TCA9548A quando o canal realmente muda.
// Isso NÃO faz cache das leituras dos sensores; apenas evita transmissões I2C repetidas.
void selectChannel(uint8_t channel) {
  if (channel == canalAtualMUX) return;

  Wire.beginTransmission(0x70);
  Wire.write((uint8_t)(1u << channel));
  Wire.endTransmission(false);
  canalAtualMUX = channel;
}

/*
 * lerSensores()
 * -------------------------------------------------------
 * O QUE FAZ : Lê os 5 sensores infravermelhos e armazena
 *             o resultado nas variáveis globais isSensor_.
 *             Também troca para o canal do sensor de distância
 *             no multiplexador I2C (selectChannel) e lê o
 *             VL53L0X (modo contínuo, não-bloqueante) e, quando
 *             há uma medição nova pronta, atualiza distanciaCM
 *             (em cm). Se não houver medição nova, mantém o
 *             último valor lido.
 *             Também troca para os canais 3 e 4 do multiplexador
 *             I2C e lê os sensores de cor TCS34725 (esquerda e
 *             direita), armazenando os valores brutos de R, G, B
 *             e C (clear) nas variáveis globais correspondentes.
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
  // CRÍTICO: sensores de linha ficam fora do I2C e são lidos em TODO ciclo.
  isSensorPE = digitalRead(PIN_SENSOR_PE);
  isSensorCE = digitalRead(PIN_SENSOR_CE);
  isSensorCM = digitalRead(PIN_SENSOR_CM);
  isSensorCD = digitalRead(PIN_SENSOR_CD);
  isSensorPD = digitalRead(PIN_SENSOR_PD);

  // I2C é executado de forma agendada, sem bloquear cada ciclo da linha.
  lerSensoresI2C();
}

void lerSensoresI2C() {
  static uint32_t ultimaDistancia = 0;
  static uint32_t ultimaCor = 0;
  static bool proximaCorEsquerda = true;
  static bool proximaDistanciaCentral = true;

  const uint32_t agora = millis();

  // ======================================================
  // DISTÂNCIA
  // ======================================================
  // Os VL53 ficam em modo contínuo. Aqui NÃO usamos readRange(),
  // pois essa função é de leitura single-shot e pode esperar uma
  // medição. Apenas verificamos se a medição contínua terminou.
  if ((uint32_t)(agora - ultimaDistancia) >= INTERVALO_DISTANCIA_MS) {
    ultimaDistancia = agora;

    if (proximaDistanciaCentral) {
      selectChannel(I2C_CANAL_DISTANCIA_C);

      if (distanciaC.isRangeComplete()) {
        uint16_t mm = distanciaC.readRangeResult();

        if (distanciaC.readRangeStatus() == 0 && mm > 0) {
          intDistanciaC = mm / 10;
        }
      }
    } else {
      selectChannel(I2C_CANAL_DISTANCIA_L);

      if (distanciaL.isRangeComplete()) {
        uint16_t mm = distanciaL.readRangeResult();

        if (distanciaL.readRangeStatus() == 0 && mm > 0) {
          intDistanciaL = mm / 10;
        }
      }
    }

    proximaDistanciaCentral = !proximaDistanciaCentral;
  }

  // ======================================================
  // COR
  // ======================================================
  // Nunca lê os dois TCS no mesmo instante.
  if ((uint32_t)(agora - ultimaCor) >= INTERVALO_COR_MS) {
    ultimaCor = agora;

    if (proximaCorEsquerda) {
      selectChannel(I2C_CANAL_COR_ESQUERDA);

      corEsquerda.getRawData(
        &corEsquerdaR,
        &corEsquerdaG,
        &corEsquerdaB,
        &corEsquerdaC
      );
    } else {
      selectChannel(I2C_CANAL_COR_DIREITA);

      corDireita.getRawData(
        &corDireitaR,
        &corDireitaG,
        &corDireitaB,
        &corDireitaC
      );
    }

    proximaCorEsquerda = !proximaCorEsquerda;
  }
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
  // 1) Verde à direita. A margem evita que qualquer leitura com G apenas
  // ligeiramente maior que R/B seja interpretada como verde.
  if (corDireitaC > 80 &&
      corDireitaG > (uint16_t)(corDireitaR * 1.25f) &&
      corDireitaG > (uint16_t)(corDireitaB * 1.15f)) {

    desafioAtual = VERDE_DIREITA;
    return;
  }

  // 2) Obstáculo central.
  if (intDistanciaC > 0 && intDistanciaC <= 10) {
    desafioAtual = OBSTACULO;
    return;
  }

  // 3) Seguimento da linha.
  if (isSensorPE || isSensorCE || isSensorCM || isSensorCD || isSensorPD) {
    if (isSensorPE && isSensorPD && isSensorCM) {
      desafioAtual = INTERSECAO_SEM_MARCACAO;
    } else if (isSensorPE && !isSensorPD) {
      desafioAtual = NOVENTA_GRAUS_ESQUERDA;
    } else if (isSensorPD && !isSensorPE) {
      desafioAtual = NOVENTA_GRAUS_DIREITA;
    } else if (isSensorCE) {
      desafioAtual = CURVA_LEVE_ESQUERDA;
    } else if (isSensorCD) {
      desafioAtual = CURVA_LEVE_DIREITA;
    } else {
      desafioAtual = NENHUM;
    }
  } else {
    // IMPORTANTE: nunca deixa um desafio antigo preso.
    desafioAtual = NENHUM;
  }
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

    case VERDE_DIREITA:
      // -------- INTERSEÇÃO COM MARCAÇÃO NA DIREITA --------
      mover(PARAR, VEL_BASE, 100);
      mover(FRENTE, VEL_BASE, 400);
      break;

    case OBSTACULO:
      // -------- OBSTACULO --------
      mover(PARAR, VEL_BASE, 100);
      mover(TRAS, VEL_DEFAULT, 150);
      mover(DIREITA, VEL_CURVA, 200);
      while (!(intDistanciaL >= 15 && intDistanciaL <= 20)) {  //adicionar redundancia ao while, ou filtro a leitura
        lerSensores();
        mover(DIREITA, VEL_CURVA, 3);
      }
      mover(DIREITA, VEL_CURVA, 250);
      mover(PARAR, VEL_BASE, 100);
      mover(FRENTE, VEL_BASE, 2875);
      mover(ESQUERDA, VEL_CURVA, 300);
      mover(PARAR, VEL_BASE, 100);
      while (!(intDistanciaL >= 10 && intDistanciaL <= 15)) {
        lerSensores();
        mover(ESQUERDA, VEL_CURVA, 3);
      }
      mover(PARAR, VEL_BASE, 100);
      mover(ESQUERDA, VEL_CURVA, 475);
      mover(PARAR, VEL_BASE, 100);
      mover(FRENTE, VEL_BASE, 3250);
      mover(ESQUERDA, VEL_CURVA, 200);
      mover(PARAR, VEL_BASE, 100);
      while (!(intDistanciaL >= 10 && intDistanciaL <= 15)) {
        lerSensores();
        mover(ESQUERDA, VEL_CURVA, 3);
      }
      mover(PARAR, VEL_BASE, 100);
      mover(ESQUERDA, VEL_CURVA, 1425);
      mover(FRENTE, VEL_BASE, 2000);
      while (!isSensorCM) {
        lerSensores();
        mover(FRENTE, VEL_DEFAULT, 3);
      }
      mover(DIREITA, VEL_CURVA, 200);
      mover(PARAR, VEL_BASE, 100);
      break;
    case INTERSECAO_SEM_MARCACAO:
      // -------- INTERSEÇÃO DUAS LINHAS SEM COR --------
      mover(PARAR, VEL_BASE, 100);
      mover(FRENTE, VEL_BASE, 425);
      break;

    case NOVENTA_GRAUS_ESQUERDA:
      // -------- CURVA DE 90° PARA A ESQUERDA --------
      mover(FRENTE, VEL_BASE, 300);

      if (isSensorCM && isSensorPE && isSensorCE) {
        // -------- INTERSEÇÃO (uma ou duas linhas sem cor) --------
        mover(FRENTE, VEL_BASE, 125);
      } else {
        // -------- CURVA 90° "PURA" --------
        mover(FRENTE, VEL_BASE, 50);

        while (!isSensorCM) {
          mover(ESQUERDA, VEL_CURVA, 3);
        }

        mover(ESQUERDA, VEL_CURVA, 150);
      }

      break;

    case NOVENTA_GRAUS_DIREITA:
      // -------- CURVA DE 90° PARA A DIREITA --------
      mover(FRENTE, VEL_BASE, 300);

      if (isSensorCM && isSensorPD && isSensorCD) {
        // -------- INTERSEÇÃO (uma ou duas linhas sem cor) --------
        mover(FRENTE, VEL_BASE, 125);
      } else {
        // -------- CURVA 90° "PURA" --------
        mover(FRENTE, VEL_BASE, 50);

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
      // -------- LINHA RETA / NENHUM DESAFIO --------
      // NÃO usar mover(..., 5) aqui: isso liga o motor por 5 ms
      // e manda parar logo em seguida. Em motores pequenos isso
      // pode não vencer o atrito estático, fazendo o robô parecer
      // que "não anda".
      //
      // Mantemos os motores ligados continuamente. O próximo ciclo
      // do loop atualiza os sensores e pode mudar a direção.
      motors.forward(VEL_DEFAULT);
      break;
  }
}