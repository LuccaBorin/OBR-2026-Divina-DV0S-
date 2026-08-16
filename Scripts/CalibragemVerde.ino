#include <Wire.h>
#include <Adafruit_TCS34725.h>

#define led 15

#define canalDireito 4
#define canalEsquerdo 3

Adafruit_TCS34725 sensor(
  TCS34725_INTEGRATIONTIME_2_4MS,
  TCS34725_GAIN_4X
);

int r, g, b, c;

int medR, medG, medB, medC;

int valG1, valG2, valG3;
int valR1, valR2, valR3;
int valB1, valB2, valB3;
int valC1, valC2, valC3;

float porcentagemG;
float porcentagemGR;
float porcentagemGB;

void selecionarCanal(int canal) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << canal);
  Wire.endTransmission();
}

void setup() {

  pinMode(led, OUTPUT);

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  Serial.println("==== CALIBRAGEM DO VERDE ====");
  Serial.println("Primeiro: DIREITO");
  Serial.println("Depois: ESQUERDO");

  // Sensor direito
  selecionarCanal(canalDireito);

  if (!sensor.begin()) {
    Serial.println("Erro no sensor direito!");
    while (true);
  }

  // Sensor esquerdo
  selecionarCanal(canalEsquerdo);

  if (!sensor.begin()) {
    Serial.println("Erro no sensor esquerdo!");
    while (true);
  }
}

void loop() {

  // ==================================================
  // DIREITO
  // ==================================================

  selecionarCanal(canalDireito);

  Serial.println();
  Serial.println("========== VERDE DIREITO ==========");

  // LEITURA 1
  digitalWrite(led, HIGH);
  delay(3000);
  digitalWrite(led, LOW);

  sensor.getRawData(
    (uint16_t*)&r,
    (uint16_t*)&g,
    (uint16_t*)&b,
    (uint16_t*)&c
  );

  valR1 = r;
  valG1 = g;
  valB1 = b;
  valC1 = c;

  Serial.println("Leitura 1 feita.");

  // LEITURA 2
  digitalWrite(led, HIGH);
  delay(3000);
  digitalWrite(led, LOW);

  sensor.getRawData(
    (uint16_t*)&r,
    (uint16_t*)&g,
    (uint16_t*)&b,
    (uint16_t*)&c
  );

  valR2 = r;
  valG2 = g;
  valB2 = b;
  valC2 = c;

  Serial.println("Leitura 2 feita.");

  // LEITURA 3
  digitalWrite(led, HIGH);
  delay(3000);
  digitalWrite(led, LOW);

  sensor.getRawData(
    (uint16_t*)&r,
    (uint16_t*)&g,
    (uint16_t*)&b,
    (uint16_t*)&c
  );

  valR3 = r;
  valG3 = g;
  valB3 = b;
  valC3 = c;

  Serial.println("Leitura 3 feita.");

  // MÉDIAS
  medR = (valR1 + valR2 + valR3) / 3;
  medG = (valG1 + valG2 + valG3) / 3;
  medB = (valB1 + valB2 + valB3) / 3;
  medC = (valC1 + valC2 + valC3) / 3;

  // PORCENTAGENS
  porcentagemGR = ((float)(medG - medR) / medR) * 100.0;
  porcentagemGB = ((float)(medG - medB) / medB) * 100.0;
  porcentagemG = ((float)medG / (medR + medG + medB)) * 100.0;

  // RESULTADO DIREITO
  Serial.println();
  Serial.println("===== RESULTADO DIREITO =====");

  Serial.print("R: ");
  Serial.println(medR);

  Serial.print("G: ");
  Serial.println(medG);

  Serial.print("B: ");
  Serial.println(medB);

  Serial.print("C: ");
  Serial.println(medC);

  Serial.print("G maior que R: ");
  Serial.print(porcentagemGR, 2);
  Serial.println("%");

  Serial.print("G maior que B: ");
  Serial.print(porcentagemGB, 2);
  Serial.println("%");

  Serial.print("G de R+G+B: ");
  Serial.print(porcentagemG, 2);
  Serial.println("%");


  // ==================================================
  // ESQUERDO
  // ==================================================

  selecionarCanal(canalEsquerdo);

  Serial.println();
  Serial.println("========== VERDE ESQUERDO ==========");

  // LEITURA 1
  digitalWrite(led, HIGH);
  delay(3000);
  digitalWrite(led, LOW);

  sensor.getRawData(
    (uint16_t*)&r,
    (uint16_t*)&g,
    (uint16_t*)&b,
    (uint16_t*)&c
  );

  valR1 = r;
  valG1 = g;
  valB1 = b;
  valC1 = c;

  Serial.println("Leitura 1 feita.");

  // LEITURA 2
  digitalWrite(led, HIGH);
  delay(3000);
  digitalWrite(led, LOW);

  sensor.getRawData(
    (uint16_t*)&r,
    (uint16_t*)&g,
    (uint16_t*)&b,
    (uint16_t*)&c
  );

  valR2 = r;
  valG2 = g;
  valB2 = b;
  valC2 = c;

  Serial.println("Leitura 2 feita.");

  // LEITURA 3
  digitalWrite(led, HIGH);
  delay(3000);
  digitalWrite(led, LOW);

  sensor.getRawData(
    (uint16_t*)&r,
    (uint16_t*)&g,
    (uint16_t*)&b,
    (uint16_t*)&c
  );

  valR3 = r;
  valG3 = g;
  valB3 = b;
  valC3 = c;

  Serial.println("Leitura 3 feita.");

  // MÉDIAS
  medR = (valR1 + valR2 + valR3) / 3;
  medG = (valG1 + valG2 + valG3) / 3;
  medB = (valB1 + valB2 + valB3) / 3;
  medC = (valC1 + valC2 + valC3) / 3;

  // PORCENTAGENS
  porcentagemGR = ((float)(medG - medR) / medR) * 100.0;
  porcentagemGB = ((float)(medG - medB) / medB) * 100.0;
  porcentagemG = ((float)medG / (medR + medG + medB)) * 100.0;

  // RESULTADO ESQUERDO
  Serial.println();
  Serial.println("===== RESULTADO ESQUERDO =====");

  Serial.print("R: ");
  Serial.println(medR);

  Serial.print("G: ");
  Serial.println(medG);

  Serial.print("B: ");
  Serial.println(medB);

  Serial.print("C: ");
  Serial.println(medC);

  Serial.print("G maior que R: ");
  Serial.print(porcentagemGR, 2);
  Serial.println("%");

  Serial.print("G maior que B: ");
  Serial.print(porcentagemGB, 2);
  Serial.println("%");

  Serial.print("G de R+G+B: ");
  Serial.print(porcentagemG, 2);
  Serial.println("%");

  Serial.println();
  Serial.println("===== CALIBRAGEM TERMINADA =====");

  digitalWrite(led, HIGH);

  while (true);
}
