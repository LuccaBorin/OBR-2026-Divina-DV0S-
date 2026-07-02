#define out1 36
#define out2 39
#define led 15

int medB, maxB, minB, valB1, valB2, valB3;
int medP, maxP, minP, valP1, valP2, valP3;
int leitura1, leitura2;

void setup() {
  pinMode(out1, INPUT);
  pinMode(out2, INPUT);
  pinMode(15, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  //--- PRETO 1 ---
  digitalWrite(led,LOW);
  delay(4000);
  leitura1 = analogRead(out1);
  leitura2 = analogRead(out2);
  delay(1000);
  digitalWrite(led,HIGH);
  valP1 = (leitura1 + leitura2) / 2;
  delay(1000);

  //--- BRANCO 1 ---
  digitalWrite(led,LOW);
  delay(4000);
  leitura1 = analogRead(out1);
  leitura2 = analogRead(out2);
  delay(1000);
  digitalWrite(led,HIGH);
  valB1 = (leitura1 + leitura2) / 2;
  delay(1000);

  //--- PRETO 2 ---
  digitalWrite(led,LOW);
  delay(4000);
  leitura1 = analogRead(out1);
  leitura2 = analogRead(out2);
  delay(1000);
  digitalWrite(led,HIGH);
  valP2 = (leitura1 + leitura2) / 2;
  delay(1000);

  //--- BRANCO 2 ---
  digitalWrite(led,LOW);
  delay(4000);
  leitura1 = analogRead(out1);
  leitura2 = analogRead(out2);
  delay(1000);
  digitalWrite(led,HIGH);
  valB2 = (leitura1 + leitura2) / 2;
  delay(1000);

  //--- PRETO 3 ---
  digitalWrite(led,LOW);
  delay(4000);
  leitura1 = analogRead(out1);
  leitura2 = analogRead(out2);
  delay(1000);
  digitalWrite(led,HIGH);
  valP3 = (leitura1 + leitura2) / 2;
  delay(1000);

  //--- BRANCO 3 ---
  digitalWrite(led,LOW);
  delay(4000);
  leitura1 = analogRead(out1);
  leitura2 = analogRead(out2);
  delay(1000);
  digitalWrite(led,HIGH);
  valB3 = (leitura1 + leitura2) / 2;
  delay(1000);

  //--- CALCULA MÉDIAS ---
  medP = (valP1 + valP2 + valP3) / 3;
  maxP = medP + 20;
  minP = medP - 20;

  medB = (valB1 + valB2 + valB3) / 3;
  maxB = medB + 20;
  minB = medB - 20;

  //--- MOSTRA RESULTADOS ---
  Serial.println("==== Calibragem ====");
  
  Serial.print("Branco -> Media: ");
  Serial.print(medB);
  Serial.print(" | Max: ");
  Serial.print(maxB);
  Serial.print(" | Min: ");
  Serial.println(minB);

  Serial.print("Preto -> Media: ");
  Serial.print(medP);
  Serial.print(" | Max: ");
  Serial.print(maxP);
  Serial.print(" | Min: ");
  Serial.println(minP);

  while (true);
}