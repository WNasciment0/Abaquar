#include <HX711_ADC.h>

// Pinos da célula de carga:
const int HX711_dout = 4; // MCU > HX711 dout pin
const int HX711_sck = 5;  // MCU > HX711 sck pin

HX711_ADC LoadCell(HX711_dout, HX711_sck);

const float CONVERSION_FACTOR = 9.81; // Conversão de KG para Newton
long t;
bool stop = false; // Variável para parar o loop

void setup() {
  Serial.begin(57600);
  Serial.println("Calibração da Célula de Carga");

  LoadCell.begin();
  long stabilizingTime = 2000;
  boolean _tare = false;
  LoadCell.start(stabilizingTime, _tare);

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Erro de conexão, verifique o circuito");
    while (1); // Pausa infinita em caso de erro
  } else {
    LoadCell.setCalFactor(1.0); // Valor de calibração inicial
    Serial.println("Inicialização completa");
  }

  while (!LoadCell.update());
  tare(); // Executa a tare (zerar) no início
  calibrate(); // Inicia o procedimento de calibração
}

void loop() {
  // Verifica se o comando 'p' foi recebido para parar o loop
  if (stop) {
    return;  // Sai completamente do loop
  }

  static boolean newDataReady = 0;
  const int serialPrintInterval = 100; // Intervalo entre as leituras

  if (LoadCell.update()) newDataReady = true;

  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      // Corrigido para ler o valor real da célula de carga
      float massa = LoadCell.getData() / 1000.0; // Convertendo de gramas para quilogramas
      float forca = massa * CONVERSION_FACTOR;

      // Exibir o valor da força em KG e Newtons
      Serial.print(massa, 3);
      Serial.print(" KG | ");
      Serial.print(forca, 3);
      Serial.println(" N");

      newDataReady = 0;
      t = millis();
    }
  }

  // Verifica se o comando para interromper o loop foi enviado
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 'p') {  // Comando 'p' para parar a medição
      stop = true; // Define a variável para interromper a medição
      Serial.println("Medição interrompida.");
    }
  }
}

void tare() {
  Serial.println("Zerando o peso. Aguarde...");
  LoadCell.tareNoDelay();

  while (LoadCell.getTareStatus() == false) {
    LoadCell.update();
  }

  Serial.println("Zeragem completa.");
}

void calibrate() {
  Serial.println("Coloque um peso conhecido e insira o valor em gramas no monitor serial.");

  float peso_conhecido = 0;
  boolean calibrado = false;

  while (!calibrado) {
    if (Serial.available() > 0) {
      peso_conhecido = Serial.parseFloat();
      if (peso_conhecido != 0) {
        LoadCell.refreshDataSet(); // Atualiza o conjunto de dados
        float novoCalFactor = LoadCell.getNewCalibration(peso_conhecido); // Obtém o novo valor de calibração
        LoadCell.setCalFactor(novoCalFactor); // Define o novo fator de calibração
        
        Serial.print("Calibração concluída. Novo valor: ");
        Serial.println(novoCalFactor);
        calibrado = true;
      }
    }
  }
}
