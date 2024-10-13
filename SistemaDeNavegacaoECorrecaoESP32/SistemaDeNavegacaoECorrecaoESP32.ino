#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// Definindo os pinos dos servos
const int servoPin1 = 12; // Servo 1 (Eixo X Positivo)
const int servoPin2 = 13; // Servo 2 (Eixo X Negativo)
const int servoPin3 = 14; // Servo 3 (Eixo Y)
const int servoPin4 = 27; // Servo 4 (Eixo Y)

// Criação dos objetos Servo e MPU6050
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
MPU6050 mpu;

// Criando objeto de servidor web e WebSocket
WebServer server;
WebSocketsServer webSocket = WebSocketsServer(81);

// Dados Wi-Fi
const char* ssid = "7LINK_WDLP"; // Nome da rede Wi-Fi
const char* password = "#SASpt92"; // Senha da rede Wi-Fi

// Variáveis de controle de tempo
unsigned long previousMillis = 0; 
const long interval = 100; // Intervalo de 100 ms para leitura do MPU6050

// Variáveis para calibração
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
const int calibration_samples = 100; // Número de amostras para calibração

// Variáveis para suavização do ângulo Z
float angleZ = 0; // Ângulo Z atual
float filteredAngleZ = 0; // Ângulo Z filtrado
const float alpha = 0.1; // Fator de suavização (0 < alpha < 1)

// Limite para detectar movimento
const float movementThreshold = 0.1; // Ajuste conforme necessário

// Declarações de funções
void handleRoot();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void calibrateMPU6050();

void setup() {
  // Inicializando comunicação serial
  Serial.begin(115200);

  // Inicializando os servos
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);

  // Configurando a posição inicial dos servos
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  
  // Delay para garantir que os servos se posicionem
  delay(1000);

  // Inicializando o MPU6050
  Wire.begin();
  mpu.initialize();
  Serial.println("MPU6050 inicializado.");

  // Calibrando o MPU6050 assim que inicia
  calibrateMPU6050();

  // Conectando ao Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Conectando ao Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado!");

  // Exibir o endereço IP no Serial Monitor
  Serial.print("Endereço IP do ESP32: ");
  Serial.println(WiFi.localIP()); // Adicionando esta linha

  // Definindo a rota para a página web
  server.on("/", handleRoot);
  server.begin();
  
  // Iniciando o WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

// Função de calibração do MPU6050
void calibrateMPU6050() {
  Serial.println("Calibrando o MPU6050...");
  
  int16_t sum[6] = {0}; // Array para armazenar a soma dos valores [ax, ay, az, gx, gy, gz]

  // Coletar dados para calibração
  for (int i = 0; i < calibration_samples; i++) {
    int16_t data[6];
    mpu.getMotion6(&data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
    
    // Somar os valores
    for (int j = 0; j < 6; j++) {
      sum[j] += data[j];
    }
    
    delay(10); // Pequeno delay para estabilizar as leituras
  }

  // Calcular os offsets e armazená-los
  ax_offset = sum[0] / calibration_samples;
  ay_offset = sum[1] / calibration_samples;
  az_offset = sum[2] / calibration_samples;
  gx_offset = sum[3] / calibration_samples;
  gy_offset = sum[4] / calibration_samples;
  gz_offset = sum[5] / calibration_samples;

  Serial.println("Calibração concluída.");
  Serial.print("Offsets: Ax: "); Serial.print(ax_offset); 
  Serial.print(", Ay: "); Serial.print(ay_offset);
  Serial.print(", Az: "); Serial.print(az_offset); 
  Serial.print(", Gx: "); Serial.print(gx_offset);
  Serial.print(", Gy: "); Serial.print(gy_offset);
  Serial.print(", Gz: "); Serial.println(gz_offset);
}


void loop() {
  // Lendo dados do MPU6050 a cada 100 ms
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Armazena o tempo atual

    // Lendo dados do MPU6050
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Subtraindo os valores de calibração
    ax -= ax_offset;
    ay -= ay_offset;
    az -= az_offset;
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;

    // Cálculo das acelerações em m/s²
    float accelX = abs(ax / 16384.0 * 9.81); // Convertendo para m/s²
    float accelY = abs(ay / 16384.0 * 9.81); // Convertendo para m/s²
    float accelZ = abs((az / 16384.0 * 9.81) - 8.68); // Convertendo para m/s²

    // Cálculo dos ângulos
    float angleX = atan2(ay, az) * 180 / PI; // -180 a 180
    float angleY = atan2(ax, az) * 180 / PI; // -180 a 180
    float angleZ = atan2(ax, ay) * 180 / PI; // -180 a 180 

    // Suavizando o ângulo Z usando filtro exponencial
    filteredAngleZ = alpha * angleZ + (1 - alpha) * filteredAngleZ;

    // Atualizando a posição dos servos
    // Invertendo o sentido de rotação
    int servo1Angle = 90 - angleX; // Servo 1 (Eixo X Positivo)
    int servo2Angle = 90 + angleX; // Servo 2 (Eixo X Negativo)
    int servo3Angle = 90 - angleY; // Servo 3 (Eixo Y)
    int servo4Angle = 90 + angleY; // Servo 4 (Eixo Y)

    // Definindo o limite dos ângulos dos servos
    servo1Angle = constrain(servo1Angle, 20, 160);
    servo2Angle = constrain(servo2Angle, 20, 160);
    servo3Angle = constrain(servo3Angle, 20, 160);
    servo4Angle = constrain(servo4Angle, 20, 160);

    // Posicionando os servos
    servo1.write(servo1Angle);
    servo2.write(servo2Angle);
    servo3.write(servo3Angle);
    servo4.write(servo4Angle);

    // Enviar dados pelo WebSocket para todos os clientes conectados
    String json = String("{\"ax\":") + accelX + ",\"ay\":" + accelY + ",\"az\":" + accelZ + 
                   ",\"angleX\":" + angleX + ",\"angleY\":" + angleY + 
                   ",\"angleZ\":" + filteredAngleZ + "}"; // Usando filteredAngleZ

    // Enviar dados para todos os clientes conectados
    for (int i = 0; i < webSocket.connectedClients(); i++) {
        webSocket.sendTXT(i, json); // Enviar dados para cada cliente conectado
    }
  }

  // Processar as requisições do servidor web
  server.handleClient();
  webSocket.loop(); // Manter o WebSocket ativo
}


void handleRoot() {
  String html = R"rawliteral(
  <html>
  <head>
    <meta charset='UTF-8'>
    <title>Dados do MPU6050</title>
    <script src='https://cdnjs.cloudflare.com/ajax/libs/socket.io/3.0.0/socket.io.js'></script>
    <script>
      var socket = new WebSocket('ws://' + window.location.hostname + ':81/');
      socket.onmessage = function(event) {
        var data = JSON.parse(event.data);
        document.getElementById('ax').innerText = data.ax;
        document.getElementById('ay').innerText = data.ay;
        document.getElementById('az').innerText = data.az;
        document.getElementById('angleX').innerText = data.angleX;
        document.getElementById('angleY').innerText = data.angleY;
        document.getElementById('angleZ').innerText = data.angleZ; // Atualização para ângulo Z
      };
    </script>
  </head>
  <body>
    <h1>Dados do MPU6050</h1>
    <p>Aceleração X: <span id='ax'>0</span> m/s²</p>
    <p>Aceleração Y: <span id='ay'>0</span> m/s²</p>
    <p>Aceleração Z: <span id='az'>0</span> m/s²</p>
    <p>Ângulo X: <span id='angleX'>0</span> º</p>
    <p>Ângulo Y: <span id='angleY'>0</span> º</p>
    <p>Ângulo Z: <span id='angleZ'>0</span> º</p>
  </body>
  </html>
  )rawliteral";
  
  server.send(200, "text/html", html);
}

// Função do WebSocket
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  // Lidar com eventos de WebSocket
  if (type == WStype_DISCONNECTED) {
    Serial.printf("Cliente %u desconectado\n", num);
  } else if (type == WStype_CONNECTED) {
    Serial.printf("Cliente %u conectado\n", num);
    // Enviar dados iniciais ao novo cliente
    String json = String("{\"ax\":0,\"ay\":0,\"az\":0,\"correctedAz\":0,\"angleX\":0,\"angleY\":0,\"angleZ\":0}");
    webSocket.sendTXT(num, json);
  } else if (type == WStype_TEXT) {
    // Você pode adicionar lógica para lidar com mensagens recebidas do cliente
    Serial.printf("Mensagem recebida do cliente %u: %s\n", num, payload);
  }
}
