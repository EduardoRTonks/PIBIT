/*
  Sketch para o ESP8266
  - Ouve na porta Serial (conectada ao Mega).
  - Conecta-se a uma rede Wi-Fi.
  - Transmite os dados recebidos via UDP para toda a rede.
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// --- Configurações de Rede ---
const char* ssid = "nome";    // <<-- INSIRA O NOME DA SUA REDE AQUI
const char* password = "senha"; // <<-- INSIRA A SENHA DA SUA REDE AQUI

// --- Configurações UDP ---
WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // Porta para ouvir e enviar pacotes
char incomingPacket[255];          // Buffer para os pacotes recebidos

void setup() {
  // A porta Serial do ESP é usada para comunicar com o Mega
  Serial.begin(115200);
  
  // Conecta ao Wi-Fi
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Conectando a ");
  Serial.print(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nConectado com sucesso!");
  Serial.print("Endereço IP do ESP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Transmitindo pacotes UDP para a porta: ");
  Serial.println(localUdpPort);

  // Inicia o listener UDP
  Udp.begin(localUdpPort);
}

void loop() {
  // Se houver dados chegando do Mega
  if (Serial.available() > 0) {
    // Lê a linha de dados
    String line = Serial.readStringUntil('\n');
    line.trim(); // Remove espaços em branco ou caracteres de nova linha

    if (line.length() > 0) {
      // Envia a linha como um pacote UDP para toda a rede (broadcast)
      // O computador com o script Python irá capturar este pacote.
      Udp.beginPacket(WiFi.broadcastIP(), localUdpPort);
      Udp.write(line.c_str(), line.length());
      Udp.endPacket();
    }
  }
}
