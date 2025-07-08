#include <Wire.h> // Biblioteca padrão do Arduino para comunicação I2C

// Endereço I2C padrão do TCA9548A
// Se você alterou os pinos A0, A1, A2 do seu multiplexador,
// mude este endereço para o que você configurou (ex: 0x71, 0x72, etc.)
#define TCAADDR 0x70

// --- Função Auxiliar para Selecionar a Porta do Multiplexador ---
// Esta função envia um comando I2C para o TCA9548A
// para que ele ative a conexão com a porta 'i' (0 a 7).
void tcaselect(uint8_t i) {
  if (i > 7) { // Garante que o número da porta esteja dentro do limite (0 a 7)
    return;
  }
  Wire.beginTransmission(TCAADDR); // Inicia a comunicação com o multiplexador
  Wire.write(1 << i);             // Envia o byte de comando para ativar a porta 'i'
                                  // Ex: 1 << 0 = 0b00000001 (Porta 0)
                                  //     1 << 3 = 0b00001000 (Porta 3)
  Wire.endTransmission();         // Finaliza a transmissão I2C
}

void setup() {
  // Inicia a comunicação serial para exibir os resultados
  Serial.begin(115200);
  while (!Serial) {
    ; // Espera a porta serial conectar (útil em algumas placas)
  }

  Serial.println("\n--- Scanner I2C para TCA9548A ---");
  Wire.begin(); // Inicia a comunicação I2C (Arduino como mestre)

  // --- Testar se o TCA9548A está presente no barramento principal ---
  Serial.print("Procurando o multiplexador TCA9548A (0x");
  Serial.print(TCAADDR, HEX);
  Serial.println(")...");

  Wire.beginTransmission(TCAADDR);
  byte error = Wire.endTransmission(); // Tenta se comunicar com o multiplexador

  if (error == 0) {
    Serial.println("TCA9548A ENCONTRADO! Iniciando scan das portas...");
    Serial.println("------------------------------------");

    // --- Loop para escanear cada uma das 8 portas do multiplexador ---
    for (uint8_t t = 0; t < 8; t++) {
      tcaselect(t); // Seleciona a porta 't' do multiplexador
      Serial.print("Escaneando Porta #");
      Serial.println(t);

      // --- Scanner I2C para a porta selecionada ---
      bool foundDeviceOnPort = false;
      for (uint8_t addr = 1; addr <= 127; addr++) { // Percorre todos os possíveis endereços I2C (1 a 127)
        if (addr == TCAADDR) { // Ignora o endereço do próprio multiplexador, se ele aparecer no scan interno (não deveria)
          continue;
        }

        Wire.beginTransmission(addr); // Inicia transmissão com o endereço atual
        if (Wire.endTransmission() == 0) { // Se endTransmission retorna 0, o dispositivo respondeu
          Serial.print("  Dispositivo I2C encontrado em 0x");
          if (addr < 0x10) { // Formatação para endereços de um dígito
            Serial.print("0");
          }
          Serial.println(addr, HEX); // Imprime o endereço em hexadecimal
          foundDeviceOnPort = true;
        }
      }
      if (!foundDeviceOnPort) {
        Serial.println("  Nenhum dispositivo encontrado nesta porta.");
      }
      Serial.println("------------------------------------");
    }
    Serial.println("\nScan de todas as portas do TCA9548A completo.");

  } else {
    // Se o multiplexador não for encontrado no endereço TCAADDR
    Serial.print("TCA9548A NAO ENCONTRADO no endereco 0x");
    Serial.print(TCAADDR, HEX);
    Serial.println(". Verifique a fiação e o endereço.");
    Serial.print("Erro: ");
    Serial.println(error);
  }
}

void loop() {
  // Nada a fazer no loop para um scanner
}