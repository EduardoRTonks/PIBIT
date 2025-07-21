
# PIBIT - Monitoramento de Ambiente com ESP32 e Python

Este repositório contém os códigos-fonte para um projeto de monitoramento de ambiente utilizando um microcontrolador da família ESP (ESP32/ESP8266) e scripts em Python para coleta, visualização e processamento de dados.

## 📝 Descrição do Projeto

O sistema é composto por duas partes principais:

1.  **Firmware para o ESP:** Desenvolvido em C/C++ (Arduino), o firmware é responsável por ler os dados de sensores conectados ao microcontrolador e enviá-los via comunicação serial.
2.  **Scripts Python:** Um conjunto de scripts que recebem os dados do ESP, os processam, salvam em arquivos `.csv` e geram visualizações gráficas em tempo real.

-----

## 🛠️ Pré-requisitos

Para executar este projeto, você precisará do seguinte ambiente configurado:

  * **Sistema Operacional:** **Ubuntu** (ou uma distribuição Linux baseada em Debian).
  * **Python:** Versão 3.8 ou superior.
  * **Arduino IDE:** Versão 2.0 ou superior.
  * **Hardware:**
      * Um microcontrolador da família **ESP32** ou **ESP8266**.
      * Sensores compatíveis com o projeto (ex: DHT22, LDR, etc.).

-----

## 🐍 Configuração do Ambiente Python

Siga estes passos para configurar o ambiente virtual e instalar as bibliotecas Python necessárias.

### 1\. Crie e Ative um Ambiente Virtual

É uma boa prática usar um ambiente virtual para isolar as dependências do projeto. No terminal, dentro da pasta do projeto, execute:

```bash
# Instala o pacote para criar ambientes virtuais (se ainda não tiver)
sudo apt update
sudo apt install python3-venv -y

# Cria o ambiente virtual na pasta 'venv'
python3 -m venv venv

# Ativa o ambiente virtual
source venv/bin/activate
```

> 💡 **Dica:** Quando o ambiente está ativo, seu terminal mostrará `(venv)` no início da linha. Para desativar, basta digitar `deactivate`.

### 2\. Instale as Bibliotecas

Com o ambiente ativo, instale as bibliotecas listadas no arquivo `requirements.txt`.

```bash
# Instala todas as dependências de uma vez
pip install pyserial matplotlib pandas
```

As principais bibliotecas utilizadas são:

  * **pyserial:** Para comunicação com a porta serial do ESP.
  * **matplotlib:** Para a plotagem dos gráficos.
  * **pandas:** Para manipulação e armazenamento dos dados em formato `.csv`.

-----

## 🤖 Configuração do Ambiente Arduino (ESP)

Siga estes passos para configurar a Arduino IDE para programar seu ESP32/ESP8266.

### 1\. Instale a Arduino IDE

Se ainda não tiver, baixe e instale a **[Arduino IDE](https://www.arduino.cc/en/software)** para Linux.

### 2\. Configure a Placa (ESP32/ESP8266)

Você precisa adicionar o suporte para as placas ESP na IDE.

1.  Abra a Arduino IDE.
2.  Vá em **File \> Preferences** (Arquivo \> Preferências).
3.  No campo **"Additional boards manager URLs"** (URLs Adicionais de Gerenciadores de Placas), adicione a URL correspondente à sua placa:
      * **Para ESP32:**
        ```
        https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
        ```
      * **Para ESP8266:**
        ```
        http://arduino.esp8266.com/stable/package_esp8266com_index.json
        ```
4.  Clique em **OK**.
5.  Vá em **Tools \> Board \> Boards Manager** (Ferramentas \> Placa \> Gerenciador de Placas).
6.  Pesquise por **`esp32`** ou **`esp8266`**, selecione o pacote e clique em **Install**.

### 3\. Instale as Bibliotecas do Arduino

Os scripts `.ino` deste projeto requerem bibliotecas específicas. Instale-as usando o Gerenciador de Bibliotecas.

1.  Vá em **Tools \> Manage Libraries...** (Ferramentas \> Gerenciar Bibliotecas...).
2.  Pesquise e instale as seguintes bibliotecas (utilize os nomes exatos para facilitar a busca):
      * `DHT sensor library` por Adafruit
      * `Adafruit Unified Sensor` por Adafruit

### 4\. Faça o Upload do Código

1.  Conecte sua placa ESP ao computador.
2.  Abra o arquivo `.ino` desejado na Arduino IDE.
3.  Vá em **Tools \> Board** e selecione o modelo exato da sua placa (ex: `DOIT ESP32 DEVKIT V1` ou `NodeMCU 1.0 (ESP-12E Module)`).
4.  Vá em **Tools \> Port** e selecione a porta serial correta (ex: `/dev/ttyUSB0` ou `/dev/ttyS0`).
5.  Clique no botão **Upload** (seta para a direita) para gravar o firmware na placa.

-----

## 🚀 Como Executar o Projeto

1.  **Grave o firmware** no ESP seguindo os passos da seção anterior.
2.  **Ative o ambiente virtual** Python: `source venv/bin/activate`.
3.  **Execute o script Python principal** para começar a coletar e visualizar os dados:
    ```bash
    # Exemplo de comando para executar o script
    python3 seu_script_principal.py
    ```

Os dados serão salvos em arquivos `.csv` e um gráfico será exibido em tempo real.
