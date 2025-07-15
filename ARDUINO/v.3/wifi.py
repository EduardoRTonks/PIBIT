import socket

# --- Configurações de Rede ---
# Deve ser igual ao que está no script do ESP8266 e no plot.py
UDP_IP = "0.0.0.0"  # Ouve em todas as interfaces de rede
UDP_PORT = 4210     # Porta para a qual o ESP está a transmitir

# --- Cria e configura o socket ---
# AF_INET para IPv4, SOCK_DGRAM para UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Liga o socket ao endereço e porta para poder receber dados
sock.bind((UDP_IP, UDP_PORT))

print(f"--- Ouvinte de Pacotes Wi-Fi (UDP) ---")
print(f"A aguardar pacotes na porta {UDP_PORT}...")
print("Pressione Ctrl+C para sair.")

try:
    # Laço infinito para receber dados continuamente
    while True:
        # Espera para receber dados (o buffer de 1024 bytes é mais que suficiente)
        data, addr = sock.recvfrom(1024) 
        
        # Decodifica os bytes recebidos para uma string e imprime
        line = data.decode('utf-8').strip()
        print(f"Pacote Wi-Fi recebido de {addr}: '{line}'")

except KeyboardInterrupt:
    print("\nPrograma terminado pelo utilizador.")
finally:
    # Fecha o socket de forma limpa ao sair
    sock.close()
    print("Socket fechado.")

