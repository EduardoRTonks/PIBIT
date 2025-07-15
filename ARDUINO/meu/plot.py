import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import sys
import random # Para o modo de simulação

# --- Configurações da Porta Serial ---
SERIAL_PORT = '/dev/ttyUSB0' # <--- ALtere para a porta correta do seu Arduino!
BAUD_RATE = 115200   # <--- Deve ser o mesmo baud rate definido no Arduino!

# --- ATENÇÃO: MODO DE SIMULAÇÃO ---
# Se TRUE, o script não tentará conectar à serial e gerará dados aleatórios.
# Use para testar a parte de plotagem sem o Arduino conectado.
SIMULATION_MODE = False

# --- Configurações das Portas do MUX (Devem ser as mesmas do Arduino!) ---
MPU_PORT_1 = 2 # MPU na porta 5 do MUX
MPU_PORT_2 = 4 # MPU na porta 3 do MUX

# --- Parâmetros de Plotagem ---
PLOT_LIMIT = 2 # Tamanho da caixa de visualização 3D
PLOT_INTERVAL_MS = 10 # Atraso em ms entre os frames da animação (10ms = 100 FPS max)

# --- Inicializa a figura e o eixo 3D ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Orientação dos MPU6050s em 3D")

cube1_plot = None
cube2_plot = None

# Função para criar um cubo simples (8 vértices)
def get_cube_vertices():
    # Define os vértices de um cubo unitário
    v = np.array([
        [-0.5, -0.5, -0.5], [ 0.5, -0.5, -0.5], [ 0.5,  0.5, -0.5], [-0.5,  0.5, -0.5],
        [-0.5, -0.5,  0.5], [ 0.5, -0.5,  0.5], [ 0.5,  0.5,  0.5], [-0.5,  0.5,  0.5]
    ])
    return v

# Matriz de rotação a partir de ângulos de Euler (Roll, Pitch, Yaw)
# ZYX Euler angles (intrinsic) -> Rz * Ry * Rx
def get_rotation_matrix(roll, pitch, yaw):
    # Converte para radianos
    roll_rad = np.deg2rad(roll)
    pitch_rad = np.deg2rad(pitch)
    yaw_rad = np.deg2rad(yaw)

    # Matriz de rotação em X (Roll)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll_rad), -np.sin(roll_rad)],
        [0, np.sin(roll_rad), np.cos(roll_rad)]
    ])

    # Matriz de rotação em Y (Pitch)
    Ry = np.array([
        [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
        [0, 1, 0],
        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])

    # Matriz de rotação em Z (Yaw)
    Rz = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])

    # Combina as rotações (ordem ZYX intrínseca)
    # A ordem pode precisar ser ajustada se a definição de Roll/Pitch/Yaw do MPU for diferente
    # np.dot é para multiplicação de matrizes
    # R = np.dot(Rz, np.dot(Ry, Rx)) # Ordem ZYX (Yaw, Pitch, Roll)
    # Experimente outras ordens se o movimento estiver incorreto, ex:
    # R = np.dot(Rx, np.dot(Ry, Rz)) # Ordem XYZ (Roll, Pitch, Yaw)
    # R = np.dot(Rz, np.dot(Rx, Ry)) # Ordem ZXY
    # Mantenha a original por enquanto e veja o que os testes dizem:
    R = np.dot(Rz, np.dot(Ry, Rx))
    return R

# --- Função de inicialização para a animação ---
def init_plot():
    ax.set_xlim([-PLOT_LIMIT, PLOT_LIMIT])
    ax.set_ylim([-PLOT_LIMIT, PLOT_LIMIT])
    ax.set_zlim([-PLOT_LIMIT, PLOT_LIMIT])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_aspect('auto')

    global cube1_plot, cube2_plot
    
    # Inicializa as linhas vazias. Matplotlib as preencherá na primeira atualização.
    # Usamos 'r-' e 'b-' para linhas. Para polígonos preenchidos, seria ax.add_collection3d(Poly3DCollection(...))
    cube1_plot, = ax.plot([], [], [], 'r-', label=f'MPU Port {MPU_PORT_1}', linewidth=2)
    cube2_plot, = ax.plot([], [], [], 'b-', label=f'MPU Port {MPU_PORT_2}', linewidth=2)
    
    ax.legend()
    return cube1_plot, cube2_plot

# --- Buffer para armazenar os últimos dados de cada MPU ---
# O formato será: {ID_PORTA: {'roll': valor, 'pitch': valor, 'yaw': valor}}
mpu_data = {
    MPU_PORT_1: {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    MPU_PORT_2: {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
}

# --- Controle de estado para ignorar mensagens de inicialização/calibração ---
receiving_data = False

# --- Abre a porta serial (se não estiver em modo de simulação) ---
ser = None
if not SIMULATION_MODE:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2) # Espera a conexão serial se estabelecer
        print(f"DEBUG: Conectado à porta serial {SERIAL_PORT}")
        ser.flushInput() # Limpa qualquer dado pendente na porta serial
    except serial.SerialException as e:
        print(f"ERRO: Não foi possível conectar à porta serial {SERIAL_PORT}. Erro: {e}")
        print("Por favor, verifique se a porta está correta e se o Arduino não está sendo usado por outro programa.")
        print("Tente definir SIMULATION_MODE = True para testar a plotagem sem o Arduino.")
        sys.exit(1)
else:
    print("DEBUG: Modo de SIMULAÇÃO ATIVO. Gerando dados aleatórios.")

# Variável para controlar a taxa de print (para não floodar o console)
last_print_time = time.time()
print_interval = 0.5 # Imprime a cada 0.5 segundos

# --- Função de atualização para a animação ---
def update_plot(frame):
    global cube1_plot, cube2_plot, receiving_data, last_print_time
    
    if SIMULATION_MODE:
        # Gera dados aleatórios para simulação
        mpu_data[MPU_PORT_1]['roll'] = random.uniform(-90, 90)
        mpu_data[MPU_PORT_1]['pitch'] = random.uniform(-90, 90)
        mpu_data[MPU_PORT_1]['yaw'] = random.uniform(-180, 180)

        mpu_data[MPU_PORT_2]['roll'] = random.uniform(-90, 90)
        mpu_data[MPU_PORT_2]['pitch'] = random.uniform(-90, 90)
        mpu_data[MPU_PORT_2]['yaw'] = random.uniform(-180, 180)

        current_time = time.time()
        if current_time - last_print_time > print_interval:
            print(f"SIMULAÇÃO: MPU {MPU_PORT_1} -> R:{mpu_data[MPU_PORT_1]['roll']:.2f}, P:{mpu_data[MPU_PORT_1]['pitch']:.2f}, Y:{mpu_data[MPU_PORT_1]['yaw']:.2f}")
            print(f"SIMULAÇÃO: MPU {MPU_PORT_2} -> R:{mpu_data[MPU_PORT_2]['roll']:.2f}, P:{mpu_data[MPU_PORT_2]['pitch']:.2f}, Y:{mpu_data[MPU_PORT_2]['yaw']:.2f}")
            last_print_time = current_time

    else: # Modo Real: Lendo da Serial
        # Processar TODAS as linhas disponíveis no buffer serial antes de plotar
        # Isso garante que a plotagem use os dados mais recentes e descarte dados antigos
        while ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if not line: continue # Ignora linhas vazias

                # DEBUG: Imprime a linha bruta
                # Descomente a linha abaixo para ver TUDO que chega na serial
                # print(f"DEBUG: Linha serial bruta: '{line}'") 

                # Verificadores de estado para ignorar mensagens de texto
                if "DATA_START" in line:
                    receiving_data = True
                    print("DEBUG: Sinal 'DATA_START' recebido. Iniciando recebimento de dados do MPU.")
                    continue # Pula para a próxima linha
                if "RECALIBRATING_START" in line:
                    receiving_data = False
                    print("DEBUG: Sinal 'RECALIBRATING_START' recebido. Pausando atualização de dados.")
                    continue
                if "RECALIBRATING_END" in line:
                    print("DEBUG: Sinal 'RECALIBRATING_END' recebido. Recalibracao finalizada.")
                    # 'DATA_START' virá depois de RECALIBRATING_END no Arduino, então não ativa aqui.
                    continue

                if not receiving_data: # Se não estamos no modo de dados, ignora a linha (é uma mensagem de texto)
                    # print(f"DEBUG: Ignorando mensagem de inicializacao/calibracao: {line}") # Pode descomentar para debug detalhado
                    continue

                # Tenta parsear a linha como CSV (apenas se receiving_data for True)
                try:
                    parts = line.split(',')
                    if len(parts) == 4:
                        sensor_id = int(parts[0])
                        roll = float(parts[1])
                        pitch = float(parts[2])
                        yaw = float(parts[3])

                        if sensor_id in mpu_data:
                            mpu_data[sensor_id]['roll'] = roll
                            mpu_data[sensor_id]['pitch'] = pitch
                            mpu_data[sensor_id]['yaw'] = yaw
                            
                            # DEBUG: Imprime os dados parseados (com limite de taxa)
                            current_time = time.time()
                            if current_time - last_print_time > print_interval:
                                print(f"DEBUG: MPU {sensor_id} -> R:{roll:.2f}, P:{pitch:.2f}, Y:{yaw:.2f}")
                                last_print_time = current_time
                        else:
                            print(f"ALERTA: ID de sensor desconhecido: {sensor_id} na linha: '{line}'")
                    else:
                        print(f"ALERTA: Formato de linha inválido (esperado 4 partes, obtido {len(parts)}): '{line}'")
                except ValueError as ve:
                    print(f"ERRO DE PARSING: Erro de conversão de dados na linha '{line}': {ve}")
                except IndexError as ie:
                    print(f"ERRO DE PARSING: Erro de índice ao parsear (linha incompleta?) na linha '{line}': {ie}")
                except Exception as e:
                    print(f"ERRO INESPERADO no processamento de linha serial: {e} na linha '{line}'")

            except serial.SerialTimeoutException:
                pass # Timeout da leitura (normal se não houver dados constantemente)
            except Exception as e:
                print(f"ERRO GERAL na leitura serial: {e}")
                return [] # Retorna vazio para evitar erro na animação

    # --- Atualiza a posição do Cubo 1 (MPU_PORT_1) ---
    roll1 = mpu_data[MPU_PORT_1]['roll']
    pitch1 = mpu_data[MPU_PORT_1]['pitch']
    yaw1 = mpu_data[MPU_PORT_1]['yaw']

    R1 = get_rotation_matrix(roll1, pitch1, yaw1)
    # As coordenadas do cubo unitário são [-0.5, 0.5]
    # Aplicar Rotação: np.dot(vertices, R.T) para girar os pontos
    # Para visualizar dois objetos distintos, vamos transladá-los após a rotação.
    rotated_cube_verts1 = np.dot(get_cube_vertices(), R1.T) + np.array([0.7, 0, 0]) # Desloca MPU1 em X positivo

    # --- Atualiza a posição do Cubo 2 (MPU_PORT_2) ---
    roll2 = mpu_data[MPU_PORT_2]['roll']
    pitch2 = mpu_data[MPU_PORT_2]['pitch']
    yaw2 = mpu_data[MPU_PORT_2]['yaw']

    R2 = get_rotation_matrix(roll2, pitch2, yaw2)
    rotated_cube_verts2 = np.dot(get_cube_vertices(), R2.T) + np.array([-0.7, 0, 0]) # Desloca MPU2 em X negativo

    # --- Re-desenha as arestas dos cubos ---
    edges = [
        [0,1],[1,2],[2,3],[3,0], # Base inferior
        [4,5],[5,6],[6,7],[7,4], # Base superior
        [0,4],[1,5],[2,6],[3,7]  # Laterais
    ]
    
    # Segmentos para o cubo 1
    lines1_x, lines1_y, lines1_z = [], [], []
    for edge in edges:
        v0, v1 = rotated_cube_verts1[edge[0]], rotated_cube_verts1[edge[1]]
        lines1_x.extend([v0[0], v1[0], np.nan]) # np.nan cria uma quebra de linha
        lines1_y.extend([v0[1], v1[1], np.nan])
        lines1_z.extend([v0[2], v1[2], np.nan])
    cube1_plot.set_data_3d(lines1_x, lines1_y, lines1_z)

    # Segmentos para o cubo 2
    lines2_x, lines2_y, lines2_z = [], [], []
    for edge in edges:
        v0, v1 = rotated_cube_verts2[edge[0]], rotated_cube_verts2[edge[1]]
        lines2_x.extend([v0[0], v1[0], np.nan])
        lines2_y.extend([v0[1], v1[1], np.nan])
        lines2_z.extend([v0[2], v1[2], np.nan])
    cube2_plot.set_data_3d(lines2_x, lines2_y, lines2_z)
    
    return cube1_plot, cube2_plot # Retorna as coleções de plot atualizadas

# --- Cria e roda a animação ---
ani = FuncAnimation(fig, update_plot, init_func=init_plot, interval=PLOT_INTERVAL_MS, blit=True, cache_frame_data=False)

plt.show()

# Fecha a porta serial quando a janela de plotagem é fechada
def on_close(event):
    if not SIMULATION_MODE and ser and ser.is_open:
        ser.close()
        print("DEBUG: Porta serial fechada.")
fig.canvas.mpl_connect('close_event', on_close)