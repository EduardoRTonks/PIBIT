import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import sys

# --- Configurações da Porta Serial ---
SERIAL_PORT = '/dev/ttyUSB0' # <--- ALtere para a porta correta do seu Arduino!
BAUD_RATE = 115200   # <--- Deve ser o mesmo baud rate definido no Arduino!

# --- Configurações das Portas do MUX (Devem ser as mesmas do Arduino!) ---
MPU_PORT_1 = 5 # MPU na porta 5 do MUX
MPU_PORT_2 = 3 # MPU na porta 3 do MUX

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
    cube1_plot, = ax.plot([], [], [], 'r-', label=f'MPU Port {MPU_PORT_1}', linewidth=2)
    cube2_plot, = ax.plot([], [], [], 'b-', label=f'MPU Port {MPU_PORT_2}', linewidth=2)
    
    ax.legend()
    return cube1_plot, cube2_plot

# --- Buffer para armazenar os últimos dados de cada MPU ---
# O formato será: {ID_PORTA: {'roll': valor, 'pitch': valor, 'yaw': valor}}
mpu_data = {
    MPU_PORT_1: {'roll': 0, 'pitch': 0, 'yaw': 0},
    MPU_PORT_2: {'roll': 0, 'pitch': 0, 'yaw': 0}
}

# --- Controle de estado para ignorar mensagens de inicialização/calibração ---
receiving_data = False

# --- Abre a porta serial ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2) # Espera a conexão serial se estabelecer
    print(f"Conectado à porta serial {SERIAL_PORT}")
    ser.flushInput() # Limpa qualquer dado pendente na porta serial
except serial.SerialException:
    print(f"ERRO: Não foi possível conectar à porta serial {SERIAL_PORT}.")
    print("Por favor, verifique se a porta está correta e se o Arduino não está sendo usado por outro programa.")
    sys.exit(1)

# --- Função de atualização para a animação ---
def update_plot(frame):
    global cube1_plot, cube2_plot, receiving_data
    
    # Processar TODAS as linhas disponíveis no buffer serial antes de plotar
    # Isso garante que a plotagem use os dados mais recentes e descarte dados antigos
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line: continue # Ignora linhas vazias

            # Verificadores de estado para ignorar mensagens de texto
            if "DATA_START" in line:
                receiving_data = True
                print("Iniciando recebimento de dados do MPU.")
                continue # Pula para a próxima linha
            if "RECALIBRATING_START" in line:
                receiving_data = False
                print("Iniciando recalibracao... pausando atualizacao de dados.")
                continue
            if "RECALIBRATING_END" in line:
                print("Recalibracao finalizada.")
                # 'DATA_START' virá depois de RECALIBRATING_END no Arduino, então não ativa aqui.
                continue

            if not receiving_data: # Se não estamos no modo de dados, ignora a linha (é uma mensagem de texto)
                # print(f"Ignorando mensagem de inicializacao: {line}") # Pode descomentar para debug
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
                        # print(f"  Dados parseados para MPU {sensor_id}: R={roll:.2f}, P={pitch:.2f}, Y={yaw:.2f}") # Para debug
                    # else:
                        # print(f"  ID de sensor desconhecido: {sensor_id}") # Para debug
                # else:
                    # print(f"  Formato de linha inválido (não 4 partes): {line}") # Para debug
            except ValueError as ve:
                # print(f"  Erro de conversão de dados na linha '{line}': {ve}") # Para debug
                pass # Ignora linhas que não podem ser convertidas (ex: strings não-CSV)
            except IndexError as ie:
                # print(f"  Erro de índice ao parsear na linha '{line}': {ie}") # Para debug
                pass # Ignora erros de índice (ex: linha incompleta)

        except serial.SerialTimeoutException:
            pass # Timeout da leitura (normal se não houver dados constantemente)
        except Exception as e:
            print(f"Erro inesperado na leitura serial: {e}")
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
    if ser.is_open:
        ser.close()
        print("Porta serial fechada.")
fig.canvas.mpl_connect('close_event', on_close)