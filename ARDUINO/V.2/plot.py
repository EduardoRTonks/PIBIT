import serial
import time
import numpy as np
import pyvista as pv
import sys
import random
import csv
from datetime import datetime

# --- Configurações da Porta Serial ---
SERIAL_PORT = '/dev/ttyUSB0' # <--- Altere para a porta correta do seu Arduino!
BAUD_RATE = 115200   # <--- Deve ser o mesmo baud rate definido no Arduino!

# --- ATENÇÃO: MODO DE SIMULAÇÃO ---
# Se TRUE, o script não tentará conectar à serial e gerará dados aleatórios.
SIMULATION_MODE = False

# --- Configurações das Portas do MUX (Devem ser as mesmas do Arduino!) ---
MPU_PORT_X = 6 # Sensor que controlará o Roll (eixo X) do objeto
MPU_PORT_Y = 4 # Sensor que controlará o Pitch (eixo Y) do objeto
MPU_PORT_Z = 2 # Sensor que controlará o Yaw (eixo Z) do objeto

# --- Configurações de Salvamento CSV ---
SAVE_TO_CSV = True
CSV_FILENAME = f"dados_sensores_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"

# --- Buffer para armazenar os últimos dados de cada MPU ---
mpu_data = {
    MPU_PORT_X: {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    MPU_PORT_Y: {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    MPU_PORT_Z: {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
}

# --- Setup do CSV ---
csv_file = None
csv_writer = None
if SAVE_TO_CSV:
    try:
        csv_file = open(CSV_FILENAME, 'w', newline='', encoding='utf-8')
        csv_writer = csv.writer(csv_file)
        # Escreve o cabeçalho
        csv_writer.writerow(['timestamp', 'sensor_id', 'roll', 'pitch', 'yaw'])
        print(f"Salvando dados no arquivo: {CSV_FILENAME}")
    except IOError as e:
        print(f"ERRO: Não foi possível abrir o arquivo CSV para escrita: {e}")
        SAVE_TO_CSV = False


# --- Funções de Geometria e Rotação ---
def get_rotation_matrix(roll, pitch, yaw):
    roll_rad, pitch_rad, yaw_rad = np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)
    Rx = np.array([[1, 0, 0], [0, np.cos(roll_rad), -np.sin(roll_rad)], [0, np.sin(roll_rad), np.cos(roll_rad)]])
    Ry = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)], [0, 1, 0], [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
    Rz = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0], [np.sin(yaw_rad), np.cos(yaw_rad), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx # Usando @ para multiplicação de matrizes (Python 3.5+)

# --- Setup da Visualização com PyVista ---
plotter = pv.Plotter(window_size=[1024, 768])
plotter.set_background('white')

# Cria a geometria do drone
body = pv.Cube(center=(0, 0, 0), x_length=0.4, y_length=0.8, z_length=0.1)
arm_front = pv.Cylinder(center=(0, 0.6, 0), direction=(0, 1, 0), radius=0.05, height=0.8)
arm_back = pv.Cylinder(center=(0, -0.6, 0), direction=(0, 1, 0), radius=0.05, height=0.8)
arm_left = pv.Cylinder(center=(-0.4, 0, 0), direction=(1, 0, 0), radius=0.05, height=0.8)
arm_right = pv.Cylinder(center=(0.4, 0, 0), direction=(1, 0, 0), radius=0.05, height=0.8)
front_marker = pv.Cone(center=(0, 1.0, 0), direction=(0, 1, 0), height=0.2, radius=0.1)

# Agrupa todas as partes do drone em um único objeto
drone_mesh = body + arm_front + arm_back + arm_left + arm_right + front_marker

# Adiciona o drone ao plotter
drone_actor = plotter.add_mesh(drone_mesh, color='#333333', pbr=True, metallic=0.5, roughness=0.5, name='drone')

# Criar eixos manualmente com setas para máxima compatibilidade.
arrow_scale = 0.5
x_arrow = pv.Arrow(direction=(1, 0, 0), scale=arrow_scale)
y_arrow = pv.Arrow(direction=(0, 1, 0), scale=arrow_scale)
z_arrow = pv.Arrow(direction=(0, 0, 1), scale=arrow_scale)

# Adiciona cada seta como um ator de malha (mesh) separado.
plotter.add_mesh(x_arrow, color='red', name='drone_axis_x')
plotter.add_mesh(y_arrow, color='green', name='drone_axis_y')
plotter.add_mesh(z_arrow, color='blue', name='drone_axis_z')

# Adiciona eixos globais no canto, associando-os ao ator principal (drone).
plotter.add_orientation_widget(drone_actor)

# 'xz' para uma visão frontal.
plotter.camera_position = 'xz'
plotter.camera.zoom(1.5)

# --- Controle de estado e Conexão Serial ---
ser = None
if not SIMULATION_MODE:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        print(f"DEBUG: Conectado à porta serial {SERIAL_PORT}")
        ser.flushInput()
    except serial.SerialException as e:
        print(f"ERRO: Não foi possível conectar à porta serial {SERIAL_PORT}. Erro: {e}")
        sys.exit(1)
else:
    print("DEBUG: Modo de SIMULAÇÃO ATIVO. Gerando dados aleatórios.")


def update_scene():
    if SIMULATION_MODE:
        for port in mpu_data:
            mpu_data[port]['yaw'] = random.uniform(-180, 180)
    elif ser and ser.is_open:
        while ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                
                # --- DEBUG ADICIONADO ---
                # Imprime a linha bruta recebida para diagnóstico
                if line:
                    print(f"RAW SERIAL: '{line}'")

                # Tenta processar a linha como dados CSV
                parts = line.split(',')
                if len(parts) == 4:
                    sensor_id = int(parts[0])
                    if sensor_id in mpu_data:
                        roll = float(parts[1])
                        pitch = float(parts[2])
                        yaw = float(parts[3])
                        mpu_data[sensor_id]['roll'] = roll
                        mpu_data[sensor_id]['pitch'] = pitch
                        mpu_data[sensor_id]['yaw'] = yaw
                        
                        # --- MODIFICAÇÃO AQUI ---
                        # Imprime o status do objeto a cada linha de dados recebida
                        object_roll_now = mpu_data[MPU_PORT_X]['yaw']
                        object_pitch_now = mpu_data[MPU_PORT_Y]['yaw']
                        object_yaw_now = mpu_data[MPU_PORT_Z]['yaw']
                        print(f"Objeto -> Roll:{object_roll_now:.2f}, Pitch:{object_pitch_now:.2f}, Yaw:{object_yaw_now:.2f}")

                        # Salva no CSV se habilitado
                        if SAVE_TO_CSV and csv_writer:
                            timestamp = datetime.now().isoformat()
                            csv_writer.writerow([timestamp, sensor_id, roll, pitch, yaw])
                # Se não for uma linha de dados válida, pode ser uma mensagem de status
                elif "RECALIBRATING" in line:
                    print(f"STATUS: {line}")

            except Exception as e:
                print(f"ERRO no processamento da linha serial: '{line}' -> {e}")

    # Mapeamento dos eixos
    object_roll = mpu_data[MPU_PORT_X]['yaw']
    object_pitch = mpu_data[MPU_PORT_Y]['yaw']
    object_yaw = mpu_data[MPU_PORT_Z]['yaw']

    # Calcula a matriz de rotação
    R = get_rotation_matrix(object_roll, object_pitch, object_yaw)
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = R

    # Aplica a MESMA transformação a todos os atores (drone e seus eixos)
    plotter.actors['drone'].user_matrix = transform_matrix
    plotter.actors['drone_axis_x'].user_matrix = transform_matrix
    plotter.actors['drone_axis_y'].user_matrix = transform_matrix
    plotter.actors['drone_axis_z'].user_matrix = transform_matrix
    
    # Força a renderização
    plotter.render()

# Laço de atualização interativo
print("Iniciando visualização 3D. Feche a janela para sair.")
plotter.show(interactive_update=True)

# Laço principal que mantém a janela aberta e atualiza a cena.
# A forma mais compatível de verificar se a janela foi fechada.
while plotter.window_size[0] > 0 and plotter.window_size[1] > 0:
    update_scene()
    # Pequena pausa para não sobrecarregar a CPU
    time.sleep(0.01)

# --- Limpeza ao fechar ---
if not SIMULATION_MODE and ser and ser.is_open:
    ser.close()
    print("DEBUG: Porta serial fechada.")
if SAVE_TO_CSV and csv_file:
    csv_file.close()
    print(f"DEBUG: Arquivo CSV '{CSV_FILENAME}' fechado.")
