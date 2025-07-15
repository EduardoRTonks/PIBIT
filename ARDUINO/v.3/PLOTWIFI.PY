import socket
import time
import numpy as np
import pyvista as pv
import sys
import random
import csv
from datetime import datetime

# --- Configurações de Rede UDP ---
UDP_IP = "0.0.0.0"  # Ouve em todas as interfaces de rede disponíveis
UDP_PORT = 4210     # Porta que o ESP está a usar para transmitir

# --- ATENÇÃO: MODO DE SIMULAÇÃO ---
SIMULATION_MODE = False

# --- Configurações das Portas do MUX (Devem ser as mesmas do Arduino!) ---
MPU_PORT_X = 6 # Sensor que controlará o Roll (eixo X) do objeto
MPU_PORT_Y = 4 # Sensor que controlará o Pitch (eixo Y) do objeto
MPU_PORT_Z = 2 # Sensor que controlará o Yaw (eixo Z) do objeto

# --- Configurações de Salvamento CSV ---
SAVE_TO_CSV = True
CSV_FILENAME = f"dados_sensores_wifi_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"

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
        csv_writer.writerow(['timestamp', 'sensor_id', 'roll', 'pitch', 'yaw'])
        print(f"Salvando dados no arquivo: {CSV_FILENAME}")
    except IOError as e:
        print(f"ERRO: Não foi possível abrir o arquivo CSV para escrita: {e}")
        SAVE_TO_CSV = False

# --- Setup do Socket UDP ---
sock = None
if not SIMULATION_MODE:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        sock.settimeout(0.01) # Define um timeout pequeno para não bloquear o loop
        print(f"Ouvindo pacotes UDP na porta {UDP_PORT}...")
    except OSError as e:
        print(f"ERRO ao ligar o socket: {e}")
        print("Verifique se outro programa já não está a usar a porta 4210.")
        sys.exit(1)


# --- Funções de Geometria e Rotação ---
def get_rotation_matrix(roll, pitch, yaw):
    roll_rad, pitch_rad, yaw_rad = np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)
    Rx = np.array([[1, 0, 0], [0, np.cos(roll_rad), -np.sin(roll_rad)], [0, np.sin(roll_rad), np.cos(roll_rad)]])
    Ry = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)], [0, 1, 0], [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
    Rz = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0], [np.sin(yaw_rad), np.cos(yaw_rad), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

# --- Setup da Visualização com PyVista ---
plotter = pv.Plotter(window_size=[1024, 768])
plotter.set_background('white')
body = pv.Cube(center=(0, 0, 0), x_length=0.4, y_length=0.8, z_length=0.1)
arm_front = pv.Cylinder(center=(0, 0.6, 0), direction=(0, 1, 0), radius=0.05, height=0.8)
arm_back = pv.Cylinder(center=(0, -0.6, 0), direction=(0, 1, 0), radius=0.05, height=0.8)
arm_left = pv.Cylinder(center=(-0.4, 0, 0), direction=(1, 0, 0), radius=0.05, height=0.8)
arm_right = pv.Cylinder(center=(0.4, 0, 0), direction=(1, 0, 0), radius=0.05, height=0.8)
front_marker = pv.Cone(center=(0, 1.0, 0), direction=(0, 1, 0), height=0.2, radius=0.1)
drone_mesh = body + arm_front + arm_back + arm_left + arm_right + front_marker
drone_actor = plotter.add_mesh(drone_mesh, color='#333333', pbr=True, metallic=0.5, roughness=0.5, name='drone')
arrow_scale = 0.5
x_arrow = pv.Arrow(direction=(1, 0, 0), scale=arrow_scale)
y_arrow = pv.Arrow(direction=(0, 1, 0), scale=arrow_scale)
z_arrow = pv.Arrow(direction=(0, 0, 1), scale=arrow_scale)
plotter.add_mesh(x_arrow, color='red', name='drone_axis_x')
plotter.add_mesh(y_arrow, color='green', name='drone_axis_y')
plotter.add_mesh(z_arrow, color='blue', name='drone_axis_z')
plotter.add_orientation_widget(drone_actor)
plotter.camera_position = 'xz'
plotter.camera.zoom(1.5)


def update_scene():
    if SIMULATION_MODE:
        for port in mpu_data:
            mpu_data[port]['yaw'] = random.uniform(-180, 180)
    elif sock:
        try:
            while True: # Processa todos os pacotes no buffer
                data, addr = sock.recvfrom(1024)
                line = data.decode('utf-8').strip()
                
                parts = line.split(',')
                if len(parts) == 4:
                    sensor_id = int(parts[0])
                    if sensor_id in mpu_data:
                        mpu_data[sensor_id]['roll'] = float(parts[1])
                        mpu_data[sensor_id]['pitch'] = float(parts[2])
                        mpu_data[sensor_id]['yaw'] = float(parts[3])
                        if SAVE_TO_CSV and csv_writer:
                            timestamp = datetime.now().isoformat()
                            csv_writer.writerow([timestamp, sensor_id, float(parts[1]), float(parts[2]), float(parts[3])])
        except socket.timeout:
            pass # Normal, significa que não há mais pacotes no momento
        except BlockingIOError:
            pass # Também normal em sistemas não-bloqueantes
        except Exception as e:
            print(f"ERRO no processamento do pacote UDP: {e}")

    # Mapeamento e rotação do objeto (sem alterações)
    object_roll = mpu_data[MPU_PORT_X]['yaw']
    object_pitch = mpu_data[MPU_PORT_Y]['yaw']
    object_yaw = mpu_data[MPU_PORT_Z]['yaw']
    R = get_rotation_matrix(object_roll, object_pitch, object_yaw)
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = R
    plotter.actors['drone'].user_matrix = transform_matrix
    plotter.actors['drone_axis_x'].user_matrix = transform_matrix
    plotter.actors['drone_axis_y'].user_matrix = transform_matrix
    plotter.actors['drone_axis_z'].user_matrix = transform_matrix
    plotter.render()

# Laço de atualização
print("Iniciando visualização 3D. Feche a janela para sair.")
plotter.show(interactive_update=True)
while plotter.window_size[0] > 0 and plotter.window_size[1] > 0:
    update_scene()
    time.sleep(0.01)

# Limpeza
if sock:
    sock.close()
    print("Socket UDP fechado.")
if SAVE_TO_CSV and csv_file:
    csv_file.close()
    print(f"Arquivo CSV '{CSV_FILENAME}' fechado.")
