#!/usr/bin/env python3
"""
PSO Data Acquisition - Python Real-Time Plotter
================================================================
Sistema de aquisição de dados em tempo real com visualização
gráfica online para o sistema PSO (Propulsion System Optimizer)

Autor: Baseado no código MATLAB original
Data: Dezembro 2025
"""

import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
from collections import deque
import time
import sys
import platform
import struct
from datetime import datetime
import argparse

# ============================================================================
# CONFIGURAÇÕES GLOBAIS
# ============================================================================

class Config:
    """Configurações do sistema"""
    # Porta serial baseada no sistema operacional
    if platform.system() == 'Windows':
        PORT = 'COM3'
    elif platform.system() == 'Linux':
        PORT = '/dev/ttyACM0'
    elif platform.system() == 'Darwin':  # macOS
        PORT = '/dev/cu.usbmodem'
    else:
        PORT = '/dev/ttyACM0'
    
    BAUDRATE = 115200
    TIMEOUT = 1.0
    
    # Protocolo
    STX = 0xFE
    PACKET_LENGTH = 21
    MSG_ID_PSO_DATA_LEN = 14
    
    # Calibração
    CALIBRATION_MODE = False
    
    # Timeout de inatividade
    INACTIVITY_TIMEOUT = 5.0
    
    # Buffer de visualização (quantos pontos mostrar)
    PLOT_WINDOW = 500  # Últimos 500 pontos
    
    # Taxa de atualização do gráfico (ms)
    PLOT_UPDATE_INTERVAL = 100  # 100ms = 10 FPS


# ============================================================================
# CLASSE DE DADOS
# ============================================================================

class PSO_Data:
    """Armazena dados coletados do PSO"""
    
    def __init__(self, max_samples=10000):
        self.max_samples = max_samples
        self.idx = 0
        
        # Buffers de dados (usando deque para eficiência)
        self.time = deque(maxlen=max_samples)
        self.index = deque(maxlen=max_samples)
        self.rpm = deque(maxlen=max_samples)
        self.throttle = deque(maxlen=max_samples)
        self.accel_x = deque(maxlen=max_samples)
        self.accel_y = deque(maxlen=max_samples)
        self.accel_z = deque(maxlen=max_samples)
        self.current = deque(maxlen=max_samples)
        self.voltage = deque(maxlen=max_samples)
        self.thrust = deque(maxlen=max_samples)
        self.power = deque(maxlen=max_samples)
        
        # Estatísticas
        self.packet_count = 0
        self.error_count = 0
        self.checksum_errors = 0
        self.start_time = time.time()
    
    def add_sample(self, sample_data):
        """Adiciona uma amostra aos buffers"""
        elapsed_time = time.time() - self.start_time
        
        self.time.append(elapsed_time)
        self.index.append(sample_data['index'])
        self.rpm.append(sample_data['rpm'])
        self.throttle.append(sample_data['throttle'])
        self.accel_x.append(sample_data['accel_x'])
        self.accel_y.append(sample_data['accel_y'])
        self.accel_z.append(sample_data['accel_z'])
        self.current.append(sample_data['current'])
        self.voltage.append(sample_data['voltage'])
        self.thrust.append(sample_data['thrust'])
        self.power.append(sample_data['power'])
        
        self.idx += 1
        self.packet_count += 1
    
    def get_stats(self):
        """Retorna estatísticas dos dados"""
        if self.idx == 0:
            return None
        
        return {
            'rpm': {
                'min': min(self.rpm) if self.rpm else 0,
                'max': max(self.rpm) if self.rpm else 0,
                'mean': np.mean(self.rpm) if self.rpm else 0
            },
            'current': {
                'min': min(self.current) if self.current else 0,
                'max': max(self.current) if self.current else 0,
                'mean': np.mean(self.current) if self.current else 0
            },
            'voltage': {
                'min': min(self.voltage) if self.voltage else 0,
                'max': max(self.voltage) if self.voltage else 0,
                'mean': np.mean(self.voltage) if self.voltage else 0
            },
            'thrust': {
                'min': min(self.thrust) if self.thrust else 0,
                'max': max(self.thrust) if self.thrust else 0,
                'mean': np.mean(self.thrust) if self.thrust else 0
            },
            'power': {
                'min': min(self.power) if self.power else 0,
                'max': max(self.power) if self.power else 0,
                'mean': np.mean(self.power) if self.power else 0
            }
        }


# ============================================================================
# FUNÇÕES DE CHECKSUM (X.25 CRC-16)
# ============================================================================

def crc_init():
    """Inicializa CRC"""
    return 0xFFFF

def accumulate_checksum(data, crc):
    """Acumula byte no CRC (compatível com implementação C)"""
    data = int(data) & 0xFF
    crc = int(crc) & 0xFFFF
    
    # tmp = data ^ (crc & 0xff)
    tmp = data ^ (crc & 0xFF)
    
    # tmp ^= (tmp << 4) & 0xff
    tmp = tmp ^ ((tmp << 4) & 0xFF)
    
    # crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    
    return crc & 0xFFFF

def calculate_checksum(packet_bytes):
    """Calcula checksum do pacote"""
    crc = crc_init()
    
    # Processar bytes 1-18 (índices 1-18 em Python)
    for i in range(1, 19):
        crc = accumulate_checksum(packet_bytes[i], crc)
    
    return crc


# ============================================================================
# PARSER DE PACOTES
# ============================================================================

def parse_packet(packet_data):
    """Parse de um pacote PSO válido"""
    try:
        # Index (bytes 2-3, big-endian)
        pkt_index = (packet_data[2] << 8) | packet_data[3]
        
        # Acelerações (signed int16, big-endian)
        accel_x = struct.unpack('>h', bytes([packet_data[4], packet_data[5]]))[0]
        accel_y = struct.unpack('>h', bytes([packet_data[6], packet_data[7]]))[0]
        accel_z = struct.unpack('>h', bytes([packet_data[8], packet_data[9]]))[0]
        
        # RPM (unsigned int16, big-endian)
        rpm = (packet_data[10] << 8) | packet_data[11]
        
        # Current (unsigned, em mA)
        current_raw = (packet_data[12] << 8) | packet_data[13]
        
        # Voltage (unsigned, em mV)
        voltage_raw = (packet_data[14] << 8) | packet_data[15]
        
        # Thrust (signed int16, big-endian)
        thrust_raw = struct.unpack('>h', bytes([packet_data[16], packet_data[17]]))[0]
        
        # Throttle
        throttle = packet_data[18]
        
        # Converter para unidades reais
        current = current_raw / 1000.0  # mA -> A
        voltage = voltage_raw / 1000.0  # mV -> V
        thrust = thrust_raw  # Manter em unidade bruta
        power = voltage * current
        
        return {
            'index': pkt_index,
            'accel_x': accel_x,
            'accel_y': accel_y,
            'accel_z': accel_z,
            'rpm': rpm,
            'current': current,
            'voltage': voltage,
            'thrust': thrust,
            'throttle': throttle,
            'power': power
        }
    
    except Exception as e:
        print(f"Erro ao fazer parse do pacote: {e}")
        return None


# ============================================================================
# PLOTAGEM EM TEMPO REAL
# ============================================================================

class RealTimePlotter:
    """Plotter de dados em tempo real"""
    
    def __init__(self, data_obj, window_size=500):
        self.data = data_obj
        self.window_size = window_size
        
        # Configurar matplotlib para modo interativo
        plt.ion()
        
        # Criar figura
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.canvas.manager.set_window_title('PSO Data Acquisition - Real-Time')
        
        # Criar grid de subplots
        gs = GridSpec(4, 2, figure=self.fig, hspace=0.3, wspace=0.3)
        
        # Subplots
        self.ax_rpm = self.fig.add_subplot(gs[0, 0])
        self.ax_throttle = self.fig.add_subplot(gs[0, 1])
        self.ax_current = self.fig.add_subplot(gs[1, 0])
        self.ax_voltage = self.fig.add_subplot(gs[1, 1])
        self.ax_thrust = self.fig.add_subplot(gs[2, 0])
        self.ax_power = self.fig.add_subplot(gs[2, 1])
        self.ax_accel = self.fig.add_subplot(gs[3, :])
        
        # Inicializar linhas de plot
        self.line_rpm, = self.ax_rpm.plot([], [], 'b-', linewidth=1.5)
        self.line_throttle, = self.ax_throttle.plot([], [], 'g-', linewidth=1.5)
        self.line_current, = self.ax_current.plot([], [], 'r-', linewidth=1.5)
        self.line_voltage, = self.ax_voltage.plot([], [], color='orange', linewidth=1.5)
        self.line_thrust, = self.ax_thrust.plot([], [], 'm-', linewidth=1.5)
        self.line_power, = self.ax_power.plot([], [], 'c-', linewidth=1.5)
        
        # Acelerações (3 linhas)
        self.line_accel_x, = self.ax_accel.plot([], [], 'r-', linewidth=1.2, label='Accel X')
        self.line_accel_y, = self.ax_accel.plot([], [], 'g-', linewidth=1.2, label='Accel Y')
        self.line_accel_z, = self.ax_accel.plot([], [], 'b-', linewidth=1.2, label='Accel Z')
        
        # Configurar eixos
        self._setup_axes()
        
        # Título principal
        self.fig.suptitle('PSO Data Acquisition - Análise em Tempo Real', 
                         fontsize=14, fontweight='bold')
        
        # Mostrar figura
        plt.show(block=False)
        plt.pause(0.001)
    
    def _setup_axes(self):
        """Configura os eixos dos gráficos"""
        # RPM
        self.ax_rpm.set_ylabel('RPM')
        self.ax_rpm.set_title('Rotação do Motor')
        self.ax_rpm.grid(True, alpha=0.3)
        
        # Throttle
        self.ax_throttle.set_ylabel('Throttle (%)')
        self.ax_throttle.set_title('Posição do Acelerador')
        self.ax_throttle.set_ylim([0, 105])
        self.ax_throttle.grid(True, alpha=0.3)
        
        # Current
        self.ax_current.set_ylabel('Corrente (A)')
        self.ax_current.set_title('Corrente Elétrica')
        self.ax_current.grid(True, alpha=0.3)
        
        # Voltage
        self.ax_voltage.set_ylabel('Tensão (V)')
        self.ax_voltage.set_title('Tensão de Alimentação')
        self.ax_voltage.grid(True, alpha=0.3)
        
        # Thrust
        self.ax_thrust.set_ylabel('Empuxo (g.f)')
        self.ax_thrust.set_title('Força de Empuxo')
        self.ax_thrust.grid(True, alpha=0.3)
        
        # Power
        self.ax_power.set_ylabel('Potência (W)')
        self.ax_power.set_title('Potência Elétrica')
        self.ax_power.grid(True, alpha=0.3)
        
        # Acelerações
        self.ax_accel.set_xlabel('Tempo (s)')
        self.ax_accel.set_ylabel('Aceleração')
        self.ax_accel.set_title('Aceleração nos 3 Eixos')
        self.ax_accel.legend(loc='upper right')
        self.ax_accel.grid(True, alpha=0.3)
    
    def update(self):
        """Atualiza os gráficos com novos dados"""
        if self.data.idx == 0:
            return
        
        # Pegar últimos N pontos
        n = min(self.window_size, len(self.data.time))
        
        time_data = list(self.data.time)[-n:]
        rpm_data = list(self.data.rpm)[-n:]
        throttle_data = list(self.data.throttle)[-n:]
        current_data = list(self.data.current)[-n:]
        voltage_data = list(self.data.voltage)[-n:]
        thrust_data = list(self.data.thrust)[-n:]
        power_data = list(self.data.power)[-n:]
        accel_x_data = list(self.data.accel_x)[-n:]
        accel_y_data = list(self.data.accel_y)[-n:]
        accel_z_data = list(self.data.accel_z)[-n:]
        
        # Atualizar dados das linhas
        self.line_rpm.set_data(time_data, rpm_data)
        self.line_throttle.set_data(time_data, throttle_data)
        self.line_current.set_data(time_data, current_data)
        self.line_voltage.set_data(time_data, voltage_data)
        self.line_thrust.set_data(time_data, thrust_data)
        self.line_power.set_data(time_data, power_data)
        self.line_accel_x.set_data(time_data, accel_x_data)
        self.line_accel_y.set_data(time_data, accel_y_data)
        self.line_accel_z.set_data(time_data, accel_z_data)
        
        # Ajustar limites dos eixos X
        if len(time_data) > 0:
            x_min, x_max = min(time_data), max(time_data)
            x_margin = (x_max - x_min) * 0.05 if x_max > x_min else 1
            
            for ax in [self.ax_rpm, self.ax_throttle, self.ax_current, 
                      self.ax_voltage, self.ax_thrust, self.ax_power, self.ax_accel]:
                ax.set_xlim([x_min - x_margin, x_max + x_margin])
        
        # Ajustar limites dos eixos Y (auto-escala)
        self._autoscale_y(self.ax_rpm, rpm_data)
        # Throttle já tem limite fixo
        self._autoscale_y(self.ax_current, current_data)
        self._autoscale_y(self.ax_voltage, voltage_data)
        self._autoscale_y(self.ax_thrust, thrust_data)
        self._autoscale_y(self.ax_power, power_data)
        
        # Acelerações (combinar todos)
        all_accel = accel_x_data + accel_y_data + accel_z_data
        self._autoscale_y(self.ax_accel, all_accel)
        
        # Redesenhar
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
    
    def _autoscale_y(self, ax, data):
        """Auto-escala eixo Y com margem"""
        if len(data) > 0:
            y_min, y_max = min(data), max(data)
            y_range = y_max - y_min
            
            if y_range < 0.01:  # Evitar divisão por zero
                y_margin = 1
            else:
                y_margin = y_range * 0.1
            
            ax.set_ylim([y_min - y_margin, y_max + y_margin])


# ============================================================================
# FUNÇÃO PRINCIPAL DE AQUISIÇÃO
# ============================================================================

def acquire_data(port=None, baudrate=115200, plot_window=500):
    """
    Função principal de aquisição de dados
    
    Args:
        port: Porta serial (None para usar padrão do sistema)
        baudrate: Taxa de transmissão
        plot_window: Tamanho da janela de plotagem
    """
    
    # Configurar porta
    if port is None:
        port = Config.PORT
    
    print("=" * 70)
    print("PSO Data Acquisition - Python Real-Time Plotter")
    print("=" * 70)
    print(f"\nConectando a {port} @ {baudrate} bps...")
    
    # Abrir porta serial
    try:
        ser = serial.Serial(port, baudrate, timeout=Config.TIMEOUT)
        print(f"✓ Conectado com sucesso!")
    except Exception as e:
        print(f"✗ Erro ao conectar: {e}")
        return
    
    # Criar objeto de dados
    data = PSO_Data()
    
    # Criar plotter
    print("\n✓ Inicializando gráficos em tempo real...")
    plotter = RealTimePlotter(data, window_size=plot_window)
    
    # Buffers
    sync_buffer = bytearray()
    last_packet_time = time.time()
    last_plot_update = time.time()
    
    print("\n" + "=" * 70)
    print("=== Iniciando coleta de dados ===")
    print("Pressione Ctrl+C para parar")
    print(f"Timeout automático: {Config.INACTIVITY_TIMEOUT} segundos sem dados")
    print("=" * 70)
    print(f"{'Pacotes':<10} {'RPM':<8} {'Thr%':<6} {'Corrente':<10} "
          f"{'Tensão':<10} {'Empuxo':<10} {'Taxa':<8}")
    print("-" * 70)
    
    try:
        while True:
            # Verificar timeout
            if time.time() - last_packet_time > Config.INACTIVITY_TIMEOUT:
                print(f"\n\n=== Timeout: {Config.INACTIVITY_TIMEOUT:.1f} "
                      f"segundos sem dados ===")
                break
            
            # Ler dados disponíveis
            if ser.in_waiting > 0:
                new_bytes = ser.read(ser.in_waiting)
                sync_buffer.extend(new_bytes)
            else:
                time.sleep(0.001)
                
                # Atualizar gráfico periodicamente
                if time.time() - last_plot_update > Config.PLOT_UPDATE_INTERVAL / 1000.0:
                    plotter.update()
                    last_plot_update = time.time()
                
                continue
            
            # Processar buffer
            while len(sync_buffer) >= Config.PACKET_LENGTH:
                # Procurar STX
                try:
                    stx_pos = sync_buffer.index(Config.STX)
                except ValueError:
                    sync_buffer.clear()
                    break
                
                # Alinhar ao STX
                if stx_pos > 0:
                    sync_buffer = sync_buffer[stx_pos:]
                
                # Verificar se temos pacote completo
                if len(sync_buffer) < Config.PACKET_LENGTH:
                    break
                
                # Extrair pacote
                packet_data = sync_buffer[:Config.PACKET_LENGTH]
                
                # Validação 1: Verificar byte de length
                if packet_data[1] != Config.MSG_ID_PSO_DATA_LEN:
                    data.error_count += 1
                    sync_buffer = sync_buffer[1:]
                    continue
                
                # Validação 2: Calcular e verificar checksum
                calculated_crc = calculate_checksum(packet_data)
                received_crc = (packet_data[19] << 8) | packet_data[20]
                
                if calculated_crc != received_crc:
                    data.checksum_errors += 1
                    data.error_count += 1
                    sync_buffer = sync_buffer[1:]
                    continue
                
                # Parse do pacote
                sample = parse_packet(packet_data)
                
                if sample is not None:
                    # Adicionar aos dados
                    data.add_sample(sample)
                    last_packet_time = time.time()
                    
                    # Imprimir a cada 10 pacotes
                    if data.packet_count % 10 == 0:
                        elapsed = time.time() - data.start_time
                        rate = data.packet_count / elapsed if elapsed > 0 else 0
                        
                        print(f"{data.packet_count:<10} {sample['rpm']:<8} "
                              f"{sample['throttle']:<6} {sample['current']:<10.3f} "
                              f"{sample['voltage']:<10.2f} {sample['thrust']:<10.2f} "
                              f"{rate:<8.1f}")
                    
                    # Atualizar gráfico
                    if time.time() - last_plot_update > Config.PLOT_UPDATE_INTERVAL / 1000.0:
                        plotter.update()
                        last_plot_update = time.time()
                
                # Remover pacote processado
                sync_buffer = sync_buffer[Config.PACKET_LENGTH:]
    
    except KeyboardInterrupt:
        print("\n\n=== Coleta interrompida pelo usuário ===")
    
    except Exception as e:
        print(f"\n\nErro: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Fechar porta
        ser.close()
        print("\n✓ Porta serial fechada")
        
        # Atualização final dos gráficos
        plotter.update()
        
        # Exibir estatísticas
        print_statistics(data)
        
        # Salvar dados
        save_data(data)
        
        # Manter gráficos abertos
        print("\nGráficos permanecerão abertos.")
        print("Feche a janela ou pressione Ctrl+C para sair.")
        plt.ioff()
        plt.show()


# ============================================================================
# FUNÇÕES AUXILIARES
# ============================================================================

def print_statistics(data):
    """Imprime estatísticas dos dados coletados"""
    if data.idx == 0:
        print("\nNenhum dado foi coletado!")
        return
    
    total_time = time.time() - data.start_time
    avg_rate = data.packet_count / total_time if total_time > 0 else 0
    total_packets = data.packet_count + data.error_count
    error_rate = (data.error_count / total_packets * 100) if total_packets > 0 else 0
    
    stats = data.get_stats()
    
    print("\n" + "=" * 70)
    print("=== Estatísticas ===")
    print("=" * 70)
    print(f"Pacotes válidos:      {data.packet_count}")
    print(f"Erros totais:         {data.error_count} ({error_rate:.2f}%)")
    print(f"Erros de checksum:    {data.checksum_errors}")
    print(f"Tempo total:          {total_time:.2f} s")
    print(f"Taxa média:           {avg_rate:.1f} pkt/s")
    
    if stats:
        print("\n=== Análise Estatística ===")
        print(f"RPM:      min={stats['rpm']['min']:<8.0f}  "
              f"max={stats['rpm']['max']:<8.0f}  "
              f"média={stats['rpm']['mean']:<10.1f}")
        print(f"Corrente: min={stats['current']['min']:<8.3f}  "
              f"max={stats['current']['max']:<8.3f}  "
              f"média={stats['current']['mean']:<10.3f} A")
        print(f"Tensão:   min={stats['voltage']['min']:<8.2f}  "
              f"max={stats['voltage']['max']:<8.2f}  "
              f"média={stats['voltage']['mean']:<10.2f} V")
        print(f"Empuxo:   min={stats['thrust']['min']:<8.2f}  "
              f"max={stats['thrust']['max']:<8.2f}  "
              f"média={stats['thrust']['mean']:<10.2f} g.f")
        print(f"Potência: min={stats['power']['min']:<8.2f}  "
              f"max={stats['power']['max']:<8.2f}  "
              f"média={stats['power']['mean']:<10.2f} W")


def save_data(data, filename=None):
    """Salva dados em arquivo CSV"""
    if data.idx == 0:
        print("\nNenhum dado para salvar!")
        return
    
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"pso_data_{timestamp}.csv"
    
    print(f"\nSalvando dados em {filename}...")
    
    try:
        with open(filename, 'w') as f:
            # Header
            f.write("Time(s),Index,RPM,Throttle(%),AccelX,AccelY,AccelZ,"
                   "Current(A),Voltage(V),Thrust(g.f),Power(W)\n")
            
            # Data
            for i in range(data.idx):
                f.write(f"{list(data.time)[i]:.3f},"
                       f"{list(data.index)[i]},"
                       f"{list(data.rpm)[i]},"
                       f"{list(data.throttle)[i]},"
                       f"{list(data.accel_x)[i]},"
                       f"{list(data.accel_y)[i]},"
                       f"{list(data.accel_z)[i]},"
                       f"{list(data.current)[i]:.3f},"
                       f"{list(data.voltage)[i]:.2f},"
                       f"{list(data.thrust)[i]:.2f},"
                       f"{list(data.power)[i]:.2f}\n")
        
        print(f"✓ Arquivo salvo: {filename}")
    
    except Exception as e:
        print(f"✗ Erro ao salvar: {e}")


# ============================================================================
# MAIN
# ============================================================================

def main():
    """Função principal"""
    parser = argparse.ArgumentParser(
        description='PSO Data Acquisition - Real-Time Plotter')
    
    parser.add_argument('-p', '--port', 
                       help='Porta serial (ex: COM3 ou /dev/ttyACM0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='Taxa de transmissão (padrão: 115200)')
    parser.add_argument('-w', '--window', type=int, default=500,
                       help='Tamanho da janela de plotagem (padrão: 500)')
    
    args = parser.parse_args()
    
    acquire_data(port=args.port, baudrate=args.baudrate, plot_window=args.window)


if __name__ == '__main__':
    main()