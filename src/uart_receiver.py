#!/usr/bin/env python3
"""
PSO Data Acquisition - UART Receiver
=====================================

Script Python para receber, decodificar e visualizar dados do sistema PSO
via comunicação UART.

Requisitos:
    pip install pyserial numpy matplotlib

Uso:
    python pso_uart_receiver.py --port /dev/ttyUSB0 --baudrate 115200

Autor: Refatoração 2025
"""

import serial
import struct
import argparse
import time
from datetime import datetime
from collections import deque
import sys

# Tentar importar matplotlib para gráficos (opcional)
try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    print("Warning: matplotlib não disponível. Gráficos desabilitados.")
    MATPLOTLIB_AVAILABLE = False

# Constantes do protocolo
STX = 0xFE
PACKET_LENGTH = 21
MAX_BUFFER_SIZE = 1000  # Número máximo de amostras no buffer de plotagem


class PSOPacket:
    """Classe para representar um pacote de dados PSO"""
    
    def __init__(self, data=None):
        self.stx = 0
        self.length = 0
        self.index = 0
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.rpm = 0
        self.current = 0.0
        self.voltage = 0.0
        self.thrust = 0.0
        self.throttle = 0
        self.checksum = 0
        self.valid = False
        
        if data:
            self.parse(data)
    
    def parse(self, data):
        """
        Parse pacote binário
        
        Formato (21 bytes):
        [0]      STX (0xFE)
        [1]      Length (14)
        [2-3]    Index (uint16, big-endian)
        [4-5]    Accel X (int16, big-endian)
        [6-7]    Accel Y (int16, big-endian)
        [8-9]    Accel Z (int16, big-endian)
        [10-11]  RPM (uint16, big-endian)
        [12-13]  Current (int16, big-endian, em mA)
        [14-15]  Voltage (int16, big-endian, em mV)
        [16-17]  Thrust (int16, big-endian, em cN)
        [18]     Throttle (uint8, 0-100%)
        [19-20]  Checksum (uint16, big-endian)
        """
        if len(data) != PACKET_LENGTH:
            return False
        
        try:
            # Unpack header
            self.stx, self.length = struct.unpack('BB', data[0:2])
            
            # Verificar STX
            if self.stx != STX:
                return False
            
            # Unpack payload
            (self.index, self.accel_x, self.accel_y, self.accel_z,
             self.rpm, current_raw, voltage_raw, thrust_raw, self.throttle) = \
                struct.unpack('>HhhhHhhhB', data[2:19])
            
            # Converter para unidades reais
            self.current = current_raw / 1000.0  # mA para A
            self.voltage = voltage_raw / 1000.0  # mV para V
            self.thrust = thrust_raw / 100.0     # cN para N
            
            # Unpack checksum
            self.checksum = struct.unpack('>H', data[19:21])[0]
            
            # TODO: Validar checksum (implementar CRC-16)
            self.valid = True
            return True
            
        except struct.error:
            return False
    
    def __str__(self):
        """Representação em string do pacote"""
        return (f"[{self.index:05d}] "
                f"RPM:{self.rpm:5d} "
                f"Thr:{self.throttle:3d}% "
                f"Acc:({self.accel_x:5d},{self.accel_y:5d},{self.accel_z:5d}) "
                f"V:{self.voltage:6.2f}V "
                f"I:{self.current:6.3f}A "
                f"F:{self.thrust:6.2f}N")


class PSOReceiver:
    """Classe para gerenciar recepção de dados PSO via UART"""
    
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.packet_count = 0
        self.error_count = 0
        self.start_time = None
        
        # Buffers para plotagem
        self.time_buffer = deque(maxlen=MAX_BUFFER_SIZE)
        self.rpm_buffer = deque(maxlen=MAX_BUFFER_SIZE)
        self.throttle_buffer = deque(maxlen=MAX_BUFFER_SIZE)
        self.current_buffer = deque(maxlen=MAX_BUFFER_SIZE)
        self.voltage_buffer = deque(maxlen=MAX_BUFFER_SIZE)
        self.thrust_buffer = deque(maxlen=MAX_BUFFER_SIZE)
    
    def connect(self):
        """Conectar à porta serial"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print(f"Conectado a {self.port} @ {self.baudrate} bps")
            self.start_time = time.time()
            return True
        except serial.SerialException as e:
            print(f"Erro ao conectar: {e}")
            return False
    
    def disconnect(self):
        """Desconectar da porta serial"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Desconectado")
    
    def find_sync(self):
        """Procurar byte de sincronização (STX)"""
        while True:
            byte = self.serial.read(1)
            if not byte:
                return False
            if byte[0] == STX:
                return True
    
    def read_packet(self):
        """Ler um pacote completo da serial"""
        # Procurar STX
        if not self.find_sync():
            return None
        
        # Ler resto do pacote
        packet_data = bytes([STX]) + self.serial.read(PACKET_LENGTH - 1)
        
        if len(packet_data) != PACKET_LENGTH:
            self.error_count += 1
            return None
        
        # Parse pacote
        packet = PSOPacket(packet_data)
        
        if packet.valid:
            self.packet_count += 1
            return packet
        else:
            self.error_count += 1
            return None
    
    def update_buffers(self, packet):
        """Atualizar buffers de dados para plotagem"""
        elapsed_time = time.time() - self.start_time
        
        self.time_buffer.append(elapsed_time)
        self.rpm_buffer.append(packet.rpm)
        self.throttle_buffer.append(packet.throttle)
        self.current_buffer.append(packet.current)
        self.voltage_buffer.append(packet.voltage)
        self.thrust_buffer.append(packet.thrust)
    
    def run_console_mode(self):
        """Modo console - apenas imprime dados"""
        print("\n=== Modo Console ===")
        print("Pressione Ctrl+C para parar\n")
        
        self.running = True
        
        try:
            while self.running:
                packet = self.read_packet()
                
                if packet:
                    print(packet)
                    
                    # Estatísticas a cada 100 pacotes
                    if self.packet_count % 100 == 0:
                        elapsed = time.time() - self.start_time
                        rate = self.packet_count / elapsed
                        error_rate = (self.error_count / 
                                     (self.packet_count + self.error_count) * 100)
                        print(f"\n--- Stats: {self.packet_count} pacotes, "
                              f"{rate:.1f} pkt/s, {error_rate:.1f}% erros ---\n")
        
        except KeyboardInterrupt:
            print("\n\nParando...")
            self.running = False
    
    def run_plot_mode(self):
        """Modo gráfico - plota dados em tempo real"""
        if not MATPLOTLIB_AVAILABLE:
            print("Matplotlib não disponível. Use modo console.")
            return
        
        print("\n=== Modo Gráfico ===")
        print("Fechando a janela irá parar o programa\n")
        
        # Configurar figura e subplots
        fig, axes = plt.subplots(3, 2, figsize=(12, 8))
        fig.suptitle('PSO Data Acquisition - Real Time Monitor')
        
        # Configurar cada subplot
        ax_rpm = axes[0, 0]
        ax_thr = axes[0, 1]
        ax_cur = axes[1, 0]
        ax_vol = axes[1, 1]
        ax_thr_plot = axes[2, 0]
        ax_power = axes[2, 1]
        
        ax_rpm.set_title('RPM')
        ax_rpm.set_ylabel('RPM')
        ax_rpm.grid(True)
        
        ax_thr.set_title('Throttle')
        ax_thr.set_ylabel('%')
        ax_thr.grid(True)
        
        ax_cur.set_title('Current')
        ax_cur.set_ylabel('A')
        ax_cur.grid(True)
        
        ax_vol.set_title('Voltage')
        ax_vol.set_ylabel('V')
        ax_vol.grid(True)
        
        ax_thr_plot.set_title('Thrust')
        ax_thr_plot.set_ylabel('N')
        ax_thr_plot.set_xlabel('Time (s)')
        ax_thr_plot.grid(True)
        
        ax_power.set_title('Power')
        ax_power.set_ylabel('W')
        ax_power.set_xlabel('Time (s)')
        ax_power.grid(True)
        
        # Linhas de plotagem
        line_rpm, = ax_rpm.plot([], [], 'b-')
        line_thr, = ax_thr.plot([], [], 'g-')
        line_cur, = ax_cur.plot([], [], 'r-')
        line_vol, = ax_vol.plot([], [], 'orange')
        line_thrust, = ax_thr_plot.plot([], [], 'm-')
        line_power, = ax_power.plot([], [], 'c-')
        
        def animate(frame):
            """Função de animação"""
            # Ler novo pacote
            packet = self.read_packet()
            
            if packet:
                self.update_buffers(packet)
                
                # Calcular potência
                power = packet.voltage * packet.current
                
                # Atualizar dados das linhas
                line_rpm.set_data(self.time_buffer, self.rpm_buffer)
                line_thr.set_data(self.time_buffer, self.throttle_buffer)
                line_cur.set_data(self.time_buffer, self.current_buffer)
                line_vol.set_data(self.time_buffer, self.voltage_buffer)
                line_thrust.set_data(self.time_buffer, self.thrust_buffer)
                
                # Potência
                power_buffer = [v * i for v, i in 
                               zip(self.voltage_buffer, self.current_buffer)]
                line_power.set_data(self.time_buffer, power_buffer)
                
                # Ajustar limites dos eixos
                if len(self.time_buffer) > 0:
                    for ax in [ax_rpm, ax_thr, ax_cur, ax_vol, ax_thr_plot, ax_power]:
                        ax.relim()
                        ax.autoscale_view()
            
            return (line_rpm, line_thr, line_cur, line_vol, 
                   line_thrust, line_power)
        
        # Iniciar animação
        ani = animation.FuncAnimation(
            fig, animate, interval=10, blit=True, cache_frame_data=False
        )
        
        plt.tight_layout()
        plt.show()
    
    def save_to_csv(self, filename):
        """Salvar dados em arquivo CSV"""
        print(f"\nSalvando dados em {filename}...")
        
        with open(filename, 'w') as f:
            # Cabeçalho
            f.write("Time,Index,RPM,Throttle,AccelX,AccelY,AccelZ,"
                   "Current,Voltage,Thrust,Power\n")
            
            # Dados
            for i in range(len(self.time_buffer)):
                power = self.voltage_buffer[i] * self.current_buffer[i]
                f.write(f"{self.time_buffer[i]:.3f},"
                       f"{i},"
                       f"{self.rpm_buffer[i]},"
                       f"{self.throttle_buffer[i]},"
                       f"0,0,0,"  # Accel não está nos buffers
                       f"{self.current_buffer[i]:.3f},"
                       f"{self.voltage_buffer[i]:.2f},"
                       f"{self.thrust_buffer[i]:.2f},"
                       f"{power:.2f}\n")
        
        print(f"Dados salvos: {len(self.time_buffer)} amostras")


def main():
    """Função principal"""
    parser = argparse.ArgumentParser(
        description='PSO UART Data Receiver'
    )
    parser.add_argument(
        '--port', '-p',
        default='/dev/ttyACM0', # '/dev/ttyUSB0'
        help='Porta serial (default: /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--baudrate', '-b',
        type=int,
        default=115200,
        help='Baudrate (default: 115200)'
    )
    parser.add_argument(
        '--mode', '-m',
        choices=['console', 'plot'],
        default='console',
        help='Modo de operação (default: console)'
    )
    parser.add_argument(
        '--output', '-o',
        help='Arquivo CSV de saída (opcional)'
    )
    
    args = parser.parse_args()
    
    # Criar receiver
    receiver = PSOReceiver(args.port, args.baudrate)
    
    # Conectar
    if not receiver.connect():
        sys.exit(1)
    
    try:
        # Executar modo selecionado
        if args.mode == 'console':
            receiver.run_console_mode()
        elif args.mode == 'plot':
            receiver.run_plot_mode()
        
        # Salvar dados se solicitado
        if args.output:
            receiver.save_to_csv(args.output)
    
    finally:
        receiver.disconnect()


if __name__ == '__main__':
    main()