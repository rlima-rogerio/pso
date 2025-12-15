#!/usr/bin/env python3
"""
PSO Data Acquisition - UART Receiver (INA169 VERSION)
======================================================

HARDWARE:
- Voltage: Divisor resistivo (R3=1.5kΩ, R4=13.7kΩ)
- Current: INA169 + Rshunt=0.5mΩ + RL=110kΩ

CONVERSÃO:
- Voltage: V = mV / 1000 (direto)
- Current: A = mA / 1000 (direto)

FAIXAS:
- Voltage: 0-33400 mV (0-33.4V)
- Current: 0-60000 mA (0-60A)

Uso:
    python uart_receiver.py --port /dev/ttyACM0 --baudrate 115200
    
# Modo console
    python uart_receiver.py --port /dev/ttyACM0 --mode console

# Modo gráfico
    python uart_receiver.py --port /dev/ttyACM0 --mode plot

# Salvar em CSV
    python uart_receiver.py --port /dev/ttyACM0 --output data.csv
```
"""

import serial
import struct
import argparse
import time
from collections import deque
import sys

# Tentar importar matplotlib
try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    print("Warning: matplotlib não disponível. Gráficos desabilitados.")
    MATPLOTLIB_AVAILABLE = False

# ===========================================================================
# CONSTANTES DO PROTOCOLO
# ===========================================================================
STX = 0xFE
PACKET_LENGTH = 21
MAX_BUFFER_SIZE = 1000

# ===========================================================================
# HARDWARE PARAMETERS (INA169)
# ===========================================================================
# Voltage Divider
R3 = 1500      # Ω
R4 = 13700     # Ω
VMAX = 33.4    # V

# Current Monitor (INA169)
RSHUNT = 0.5e-3      # 0.5 mΩ
RL = 110e3           # 110 kΩ
GM = 1e-3            # 1000 μA/V = 0.001 A/V
VOUT_PER_AMP = RSHUNT * GM * RL  # 0.055 V/A
IMAX = 3.3 / VOUT_PER_AMP        # 60 A


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
        Parse pacote binário (INA169 VERSION)
        
        Formato (21 bytes):
        [0]      STX (0xFE)
        [1]      Length (14)
        [2-3]    Index (uint16, big-endian)
        [4-5]    Accel X (int16, big-endian)
        [6-7]    Accel Y (int16, big-endian)
        [8-9]    Accel Z (int16, big-endian)
        [10-11]  RPM (uint16, big-endian)
        [12-13]  Current (uint16, big-endian, em mA, 0-60000)
        [14-15]  Voltage (uint16, big-endian, em mV, 0-33400)
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
            
            # ================================================================
            # INA169 CONFIGURATION: Current e Voltage são uint16 (H)
            # ================================================================
            # Format: '>HhhhHHHhB'
            #   H = uint16 (index, rpm, current, voltage)
            #   h = int16 (accel_x, accel_y, accel_z, thrust)
            #   B = uint8 (throttle)
            # ================================================================
            
            (self.index, self.accel_x, self.accel_y, self.accel_z,
             self.rpm, current_ma, voltage_mv, thrust_raw, self.throttle) = \
                struct.unpack('>HhhhHHHhB', data[2:19])
            
            # ================================================================
            # CONVERSÃO DIRETA (INA169)
            # ================================================================
            
            # Voltage: mV → V
            self.voltage = voltage_mv / 1000.0
            
            # Current: mA → A (DIRETO, sem fator de escala!)
            self.current = current_ma / 1000.0
            
            # Thrust
            self.thrust = thrust_raw  # / 100.0 se for em cN
            
            # ================================================================
            
            # Unpack checksum
            self.checksum = struct.unpack('>H', data[19:21])[0]
            
            # TODO: Validar checksum (implementar CRC-16)
            self.valid = True
            return True
            
        except struct.error as e:
            print(f"Erro no parse: {e}")
            return False
    
    def __str__(self):
        """Representação em string do pacote"""
        power_kw = (self.voltage * self.current) / 1000.0
        return (f"[{self.index:05d}] "
                f"RPM:{self.rpm:5d} "
                f"Thr:{self.throttle:3d}% "
                f"Acc:({self.accel_x:5d},{self.accel_y:5d},{self.accel_z:5d}) "
                f"V:{self.voltage:6.3f} V "
                f"I:{self.current:6.2f} A "
                f"F:{self.thrust:6.2f} N "
                f"P:{power_kw:6.2f} kW")


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
        self.power_buffer = deque(maxlen=MAX_BUFFER_SIZE)
    
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
            print(f"\nHardware Configuration (INA169):")
            print(f"  Voltage range: 0-{VMAX:.1f} V")
            print(f"  Current range: 0-{IMAX:.0f} A")
            print(f"  Transfer function: Vout = Is × {VOUT_PER_AMP:.3f} [V/A]")
            print(f"  Rshunt: {RSHUNT*1000:.1f} mΩ")
            print(f"  RL: {RL/1000:.0f} kΩ\n")
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
        
        # Calculate power
        power_kw = (packet.voltage * packet.current) / 1000.0
        self.power_buffer.append(power_kw)
    
    def run_console_mode(self):
        """Modo console - apenas imprime dados"""
        print("\n=== Modo Console (INA169 Configuration) ===")
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
                        rate = self.packet_count / elapsed if elapsed > 0 else 0
                        total = self.packet_count + self.error_count
                        error_rate = (self.error_count / total * 100) if total > 0 else 0
                        
                        # Estatísticas dos dados
                        if len(self.current_buffer) > 0:
                            avg_current = sum(self.current_buffer) / len(self.current_buffer)
                            max_current = max(self.current_buffer)
                            avg_voltage = sum(self.voltage_buffer) / len(self.voltage_buffer)
                            avg_power = sum(self.power_buffer) / len(self.power_buffer)
                            
                            print(f"\n--- Stats: {self.packet_count} pacotes, "
                                  f"{rate:.1f} pkt/s, {error_rate:.1f}% erros ---")
                            print(f"    Avg: V={avg_voltage:.2f}V, I={avg_current:.2f}A, "
                                  f"P={avg_power:.2f}kW, Max I={max_current:.2f}A\n")
        
        except KeyboardInterrupt:
            print("\n\nParando...")
            self.running = False
    
    def run_plot_mode(self):
        """Modo gráfico - plota dados em tempo real"""
        if not MATPLOTLIB_AVAILABLE:
            print("Matplotlib não disponível. Use modo console.")
            return
        
        print("\n=== Modo Gráfico (INA169 Configuration) ===")
        print("Fechando a janela irá parar o programa\n")
        
        # Configurar figura e subplots
        fig, axes = plt.subplots(3, 2, figsize=(12, 8))
        fig.suptitle('PSO Data Acquisition - Real Time Monitor (INA169)')
        
        # Configurar cada subplot
        ax_rpm = axes[0, 0]
        ax_thr = axes[0, 1]
        ax_cur = axes[1, 0]
        ax_vol = axes[1, 1]
        ax_thrust = axes[2, 0]
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
        
        ax_thrust.set_title('Thrust')
        ax_thrust.set_ylabel('N')
        ax_thrust.set_xlabel('Time (s)')
        ax_thrust.grid(True)
        
        ax_power.set_title('Power')
        ax_power.set_ylabel('kW')
        ax_power.set_xlabel('Time (s)')
        ax_power.grid(True)
        
        # Linhas de plotagem
        line_rpm, = ax_rpm.plot([], [], 'b-', linewidth=1.5)
        line_thr, = ax_thr.plot([], [], 'g-', linewidth=1.5)
        line_cur, = ax_cur.plot([], [], 'r-', linewidth=1.5)
        line_vol, = ax_vol.plot([], [], 'orange', linewidth=1.5)
        line_thrust, = ax_thrust.plot([], [], 'm-', linewidth=1.5)
        line_power, = ax_power.plot([], [], 'c-', linewidth=1.5)
        
        def animate(frame):
            """Função de animação"""
            # Ler novo pacote
            packet = self.read_packet()
            
            if packet:
                self.update_buffers(packet)
                
                # Atualizar dados das linhas
                line_rpm.set_data(self.time_buffer, self.rpm_buffer)
                line_thr.set_data(self.time_buffer, self.throttle_buffer)
                line_cur.set_data(self.time_buffer, self.current_buffer)
                line_vol.set_data(self.time_buffer, self.voltage_buffer)
                line_thrust.set_data(self.time_buffer, self.thrust_buffer)
                line_power.set_data(self.time_buffer, self.power_buffer)
                
                # Ajustar limites dos eixos
                if len(self.time_buffer) > 0:
                    for ax in [ax_rpm, ax_thr, ax_cur, ax_vol, ax_thrust, ax_power]:
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
            f.write("Time,Index,RPM,Throttle,Current_A,Voltage_V,Thrust,Power_kW\n")
            
            # Dados
            for i in range(len(self.time_buffer)):
                f.write(f"{self.time_buffer[i]:.3f},"
                       f"{i},"
                       f"{self.rpm_buffer[i]},"
                       f"{self.throttle_buffer[i]},"
                       f"{self.current_buffer[i]:.3f},"
                       f"{self.voltage_buffer[i]:.3f},"
                       f"{self.thrust_buffer[i]:.2f},"
                       f"{self.power_buffer[i]:.3f}\n")
        
        print(f"Dados salvos: {len(self.time_buffer)} amostras")
        
        # Estatísticas finais
        if len(self.current_buffer) > 0:
            print(f"\n=== Estatísticas Finais ===")
            print(f"Amostras: {len(self.current_buffer)}")
            print(f"Tensão média: {sum(self.voltage_buffer)/len(self.voltage_buffer):.2f} V")
            print(f"Corrente média: {sum(self.current_buffer)/len(self.current_buffer):.2f} A")
            print(f"Corrente máxima: {max(self.current_buffer):.2f} A")
            print(f"Potência média: {sum(self.power_buffer)/len(self.power_buffer):.2f} kW")
            print(f"Potência máxima: {max(self.power_buffer):.2f} kW")


def main():
    """Função principal"""
    parser = argparse.ArgumentParser(
        description='PSO UART Data Receiver (INA169 Configuration)'
    )
    parser.add_argument(
        '--port', '-p',
        default='/dev/ttyACM0',
        help='Porta serial (default: /dev/ttyACM0)'
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