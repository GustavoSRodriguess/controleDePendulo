import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from fuzzy_controler import create_inverted_pendulum_controller, control_pendulum
import time

class PendulumControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Controlador do Pêndulo Invertido")
        
        # Criar o controlador fuzzy
        self.controller = create_inverted_pendulum_controller()
        
        # Variáveis de simulação
        self.is_running = False
        self.simulation_start_time = None
        self.dt = 0.01
        self.target_angle = 0.0
        self.angle_tolerance = 0.1
        
        # Métricas de desempenho
        self.metrics = {
            'execution_time': 0,
            'stability_percentage': 0,
            'total_error': 0,
            'max_deviation': 0,
            'energy_used': 0
        }
        
        # Configurar o layout principal
        self.setup_layout()
        
        # Histórico para os gráficos
        self.history = {
            'angle': [],
            'angular_velocity': [],
            'position': [],
            'velocity': [],
            'force': [],
            'time': []
        }
        
        # Inicializar a figura do matplotlib
        self.setup_plots()

    def setup_layout(self):
        # Frame principal
        main_frame = ttk.Frame(self.root)
        main_frame.grid(row=0, column=0, sticky="nsew")
        
        # Frame para controles
        control_frame = ttk.LabelFrame(main_frame, text="Controles", padding="10")
        control_frame.grid(row=0, column=0, padx=10, pady=5, sticky="nsew")
        
        # Variáveis de controle
        self.angle_var = tk.DoubleVar(value=0.1)
        self.angular_vel_var = tk.DoubleVar(value=0.0)
        self.position_var = tk.DoubleVar(value=0.0)
        self.velocity_var = tk.DoubleVar(value=0.0)
        self.simulation_time_var = tk.DoubleVar(value=10.0)
        
        # Configurações iniciais
        ttk.Label(control_frame, text="Configurações Iniciais:").grid(row=0, column=0, columnspan=2, pady=5)
        
        # Campos de entrada para configurações iniciais
        input_fields = [
            ("Ângulo Inicial (rad):", self.angle_var),
            ("Velocidade Angular Inicial (rad/s):", self.angular_vel_var),
            ("Posição Inicial (m):", self.position_var),
            ("Velocidade Inicial (m/s):", self.velocity_var),
            ("Tempo de Simulação (s):", self.simulation_time_var)
        ]
        
        for i, (label, var) in enumerate(input_fields):
            ttk.Label(control_frame, text=label).grid(row=i+1, column=0, padx=5, pady=2, sticky="e")
            ttk.Entry(control_frame, textvariable=var, width=10).grid(row=i+1, column=1, padx=5, pady=2, sticky="w")
        
        # Estado atual
        ttk.Label(control_frame, text="Estado Atual:").grid(row=len(input_fields)+1, column=0, columnspan=2, pady=10)
        
        labels = ["Ângulo:", "Vel. Angular:", "Posição:", "Velocidade:", "Força:"]
        self.state_labels = {}
        for i, label in enumerate(labels):
            ttk.Label(control_frame, text=label).grid(row=i+len(input_fields)+2, column=0, padx=5, pady=2)
            self.state_labels[label] = ttk.Label(control_frame, text="0.000")
            self.state_labels[label].grid(row=i+len(input_fields)+2, column=1, padx=5, pady=2)
        
        # Frame para métricas
        metrics_frame = ttk.LabelFrame(main_frame, text="Métricas de Desempenho", padding="10")
        metrics_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        
        self.metric_labels = {}
        metric_names = [
            "Tempo de Execução (s):",
            "Estabilidade (%):",
            "Erro Total:",
            "Desvio Máximo:",
            "Energia Utilizada:"
        ]
        
        for i, metric in enumerate(metric_names):
            ttk.Label(metrics_frame, text=metric).grid(row=i, column=0, padx=5, pady=2)
            self.metric_labels[metric] = ttk.Label(metrics_frame, text="0.000")
            self.metric_labels[metric].grid(row=i, column=1, padx=5, pady=2)
        
        # Botões de controle
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=2, column=0, pady=10)
        
        self.start_button = ttk.Button(button_frame, text="Iniciar Simulação", command=self.toggle_simulation)
        self.start_button.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(button_frame, text="Resetar", command=self.reset).pack(side=tk.LEFT, padx=5)

    def setup_plots(self):
        self.fig = Figure(figsize=(10, 6))
        
        # Subplot para ângulo
        self.ax1 = self.fig.add_subplot(311)
        self.ax1.set_title('Ângulo do Pêndulo')
        self.ax1.grid(True)
        self.line_angle, = self.ax1.plot([], [], 'b-', label='Ângulo')
        self.ax1.set_ylabel('Radianos')
        self.ax1.legend()
        
        # Subplot para posição
        self.ax2 = self.fig.add_subplot(312)
        self.ax2.set_title('Posição do Carro')
        self.ax2.grid(True)
        self.line_position, = self.ax2.plot([], [], 'g-', label='Posição')
        self.ax2.set_ylabel('Metros')
        self.ax2.legend()
        
        # Subplot para força
        self.ax3 = self.fig.add_subplot(313)
        self.ax3.set_title('Força de Controle')
        self.ax3.grid(True)
        self.line_force, = self.ax3.plot([], [], 'r-', label='Força')
        self.ax3.set_xlabel('Tempo (s)')
        self.ax3.set_ylabel('Newtons')
        self.ax3.legend()
        
        self.fig.tight_layout()
        
        # Adicionar canvas do matplotlib ao tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=1, rowspan=3, padx=10, pady=5)

    def update_metrics(self):
        if len(self.history['angle']) > 0:
            current_time = time.time() - self.simulation_start_time
            angles = np.array(self.history['angle'])
            forces = np.array(self.history['force'])
            
            self.metrics['execution_time'] = current_time
            self.metrics['stability_percentage'] = (np.sum(np.abs(angles) < self.angle_tolerance) / len(angles)) * 100
            self.metrics['total_error'] = np.sum(np.abs(angles - self.target_angle))
            self.metrics['max_deviation'] = np.max(np.abs(angles - self.target_angle))
            self.metrics['energy_used'] = np.sum(np.abs(forces)) * self.dt
            
            for label, value in [
                ("Tempo de Execução (s):", self.metrics['execution_time']),
                ("Estabilidade (%):", self.metrics['stability_percentage']),
                ("Erro Total:", self.metrics['total_error']),
                ("Desvio Máximo:", self.metrics['max_deviation']),
                ("Energia Utilizada:", self.metrics['energy_used'])
            ]:
                self.metric_labels[label].config(text=f"{value:.3f}")

    def update_simulation(self):
        if not self.is_running:
            return
            
        current_time = time.time() - self.simulation_start_time
        if current_time >= self.simulation_time_var.get():
            self.stop_simulation()
            return
            
        # Simulação física simplificada do pêndulo
        angle = self.angle_var.get()
        angular_velocity = self.angular_vel_var.get()
        position = self.position_var.get()
        velocity = self.velocity_var.get()
        
        # Calcular força de controle
        force = control_pendulum(self.controller, angle, angular_velocity, position, velocity)
        
        # Atualizar estado do sistema (simulação física simplificada)
        new_angular_velocity = angular_velocity + force * self.dt
        new_angle = angle + new_angular_velocity * self.dt
        new_velocity = velocity + force * self.dt
        new_position = position + velocity * self.dt
        
        # Atualizar variáveis
        self.angle_var.set(new_angle)
        self.angular_vel_var.set(new_angular_velocity)
        self.position_var.set(new_position)
        self.velocity_var.set(new_velocity)
        
        # Atualizar histórico
        self.history['time'].append(current_time)
        self.history['angle'].append(new_angle)
        self.history['position'].append(new_position)
        self.history['force'].append(force)
        
        # Atualizar labels de estado
        for label, value in [
            ("Ângulo:", new_angle),
            ("Vel. Angular:", new_angular_velocity),
            ("Posição:", new_position),
            ("Velocidade:", new_velocity),
            ("Força:", force)
        ]:
            self.state_labels[label].config(text=f"{value:.3f}")
        
        # Atualizar gráficos
        self.line_angle.set_data(self.history['time'], self.history['angle'])
        self.line_position.set_data(self.history['time'], self.history['position'])
        self.line_force.set_data(self.history['time'], self.history['force'])
        
        # Ajustar limites dos eixos
        for ax in [self.ax1, self.ax2, self.ax3]:
            ax.relim()
            ax.autoscale_view()
        
        self.canvas.draw()
        
        # Atualizar métricas
        self.update_metrics()
        
        # Agendar próxima atualização
        self.root.after(int(self.dt * 1000), self.update_simulation)

    def toggle_simulation(self):
        if not self.is_running:
            self.start_simulation()
        else:
            self.stop_simulation()

    def start_simulation(self):
        self.is_running = True
        self.simulation_start_time = time.time()
        self.start_button.config(text="Parar Simulação")
        self.update_simulation()

    def stop_simulation(self):
        self.is_running = False
        self.start_button.config(text="Iniciar Simulação")

    def reset(self):
        # Parar simulação se estiver rodando
        self.stop_simulation()
        
        # Resetar valores para os padrões iniciais
        self.angle_var.set(0.1)
        self.angular_vel_var.set(0.0)
        self.position_var.set(0.0)
        self.velocity_var.set(0.0)
        
        # Limpar histórico
        for key in self.history:
            self.history[key] = []
        
        # Resetar métricas
        for metric in self.metric_labels.values():
            metric.config(text="0.000")
        
        # Limpar gráficos
        self.line_angle.set_data([], [])
        self.line_position.set_data([], [])
        self.line_force.set_data([], [])
        self.canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = PendulumControlGUI(root)
    root.mainloop()