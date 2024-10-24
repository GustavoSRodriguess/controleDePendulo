import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

def create_inverted_pendulum_controller():
    # Criar variáveis de entrada do pêndulo
    angle = ctrl.Antecedent(np.linspace(-1, 1, 100), 'angle')
    angular_velocity = ctrl.Antecedent(np.linspace(-1, 1, 100), 'angular_velocity')
    
    # Criar variáveis de entrada do carro
    position = ctrl.Antecedent(np.linspace(-1, 1, 100), 'position')
    velocity = ctrl.Antecedent(np.linspace(-1, 1, 100), 'velocity')
    
    # Criar variável de saída
    force = ctrl.Consequent(np.linspace(-1, 1, 100), 'force')
    
    # Definir funções de pertinência para o ângulo
    angle['N'] = fuzz.trimf(angle.universe, [-1, -1, 0])  # Negativo
    angle['Z'] = fuzz.trimf(angle.universe, [-0.5, 0, 0.5])  # Zero
    angle['P'] = fuzz.trimf(angle.universe, [0, 1, 1])  # Positivo
    
    # Definir funções de pertinência para velocidade angular
    angular_velocity['N'] = fuzz.trimf(angular_velocity.universe, [-1, -1, 0])
    angular_velocity['Z'] = fuzz.trimf(angular_velocity.universe, [-0.5, 0, 0.5])
    angular_velocity['P'] = fuzz.trimf(angular_velocity.universe, [0, 1, 1])
    
    # Definir funções de pertinência para posição do carro
    position['N'] = fuzz.trimf(position.universe, [-1, -1, 0])
    position['Z'] = fuzz.trimf(position.universe, [-0.5, 0, 0.5])
    position['P'] = fuzz.trimf(position.universe, [0, 1, 1])
    
    # Definir funções de pertinência para velocidade do carro
    velocity['N'] = fuzz.trimf(velocity.universe, [-1, -1, 0])
    velocity['Z'] = fuzz.trimf(velocity.universe, [-0.5, 0, 0.5])
    velocity['P'] = fuzz.trimf(velocity.universe, [0, 1, 1])
    
    # Definir funções de pertinência para força de saída
    force['NB'] = fuzz.trimf(force.universe, [-1, -1, -0.6])    # Negativo Grande
    force['N'] = fuzz.trimf(force.universe, [-0.8, -0.4, 0])    # Negativo
    force['Z'] = fuzz.trimf(force.universe, [-0.2, 0, 0.2])     # Zero
    force['P'] = fuzz.trimf(force.universe, [0, 0.4, 0.8])      # Positivo
    force['PB'] = fuzz.trimf(force.universe, [0.6, 1, 1])       # Positivo Grande
    
    # Regras para o pêndulo
    rule1 = ctrl.Rule(angle['N'] & angular_velocity['N'], force['NB'])
    rule2 = ctrl.Rule(angle['N'] & angular_velocity['Z'], force['N'])
    rule3 = ctrl.Rule(angle['N'] & angular_velocity['P'], force['Z'])
    rule4 = ctrl.Rule(angle['Z'] & angular_velocity['N'], force['N'])
    rule5 = ctrl.Rule(angle['Z'] & angular_velocity['Z'], force['Z'])
    rule6 = ctrl.Rule(angle['Z'] & angular_velocity['P'], force['P'])
    rule7 = ctrl.Rule(angle['P'] & angular_velocity['N'], force['Z'])
    rule8 = ctrl.Rule(angle['P'] & angular_velocity['Z'], force['P'])
    rule9 = ctrl.Rule(angle['P'] & angular_velocity['P'], force['PB'])
    
    # Regras para o carro
    rule10 = ctrl.Rule(position['N'] & velocity['N'], force['PB'])
    rule11 = ctrl.Rule(position['N'] & velocity['Z'], force['P'])
    rule12 = ctrl.Rule(position['N'] & velocity['P'], force['Z'])
    rule13 = ctrl.Rule(position['Z'] & velocity['N'], force['P'])
    rule14 = ctrl.Rule(position['Z'] & velocity['Z'], force['Z'])
    rule15 = ctrl.Rule(position['Z'] & velocity['P'], force['N'])
    rule16 = ctrl.Rule(position['P'] & velocity['N'], force['Z'])
    rule17 = ctrl.Rule(position['P'] & velocity['Z'], force['N'])
    rule18 = ctrl.Rule(position['P'] & velocity['P'], force['NB'])
    
    # Criar sistema de controle
    control_system = ctrl.ControlSystem([
        rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9,
        rule10, rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18
    ])
    
    return ctrl.ControlSystemSimulation(control_system)

# Função para controlar o pêndulo
def control_pendulum(controller, angle, angular_velocity, position, velocity):
    # Definir entradas
    controller.input['angle'] = angle
    controller.input['angular_velocity'] = angular_velocity
    controller.input['position'] = position
    controller.input['velocity'] = velocity
    
    # Computar a saída
    try:
        controller.compute()
        return controller.output['force']
    except:
        return 0