import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# INPUTS
# Distância para obstáculos (classificada como "próxima", "média", "distante")
# Espaço de manobra disponível ("limitado", "amplo"), cada parte vai ter uma verificação limitado, amplo diferente
# Velocidade do robô ("lenta", "moderada", "rápida")
# Velocidade dos obstáculos ("estático", "lento", "rápido"), afastando, aproximando

# OUTPUTS
# alpha, beta, gamma

def fuzzy_logic_maneuvering_space(space_parts, safety_distance):
    """Find the occupancy rate of the obstacle area to discover the available maneuvering space.
    
    Args:
        space_parts (list): mean distance values of parts that divide the space.
        safety_distance (float): safety distance that define obstacle influence area.
        
    Returns:
        occupancy_rate_output (float): occupancy rate of the obstacle area."""

    # Define fuzzy variables, each variable is created as Antecedent (input) or Consequent (output).
    distancia_a = ctrl.Antecedent(np.arange(0, safety_distance, 0.1), 'distance_a')
    distancia_b = ctrl.Antecedent(np.arange(0, safety_distance, 0.1), 'distance_b')
    distancia_c = ctrl.Antecedent(np.arange(0, safety_distance, 0.1), 'distance_c')
    occupancy_rate = ctrl.Consequent(np.arange(0, 100, 1), 'occupancy_rate')

    # Define parameters for the sigmoidal membership functions
    midpoint_1 = safety_distance/2
    midpoint_2 = midpoint_1 + 1.0

    # Define the sigmoidal membership functions for the distance to the obstacle
    distancia_a['perto'] = fuzz.sigmf(distancia_a.universe, midpoint_1, -safety_distance)  # ajusta o ponto médio e a inclinação -10
    distancia_a['distante'] = fuzz.sigmf(distancia_a.universe, midpoint_2, safety_distance)

    distancia_b['perto'] = fuzz.sigmf(distancia_b.universe, midpoint_1, -safety_distance)
    distancia_b['distante'] = fuzz.sigmf(distancia_b.universe, midpoint_2, safety_distance)

    distancia_c['perto'] = fuzz.sigmf(distancia_c.universe, midpoint_1, -safety_distance)
    distancia_c['distante'] = fuzz.sigmf(distancia_c.universe, midpoint_2, safety_distance)

    # Define the sigmoidal membership functions for the occupancy rate
    occupancy_rate['amplo'] = fuzz.sigmf(occupancy_rate.universe, 50, -0.1)
    occupancy_rate['limitado'] = fuzz.sigmf(occupancy_rate.universe, 75, 0.1)

    # Plot graphs of relevance (optional)
    # distancia_a.view()
    # distancia_b.view()
    # distancia_c.view()
    # occupancy_rate.view()

    # Fuzzy rules
    rule1 = ctrl.Rule(distancia_a['perto'] & distancia_b['perto'] & distancia_c['perto'], occupancy_rate['limitado'])
    rule2 = ctrl.Rule(distancia_a['perto'] & distancia_b['distante'] & distancia_c['perto'], occupancy_rate['limitado'])
    rule3 = ctrl.Rule(distancia_a['perto'] & distancia_b['perto'] & distancia_c['distante'], occupancy_rate['limitado'])
    rule4 = ctrl.Rule(distancia_a['perto'] & distancia_b['distante'] & distancia_c['distante'], occupancy_rate['amplo'])

    rule5 = ctrl.Rule(distancia_a['distante'] & distancia_b['perto'] & distancia_c['perto'], occupancy_rate['limitado'])
    rule6 = ctrl.Rule(distancia_a['distante'] & distancia_b['distante'] & distancia_c['perto'], occupancy_rate['amplo'])
    rule7 = ctrl.Rule(distancia_a['distante'] & distancia_b['perto'] & distancia_c['distante'], occupancy_rate['amplo'])
    rule8 = ctrl.Rule(distancia_a['distante'] & distancia_b['distante'] & distancia_c['distante'], occupancy_rate['amplo'])

    # Fuzzy controller
    occupancy_rate_indicator = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8])
    simulator = ctrl.ControlSystemSimulation(occupancy_rate_indicator)

    # Input values for simulation
    simulator.input['distance_a'] = space_parts[0]
    simulator.input['distance_b'] = space_parts[1]
    simulator.input['distance_c'] = space_parts[2]

    # Simulation
    simulator.compute()

    # Output
    occupancy_rate_output = simulator.output['occupancy_rate']

    # Plot graphs of relevance (optional)
    # occupancy_rate.view(simulator)

    return occupancy_rate_output

def fuzzy_logic(dist_input, obst_input, space_input, safety_distance, obst_max_speed=6.0):
    """Find the alpha, beta and gamma that fits best for each situation predicted.
    
    Args:   
        dist_input (float): distance to the nearest obstacle in m.
        obst_input (list): obstacle velocity in m/s.
        space_input (float): maneuvering space in m.
        safety_distance (float): safety distance that define obstacle influence area.
        obst_max_speed (float): maximum speed of the obstacle in m/s.
    
    Returns:
        alpha_output (float): alpha value.
        beta_output (float): beta value.
        gamma_output (float): gamma value."""
    
    rules = []

    # Define fuzzy variables, each variable is created as Antecedent (input) or Consequent (output).
    distancia_to_obstacle = ctrl.Antecedent(np.arange(0, safety_distance, 0.1), 'distance_to_obstacle') # Represent the robot distance to the obstacle
    maneuvering_space = ctrl.Antecedent(np.arange(0, 100, 0.1), 'maneuvering_space')
    obstacle_velocity = ctrl.Antecedent(np.arange(0, obst_max_speed, 0.1), 'obstacle_velocity')
    # obstacle_velocity_b = ctrl.Antecedent(np.arange(0, obst_max_speed, 0.1), 'obstacle_velocity_b')
    # obstacle_velocity_c = ctrl.Antecedent(np.arange(0, obst_max_speed, 0.1), 'obstacle_velocity_c')
    alpha = ctrl.Consequent(np.arange(0, 1, 0.1), 'alpha') 
    beta = ctrl.Consequent(np.arange(0, 1, 0.1), 'beta')
    gamma = ctrl.Consequent(np.arange(0, 1, 0.1), 'gamma')

    # Define parameters for the sigmoidal membership functions
    # midpoint_1 = safety_distance/2
    # midpoint_2 = midpoint_1 + 1.0
    # midpoint_3 = (midpoint_2 - midpoint_1)/2 + midpoint_1

    # Definindo as funções de pertinência para a distância ao obstáculo
    # distancia_to_obstacle['proxima'] = fuzz.sigmf(distancia_to_obstacle.universe, midpoint_1, -safety_distance)
    # distancia_to_obstacle['media'] = fuzz.trimf(distancia_to_obstacle.universe, [midpoint_3 - 0.5, midpoint_3, midpoint_3 + 0.5])
    # distancia_to_obstacle['distante'] = fuzz.sigmf(distancia_to_obstacle.universe, midpoint_2, safety_distance)

    distancia_to_obstacle['proxima'] = fuzz.sigmf(distancia_to_obstacle.universe, 2, -safety_distance)
    distancia_to_obstacle['media'] = fuzz.trimf(distancia_to_obstacle.universe, [1.7, 2.0, 2.5])
    # distancia_to_obstacle['distante'] = fuzz.sigmf(distancia_to_obstacle.universe, 2.5, safety_distance)
    distancia_to_obstacle['distante'] = fuzz.sigmf(distancia_to_obstacle.universe, 3.0, safety_distance)

    # Definindo as funções de pertinência para a velocidade do obstáculo
    obstacle_velocity['estatico'] = fuzz.trimf(obstacle_velocity.universe, [-obst_max_speed, -obst_max_speed, 1.5])
    obstacle_velocity['lento'] = fuzz.trimf(obstacle_velocity.universe, [1.0, 1.5, 2.5])
    obstacle_velocity['rapido'] = fuzz.trimf(obstacle_velocity.universe, [2.0, obst_max_speed, obst_max_speed])

    # obstacle_velocity_b['estatico'] = fuzz.trimf(obstacle_velocity_b.universe, [0, 0, 1.5])
    # obstacle_velocity_b['lento'] = fuzz.trimf(obstacle_velocity_b.universe, [1.0, 1.5, 2.5])
    # obstacle_velocity_b['rapido'] = fuzz.trimf(obstacle_velocity_b.universe, [2.0, obst_max_speed, obst_max_speed])

    # obstacle_velocity_c['estatico'] = fuzz.trimf(obstacle_velocity_c.universe, [0, 0, 1.5])
    # obstacle_velocity_c['lento'] = fuzz.trimf(obstacle_velocity_c.universe, [1.0, 1.5, 2.5])
    # obstacle_velocity_c['rapido'] = fuzz.trimf(obstacle_velocity_c.universe, [2.0, obst_max_speed, obst_max_speed])

    # Definindo as funções de pertinência para o espaço de manobra
    # maneuvering_space['limitado'] = fuzz.sigmf(maneuvering_space.universe, 50, -0.1)
    # maneuvering_space['amplo'] = fuzz.sigmf(maneuvering_space.universe, 75, 0.1)

    maneuvering_space['amplo'] = fuzz.sigmf(maneuvering_space.universe, 50, -0.1)
    maneuvering_space['limitado'] = fuzz.sigmf(maneuvering_space.universe, 75, 0.1)

    # Definindo as funções de pertinência para os valores de saída
    alpha['baixo'] = fuzz.trimf(alpha.universe, [0, 0, 0.6])
    alpha['medio'] = fuzz.trimf(alpha.universe, [0.5, 0.7, 0.7])
    alpha['alto'] = fuzz.trimf(alpha.universe, [0.7, 1, 1])

    beta['baixo'] = fuzz.trimf(beta.universe, [0, 0, 0.6])
    beta['medio'] = fuzz.trimf(beta.universe, [0.5, 0.7, 0.7])
    beta['alto'] = fuzz.trimf(beta.universe, [0.7, 1, 1])

    gamma['baixo'] = fuzz.trimf(gamma.universe, [0, 0, 0.5])
    gamma['medio'] = fuzz.trimf(gamma.universe, [0.4, 0.7, 0.7])
    gamma['alto'] = fuzz.trimf(gamma.universe, [0.7, 1, 1])

    # # Plot graphs of relevance (optional)
    # distancia_to_obstacle.view()
    # obstacle_velocity.view()
    # # obstacle_velocity_a.view()
    # # obstacle_velocity_b.view()
    # # obstacle_velocity_c.view()
    # maneuvering_space.view()
    # alpha.view()
    # beta.view()
    # gamma.view()

    # Define fuzzy rules
    rules.append(ctrl.Rule(distancia_to_obstacle['proxima'] & maneuvering_space['limitado'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), alpha['baixo']))
    rules.append(ctrl.Rule(distancia_to_obstacle['proxima'] & maneuvering_space['limitado'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), beta['alto']))
    rules.append(ctrl.Rule(distancia_to_obstacle['proxima'] & maneuvering_space['limitado'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), gamma['baixo']))
    rules.append(ctrl.Rule(distancia_to_obstacle['proxima'] & maneuvering_space['limitado'] & obstacle_velocity['rapido'], alpha['baixo']))
    rules.append(ctrl.Rule(distancia_to_obstacle['proxima'] & maneuvering_space['limitado'] & obstacle_velocity['rapido'], beta['alto']))
    rules.append(ctrl.Rule(distancia_to_obstacle['proxima'] & maneuvering_space['limitado'] & obstacle_velocity['rapido'], gamma['medio']))
    rules.append(ctrl.Rule(distancia_to_obstacle['proxima'] & maneuvering_space['amplo'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), alpha['baixo']))
    rules.append(ctrl.Rule(distancia_to_obstacle['proxima'] & maneuvering_space['amplo'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), beta['alto']))
    rules.append(ctrl.Rule(distancia_to_obstacle['proxima'] & maneuvering_space['amplo'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), gamma['baixo']))
    rules.append(ctrl.Rule(distancia_to_obstacle['proxima'] & maneuvering_space['amplo'] & obstacle_velocity['rapido'], alpha['baixo']))
    rules.append(ctrl.Rule(distancia_to_obstacle['proxima'] & maneuvering_space['amplo'] & obstacle_velocity['rapido'], beta['alto']))
    rules.append(ctrl.Rule(distancia_to_obstacle['proxima'] & maneuvering_space['amplo'] & obstacle_velocity['rapido'], gamma['medio']))

    rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['limitado'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), alpha['medio']))
    rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['limitado'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), beta['medio']))
    rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['limitado'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), gamma['baixo']))
    rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['limitado'] & obstacle_velocity['rapido'], alpha['medio']))
    rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['limitado'] & obstacle_velocity['rapido'], beta['medio']))
    rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['limitado'] & obstacle_velocity['rapido'], gamma['medio']))
    rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['amplo'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), alpha['alto']))
    rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['amplo'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), beta['medio']))
    rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['amplo'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), gamma['medio']))
    # rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['amplo'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), gamma['baixo']))
    rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['amplo'] & obstacle_velocity['rapido'], alpha['alto']))
    rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['amplo'] & obstacle_velocity['rapido'], beta['medio']))
    rules.append(ctrl.Rule(distancia_to_obstacle['media'] & maneuvering_space['amplo'] & obstacle_velocity['rapido'], gamma['medio']))

    rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['amplo'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), alpha['alto']))
    rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['amplo'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), beta['baixo']))
    # rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['amplo'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), gamma['alto']))
    rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['amplo'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), gamma['medio']))
    rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['limitado'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), alpha['alto']))
    rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['limitado'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), beta['baixo']))
    # rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['limitado'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), gamma['medio']))
    rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['limitado'] & (obstacle_velocity['estatico'] | obstacle_velocity['lento']), gamma['baixo']))
    rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['amplo'] & (obstacle_velocity['rapido']), alpha['alto']))
    rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['amplo'] & (obstacle_velocity['rapido']), beta['baixo']))
    # rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['amplo'] & (obstacle_velocity['rapido']), gamma['alto']))
    rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['amplo'] & (obstacle_velocity['rapido']), gamma['medio']))
    rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['limitado'] & (obstacle_velocity['rapido']), alpha['alto']))
    rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['limitado'] & (obstacle_velocity['rapido']), beta['baixo']))
    rules.append(ctrl.Rule(distancia_to_obstacle['distante'] & maneuvering_space['limitado'] & (obstacle_velocity['rapido']), gamma['medio']))

    # Fuzzy controller
    controlador_velocidade = ctrl.ControlSystem(rules)
    simulator = ctrl.ControlSystemSimulation(controlador_velocidade)

    # Input values for simulation
    simulator.input['distance_to_obstacle'] = dist_input
    simulator.input['obstacle_velocity'] = obst_input
    # simulator.input['obstacle_velocity_a'] = obst_input[0]
    # simulator.input['obstacle_velocity_b'] = obst_input[1]
    # simulator.input['obstacle_velocity_c'] = obst_input[2]
    simulator.input['maneuvering_space'] = space_input

    # Simulation
    simulator.compute()

    # Outputs
    alpha_output = simulator.output['alpha']
    beta_output = simulator.output['beta']
    gamma_output = simulator.output['gamma']

    # # Plot dos gráficos de pertinência (opcional)
    # alpha.view(simulator)
    # beta.view(simulator)
    # gamma.view(simulator)

    return alpha_output, beta_output, gamma_output

# # Exemplo de uso
# dist_input = 3.871997928619385  # Distância ao obstáculo em m
# obst_input = -0.5675925630511488 # Velocidade do obstáculo em m/s
# safety_distance = 5.0 # 3
# # parts = [3.0, 3.0, 3.0]

# # space_input = fuzzy_logic_maneuvering_space(parts, safety_distance)
# space_input = 28.12013594785211
# alpha, beta, gamma = fuzzy_logic(dist_input, obst_input, space_input, safety_distance)
# print(f"Alpha: {alpha:.2f}, Beta: {beta:.2f}, Gamma: {gamma:.2f}")


# # Exemplo de uso
# parts = [1.0, 1.0, 3.0]
# safety_distance = 3.0

# proximity = fuzzy_logic_maneuvering_space(parts, safety_distance)
# print(f"Proximity: {proximity:.2f}")