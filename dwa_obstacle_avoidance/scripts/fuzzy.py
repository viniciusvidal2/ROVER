import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

def fuzzyLogicManeuveringSpace(space_parts, safety_distance):
    """Find the occupancy rate of the obstacle area to discover the available maneuvering space.

    Args:
        space_parts (list): mean distance values of parts that divide the space.
        safety_distance (float): safety distance that define obstacle influence area.

    Returns:
        occupancy_rate_output (float): occupancy rate of the obstacle area."""

    rules = []

    # Define fuzzy variables, each variable is created as Antecedent (input) or Consequent (output).
    distancia_a = ctrl.Antecedent(
        np.arange(0, safety_distance, 0.1), 'distance_a')
    distancia_b = ctrl.Antecedent(
        np.arange(0, safety_distance, 0.1), 'distance_b')
    distancia_c = ctrl.Antecedent(
        np.arange(0, safety_distance, 0.1), 'distance_c')
    occupancy_rate = ctrl.Consequent(np.arange(0, 100, 1), 'occupancy_rate')

    # Define parameters for the sigmoidal membership functions
    midpoint_1 = safety_distance/2
    midpoint_2 = midpoint_1 + 1.0

    # Define the sigmoidal membership functions for the distance to the obstacle
    # ajusta o ponto médio e a inclinação -10
    distancia_a['near'] = fuzz.sigmf(
        distancia_a.universe, midpoint_1, -safety_distance)
    distancia_a['far'] = fuzz.sigmf(
        distancia_a.universe, midpoint_2, safety_distance)

    distancia_b['near'] = fuzz.sigmf(
        distancia_b.universe, midpoint_1, -safety_distance)
    distancia_b['far'] = fuzz.sigmf(
        distancia_b.universe, midpoint_2, safety_distance)

    distancia_c['near'] = fuzz.sigmf(
        distancia_c.universe, midpoint_1, -safety_distance)
    distancia_c['far'] = fuzz.sigmf(
        distancia_c.universe, midpoint_2, safety_distance)

    # Define the sigmoidal membership functions for the occupancy rate
    occupancy_rate['wide'] = fuzz.sigmf(occupancy_rate.universe, 50, -0.1)
    occupancy_rate['limited'] = fuzz.sigmf(occupancy_rate.universe, 75, 0.1)

    # # Plot graphs of relevance (optional)
    # distancia_a.view()
    # distancia_b.view()
    # distancia_c.view()
    # occupancy_rate.view()

    # Fuzzy rules
    rules.append(ctrl.Rule(distancia_a['near'] & distancia_b['near']
                 & distancia_c['near'], occupancy_rate['limited']))
    # rule2 = ctrl.Rule(distancia_a['near'] & distancia_b['far'] & distancia_c['near'], occupancy_rate['limited'])
    rules.append(ctrl.Rule(distancia_a['near'] & distancia_b['near']
                 & distancia_c['far'], occupancy_rate['limited']))
    rules.append(ctrl.Rule(distancia_a['far'] & distancia_b['near']
                 & distancia_c['near'], occupancy_rate['limited']))
    rules.append(ctrl.Rule(distancia_a['far'] & distancia_b['near']
                 & distancia_c['far'], occupancy_rate['limited']))

    rules.append(ctrl.Rule(
        distancia_a['near'] & distancia_b['far'] & distancia_c['near'], occupancy_rate['wide']))
    rules.append(ctrl.Rule(
        distancia_a['near'] & distancia_b['far'] & distancia_c['far'], occupancy_rate['wide']))
    rules.append(ctrl.Rule(
        distancia_a['far'] & distancia_b['far'] & distancia_c['near'], occupancy_rate['wide']))
    rules.append(ctrl.Rule(
        distancia_a['far'] & distancia_b['far'] & distancia_c['far'], occupancy_rate['wide']))

    # Fuzzy controller
    occupancy_rate_indicator = ctrl.ControlSystem(rules)
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


def fuzzyLogic(dist_input, space_input, safety_distance):
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
    # Represent the robot distance to the obstacle
    distancia_to_obstacle = ctrl.Antecedent(
        np.arange(0, safety_distance, 0.1), 'distance_to_obstacle')
    maneuvering_space = ctrl.Antecedent(
        np.arange(0, 100, 0.1), 'maneuvering_space')
    alpha = ctrl.Consequent(np.arange(0, 1, 0.1), 'alpha')
    beta = ctrl.Consequent(np.arange(0, 1, 0.1), 'beta')
    gamma = ctrl.Consequent(np.arange(0, 1, 0.1), 'gamma')

    # Define parameters for the membership functions
    distancia_to_obstacle['near'] = fuzz.sigmf(
        distancia_to_obstacle.universe, 2, -safety_distance)
    distancia_to_obstacle['medium'] = fuzz.trimf(
        distancia_to_obstacle.universe, [1.7, 2.0, 2.5])
    distancia_to_obstacle['distant'] = fuzz.sigmf(
        distancia_to_obstacle.universe, 3.0, safety_distance)

    maneuvering_space['wide'] = fuzz.sigmf(
        maneuvering_space.universe, 50, -0.1)
    maneuvering_space['limited'] = fuzz.sigmf(
        maneuvering_space.universe, 75, 0.1)

    # Definindo as funções de pertinência para os valores de saída
    alpha['low'] = fuzz.trimf(alpha.universe, [0, 0, 0.6])
    alpha['medium'] = fuzz.trimf(alpha.universe, [0.5, 0.7, 0.7])
    alpha['high'] = fuzz.trimf(alpha.universe, [0.7, 1, 1])

    beta['low'] = fuzz.trimf(beta.universe, [0, 0, 0.6])
    beta['medium'] = fuzz.trimf(beta.universe, [0.5, 0.7, 0.7])
    beta['high'] = fuzz.trimf(beta.universe, [0.7, 1, 1])

    gamma['low'] = fuzz.trimf(gamma.universe, [0, 0, 0.5])
    gamma['medium'] = fuzz.trimf(gamma.universe, [0.4, 0.7, 0.7])
    gamma['high'] = fuzz.trimf(gamma.universe, [0.7, 1, 1])

    # # Plot graphs of relevance (optional)
    # distancia_to_obstacle.view()
    # maneuvering_space.view()
    # alpha.view()
    # beta.view()
    # gamma.view()

    # Define fuzzy rules
    rules.append(ctrl.Rule(
        distancia_to_obstacle['near'] & maneuvering_space['limited'], alpha['low']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['near'] & maneuvering_space['limited'], beta['high']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['near'] & maneuvering_space['limited'], gamma['low']))
    # rules.append(ctrl.Rule(distancia_to_obstacle['near'] & maneuvering_space['wide'], alpha['low']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['near'] & maneuvering_space['wide'], alpha['medium']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['near'] & maneuvering_space['wide'], beta['high']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['near'] & maneuvering_space['wide'], gamma['low']))

    rules.append(ctrl.Rule(
        distancia_to_obstacle['medium'] & maneuvering_space['limited'], alpha['medium']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['medium'] & maneuvering_space['limited'], beta['medium']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['medium'] & maneuvering_space['limited'], gamma['low']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['medium'] & maneuvering_space['wide'], alpha['high']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['medium'] & maneuvering_space['wide'], beta['low']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['medium'] & maneuvering_space['wide'], gamma['low']))

    rules.append(ctrl.Rule(
        distancia_to_obstacle['distant'] & maneuvering_space['limited'], alpha['high']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['distant'] & maneuvering_space['limited'], beta['low']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['distant'] & maneuvering_space['limited'], gamma['low']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['distant'] & maneuvering_space['wide'], alpha['high']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['distant'] & maneuvering_space['wide'], beta['low']))
    rules.append(ctrl.Rule(
        distancia_to_obstacle['distant'] & maneuvering_space['wide'], gamma['low']))

    # Fuzzy controller
    controlador_velocidade = ctrl.ControlSystem(rules)
    simulator = ctrl.ControlSystemSimulation(controlador_velocidade)

    # Input values for simulation
    simulator.input['distance_to_obstacle'] = dist_input
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

# # space_input = fuzzyLogicManeuveringSpace(parts, safety_distance)
# space_input = 28.12013594785211
# alpha, beta, gamma = fuzzyLogic(dist_input, space_input, safety_distance)
# print(f"Alpha: {alpha:.2f}, Beta: {beta:.2f}, Gamma: {gamma:.2f}")


# # Exemplo de uso
# parts = [1.0, 1.0, 3.0]
# safety_distance = 5.0

# proximity = fuzzyLogicManeuveringSpace(parts, safety_distance)
# print(f"Proximity: {proximity:.2f}")
