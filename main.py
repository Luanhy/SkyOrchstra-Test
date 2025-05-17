import pybullet as p
import pybullet_data
import numpy as np
import time
import os



# ======= Formationsfunktionen =======

def linienformation(n, spacing, height):
    positions = []
    start_x = - (n - 1) / 2 * spacing
    for i in range(n):
        x = start_x + i * spacing
        y = 0
        z = height
        positions.append((x, y, z))
    return positions


def herzformation(n, scale, height):
    ts = np.linspace(0, 2 * np.pi, n, endpoint=False)
    coords = []
    for t in ts:
        x = 16 * np.sin(t) ** 3
        z = (13 * np.cos(t)
             - 5 * np.cos(2 * t)
             - 2 * np.cos(3 * t)
             - np.cos(4 * t))
        # X = Breite, Y = Tiefe (z.B. 0), Z = Höhe (durch Herzform + gewünschte Höhe)
        coords.append((x * scale, 0, z * scale + height))
    return coords


def quadratformation(num_drones, size, center):
    positions = []
    half = size / 2
    drones_per_side = num_drones // 4

    # Unten
    for i in range(drones_per_side):
        x = center[0] - half + i * (size / (drones_per_side - 1))
        z = center[2] - half
        positions.append([x, center[1], z])

    # Rechts
    for i in range(drones_per_side):
        x = center[0] + half
        z = center[2] - half + i * (size / (drones_per_side - 1))
        positions.append([x, center[1], z])

    # Oben
    for i in range(drones_per_side):
        x = center[0] + half - i * (size / (drones_per_side - 1))
        z = center[2] + half
        positions.append([x, center[1], z])

    # Links
    for i in range(drones_per_side):
        x = center[0] - half
        z = center[2] + half - i * (size / (drones_per_side - 1))
        positions.append([x, center[1], z])

    return positions


def kreisformation(n, radius, height, y_position=0, center=(0, 0)):
    positions = []
    angles = np.linspace(0, 2 * np.pi, n, endpoint=False)
    for angle in angles:
        x = center[0] + radius * np.cos(angle)     # X = Breite
        z = center[1] + radius * np.sin(angle)     # Z = relative Höhe im Kreis
        positions.append((x, y_position, z + height))  # Z + Höhe = absolute Höhe
    return positions


# ======= Hilfsfunktionen =======

def interpolate(current, target, step_size):
    current = np.array(current)
    target = np.array(target)
    direction = target - current
    dist = np.linalg.norm(direction)
    if dist < step_size:
        return target
    return current + direction / dist * step_size


def all_reached(drones, targets, threshold=0.05):
    for i, drone_id in enumerate(drones):
        pos, _ = p.getBasePositionAndOrientation(drone_id)
        if np.linalg.norm(np.array(pos) - np.array(targets[i])) > threshold:
            return False
    return True


def add_light_marker(position, color=(1, 1, 0, 1), radius=0.1):
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=color
    )
    body_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=position
    )
    return body_id





# ======= Hauptfunktion =======

def run_simulation():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    num_drones = 20
    drone_path = os.path.join("models", "simple_drone.urdf")

    # Startposition & Ziel-Formationen
    start_positions = linienformation(num_drones, spacing=1.5, height=0.5)
    target_herz = herzformation(num_drones, scale=0.15, height=3)
    target_quadrat = quadratformation(num_drones, size=3, center=[0, 0, 5])
    target_kreis = kreisformation(num_drones, radius=3, height=5)

    phase = "herz"  # Startphase

    # Drohnen erzeugen
    drones = []
    for i in range(num_drones):
        pos = start_positions[i]
        ori = p.getQuaternionFromEuler([0, 0, 0])
        drone_id = p.loadURDF(drone_path, basePosition=pos, baseOrientation=ori, useFixedBase=False)
        drones.append(drone_id)

    # Bewegung des Lichts
    lights_ids = []
    offset = (0, 0, 0.3)

    for drone in drones:
        pos, _ = p.getBasePositionAndOrientation(drone)
        light_pos = tuple(np.array(pos) + np.array(offset))
        light_id = add_light_marker(light_pos)
        lights_ids.append(light_id)


    # Simulationsschleife
    while True:
        # Zielpositionen je nach Phase
        if phase == "herz":
            current_targets = target_herz
        elif phase == "quadrat":
            current_targets = target_quadrat
        elif phase == "kreis":
            current_targets = target_kreis

        # Bewegung pro Drohne
        for i, drone_id in enumerate(drones):
            pos, ori = p.getBasePositionAndOrientation(drone_id)
            target = current_targets[i]
            new_pos = interpolate(pos, target, 0.05)
            p.resetBasePositionAndOrientation(drone_id, new_pos, ori)

        # Phasenwechsel
        if phase == "herz" and all_reached(drones, target_herz):
            phase = "quadrat"

        elif phase == "quadrat" and all_reached(drones, target_quadrat):
            phase = "kreis"

        #Bewegung pro Licht
        for i, drone in enumerate(drones):
            pos, _ = p.getBasePositionAndOrientation(drone)
            light_pos = tuple(np.array(pos) + np.array(offset))
            p.resetBasePositionAndOrientation(lights_ids[i], light_pos, [0, 0, 0, 1])

        p.stepSimulation()
        time.sleep(1.0 / 240)


# ======= Startpunkt =======

if __name__ == "__main__":
    run_simulation()
