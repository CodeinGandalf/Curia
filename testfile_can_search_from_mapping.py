import yaml
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from scipy.ndimage import label, center_of_mass


def read_yaml(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data


def read_pgm_image(pgm_path):
    img = Image.open(pgm_path).convert('L')  # 'L' = 8-bit grayscale
    return np.array(img)


def convert_map_to_occupancy(grid, yaml_data):
    occupied_thresh = int(yaml_data.get('occupied_thresh', 0.65) * 255)
    free_thresh = int(yaml_data.get('free_thresh', 0.196) * 255)

    occupancy = np.full(grid.shape, -1, dtype=int)
    occupancy[grid <= free_thresh] = 0
    occupancy[grid >= occupied_thresh] = 1

    return occupancy


def add_circular_object(grid, center_x, center_y, radius_cells, value=2):
    height, width = grid.shape
    for y in range(-radius_cells, radius_cells + 1):
        for x in range(-radius_cells, radius_cells + 1):
            if x**2 + y**2 <= radius_cells**2:
                px = center_x + x
                py = center_y + y
                if 0 <= px < width and 0 <= py < height:
                    grid[py, px] = value
    return grid


def manipulate_map(grid, yaml_data, position='center'):
    grid_copy = grid.copy()

    if position == 'center':
        center_x = grid.shape[1] // 2 + 20
        center_y = grid.shape[0] // 2 - 20
    else:
        center_x = 10
        center_y = 10

    radius = 2  # entspricht ca. 2-3 Zellen im Durchmesser

    print(f"Zellenkoordinaten der Dose: ({center_x}, {center_y})")

    resolution = yaml_data['resolution']
    origin_x, origin_y = yaml_data['origin'][:2]

    world_x = origin_x + (center_x * resolution)
    world_y = origin_y + ((grid.shape[0] - center_y) * resolution)

    print(f'origin: {origin_x}, {origin_y} manipulated map.')
    print(f"Weltkoordinaten der Dose: ({world_x:.3f}, {world_y:.3f})")

    grid_with_dose = add_circular_object(grid_copy, center_x, center_y, radius)

    return grid_with_dose, (world_x, world_y)


def show_colored_map(grid):
    cmap = mcolors.ListedColormap(['lightgray', 'white', 'black', 'red'])
    bounds = [-1.5, -0.5, 0.5, 1.5, 2.5]
    norm = mcolors.BoundaryNorm(bounds, cmap.N)

    plt.imshow(grid, cmap=cmap, norm=norm)
    plt.title("Map mit Dose (rot = Dose)")
    plt.colorbar(ticks=[-1, 0, 1, 2])
    plt.show()


def crop_map(map_array, margin):
    return map_array[margin:-margin, margin:-margin]


def calculatePoseCan(map1_array, map2_array):
    resolution = 0.02
    margin = 0.2
    crop_cells = int(margin / resolution)

    map1_cropped = crop_map(map1_array, crop_cells)
    map2_cropped = crop_map(map2_array, crop_cells)

    old_origin_x1 = -13.8
    old_origin_y1 = -12.2
    old_origin_x2 = -13.8
    old_origin_y2 = -12.2

    new_origin_x1 = old_origin_x1 + margin
    new_origin_y1 = old_origin_y1 + margin
    new_origin_x2 = old_origin_x2 + margin
    new_origin_y2 = old_origin_y2 + margin

    diff_x = new_origin_x2 - new_origin_x1
    diff_y = new_origin_y2 - new_origin_y1

    min_origin_x = max(new_origin_x1, new_origin_x2)
    min_origin_y = max(new_origin_y1, new_origin_y2)

    if diff_x != 0 or diff_y != 0:
        offset_x2 = abs(new_origin_x2 - min_origin_x)
        offset_y2 = abs(new_origin_y2 - min_origin_y)

        offset_cells_x2 = int(round(offset_x2 / resolution))
        offset_cells_y2 = int(round(offset_y2 / resolution))

        if offset_cells_y2 == 0:
            map2_cropped = map2_cropped[:, offset_cells_x2:]
        else:
            map2_cropped = map2_cropped[:-offset_cells_y2, offset_cells_x2:]

    final_width = min(map1_cropped.shape[1], map2_cropped.shape[1])
    final_height = min(map1_cropped.shape[0], map2_cropped.shape[0])

    if map1_cropped.shape != map2_cropped.shape:
        map1_cropped = map1_cropped[-final_height:, :final_width]
        map2_cropped = map2_cropped[-final_height:, :final_width]

    mask = map2_cropped > map1_cropped
    structure = np.ones((3, 3), dtype=int)
    labeled_mask, num_features = label(mask, structure=structure)

    world_x = []
    world_y = []
    area = []

    for i in range(1, num_features + 1):
        region = (labeled_mask == i)
        check = np.sum(region)*resolution**2*10000
        if check >= 36 and check <= 64:
            area.append(np.sum(region)*resolution**2*10000)
            centroid = center_of_mass(region)
            y_idx, x_idx = centroid
            world_x.append(min_origin_x + x_idx * resolution)
            world_y.append(min_origin_y + (final_height - y_idx) * resolution)

    return list(zip(world_x, world_y, area))


def add_noise_to_map(occupancy_grid, noise_ratio=0.01, seed=None):
    """
    Füge Rauschen zu einer Karte hinzu, indem ein kleiner Prozentsatz der Zellen zufällig verändert wird.
    :param occupancy_grid: Die Karte als numpy-Array.
    :param noise_ratio: Anteil der zu verändernden Pixel (z.B. 0.01 = 1%).
    :param seed: Optionaler Seed für Reproduzierbarkeit.
    :return: Neue Karte mit Rauschen.
    """
    if seed is not None:
        np.random.seed(seed)

    noisy_grid = occupancy_grid.copy()
    num_cells = occupancy_grid.size
    num_noisy = int(noise_ratio * num_cells)

    indices = np.unravel_index(np.random.choice(
        num_cells, num_noisy, replace=False), occupancy_grid.shape)
    for y, x in zip(*indices):
        current = noisy_grid[y, x]
        if current == 0:
            noisy_grid[y, x] = -1  # frei → unbekannt
        elif current == 1:
            noisy_grid[y, x] = 0   # belegt → frei
        elif current == -1:
            noisy_grid[y, x] = 0   # unbekannt → frei
    return noisy_grid


def main():
    yaml_file = 'test_map.yaml'
    yaml_data = read_yaml(yaml_file)
    pgm_file = yaml_data['image']
    pgm_data = read_pgm_image(pgm_file)

    occupancy_grid = convert_map_to_occupancy(pgm_data, yaml_data)

    height, width = occupancy_grid.shape

    occupancy_grid_manipulated, placed_can_world = manipulate_map(
        occupancy_grid, yaml_data)

    # show_colored_map(occupancy_grid_manipulated)

    # Füge Rauschen zur manipulierten Map hinzu
    noisy_manipulated_grid = add_noise_to_map(
        occupancy_grid_manipulated, noise_ratio=0.01, seed=42)

    # Zeige auch verrauschte Map
    # show_colored_map(noisy_manipulated_grid)
    fig = plt.figure(1)
    plt.imshow(occupancy_grid, cmap='gray')
    plt.title("occupancy_grid Cropping")
    plt.colorbar()

    crop_x = 100
    crop_y = 100
    # occupancy_grid = occupancy_grid[:-crop_y, :]

    # origin im YAML-Dict anpassen (nur x- und y-Werte)
    yaml_data['origin'][0] += crop_x * yaml_data['resolution']
    yaml_data['origin'][1] += crop_y * yaml_data['resolution']

    fig = plt.figure(2)
    plt.imshow(occupancy_grid, cmap='gray')
    plt.title("occupancy_grid nach Cropping")
    plt.colorbar()
    plt.show()

    pose = calculatePoseCan(occupancy_grid, noisy_manipulated_grid)

    x_world = None
    y_world = None
    area = None

    if pose:
        for pos in pose:
            if pos[2] >= 36 and pos[2] <= 64:
                # Finde größte Fläche = Dose
                x_world, y_world, area = pos

        if x_world:
            resolution = yaml_data['resolution']
            origin_x, origin_y = yaml_data['origin'][:2]

            x_cell = int(round((x_world - origin_x) / resolution))
            y_cell = int(round((y_world - origin_y) / resolution))

            """
            # Plot grünes X auf manipulierte Map
            fig = plt.figure()
            plt.imshow(occupancy_grid_manipulated, cmap='gray')
            plt.plot(x_cell, y_cell, 'gx', markersize=12, markeredgewidth=2)
            plt.title("Dose erkannt (grünes X)")
            plt.show()
            """

            # Zeige Abweichung (für Evaluation)
            placed_x, placed_y = placed_can_world
            dx = x_world - placed_x
            dy = y_world - placed_y
            dist = np.hypot(dx, dy)

            print(f"\nPlatzierte Dose: ({placed_x:.3f}, {placed_y:.3f})")
            print(f"Erkannte Dose:   ({x_world:.3f}, {y_world:.3f})")
            print(
                f"Abweichung: Δx = {dx:.3f} m, Δy = {dy:.3f} m, Distanz = {dist:.3f} m")
        else:
            print("Keine Dose gefunden / zu viel Rauschen")
    else:
        print("Keine Dose gefunden.")
    print(pose)


if __name__ == '__main__':
    main()
