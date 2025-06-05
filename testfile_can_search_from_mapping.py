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

    mask = map2_cropped < map1_cropped
    structure = structure = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]])

    labeled_mask, num_features = label(mask, structure=structure)

    world_x = []
    world_y = []
    area = []

    # Visualisierung der Differenzmaske
    plt.figure(figsize=(6, 6))
    plt.imshow(mask, cmap='gray')
    plt.title("Differenzmaske (mögliche neue Objekte)")
    plt.xlabel("X (Pixel)")
    plt.ylabel("Y (Pixel)")
    plt.grid(False)
    plt.show()

    for i in range(1, num_features + 1):
        region = (labeled_mask == i)
        check = np.sum(region)*resolution**2*10000
        if check >= 14 and check <= 18:
            area.append(np.sum(region)*resolution**2*10000)
            centroid = center_of_mass(region)
            y_idx, x_idx = centroid
            world_x.append(min_origin_x + x_idx * resolution)
            world_y.append(min_origin_y + (final_height - y_idx) * resolution)

    return list(zip(world_x, world_y, area))


def main():
    yaml_file_base = 'my_map_no_dose.yaml'
    yaml_data_base = read_yaml(yaml_file_base)
    pgm_file_base = yaml_data_base['image']
    pgm_data_base = read_pgm_image(pgm_file_base)

    occupancy_grid = convert_map_to_occupancy(pgm_data_base, yaml_data_base)

    yaml_file = 'my_map_dose.yaml'
    yaml_data = read_yaml(yaml_file)
    pgm_file = yaml_data['image']
    pgm_data = read_pgm_image(pgm_file)

    occupancy_grid_dose = convert_map_to_occupancy(pgm_data, yaml_data)

    pose = calculatePoseCan(occupancy_grid, occupancy_grid_dose)

    print(pose)

    x_world = None
    y_world = None
    area = None

    if pose:
        resolution = yaml_data['resolution']
        origin_x, origin_y = yaml_data['origin'][:2]
        detected_cans = []  # Liste zum Speichern der Zellenkoordinaten

        for pos in pose:
            x_world, y_world, area = pos

            # Weltkoordinaten → Zellenkoordinaten
            x_cell = int(round((x_world - origin_x) / resolution))
            y_cell = int(round((y_world - origin_y) / resolution))

            detected_cans.append((x_cell, y_cell))  # speichern

        if detected_cans:
            fig = plt.figure()
            plt.imshow(occupancy_grid_dose, cmap='gray')

            for (x_cell, y_cell) in detected_cans:
                plt.plot(x_cell, y_cell, 'gx',
                         markersize=12, markeredgewidth=2)

            plt.title("Alle erkannten Dosen (grünes X)")
            plt.show()
        else:
            print("Keine Dose mit gültiger Fläche erkannt.")
    else:
        print("Keine Dose gefunden.")


if __name__ == '__main__':
    main()
