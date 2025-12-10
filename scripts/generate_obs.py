import yaml
import random

def generate_random_cube(map_x_min, map_x_max, map_y_min, map_y_max, map_z_min, map_z_max,
                         x_len_min, x_len_max, y_len_min, y_len_max, z_len_min, z_len_max):
    # Generate random sizes for the cube
    x_len = random.uniform(x_len_min, x_len_max)
    y_len = random.uniform(y_len_min, y_len_max)
    z_len = random.uniform(z_len_min, z_len_max)

    # Ensure the cube fits within the map bounds
    cx_min = max(map_x_min + x_len / 2, map_x_min)
    cx_max = min(map_x_max - x_len / 2, map_x_max)
    cy_min = max(map_y_min + y_len / 2, map_y_min)
    cy_max = min(map_y_max - y_len / 2, map_y_max)
    cz_min = max(map_z_min + z_len / 2, map_z_min)
    cz_max = min(map_z_max - z_len / 2, map_z_max)

    if cx_min > cx_max or cy_min > cy_max or cz_min > cz_max:
        return None  # Cube cannot fit within the map

    c_x = random.uniform(cx_min, cx_max)
    c_y = random.uniform(cy_min, cy_max)
    c_z = random.uniform(cz_min, cz_max)

    x_min = c_x - x_len / 2
    x_max = c_x + x_len / 2
    y_min = c_y - y_len / 2
    y_max = c_y + y_len / 2
    z_min = c_z - z_len / 2
    z_max = c_z + z_len / 2

    # Build halfspace representation matrix
    cube_constraints = [
        [1, 0, 0, -x_max],
        [-1, 0, 0, x_min],
        [0, 1, 0, -y_max],
        [0, -1, 0, y_min],
        [0, 0, 1, -z_max],
        [0, 0, -1, z_min]
    ]
    return cube_constraints


def generate_cubes_config(num_cubes, map_size_x, map_size_y, map_size_z, map_origin_x, map_origin_y, map_origin_z,
                          x_len_min, x_len_max, y_len_min, y_len_max, z_len_min, z_len_max):
    cubes = []
    for _ in range(num_cubes):
        cube = generate_random_cube(
            map_x_min=map_origin_x,
            map_x_max=map_origin_x + map_size_x,
            map_y_min=map_origin_y,
            map_y_max=map_origin_y + map_size_y,
            map_z_min=map_origin_z,
            map_z_max=map_origin_z + map_size_z,
            x_len_min=x_len_min,
            x_len_max=x_len_max,
            y_len_min=y_len_min,
            y_len_max=y_len_max,
            z_len_min=z_len_min,
            z_len_max=z_len_max
        )
        if cube:  # Only add valid cubes
            cubes.append(cube)

    config = {'cubes': [[list(constraint) for constraint in cube] for cube in cubes]}
    return config


if __name__ == "__main__":
    # Define map parameters
    map_size_x = 100.0
    map_size_y = 100.0
    map_size_z = 10.0
    map_origin_x = -50.0
    map_origin_y = -50.0
    map_origin_z = 0.0

    # Define cube dimensions ranges
    x_len_min, x_len_max = 1, 3.0
    y_len_min, y_len_max = 1, 3.0
    z_len_min, z_len_max = 5.0, 10.0

    # Number of random cubes to generate
    num_cubes = 30

    # Generate the cubes configuration
    cubes_config = generate_cubes_config(
        num_cubes=num_cubes,
        map_size_x=map_size_x,
        map_size_y=map_size_y,
        map_size_z=map_size_z,
        map_origin_x=map_origin_x,
        map_origin_y=map_origin_y,
        map_origin_z=map_origin_z,
        x_len_min=x_len_min,
        x_len_max=x_len_max,
        y_len_min=y_len_min,
        y_len_max=y_len_max,
        z_len_min=z_len_min,
        z_len_max=z_len_max
    )

    # Save to YAML file
    with open('../cfg/obstacles.yaml', 'w') as file:
        yaml.dump(cubes_config, file, default_flow_style=None)

    print(f"Generated {len(cubes_config['cubes'])} random cubes and saved to '../cfg/obstacles.yaml'")