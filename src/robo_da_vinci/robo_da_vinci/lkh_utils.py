import subprocess
import os
from pathlib import Path
from robo_da_vinci.lkh_file_generator import generate_lkh_input  # make sure your function is imported

# 🏗 Run this after generate_lkh_input(strokes)
def run_gtsp_to_tsp(folder="tsp_files", base_name="selfie_draw"):
    tsp_in = f"{base_name}.tsp"
    clusters = f"{base_name}.clusters"
    tsp_out = f"{base_name}_converted.tsp"
    full_cmd = ["LKH/GTSP2TSP", tsp_in, clusters, tsp_out]
    subprocess.run(full_cmd, cwd=folder, check=True)
    return tsp_out

def update_par_file_for_converted_tsp(folder="tsp_files", par_file="selfie_draw.par", new_tsp_file="selfie_draw_converted.tsp"):
    path = os.path.join(folder, par_file)
    lines = []
    with open(path, 'r') as f:
        for line in f:
            if line.startswith("PROBLEM_FILE"):
                lines.append(f"PROBLEM_FILE = {new_tsp_file}\n")
            else:
                lines.append(line)
    with open(path, 'w') as f:
        f.writelines(lines)

def run_lkh(folder="tsp_files", par_file="selfie_draw.par"):
    subprocess.run(["LKH", par_file], cwd=folder, check=True)

def parse_lkh_tour(folder="tsp_files", tour_file="selfie_draw_CONVERTED_LKH.sol"):
    path = os.path.join(folder, tour_file)
    with open(path, "r") as f:
        lines = f.readlines()
    reading = False
    tour = []
    for line in lines:
        if line.strip() == "TOUR_SECTION":
            reading = True
            continue
        if line.strip() == "-1":
            break
        if reading:
            tour.append(int(line.strip()) - 1)  # convert to 0-indexed
    return tour

# One function to run the full pipeline:
def solve_stroke_order(strokes, base_name="selfie_draw", folder="tsp_files"):
    generate_lkh_input(strokes)
    
    converted_tsp = run_gtsp_to_tsp(folder, base_name)
    update_par_file_for_converted_tsp(folder, f"{base_name}.par", converted_tsp)
    run_lkh(folder, f"{base_name}.par")

    # LKH outputs to `*_CONVERTED_LKH.sol` if input is `*_converted.tsp`
    tour = parse_lkh_tour(folder, f"{base_name.upper()}_CONVERTED_LKH.sol")
    return tour


if __name__ == "__main__":
    strokes = [
        ((0, 0), (1, 1)),
        ((2, 2), (3, 1)),
        ((1, 3), (0, 2))
    ]
    ordered = solve_stroke_order(strokes)
    for i, stroke in enumerate(ordered):
        print(f"Stroke {i+1}: {stroke}")