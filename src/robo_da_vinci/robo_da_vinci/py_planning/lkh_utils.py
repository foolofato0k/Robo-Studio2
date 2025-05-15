# lkh_utils.py

import os
#import lkh
import math
import tempfile
import subprocess

def euclidean_distance(p1, p2):
    return int(round(math.hypot(p1[0] - p2[0], p1[1] - p2[1])))

def generate_distance_matrix(strokes):
    n = len(strokes)
    matrix = []
    for i in range(n):
        row = []
        for j in range(n):
            if i != j:
                # Distance from the end of stroke i to the start of stroke j
                x1, y1 = strokes[i][2], strokes[i][3]  # end of stroke i
                x0, y0 = strokes[j][0], strokes[j][1]  # start of stroke j
                row.append(euclidean_distance((x1, y1), (x0, y0)))
            else:
                row.append(0)  # Distance to itself is 0
        matrix.append(row)
    return matrix

def generate_tsp_content(matrix, name="StrokeDrawing"):
    n = len(matrix)
    lines = [
        f"NAME: {name}",
        "TYPE: TSP",
        f"DIMENSION: {n}",
        "EDGE_WEIGHT_TYPE: EXPLICIT",
        "EDGE_WEIGHT_FORMAT: FULL_MATRIX",
        "EDGE_WEIGHT_SECTION"
    ]
    for row in matrix:
        lines.append(" ".join(str(int(v)) for v in row))
    lines.append("EOF")
    return "\n".join(lines)

def solve_stroke_order_py(strokes, runs=50, solver_path='/usr/local/bin/LKH'):
    matrix = generate_distance_matrix(strokes)
    tsp_text = generate_tsp_content(matrix)
    print("TSP file content:\n", tsp_text)

    with tempfile.TemporaryDirectory() as tmpdir:
        tsp_path = os.path.join(tmpdir, "problem.tsp")
        par_path = os.path.join(tmpdir, "params.par")
        tour_path = os.path.join(tmpdir, "output.tour")

        with open(tsp_path, 'w') as f:
            f.write(tsp_text)

        with open(par_path, 'w') as f:
            f.write(f"PROBLEM_FILE = {tsp_path}\n")
            f.write(f"TOUR_FILE = {tour_path}\n")
            f.write(f"RUNS = {runs}\n")
            f.write("TRACE_LEVEL = 1\n")
            f.write("INITIAL_TOUR_ALGORITHM = GREEDY\n")
            f.write("KICK_TYPE = 4\n")
            f.write("KICKS = 10\n")
            f.write("MAX_TRIALS = 50\n")
            f.write("MAX_SWAPS = 20\n")
            f.write("TRACE_LEVEL = 2\n")


        result = subprocess.check_output([solver_path, par_path]).decode()
        print("LKH Output:\n", result)

        # Read and parse the output tour file
        tour = []
        with open(tour_path, 'r') as f:
            reading = False
            for line in f:
                if line.strip() == "TOUR_SECTION":
                    reading = True
                    continue
                if reading:
                    if line.strip() == "-1":
                        break
                    tour.append(int(line.strip()) - 1)  # 1-indexed to 0-indexed

    return tour
# Example usage:
if __name__ == "__main__":
    # Example strokes: (x0, y0, x1, y1)
    strokes = [
        (10, 10, 20, 30),
        (30, 30, 40, 40),
        (50, 50, 60, 60),
        (20, 20, 40, 60),
        (70, 70, 90, 90),
        (80, 80, 100, 100)
    ]

    order = solve_stroke_order_py(strokes,50)
    
    print("Optimal stroke order:", order)