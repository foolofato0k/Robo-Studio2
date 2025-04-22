import os
import numpy as np

def euclidean(p1, p2):
    return round(np.linalg.norm(np.array(p1) - np.array(p2)))

def build_cost_matrix(strokes):
    n = len(strokes)
    cost_matrix = np.zeros((2 * n, 2 * n), dtype=int)

    for i in range(n):
        i_fwd_end = strokes[i][1]
        i_rev_end = strokes[i][0]

        for j in range(n):
            if i == j:
                cost_matrix[2*i][2*j] = 999999
                cost_matrix[2*i][2*j+1] = 999999
                cost_matrix[2*i+1][2*j] = 999999
                cost_matrix[2*i+1][2*j+1] = 999999
                continue

            j_fwd_start = strokes[j][0]
            j_rev_start = strokes[j][1]

            cost_matrix[2*i][2*j] = euclidean(i_fwd_end, j_fwd_start)
            cost_matrix[2*i][2*j+1] = euclidean(i_fwd_end, j_rev_start)
            cost_matrix[2*i+1][2*j] = euclidean(i_rev_end, j_fwd_start)
            cost_matrix[2*i+1][2*j+1] = euclidean(i_rev_end, j_rev_start)

    return cost_matrix

def ensure_folder(folder="tsp_files"):
    if not os.path.exists(folder):
        os.makedirs(folder)
    return folder

def write_tsp_file(matrix, folder="tsp_files", filename="selfie_draw.tsp"):
    path = os.path.join(folder, filename)
    n = len(matrix)
    with open(path, "w") as f:
        f.write("NAME: selfie_draw\n")
        f.write("TYPE: ATSP\n")
        f.write(f"DIMENSION: {n}\n")
        f.write("EDGE_WEIGHT_TYPE: EXPLICIT\n")
        f.write("EDGE_WEIGHT_FORMAT: FULL_MATRIX\n")
        f.write("EDGE_WEIGHT_SECTION\n")
        for row in matrix:
            f.write(" ".join(map(str, row)) + "\n")
        f.write("EOF\n")

def write_par_file(folder="tsp_files", tsp_file="selfie_draw.tsp", filename="selfie_draw.par"):
    path = os.path.join(folder, filename)
    with open(path, "w") as f:
        f.write(f"PROBLEM_FILE = {tsp_file}\n")
        f.write("RUNS = 1\n")
        f.write("TRACE_LEVEL = 1\n")

def write_clusters(strokes, folder="tsp_files", filename="selfie_draw.clusters"):
    path = os.path.join(folder, filename)
    with open(path, "w") as f:
        f.write("GTSP_SETS:\n")
        for i in range(len(strokes)):
            f.write(f"{i+1} {2*i+1} {2*i+2}\n")  # 1-indexed
        f.write("END\n")

def generate_lkh_input(strokes):
    folder = ensure_folder()
    matrix = build_cost_matrix(strokes)
    write_tsp_file(matrix, folder)
    write_par_file(folder)
    write_clusters(strokes, folder)
    print(f"✅ Files generated in folder: {folder}/")

# Example usage
if __name__ == "__main__":
    # Example: 3 strokes with start/end coords
    strokes = [
        ((0, 0), (1, 1)),
        ((2, 2), (3, 1)),
        ((1, 3), (0, 2))
    ]
    generate_lkh_input(strokes)