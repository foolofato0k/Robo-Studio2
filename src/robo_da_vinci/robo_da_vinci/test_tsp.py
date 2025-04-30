import random
from robo_da_vinci.lkh_utils import solve_stroke_order_py
import matplotlib.pyplot as plt

def generate_fake_strokes(num_strokes=30, canvas_size=(1000, 1000), seed=None):
    if seed is not None:
        random.seed(seed)

    strokes = []
    for _ in range(num_strokes):
        x1 = random.uniform(0, canvas_size[0])
        y1 = random.uniform(0, canvas_size[1])
        x2 = random.uniform(0, canvas_size[0])
        y2 = random.uniform(0, canvas_size[1])
        strokes.append((x1, y1, x2, y2))  # flattened tuple

    return strokes

def plot_strokes_ordered(strokes, tsp_solution, title, show_index=True, optimized=False):
    plt.figure(figsize=(10, 10))
    
    # Plot the strokes in the given order (original or optimized)
    for i, idx in enumerate(tsp_solution):
        x1, y1, x2, y2 = strokes[idx]
        xs = [x1, x2]
        ys = [y1, y2]
        plt.plot(xs, ys, marker='o')
        if show_index:
            plt.text(x1, y1, str(i), fontsize=8, color='red')
    
    # Plot dashed lines between consecutive strokes in the given order
    for i in range(len(tsp_solution) - 1):
        _, _, x1_end, y1_end = strokes[tsp_solution[i]]
        x2_start, y2_start, _, _ = strokes[tsp_solution[i+1]]
        plt.plot([x1_end, x2_start], [y1_end, y2_start], 'k--', linewidth=0.5, alpha=0.5)
    
    plt.gca().invert_yaxis()
    plt.axis('equal')
    plt.title(title)
    plt.show()

# Generate fake strokes
strokes = generate_fake_strokes(num_strokes=90, canvas_size=(800, 600), seed=52)

# Solve the TSP for optimized stroke order
order_optimized = solve_stroke_order_py(strokes, runs=50)

# Plot original order (not optimized)
order_original = list(range(len(strokes)))  # Original order is just 0, 1, 2, ...
plot_strokes_ordered(strokes, order_original, title="Original Stroke Order", optimized=False)

# Plot optimized order
plot_strokes_ordered(strokes, order_optimized, title="TSP-Optimized Stroke Order", optimized=True)

print("Original stroke order:", order_original)
print("Optimal stroke order:", order_optimized)