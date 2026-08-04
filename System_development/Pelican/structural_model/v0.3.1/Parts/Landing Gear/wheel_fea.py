import matplotlib.pyplot as plt
import numpy as np
import pygad

np.random.seed(42)

NUMBER_OF_SPOKES = 12
HUB_RADIUS_MM = 25.4 / 2
RIM_RADIUS_MM = 97.8 / 2
HUB_RIM_SPAN_MM = RIM_RADIUS_MM - HUB_RADIUS_MM

SPOKE_WIDTH_MM = 22.4

YOUNGS_MODULUS_PLA_MPA = 2300.0
ULTIMATE_STRESS_MPA = 50.0
SAFETY_FACTOR = 1.6
ALLOWABLE_STRESS_MPA = ULTIMATE_STRESS_MPA / SAFETY_FACTOR
DENSITY_PLA = 1.24e-3

FORCE_LBS = 15.0
TOTAL_FORCE_NEWTONS = FORCE_LBS * 4.44822
FORCE_PER_SPOKE_NEWTONS = TOTAL_FORCE_NEWTONS / (NUMBER_OF_SPOKES / 3.0)

TARGET_DEFLECTION_MM = 1

best_fitness_history = []
mean_fitness_history = []


# discretizes 400 points along the bezier centerline
def generate_bezier_centerline(
    control_x1,
    control_y1,
    control_x2,
    control_y2,
    span_mm=HUB_RIM_SPAN_MM,
    num_points=400,
):
    point_0 = np.array([0.0, 0.0])  # locked at wheel hub
    point_1 = np.array([control_x1, control_y1])  # control points
    point_2 = np.array([control_x2, control_y2])
    point_3 = np.array([span_mm, 0.0])  # locked at wheel rim

    time_steps = np.linspace(0.0, 1.0, num_points)[:, None]
    # discretized points following the centerline
    curve_points = (
        (1 - time_steps) ** 3 * point_0
        + 3 * (1 - time_steps) ** 2 * time_steps * point_1
        + 3 * (1 - time_steps) * time_steps**2 * point_2
        + time_steps**3 * point_3
    )
    return curve_points, [point_0, point_1, point_2, point_3]


# calculuates deflection, max stress, and mass
def generalized_spoke_mechanics(
    curve_points,
    thickness_root,
    thickness_tip,
    spoke_width,
    force_per_spoke,
    youngs_modulus=YOUNGS_MODULUS_PLA_MPA,
):
    num_segments = len(curve_points)
    thickness_profile = np.linspace(thickness_root, thickness_tip, num_segments)

    segment_thickness = thickness_profile[:-1]
    moment_of_inertia = spoke_width * segment_thickness**3 / 12.0
    cross_section_area = spoke_width * segment_thickness

    delta_x = np.diff(curve_points[:, 0])
    delta_y = np.diff(curve_points[:, 1])
    segment_lengths = np.sqrt(delta_x**2 + delta_y**2)

    y_deviations = curve_points[:-1, 1]
    bending_moments = force_per_spoke * y_deviations

    cosine_angles = delta_x / np.where(segment_lengths == 0, 1e-6, segment_lengths)
    axial_compressive_forces = force_per_spoke * cosine_angles

    # Castilagno's second theorem: https://en.wikipedia.org/wiki/Castigliano's_method
    deflection_spoke = np.sum(
        (force_per_spoke * y_deviations**2)
        / (youngs_modulus * moment_of_inertia)
        * segment_lengths
    )
    deflection_axial = np.sum(
        (force_per_spoke * cosine_angles**2)
        / (youngs_modulus * cross_section_area)
        * segment_lengths
    )

    total_deflection_mm = deflection_spoke + deflection_axial

    stress_axial = axial_compressive_forces / cross_section_area
    stress_bending = (
        np.abs(bending_moments) * (segment_thickness / 2.0) / moment_of_inertia
    )

    max_stress_mpa = np.max(stress_axial + stress_bending)

    total_length = np.sum(segment_lengths)
    average_thickness = (thickness_root + thickness_tip) / 2.0
    total_mass_grams = (
        average_thickness * spoke_width * total_length * DENSITY_PLA * NUMBER_OF_SPOKES
    )

    return total_deflection_mm, max_stress_mpa, total_mass_grams


def pygad_fitness(genetic_algorithm_instance, solution_array, solution_index):
    control_x1, control_y1, control_x2, control_y2, thickness_root, thickness_tip = (
        solution_array
    )

    if control_x1 >= control_x2:
        return -500.0

    curve_points, _ = generate_bezier_centerline(
        control_x1, control_y1, control_x2, control_y2
    )

    if np.any(np.diff(curve_points[:, 0]) <= 0.02):
        return -500.0

    deflection, max_stress, total_mass = generalized_spoke_mechanics(
        curve_points,
        thickness_root,
        thickness_tip,
        SPOKE_WIDTH_MM,
        force_per_spoke=FORCE_PER_SPOKE_NEWTONS,
    )

    deflection_error = abs(deflection - TARGET_DEFLECTION_MM) / TARGET_DEFLECTION_MM
    deflection_loss = 2000.0 * (deflection_error**2)

    mass_objective = total_mass / 50.0

    if max_stress > ALLOWABLE_STRESS_MPA:
        stress_penalty = 2000.0 * (max_stress / ALLOWABLE_STRESS_MPA - 1.0) ** 2
    else:
        stress_penalty = 0.0

    total_loss = deflection_loss + mass_objective + stress_penalty
    return -total_loss  # pygad sees loss as greater meaning better so function has to return negative values, e.g. -100 is more loss than -10 but a smaller number


def log_generation(genetic_algorithm_instance):
    fitness_scores = genetic_algorithm_instance.last_generation_fitness
    best_fitness_history.append(-np.max(fitness_scores))
    mean_fitness_history.append(-np.mean(fitness_scores))


# Geometric functions for plots
def rotate_points(points_array, angle_radians):
    cosine_val = np.cos(angle_radians)
    sine_val = np.sin(angle_radians)
    rotation_matrix = np.array([[cosine_val, -sine_val], [sine_val, cosine_val]])
    return points_array @ rotation_matrix.T


def thicken_tapered_curve(centerline_points, thickness_root, thickness_tip):
    num_points = len(centerline_points)
    thickness_profile = np.linspace(thickness_root, thickness_tip, num_points)[:, None]

    gradients = np.gradient(centerline_points, axis=0)
    normal_vectors = np.stack([-gradients[:, 1], gradients[:, 0]], axis=1)
    normal_vectors /= np.linalg.norm(normal_vectors, axis=1, keepdims=True)

    top_edge = centerline_points + normal_vectors * (thickness_profile / 2.0)
    bottom_edge = centerline_points - normal_vectors * (thickness_profile / 2.0)
    return np.vstack([top_edge, bottom_edge[::-1]])


def place_repeating_sector(local_polygon, hub_radius, sector_angle_degrees=0.0):
    shifted_polygon = local_polygon.copy()
    shifted_polygon[:, 0] += hub_radius
    return rotate_points(shifted_polygon, np.deg2rad(sector_angle_degrees))


if __name__ == "__main__":
    print(
        f"Running Co-Optimization for Shape & Thickness ({NUMBER_OF_SPOKES} Spokes)..."
    )
    print(f"Fixed Face Width: {SPOKE_WIDTH_MM} mm | Driving Variable Thickness Tracker")

    # maxes and mins for the possible parameters
    gene_search_space = [
        {"low": 5.0, "high": HUB_RIM_SPAN_MM * 0.45},
        {"low": -55.0, "high": 55.0},
        {"low": HUB_RIM_SPAN_MM * 0.55, "high": HUB_RIM_SPAN_MM},
        {"low": -55.0, "high": 55.0},
        {"low": 6.0, "high": 12.0},
        {"low": 1.5, "high": 3.5},
    ]

    genetic_algorithm_instance = pygad.GA(
        num_generations=1000,
        num_parents_mating=15,
        fitness_func=pygad_fitness,
        sol_per_pop=160,
        num_genes=6,
        gene_space=gene_search_space,
        parent_selection_type="tournament",
        K_tournament=4,
        crossover_type="uniform",
        mutation_type="random",
        mutation_probability=0.40,
        keep_elitism=3,
        on_generation=log_generation,
    )

    genetic_algorithm_instance.run()

    best_solution, best_solution_fitness, _ = genetic_algorithm_instance.best_solution()
    (
        optimal_x1,
        optimal_y1,
        optimal_x2,
        optimal_y2,
        optimal_thickness_root,
        optimal_thickness_tip,
    ) = best_solution

    optimal_curve_points, optimal_control_polygon = generate_bezier_centerline(
        optimal_x1, optimal_y1, optimal_x2, optimal_y2
    )
    final_deflection, final_max_stress, final_total_mass = generalized_spoke_mechanics(
        optimal_curve_points,
        optimal_thickness_root,
        optimal_thickness_tip,
        SPOKE_WIDTH_MM,
        force_per_spoke=FORCE_PER_SPOKE_NEWTONS,
    )

    print("\n================ EVOLVED OPTIMAL DESIGN ================")
    print(f" Spoke Count:                  {NUMBER_OF_SPOKES}")
    print(f" Fixed Constant Width:         {SPOKE_WIDTH_MM:.2f} mm")
    print(f" Evolved Optimal Root Thickness: {optimal_thickness_root:.2f} mm")
    print(f" Evolved Optimal Tip Thickness:  {optimal_thickness_tip:.2f} mm")
    print("-----------------------------------------------------------------")
    print(
        f" Per-Spoke Deflection:         {final_deflection:6.2f} mm  (Target: {
            TARGET_DEFLECTION_MM
        } mm)"
    )
    print(
        f" Max Bending Stress:           {final_max_stress:6.2f} MPa (Max Allowable: {
            ALLOWABLE_STRESS_MPA:.1f} MPa)"
    )
    print(f" Total Integrated Wheel Mass:  {final_total_mass:6.2f} g")
    print("===================================================================\n")

    spoke_polygon_local = thicken_tapered_curve(
        optimal_curve_points, optimal_thickness_root, optimal_thickness_tip
    )
    spoke_polygon_placed = place_repeating_sector(
        spoke_polygon_local, HUB_RADIUS_MM, sector_angle_degrees=0.0
    )

    figure, axes = plt.subplots(2, 2, figsize=(12, 12))
    half_sector_angle = 360.0 / NUMBER_OF_SPOKES / 2.0
    hub_arc_angles = np.linspace(-half_sector_angle, half_sector_angle, 100)
    full_circle_angles = np.linspace(0, 2 * np.pi, 300)

    axes[0, 0].plot(
        best_fitness_history, label="Best Candidate", color="#1b5e20", linewidth=2
    )
    axes[0, 0].plot(
        mean_fitness_history, label="Population Mean", color="#81c784", linewidth=1.5
    )
    axes[0, 0].set_title("GA Convergence (Stress Optimization Space)")
    axes[0, 0].set_xlabel("Generation")
    axes[0, 0].set_ylabel("Loss Function Value")
    axes[0, 0].legend()
    axes[0, 0].grid(alpha=0.3)

    for boundary_angle in (-half_sector_angle, half_sector_angle):
        axes[0, 1].plot(
            [0, RIM_RADIUS_MM * 1.15 * np.cos(np.deg2rad(boundary_angle))],
            [0, RIM_RADIUS_MM * 1.15 * np.sin(np.deg2rad(boundary_angle))],
            "--",
            color="gray",
            linewidth=1,
        )
    axes[0, 1].plot(
        HUB_RADIUS_MM * np.cos(np.deg2rad(hub_arc_angles)),
        HUB_RADIUS_MM * np.sin(np.deg2rad(hub_arc_angles)),
        color="black",
        linewidth=2,
    )
    axes[0, 1].plot(
        RIM_RADIUS_MM * np.cos(np.deg2rad(hub_arc_angles)),
        RIM_RADIUS_MM * np.sin(np.deg2rad(hub_arc_angles)),
        color="black",
        linewidth=2,
    )
    axes[0, 1].fill(
        spoke_polygon_placed[:, 0],
        spoke_polygon_placed[:, 1],
        color="#2e7d32",
        alpha=0.85,
        edgecolor="black",
    )

    control_polygon_physical = np.array(optimal_control_polygon)
    control_polygon_physical[:, 0] += HUB_RADIUS_MM
    axes[0, 1].plot(
        control_polygon_physical[:, 0],
        control_polygon_physical[:, 1],
        "o--",
        color="#d32f2f",
        linewidth=1.5,
        label="Bézier Handles",
    )

    handle_labels = ["P0", "P1", "P2", "P3"]
    for handle_index, handle_label in enumerate(handle_labels):
        axes[0, 1].annotate(
            handle_label,
            (
                control_polygon_physical[handle_index, 0],
                control_polygon_physical[handle_index, 1],
            ),
            textcoords="offset points",
            xytext=(0, 8),
            ha="center",
            color="#c62828",
            weight="bold",
            fontsize=9,
        )

    axes[0, 1].set_aspect("equal")
    axes[0, 1].set_title(f"1/{NUMBER_OF_SPOKES} Repeating Sector Profile")
    axes[0, 1].legend(loc="upper right")
    axes[0, 1].grid(alpha=0.2)

    axes[1, 0].plot(
        HUB_RADIUS_MM * np.cos(full_circle_angles),
        HUB_RADIUS_MM * np.sin(full_circle_angles),
        color="black",
        linewidth=2,
    )
    axes[1, 0].plot(
        RIM_RADIUS_MM * np.cos(full_circle_angles),
        RIM_RADIUS_MM * np.sin(full_circle_angles),
        color="black",
        linewidth=2,
    )

    for spoke_index in range(NUMBER_OF_SPOKES):
        rotation_angle = spoke_index * (360.0 / NUMBER_OF_SPOKES)
        rotated_polygon = rotate_points(
            spoke_polygon_placed, np.deg2rad(rotation_angle)
        )
        axes[1, 0].fill(
            rotated_polygon[:, 0],
            rotated_polygon[:, 1],
            color="#2e7d32",
            alpha=0.85,
            edgecolor="black",
        )

    axes[1, 0].set_aspect("equal")
    axes[1, 0].set_title(f"Full Wheel System ({NUMBER_OF_SPOKES}x Symmetry)")
    axes[1, 0].grid(alpha=0.2)

    axes[1, 1].plot(
        optimal_curve_points[:, 0],
        optimal_curve_points[:, 1],
        "-",
        color="#1565c0",
        linewidth=3,
        label="Optimized Centerline",
    )
    axes[1, 1].set_title("Isolated Spoke Curvature Mapping")
    axes[1, 1].set_xlabel("Local X Span (mm)")
    axes[1, 1].set_ylabel("Local Y Deviation (mm)")
    axes[1, 1].legend()
    axes[1, 1].grid(alpha=0.3)

    figure.suptitle(
        f"Compliant PLA UAV Wheel — Co-Optimized Shape & Thickness Profile",
        fontsize=15,
        fontweight="bold",
    )
    figure.tight_layout(rect=[0, 0, 1, 0.96])
    figure.savefig("poster_summary.png", dpi=200)

    print("Saved co-optimized structural summary to: poster_summary.png")

    print("--- INVENTOR INPUT DATA ---")
    print("P0 (Hub Start): X = 0.0, Y = 0.0")
    print(f"P1 (Handle 1) : X = {optimal_x1:.3f}, Y = {optimal_y1:.3f}")
    print(f"P2 (Handle 2) : X = {optimal_x2:.3f}, Y = {optimal_y2:.3f}")
    print(f"P3 (Rim End)  : X = {HUB_RIM_SPAN_MM:.3f}, Y = 0.0")
    print(f"Root Thickness (t_root): {optimal_thickness_root:.3f} mm")
    print(f"Tip Thickness (t_tip)  : {optimal_thickness_tip:.3f} mm")
