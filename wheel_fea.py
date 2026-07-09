import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import pygad

np.random.seed(42)

N_SPOKES = 12
HUB_RADIUS_MM = 25.4 / 2
RIM_RADIUS_MM = 97.8 / 2
HUB_RIM_SPAN_MM = RIM_RADIUS_MM - HUB_RADIUS_MM

SPOKE_WIDTH_MM = 22.4

YOUNGS_MODULUS_PLA_MPA = 2300.0 # Young's Modulus of PLA
SIGMA_ULT_MPA = 50.0
SAFETY_FACTOR = 1.6
SIGMA_ALLOW_MPA = SIGMA_ULT_MPA / SAFETY_FACTOR
RHO_PLA = 1.24e-3

LBSFORCE = 15.0
FORCE_TOTAL_N = LBSFORCE * 4.44822
FORCE_PER_SPOKE_N = FORCE_TOTAL_N / (N_SPOKES / 3.0)

TARGET_DEFLECTION_MM = 1

# Cubic Bezier curve
def generate_bezier_centerline(x1, y1, x2, y2, span=HUB_RIM_SPAN_MM, n_pts=400):
    p0 = np.array([0.0, 0.0])
    p1 = np.array([x1, y1])
    p2 = np.array([x2, y2])
    p3 = np.array([span, 0.0])

    t = np.linspace(0.0, 1.0, n_pts)[:, None]
    pts = (
        (1 - t) ** 3 * p0
        + 3 * (1 - t) ** 2 * t * p1
        + 3 * (1 - t) * t**2 * p2
        + t**3 * p3
    )
    return pts, [p0, p1, p2, p3]


def generalized_spoke_mechanics(pts, t_root, t_tip, w, F, E=YOUNGS_MODULUS_PLA_MPA):
    n_seg = len(pts)
    t_profile = np.linspace(t_root, t_tip, n_seg)

    # Evaluate properties on the segments between discrete points (n_seg - 1)
    t_seg = t_profile[:-1]
    I = w * t_seg**3 / 12.0
    A = w * t_seg

    # Calculate segment vectors and arc lengths
    dx = np.diff(pts[:, 0])
    dy = np.diff(pts[:, 1])
    ds = np.sqrt(dx**2 + dy**2)

    # Local Y-deviation acts as the direct moment arm for a radial force
    y_eval = pts[:-1, 1]

    # 1. Bending Moment at each segment: M = F * y
    moments = F * y_eval

    # 2. Axial Compressive Force: Project global X-force onto the local segment angle
    # cos(theta) = dx / ds
    cos_theta = dx / np.where(ds == 0, 1e-6, ds)
    axial_forces = F * cos_theta

    # 3. Deflection via Castigliano's Energy Method
    # Bending energy deflection component
    defl_bending = np.sum((F * y_eval**2) / (E * I) * ds)
    # Axial compression energy deflection component (crucial for straight columns)
    defl_axial = np.sum((F * cos_theta**2) / (E * A) * ds)

    total_deflection_mm = defl_bending + defl_axial

    # 4. Combined Stress: Direct Axial Compression + Peak Outer Fiber Bending Stress
    sigma_axial = axial_forces / A
    sigma_bending = np.abs(moments) * (t_seg / 2.0) / I

    # Combined worst-case structural stress
    sigma_max_mpa = np.max(sigma_axial + sigma_bending)

    # 5. Integrated Mass Calculation
    total_length = np.sum(ds)
    avg_thickness = (t_root + t_tip) / 2.0
    mass_g = avg_thickness * w * total_length * RHO_PLA * N_SPOKES

    return total_deflection_mm, sigma_max_mpa, mass_g

# ------------------------------------------------------------------------
# 3. PYGAD FITNESS FUNCTION (STRESS-MINIMIZATION STRATEGY)
# ------------------------------------------------------------------------
def pygad_fitness(ga_instance, solution, solution_idx):
    # Unpack 5 parameters: 4 control points + thickness 't'
    x1, y1, x2, y2, t_root, t_tip = solution

    if x1 >= x2:
        return -500.0

    pts, _ = generate_bezier_centerline(x1, y1, x2, y2)

    if np.any(np.diff(pts[:, 0]) <= 0.02):
        return -500.0

    defl, sigma, mass = generalized_spoke_mechanics(pts, t_root, t_tip, SPOKE_WIDTH_MM, F=FORCE_PER_SPOKE_N)

    # 1. Deflection Accuracy Constraint (Keep this)
    defl_err = abs(defl - TARGET_DEFLECTION_MM) / TARGET_DEFLECTION_MM
    deflection_loss = 2000.0 * (defl_err ** 2)

    # 2. NEW: Mass Minimization Driver
    # This forces the GA to make the spokes as thin and short as possible
    mass_objective = mass / 50.0  # Normalized scaling factor

    # 3. Stress Penalty (Only penalize if it breaks the safety factor)
    if sigma > SIGMA_ALLOW_MPA:
        stress_penalty = 2000.0 * (sigma / SIGMA_ALLOW_MPA - 1.0) ** 2
    else:
        stress_penalty = 0.0

    total_loss = deflection_loss + mass_objective + stress_penalty
    return -total_loss

best_hist = []
mean_hist = []


def on_generation(ga_instance):
    fits = ga_instance.last_generation_fitness
    best_hist.append(-np.max(fits))
    mean_hist.append(-np.mean(fits))


# ------------------------------------------------------------------------
# 4. GEOMETRY UTILITIES
# ------------------------------------------------------------------------
def rotate(points, angle_rad):
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    R = np.array([[c, -s], [s, c]])
    return points @ R.T


def thicken_tapered(centerline, t_root, t_tip):
    n_pts = len(centerline)
    t_profile = np.linspace(t_root, t_tip, n_pts)[:, None] # Column vector

    d = np.gradient(centerline, axis=0)
    norm = np.stack([-d[:, 1], d[:, 0]], axis=1)
    norm /= np.linalg.norm(norm, axis=1, keepdims=True)

    # Scale the normal offset vector by the local thickness profile
    top = centerline + norm * (t_profile / 2.0)
    bot = centerline - norm * (t_profile / 2.0)
    return np.vstack([top, bot[::-1]])

def place_sector(poly_local, r_hub, sector_angle_deg=0.0):
    shifted = poly_local.copy()
    shifted[:, 0] += r_hub
    return rotate(shifted, np.deg2rad(sector_angle_deg))


# ------------------------------------------------------------------------
# 5. MAIN OPTIMIZATION LOOP
# ------------------------------------------------------------------------
if __name__ == "__main__":
    print(f"Running Co-Optimization for Shape & Thickness ({N_SPOKES} Spokes)...")
    print(f"Fixed Face Width: {SPOKE_WIDTH_MM} mm | Driving Variable Thickness Tracker")

    # 6-Gene Search Space (x1, y1, x2, y2, t_root, t_tip)
    gene_space = [
        {"low": 5.0, "high": HUB_RIM_SPAN_MM * 0.45},    # x1
        {"low": -55.0, "high": 55.0},            # y1
        {"low": HUB_RIM_SPAN_MM * 0.55, "high": HUB_RIM_SPAN_MM},# x2
        {"low": -55.0, "high": 55.0},            # y2
        {"low": 6.0, "high": 12.0},              # t_root (Thick base allowed)
        {"low": 1.5, "high": 3.5},               # t_tip  (Thin tip allowed)
    ]

    ga_instance = pygad.GA(
        num_generations=1000,
        num_parents_mating=15,
        fitness_func=pygad_fitness,
        sol_per_pop=160,
        num_genes=6,
        gene_space=gene_space,
        parent_selection_type="tournament",
        K_tournament=4,
        crossover_type="uniform",
        mutation_type="random",
        mutation_probability=0.40,
        keep_elitism=3,
        on_generation=on_generation,
    )

    ga_instance.run()

    # Extract Results
    best_solution, best_solution_fitness, _ = ga_instance.best_solution()
    x1, y1, x2, y2, t_root, t_tip = best_solution

    # FIX 1: Correctly unpack the tuple so 'pts' is a pure NumPy array
    pts, control_polygon = generate_bezier_centerline(x1, y1, x2, y2)
    final_defl, final_sigma, final_mass = generalized_spoke_mechanics(pts, t_root, t_tip, SPOKE_WIDTH_MM, F=FORCE_PER_SPOKE_N)

    print("\n================ EVOLVED OPTIMAL DESIGN ================")
    print(f" Spoke Count:                 {N_SPOKES}")
    print(f" Fixed Constant Width w:      {SPOKE_WIDTH_MM:.2f} mm")
    # FIX 2: Correct string formatting syntax from ':.2 mm' to ':.2f} mm'
    print(f" Evolved Optimal Root Thickness: {t_root:.2f} mm")
    print(f" Evolved Optimal Tip Thickness:  {t_tip:.2f} mm")
    print("-----------------------------------------------------------------")
    # FIX 3: Scope correction. Reference 'final_*' metrics generated above
    print(f" Per-Spoke Deflection:        {final_defl:6.2f} mm  (Target: {TARGET_DEFLECTION_MM} mm)")
    print(f" Max Bending Stress:          {final_sigma:6.2f} MPa (Max Allowable: {SIGMA_ALLOW_MPA:.1f} MPa)")
    print(f" Total Integrated Wheel Mass: {final_mass:6.2f} g")
    print("===================================================================\n")

    # ---------------- POSTER GENERATION PLOTS ----------------
    # FIX 4: Feed the updated 'pts', 't_root', and 't_tip' into the tapered thickener
    spoke_poly_local = thicken_tapered(pts, t_root, t_tip)
    spoke_poly = place_sector(spoke_poly_local, HUB_RADIUS_MM, sector_angle_deg=0.0)

    fig_all, axs = plt.subplots(2, 2, figsize=(12, 12))
    sector_half = 360.0 / N_SPOKES / 2.0
    hub_arc = np.linspace(-sector_half, sector_half, 100)
    theta_full = np.linspace(0, 2 * np.pi, 300)

    # Subplot 1: Convergence
    axs[0, 0].plot(best_hist, label="Best Candidate", color="#1b5e20", linewidth=2)
    axs[0, 0].plot(mean_hist, label="Population Mean", color="#81c784", linewidth=1.5)
    axs[0, 0].set_title("GA Convergence (Stress Optimization Space)")
    axs[0, 0].set_xlabel("Generation")
    axs[0, 0].set_ylabel("Loss Function Value")
    axs[0, 0].legend()
    axs[0, 0].grid(alpha=0.3)

    # Subplot 2: Physical Sector with Handle Plotting
    for ang in (-sector_half, sector_half):
        axs[0, 1].plot(
            [0, RIM_RADIUS_MM * 1.15 * np.cos(np.deg2rad(ang))],
            [0, RIM_RADIUS_MM * 1.15 * np.sin(np.deg2rad(ang))],
            "--", color="gray", linewidth=1,
        )
    axs[0, 1].plot(HUB_RADIUS_MM * np.cos(np.deg2rad(hub_arc)), HUB_RADIUS_MM * np.sin(np.deg2rad(hub_arc)), color="black", linewidth=2)
    axs[0, 1].plot(RIM_RADIUS_MM * np.cos(np.deg2rad(hub_arc)), RIM_RADIUS_MM * np.sin(np.deg2rad(hub_arc)), color="black", linewidth=2)
    axs[0, 1].fill(spoke_poly[:, 0], spoke_poly[:, 1], color="#2e7d32", alpha=0.85, edgecolor="black")

    c_poly_physical = np.array(control_polygon)
    c_poly_physical[:, 0] += HUB_RADIUS_MM
    axs[0, 1].plot(c_poly_physical[:, 0], c_poly_physical[:, 1], "o--", color="#d32f2f", linewidth=1.5, label="Bézier Handles")
    for i, txt in enumerate(["P0", "P1", "P2", "P3"]):
        axs[0, 1].annotate(txt, (c_poly_physical[i, 0], c_poly_physical[i, 1]), textcoords="offset points", xytext=(0,8), ha='center', color="#c62828", weight='bold', fontsize=9)

    axs[0, 1].set_aspect("equal")
    axs[0, 1].set_title(f"1/{N_SPOKES} Repeating Sector Profile")
    axs[0, 1].legend(loc="upper right")
    axs[0, 1].grid(alpha=0.2)

    # Subplot 3: Full Wheel Layout
    axs[1, 0].plot(HUB_RADIUS_MM * np.cos(theta_full), HUB_RADIUS_MM * np.sin(theta_full), color="black", linewidth=2)
    axs[1, 0].plot(RIM_RADIUS_MM * np.cos(theta_full), RIM_RADIUS_MM * np.sin(theta_full), color="black", linewidth=2)
    for k in range(N_SPOKES):
        ang = k * (360.0 / N_SPOKES)
        poly_k = rotate(spoke_poly, np.deg2rad(ang))
        axs[1, 0].fill(poly_k[:, 0], poly_k[:, 1], color="#2e7d32", alpha=0.85, edgecolor="black")
    axs[1, 0].set_aspect("equal")
    axs[1, 0].set_title(f"Full Wheel System ({N_SPOKES}x Symmetry)")
    axs[1, 0].grid(alpha=0.2)

    # Subplot 4: Standalone Centerline Track
    # FIX 5: Change from stale 'pts_opt' variable name to 'pts'
    axs[1, 1].plot(pts[:, 0], pts[:, 1], "-", color="#1565c0", linewidth=3, label="Optimized Centerline")
    axs[1, 1].set_title("Isolated Spoke Curvature Mapping")
    axs[1, 1].set_xlabel("Local X Span (mm)")
    axs[1, 1].set_ylabel("Local Y Deviation (mm)")
    axs[1, 1].legend()
    axs[1, 1].grid(alpha=0.3)

    fig_all.suptitle(f"Compliant PLA UAV Wheel — Co-Optimized Shape & Thickness Profile", fontsize=15, fontweight="bold")
    fig_all.tight_layout(rect=[0, 0, 1, 0.96])
    fig_all.savefig("poster_summary.png", dpi=200)

    print("Saved co-optimized structural summary to: poster_summary.png")

    print("--- INVENTOR INPUT DATA ---")
    print(f"P0 (Hub Start): X = 0.0, Y = 0.0")
    print(f"P1 (Handle 1) : X = {x1:.3f}, Y = {y1:.3f}")
    print(f"P2 (Handle 2) : X = {x2:.3f}, Y = {y2:.3f}")
    print(f"P3 (Rim End)  : X = {HUB_RIM_SPAN_MM:.3f}, Y = 0.0")
    print(f"Root Thickness (t_root): {t_root:.3f} mm")
    print(f"Tip Thickness (t_tip)  : {t_tip:.3f} mm")