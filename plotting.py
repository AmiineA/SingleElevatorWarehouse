import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import random  
from math import sqrt, log

def plot_convergence(trace_t, trace_avgW, n, lambda_val,
                     policy: str = None,
                     controller_type: str = None,
                     color: str = "C0",
                     newfigure: bool = True,
                     show: bool = True,
                     label: str | None = None,
                     alpha: float = 1.0):
    if not trace_t:
        return
    if newfigure:
        plt.figure(figsize=(8,4))

    # default label: one per controller/policy
    if label is None:
        parts = []
        if controller_type is not None:
            parts.append(controller_type.upper())
        if policy is not None:
            parts.append(policy)
        label = " - ".join(parts) if parts else "W moyen courant"

    plt.plot(trace_t, trace_avgW, color=color, label=label, alpha=alpha)

    title = f" W(t) — n = {n}, λ = {lambda_val}"
    extra = []
    if policy is not None:
        extra.append(f"policy {policy}")
    if controller_type is not None:
        extra.append(f"controller {controller_type}")
    if extra:
        title += " (" + ", ".join(extra) + ")"

    plt.title(title)
    plt.xlabel("Temps simulé")
    plt.ylabel("W moyen courant")
    plt.legend()
    plt.tight_layout()
    if show:
        plt.show()



def plot_queues_and_elevator(t_arr, queues_arr, elev_pos, n):
    fig, axes = plt.subplots(n + 1, 1, figsize=(10, 2.2 * (n + 1)), sharex=True)

    for f in range(n):
        t_steps = np.repeat(t_arr, 2)[1:]
        q_steps = np.repeat(queues_arr[:, f], 2)[:-1]
        axes[n - 1 - f].plot(t_steps, q_steps, drawstyle='steps-post')
        axes[n - 1 - f].set_ylabel(f"Étage {f}")
        axes[n - 1 - f].grid(True, alpha=0.3)
        axes[n - 1 - f].yaxis.set_major_locator(MaxNLocator(integer=True))

    ramp_t = []
    ramp_pos = []
    for i in range(1, len(t_arr)):
        ramp_t.extend([t_arr[i - 1], t_arr[i]])
        ramp_pos.extend([elev_pos[i - 1], elev_pos[i]])

    axes[n].plot(ramp_t, ramp_pos, color="black", linewidth=2)
    axes[n].set_ylabel("Ascenseur")
    axes[n].grid(True, alpha=0.3)
    axes[n].yaxis.set_major_locator(MaxNLocator(integer=True))

    axes[-1].set_xlabel("Temps simulé")
    fig.suptitle("Files d'attente par étage", fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()


def plot_all(out, cfg, controller_type: str = None, color: str = "C0",
             newfigure: bool = True, show: bool = True,
             label: str | None = None, alpha: float = 1.0):
    policy = getattr(cfg, "policy", None )
    lambda_val = cfg.lambdas[0] if hasattr(cfg, "lambdas") and cfg.lambdas else None
    if cfg.Verbose is not None:
        t_arr = np.array(out["Verbose"]["t"])
        queues_arr = np.array(out["Verbose"]["queues"])
        elev_pos = out["Verbose"]["elevator_pos"]

        plot_queues_and_elevator(t_arr, queues_arr, elev_pos, cfg.n)

    plot_convergence(
        out["trace_t"], out["trace_avgW"], cfg.n, lambda_val,
        policy=policy, controller_type=controller_type,
        color=color, newfigure=newfigure, show=show,
        label=label, alpha=alpha
    )


def plot_policy_vs_lambda_grid(SimConfig, run_one_rep, m, n_values, M_values: list[int] = [1],
                               sim_time: float = 7_000,
                               warmup: float = 0.0,
                               speed: float = 1.0,
                               door_time: float = 0.0):
    """
    Pour chaque n crée une figure différente.
    Sur chaque graphique :
      - pour k = 1,...,m, on prend
            lambda_scalar = 2.7 * k / (m * n * (n + 1))
        (même λ_i = lambda_scalar pour tous les étages),
      - pour chaque (contrôleur, politique) dans
            {naif, dp} x {passive, active},
        on lance une simu avec le même seed pour tous les 4 cas,
        on calcule W moyen et on trace les 5 points reliés en ligne.

    Args:
        SimConfig: classe de configuration (SimConfig dans simulation EA.py).
        run_one_rep: fonction de simulation run_one_rep(cfg, controller_type=...).
        sim_time, warmup, speed, door_time: paramètres communs à toutes les simus.
    """

    controllers = ["naif", "idiot", "dp"]
    #controllers = ["naif"]
    policies = ["passive"]
    colors = ["C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9"]

    # Une couleur par combinaison (contrôleur, politique)
    color_map = {}
    idx = 0
    for c in controllers:
        for p in policies:
            color_map[(c, p)] = colors[idx % len(colors)]
            idx += 1

    for n in n_values:
        plt.figure(figsize=(8, 5))

        # Pour stocker les résultats par combinaison
        results = {
            (c, p): {"lambdas": [], "avgW": []}
            for c in controllers for p in policies
        }

        for k in range(1, m+1):
            lambda_scalar = 3 * k / (m * (n * (n + 1)))

            # Seed unique pour ce couple (n, k), partagé par les 4 combinaisons
            seed = random.randint(0, 10**9)

            lambdas_per_floor = [lambda_scalar for _ in range(n)]

            for controller_type in controllers:
                for policy in policies:
                    cfg = SimConfig(
                        n_floors=n,
                        lambdas_per_floor=lambdas_per_floor,
                        uniform_dest=True,
                        P_dest=None,
                        init_floor=0,
                        speed=speed,
                        door_time=door_time,
                        sim_time=sim_time * max(n_values),
                        warmup=warmup,
                        seed=seed,
                        policy=policy,
                        Verbose=None
                    )
                    out = run_one_rep(cfg, controller_type=controller_type)
                    if k == m:
                       plot_all(out, cfg, controller_type=controller_type)
                    waits = out["wait_times"]
                    avgW = (sum(waits) / len(waits)) if waits else float("nan")

                    key = (controller_type, policy)
                    results[key]["lambdas"].append(lambda_scalar)
                    results[key]["avgW"].append(avgW)

        # Une fois qu'on a les m points pour chaque combinaison, on trace
        for (controller_type, policy), data in results.items():
            lambdas = data["lambdas"]
            avgWs = data["avgW"]
            label = f"{controller_type.upper()} - {policy}"
            color = color_map[(controller_type, policy)]
            plt.plot(lambdas, avgWs, marker="o", color=color, label=label)

        plt.xlabel(r"$\lambda$ par étage")
        plt.ylabel(r"$\bar{W}$ (temps d'attente moyen)")
        plt.title(f"Comparaison de politiques — n = {n}")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.show()

def plot_convergence_benchmark(SimConfig, run_one_rep, controllers, policies,
                               lambdas: float, reps: int = 4, n: int = 3,
                               sim_time: float = 20_000):
    colors = ["C3", "C4", "C1", "C3", "C4", "C5", "C6", "C7", "C8", "C9"]

    color_map = {}
    idx = 0
    for c in controllers:
        for p in policies:
            color_map[(c, p)] = colors[idx % len(colors)]
            idx += 1

    lambdas_per_floor = [lambdas for _ in range(n)]

    plt.figure(figsize=(8, 5))

    for controller_type in controllers:
        for policy in policies:
            color = color_map[(controller_type, policy)]

            rep_curves = []   # list of np.array(trace_avgW)
            rep_wait_means = []
            common_t = None

            for rep in range(reps):
                seed = random.randint(0, 10**9)
                cfg = SimConfig(
                    n_floors=n,
                    lambdas_per_floor=lambdas_per_floor,
                    sim_time=sim_time,
                    seed=seed,
                    policy=policy,
                )
                out = run_one_rep(cfg, controller_type=controller_type)

                # plot this rep faintly
                plot_all(out, cfg, controller_type=controller_type,
                         color=color, newfigure=False, show=False,
                         label="_nolegend_", alpha=0.2)

                # collect data for averaging
                t = np.asarray(out["trace_t"])
                y = np.asarray(out["trace_avgW"])
                if common_t is None:
                    common_t = t
                else:
                    # if time grids differ
                    m = min(len(common_t), len(t), len(y))
                    common_t = common_t[:m]
                    y = y[:m]
                    rep_curves = [c[:m] for c in rep_curves]

                rep_curves.append(y)

                waits = out.get("wait_times", [])
                if waits:
                    rep_wait_means.append(float(np.mean(waits)))

            # average curve (solid)
            if rep_curves and common_t is not None:
                Y = np.vstack(rep_curves)
                avg_curve = np.mean(Y, axis=0)

                avg_wait = float(np.mean(rep_wait_means)) if rep_wait_means else float("nan")
                label = f"{controller_type.upper()} - {policy} (E[W]={avg_wait:.3g})"

                plt.plot(common_t, avg_curve, color=color, alpha=1.0, linewidth=2.0, label=label)

    plt.xlabel("Temps simulé")
    plt.ylabel("W moyen courant")
    plt.title(f"Convergence — n={n}, λ={lambdas} (reps={reps})")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

def benchmark_naif_passive_vs_active_vs_lambda(
    SimConfig,
    run_one_rep,
    n: int,
    lambda_crit: float,
    lambda_min: float = 0.01,
    n_points: int = 20,
    reps_per_lambda: int = 5,
    sim_time: float = 30_000,
    warmup: float = 0.0,
    speed: float = 1.0,
    door_time: float = 0.0,
    seed_base: int | None = None,
):
    """
    Benchmarks NAIF controller: passive vs active as lambda increases.

    For each lambda in a grid of n_points, run reps_per_lambda seeds.
    For each seed, run BOTH passive and active with the same seed, then average E[W].
    Total seeds = n_points * reps_per_lambda.
    """

    controller_type = "naif"
    policies = ["passive", "active"]

    # λ grid (avoid exactly lambda_crit if you want)
    lambdas_grid = np.linspace(lambda_min, lambda_crit, n_points)

    # reproducible seed stream if seed_base is set
    rng = np.random.default_rng(seed_base)
    seeds = rng.integers(0, 10**9, size=n_points * reps_per_lambda, dtype=np.int64)

    results = {p: [] for p in policies}

    s_idx = 0
    for lam in lambdas_grid:
        lambdas_per_floor = [float(lam) for _ in range(n)]

        # accumulate across reps
        acc = {p: [] for p in policies}

        for _ in range(reps_per_lambda):
            seed = int(seeds[s_idx])
            s_idx += 1

            for policy in policies:
                cfg = SimConfig(
                    n_floors=n,
                    lambdas_per_floor=lambdas_per_floor,
                    uniform_dest=True,
                    P_dest=None,
                    init_floor=0,
                    speed=speed,
                    door_time=door_time,
                    sim_time=sim_time,
                    warmup=warmup,
                    seed=seed,          # SAME seed for passive & active
                    policy=policy,
                    Verbose=None
                )

                out = run_one_rep(cfg, controller_type=controller_type)
                waits = out.get("wait_times", [])
                avgW = float(np.mean(waits)) if waits else float("nan")
                acc[policy].append(avgW)

        # average across the 5 seeds for this lambda
        for policy in policies:
            results[policy].append(float(np.nanmean(acc[policy])))

    # ---- Plot
    plt.figure(figsize=(9, 5))

    plt.plot(lambdas_grid, results["passive"], marker="o", linewidth=2, label="Passive")
    plt.plot(lambdas_grid, results["active"],  marker="s", linewidth=2, label="Active")

    plt.xlabel(r"$\lambda$")
    plt.ylabel(r"$E[W]$ (Average on 5 seeds)")
    plt.title(f"NAIF: passive vs active — n={n}")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

    return lambdas_grid, results

def benchmark_passive_active_naif_dp_vs_lambda(
    SimConfig,
    run_one_rep,
    n: int,
    lambda_crit: float,
    lambda_min: float = 0.01,
    n_points: int = 20,
    reps_per_lambda: int = 5,
    sim_time: float = 30_000,
    warmup: float = 0.0,
    speed: float = 1.0,
    door_time: float = 0.0,
    seed_base: int | None = None,
):
    """
    Benchmarks passive vs active for NAIF and DP controllers.
    Same seeds are used for passive/active for each controller.

    Total seeds = n_points * reps_per_lambda.
    Produces ONE figure with 4 curves.
    """

    controllers = ["naif", "dp"]
    policies = ["passive", "active"]

    lambdas_grid = np.linspace(lambda_min, lambda_crit, n_points)

    rng = np.random.default_rng(seed_base)
    seeds = rng.integers(0, 10**9, size=n_points * reps_per_lambda, dtype=np.int64)

    # results[(controller, policy)] = list of averaged E[W] over lambdas
    results = {(c, p): [] for c in controllers for p in policies}

    s_idx = 0
    for lam in lambdas_grid:
        lambdas_per_floor = [float(lam) for _ in range(n)]

        # accumulate per lambda
        acc = {(c, p): [] for c in controllers for p in policies}

        for _ in range(reps_per_lambda):
            seed = int(seeds[s_idx])
            s_idx += 1

            for controller_type in controllers:
                for policy in policies:
                    cfg = SimConfig(
                        n_floors=n,
                        lambdas_per_floor=lambdas_per_floor,
                        uniform_dest=True,
                        P_dest=None,
                        init_floor=0,
                        speed=speed,
                        door_time=door_time,
                        sim_time=sim_time,
                        warmup=warmup,
                        seed=seed,   # same seed across passive/active
                        policy=policy,
                        Verbose=None
                    )

                    out = run_one_rep(cfg, controller_type=controller_type)
                    waits = out.get("wait_times", [])
                    avgW = float(np.mean(waits)) if waits else float("nan")

                    acc[(controller_type, policy)].append(avgW)

        # average across the 5 seeds for this lambda
        for key in acc:
            results[key].append(float(np.nanmean(acc[key])))

    # ---- Plot
    plt.figure(figsize=(9, 5))

    style = {
        ("naif", "passive"): dict(color="C0", marker="o", linestyle="-"),
        ("naif", "active"):  dict(color="C0", marker="s", linestyle="--"),
        ("dp",   "passive"): dict(color="C2", marker="^", linestyle="-"),
        ("dp",   "active"):  dict(color="C2", marker="D", linestyle="--"),
    }

    for (controller_type, policy), avgWs in results.items():
        lbl = f"{controller_type.upper()} - {policy}"
        plt.plot(lambdas_grid, avgWs, linewidth=2.2, label=lbl, **style[(controller_type, policy)])

    plt.xlabel(r"$\lambda$ per floor")
    plt.ylabel(r"$E[W]$ (Average on 5 seeds)")
    plt.title(
        f"Passive vs active — NAIF & DP (n={n})"
    )
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()

    return lambdas_grid, results

def benchmark_dp_vs_offlineM_vs_lambda(
    SimConfig,
    run_one_rep,
    n: int,
    lambda_max: float,
    lambda_min: float = 0.01,
    n_points: int = 10,
    reps_per_lambda: int = 5,
    M_values: list[int] = [1, 2, 3, 4],
    sim_time: float = 30_000,
    warmup: float = 0.0,
    speed: float = 1.0,
    door_time: float = 0.0,
    seed_base: int | None = None,
):
    """
    Compare real DP (dp, passive) vs offline truncated DP (offline_dp, passive, M in M_values)
    as lambda varies.

    For each lambda (n_points) and each rep (reps_per_lambda), we use ONE seed shared across
    all variants (dp + offline M's). Then we average E[W] over reps_per_lambda seeds.
    """

    policy = "passive"
    variants = [("dp", None)] + [("offline_dp", M) for M in M_values]

    lambdas_grid = np.linspace(lambda_min, lambda_max, n_points)

    rng = np.random.default_rng(seed_base)
    seeds = rng.integers(0, 10**9, size=n_points * reps_per_lambda, dtype=np.int64)

    results = {v: [] for v in variants}  # each entry is list over lambdas

    s_idx = 0
    for lam in lambdas_grid:
        lambdas_per_floor = [float(lam) for _ in range(n)]

        # accumulate waits for this lambda
        acc = {v: [] for v in variants}

        for _ in range(reps_per_lambda):
            seed = int(seeds[s_idx])
            s_idx += 1

            for (controller_type, M) in variants:
                cfg = SimConfig(
                    n_floors=n,
                    lambdas_per_floor=lambdas_per_floor,
                    uniform_dest=True,
                    P_dest=None,
                    init_floor=0,
                    speed=speed,
                    door_time=door_time,
                    sim_time=sim_time,
                    warmup=warmup,
                    seed=seed,
                    policy=policy,
                    Verbose=None,
                    M=(1 if M is None else int(M)),  # dp ignores it; offline_dp uses it
                )

                out = run_one_rep(cfg, controller_type=controller_type)
                waits = out.get("wait_times", [])
                avgW = float(np.mean(waits)) if waits else float("nan")
                acc[(controller_type, M)].append(avgW)

        # average across the 5 reps for this lambda
        for v in variants:
            results[v].append(float(np.nanmean(acc[v])))

    plt.figure(figsize=(9, 5))

    # style choices
    plt.plot(lambdas_grid, results[("dp", None)],
             marker="o", linewidth=2.5, label="DP inf, passive", color="C0")

    for j, M in enumerate(M_values, start=1):
        plt.plot(lambdas_grid, results[("offline_dp", M)],
                 marker="s", linewidth=2.0, linestyle="-",
                 label=f"Truncated DP (M={M}), passive", color=f"C{j}")

    plt.xlabel(r"$\lambda$ per floor")
    plt.ylabel(r"$E[W]$ (average over 5 seeds)")
    plt.title(f"DP vs Truncated DP — n={n}")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.yscale("log")
    plt.tight_layout()
    plt.show()

    return lambdas_grid, results

"""if __name__ == "__main__":
    from simulation_EA import SimConfig, run_one_rep
    controllers = ["naif"]
    policies = ["passive", "active"]
    n = 6
    l = 0.01
    plot_convergence_benchmark(SimConfig, run_one_rep, controllers = controllers, policies = policies, lambdas=l, n = n, reps = 10, sim_time=30_000)"""