import math
from collections import deque
from typing import Dict, List, Tuple, Optional
import simpy

from controllers.utils import record_snapshot, active_idle_towards_center
from controllers.dp_snapshot_controller import solve_snapshot, _snapshot_H_from_queues


# Tolerance for overflow probability in M formula 
EPSILON_M = 1e-2

# Hard cap on allowed DP state count (rough lower bound)
MAX_STATES_ESTIMATE = 1_000_000

# Single global memo for offline DP:
#   dp_memo = {"M": int, "action": { state_key -> best_target_floor }}
_DP_MEMO: Dict[Tuple, Dict] = {}



def compute_M(n: int, lambda_scalar: float, eps: float = EPSILON_M) -> int:
    """
    Compute a per-floor cap M from (n, lambda) using the earlier formula:

        rho = lambda * (2 n^2 + n - 1) / 3
        M >= (1/n) * ln(1/eps) / (-ln(rho))

    Here we just clamp rho if it gets too close to 1, to avoid numerical issues.
    """
    rho = lambda_scalar * (2 * n * n + n - 1) / 3.0
    if rho >= 0.95:
        print("Careful, rho is too close to 1, clamping to 0.9 for M computation.")
        rho = 0.9
    numerator = math.log(1.0 / eps)
    denominator = -math.log(rho)
    M_real = numerator / (n * denominator)
    M = max(1, math.ceil(M_real))
    return M


def estimate_state_count_lower_bound(n: int, M: int) -> int:
    """
    Very rough lower bound on the number of states if we only encode
    elevator floor + queue lengths:

        |S| >= n * (M + 1)^n

    If even this is too large, we abort the offline DP.
    """
    return n * (M + 1) ** n


def _snapshot_H_truncated(queues: Dict[int, deque], M: int) -> Dict[int, List[int]]:
    """
    Build H from queues (floor -> list of destinations), then truncate
    each floor's list to at most M destinations.
    """
    H = _snapshot_H_from_queues(queues)  # original helper: no cap
    for f, dests in H.items():
        if len(dests) > M:
            H[f] = dests[:M]
    return H


def _build_state_key(elev_floor: int, H: Dict[int, List[int]]) -> Tuple:
    """
    Build a canonical, hashable key from elevator floor and H.

    Key structure:
      (elev_floor, (tuple(H[f0]), tuple(H[f1]), ...))
    with floors sorted so that representation is stable.
    """
    items = tuple((f, tuple(H[f])) for f in sorted(H.keys()))
    return (elev_floor, items)


def offline_solve_first_action(elev_floor: int,
                               queues: Dict[int, deque],
                               cfg,
                               dp_memo: Dict) -> Optional[int]:
    """
    Wrapper around deterministic DP solver (solve_snapshot),
    with a memo (dp_memo["action"]) keyed on the snapshot state.

    Returns:
        next_target_floor (int) or None if no pending boxes.
    """
    M = dp_memo["M"]
    H = _snapshot_H_truncated(queues, M)
    if not H:
        return None
    key = _build_state_key(elev_floor, H)

    # If we've already solved this snapshot, reuse
    if key in dp_memo["action"]:
        return dp_memo["action"][key]

    # Otherwise, solve the deterministic DP once
    a_star = solve_snapshot(
    elev_floor, H, cfg.speed, cfg.door,
    global_memo=dp_memo["subDP"]
    )


    # Store in action table
    dp_memo["action"][key] = a_star
    return a_star


def get_or_build_dp_memo(cfg) -> Optional[Dict]:
    """
    For a given SimConfig, build (once) the offline DP memo:
       dp_memo = {"M": M, "action": {}}
    with a rough feasibility check on M via a state-count lower bound.

    Returns:
        dp_memo dict if feasible, else None (meaning: don't use offline DP).
    """
    n = cfg.n
    M = cfg.M
    key = (n, M)  # include what affects solve_snapshot costs
    if key in _DP_MEMO:
        return _DP_MEMO[key]

    
    S_lb = estimate_state_count_lower_bound(n, M)
    if S_lb > MAX_STATES_ESTIMATE:
        print(f"Offline DP: Too many states in lower bound (~{S_lb}), disabling.")
        _DP_MEMO[key] = None
        return None

    dp_memo = {"M": M, "action": {}, "subDP": {}}
    _DP_MEMO[key] = dp_memo
    print(f"Offline DP memo initialized for key={key} with M={M}, |S|lb~{S_lb}")
    return dp_memo


def dp_offline_controller(env: simpy.Environment,
                          elev,
                          queues: Dict[int, deque],
                          cfg,
                          stats: Dict[str, List[float]],
                          wakeup_ref: List[simpy.events.Event],
                          trace: Dict[str, List[float]],
                          Verbose,
                          global_pending: List[int]):
    """
    Offline DP controller (tabulated deterministic DP with per-floor cap M).

    - Before first use, build a single DP memo for the given configuration.
    - At each decision epoch:
        * if carrying a box: deliver it (same pattern as other controllers),
        * if system is empty: passive/active idle behavior,
        * otherwise: snapshot -> truncated H -> state key -> action from DP memo.
    """
    dp_memo = get_or_build_dp_memo(cfg)
    if dp_memo is None:
        raise RuntimeError("Offline DP infeasible for this configuration (state space too large).")

    carrying = None
    cum_wait, count = 0.0, 0

    while True:
        # 1) Si on transporte déjà un colis -> aller livrer
        if carrying is not None:
            if elev.door > 0:
                yield env.timeout(elev.door)
            yield from elev.travel_to(carrying.dest)
            if elev.door > 0:
                yield env.timeout(elev.door)
            carrying.t_drop = env.now
            record_snapshot(env, elev, queues, Verbose)

            # Stats
            if carrying.t_create >= cfg.warmup:
                w = carrying.t_pick - carrying.t_create
                s = carrying.t_drop - carrying.t_create
                stats["completed"] += 1
                stats["wait_times"].append(w)
                stats["system_times"].append(s)
                cum_wait += w
                count += 1
                trace["t"].append(env.now)
                trace["avgW"].append(cum_wait / max(1, count))

            carrying = None
            continue

        # 2) Système vide
        if global_pending[0] == 0:
            if cfg.policy == "passive":
                yield wakeup_ref[0]
                wakeup_ref[0] = simpy.Event(env)
                continue
            elif cfg.policy == "active":
                yield from active_idle_towards_center(env, elev, queues, cfg, wakeup_ref, Verbose, global_pending)
                continue

        # 3) Système non vide, cabine vide : choisir l'action via offline DP
        target = offline_solve_first_action(elev.floor, queues, cfg, dp_memo)

        if target is None:  # sécurité: incohérence si global_pending > 0
            raise RuntimeError("Offline DP: non-empty system but DP returned no target.")

        # 4) Déplacement vers la cible
        if target != elev.floor:
            yield from elev.travel_to(target)
            record_snapshot(env, elev, queues, Verbose)

        # 5) Pickup en tête de la file
        if queues[target]:
            carrying = queues[target].popleft()
            global_pending[0] = max(global_pending[0] - 1, 0)
            if elev.door > 0:
                yield env.timeout(elev.door)
            carrying.t_pick = env.now
            record_snapshot(env, elev, queues, Verbose)
