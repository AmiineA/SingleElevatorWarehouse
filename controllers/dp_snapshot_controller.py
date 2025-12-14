from collections import deque
from typing import Dict, List, Optional, Tuple
import simpy
from controllers.utils import record_snapshot, active_idle_towards_center


def _snapshot_H_from_queues(queues: Dict[int, deque], M_cap: int = None) -> Dict[int, List[int]]:
    """
    Builds, for each non-empty floor f, the list of destination floors of the pending requests.
    """
    H = {}
    for f, q in queues.items():
        if q:
            H[f] = [req.dest for req in q]
            if not M_cap is None:
                H[f] = H[f][:M_cap]
    return H

def _tau(l: int, a: int, h: int, speed: float, door: float, door_count: int = 3) -> float:
    """
    Computes the service time of a request:
    travel to pick up the item at floor a starting from position l, then deliver it to floor h.
    """
    sp = max(speed, 1e-12)
    travel = abs(l - a) / sp + abs(a - h) / sp
    return travel + door_count * door


def solve_snapshot(l0: int,
                   H: Dict[int, List[int]],
                   speed: float,
                   door: float,
                   door_count: int = 0,
                   global_memo: Optional[Dict] = None) -> Optional[int]:
    """
    Deterministic DP for a static snapshot.

    If global_memo is provided, it is used *in addition to* a local memo
    so computations are shared across snapshots.
    """
    floors = tuple(sorted(H.keys()))
    n_nonempty = len(floors)
    m = [len(H[f]) for f in floors]
    H_sig = tuple(tuple(H[f]) for f in floors)   
    snapshot_sig = (floors, H_sig)
    B = sum(m)
    if B == 0:
        return None

    # DP local memo for this snapshot
    local_memo: Dict[Tuple[int, Tuple], Tuple[float, int]] = {}

    def V(l: int, p_list: List[int], R: int) -> Tuple[float, int]:
        if R == 0:
            return (0.0, -1)

       # global memo key MUST include snapshot_sig
        gkey = (snapshot_sig, l, tuple(p_list))

        if global_memo is not None and gkey in global_memo:
            return global_memo[gkey]

        # local memo can stay per-snapshot
        lkey = (l, tuple(p_list))
        if lkey in local_memo:
            return local_memo[lkey]

        best_cost, best_k = float('inf'), -1

        for k in range(n_nonempty):
            if p_list[k] >= m[k]:
                continue

            a = floors[k]
            h = H[a][p_list[k]]
            tau = _tau(l, a, h, speed, door, door_count)

            p_list[k] += 1
            sub_cost, _ = V(h, p_list, R - 1)
            p_list[k] -= 1

            total = R * tau + sub_cost
            if total < best_cost:
                best_cost, best_k = total, k

        result = (best_cost, best_k)

        # Store into BOTH levels of memo
        if global_memo is not None:
            global_memo[gkey] = result
        local_memo[lkey] = result

        return result

    p0 = [0] * n_nonempty
    _, k = V(l0, p0, B)
    return None if k == -1 else floors[k]

def dp_controller(env: simpy.Environment,
               elev,
               queues: Dict[int, deque],
               cfg,
               stats: Dict[str, List[float]],
               wakeup_ref: List[simpy.events.Event],
               trace: Dict[str, List[float]],
               Verbose: Dict[str, List],
               global_pending: List[int]
               ):
    """
    Contrôleur dynamique P :
        - S'il existe au moins un colis en attente :
            Résoudre le programme dynamique déterministe sur le snapshot actuel
            pour choisir la PREMIÈRE action optimale :
                → Prendre à l'étage a* et livrer à l'étage h*.
            Puis re-snapshot et re-résoudre à l'étape suivante.

        - Si le système est vide :
            * 'passive' : dormir en attendant un réveil (les arrivées déclenchent l'événement).
            * 'active'  : dériver vers l'étage central ; si déjà sur place, dormir.
    Args:
        env (simpy.Environment): Environnement SimPy gérant le temps et les processus.
        elev (ElevatorState): État courant de l'ascenseur (position, vitesse, etc.).
        queues (Dict[int, deque]): Dictionnaire associant à chaque étage une file FIFO de requêtes en attente.
        cfg (SimConfig): Objet de configuration de la simulation (paramètres généraux).
        stats (Dict[str, List[float]]): Dictionnaire pour stocker les métriques collectées.
        wakeup_ref (List[simpy.events.Event]): Référence mutable vers l'événement de réveil utilisé pour sortir le contrôleur de sa veille.
        trace (Dict[str, List[float]]): Dictionnaire pour tracer la convergence de W(t).
        Verbose (Dict[str, List]): Dictionnaire de logs pour les visualisations.
        global_pending (List[int]): Nombre de colis actuellement en attente dans le système (utilisé pour déterminer l'inactivité du système en O(1)).
    """
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
                # Politique passive : on dort
                yield wakeup_ref[0]
                wakeup_ref[0] = simpy.Event(env)
                continue
            elif cfg.policy == "active":
                #On va vers l'étage central si aucune requête
                yield from active_idle_towards_center(env, elev, queues, cfg, wakeup_ref, Verbose, global_pending)
                continue

        # 3) Système non vide: choisir l'action à faire avec la prog. dyn.
        H = _snapshot_H_from_queues(queues)
        a_star = solve_snapshot(elev.floor, H, elev.speed, elev.door, door_count=3)

        if a_star is None: #Test de sécurité
            raise RuntimeError("DP returned no action in nonempty state.")
        target = a_star

        # 4) Déplacement vers la cible
        if target != elev.floor:
            yield from elev.travel_to(target)
            record_snapshot(env, elev, queues, Verbose)
        # 5) Pickup en tête de la file
        if queues[target]: #Test de sécurité
            carrying = queues[target].popleft()
            global_pending[0] = max(global_pending[0] - 1, 0)
            if elev.door > 0:
                yield env.timeout(elev.door)                 
            carrying.t_pick = env.now
            record_snapshot(env, elev, queues, Verbose)