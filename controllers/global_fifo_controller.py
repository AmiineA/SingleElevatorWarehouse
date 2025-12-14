from collections import deque
from typing import Dict, List, Optional
import simpy
from controllers.utils import record_snapshot, active_idle_towards_center

def choose_oldest_across_floors(queues: Dict[int, deque]) -> Optional[int]:
    """
    Retourne l'étage du colis le plus ancien (plus petit t_create) parmi toutes
    les files, en considérant le système comme une seule file globale FIFO.

    Returns:
        int | None: index d'étage de l'ancienneté minimale, ou None si toutes les files sont vides.
    """
    best_floor = None
    best_t = float("inf")
    for f, q in queues.items():
        if q:
            t = q[0].t_create   # tête de file à l'étage f
            if t < best_t:
                best_t = t
                best_floor = f
    return best_floor

def global_fifo_controller(env: simpy.Environment,
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
    Contrôleur "file globale" :
      - À chaque décision, si des colis sont présents,
        on considère conceptuellement le système comme UNE file globale,
        ordonnée par temps d'arrivée t_create.
      - On cherche l'étage dont la tête de file est la plus ancienne
        (plus petit t_create), on va à cet étage, on prend la tête,
        puis on livre, et on recommence.
      - Si le système est vide :
          * 'passive' : on dort en attendant un réveil.
          * 'active'  : on dérive vers l'étage central, puis on dort.

    Cela réalise une discipline "plus ancien colis d'abord" au niveau global.
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

        if global_pending[0] > 0:
            target = choose_oldest_across_floors(queues)
        else:
            # Système vide, comportement passif/actif
            if cfg.policy == "passive":
                yield wakeup_ref[0]
                wakeup_ref[0] = simpy.Event(env)
                continue
            elif cfg.policy == "active":
                yield from active_idle_towards_center(env, elev, queues, cfg, wakeup_ref, Verbose, global_pending)
                continue

        
        if target is None:
            # Incohérence : global_pending>0 mais toutes les files sont vides
            pending = sum(len(q) for q in queues.values())
            raise RuntimeError(
                f"global_fifo_controller: pending={global_pending[0]} mais aucune file non vide "
                f"(recount={pending}, floor={elev.floor})"
            )

        # 4) Aller à l'étage choisi 
        if target != elev.floor:
            yield from elev.travel_to(target)
            record_snapshot(env, elev, queues, Verbose)

        # 5) Pickup FIFO à cet étage 
        if queues[target]:
            carrying = queues[target].popleft()
            global_pending[0] = max(global_pending[0] - 1, 0)
            if elev.door > 0:
                yield env.timeout(elev.door)          # porte au pickup
            carrying.t_pick = env.now
            record_snapshot(env, elev, queues, Verbose)