from typing import Dict, List, Optional, Tuple
import simpy  
from collections import deque


from controllers.utils import record_snapshot, active_idle_towards_center, choose_nearest_nonempty


def naif_controller(env: simpy.Environment,
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
    Processus SimPy contrôlant le comportement de l'ascenseur selon la politique naive:
    aller chercher le colis le plus proche quand l'étage actuel est vide et qu'il existe un autre étage non vide 

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
        # 1) Si on transporte déjà un colis -> aller livrer.
        if carrying is not None:
            if elev.door > 0:
                yield env.timeout(elev.door)         
            yield from elev.travel_to(carrying.dest) 
            if elev.door > 0:
                yield env.timeout(elev.door)         
            carrying.t_drop = env.now               
            record_snapshot(env, elev, queues, Verbose)

            # Enregistrement des stats
            if carrying.t_create >= cfg.warmup:
                w = carrying.t_pick - carrying.t_create
                s = carrying.t_drop - carrying.t_create
                stats["completed"] += 1
                stats["wait_times"].append(w)
                stats["system_times"].append(s)
                cum_wait += w
                count += 1
                trace["t"].append(env.now)
                trace["avgW"].append(cum_wait / count)    

            carrying = None                          
            continue

        # 2) Cabine vide : tenter un pickup local (FIFO) à l'étage courant.
        q_here = queues[elev.floor]
        if q_here:
            carrying = q_here.popleft()              
            global_pending[0] = max(global_pending[0] - 1, 0)
            if elev.door > 0:
                yield env.timeout(elev.door)         
            carrying.t_pick = env.now               
            record_snapshot(env, elev, queues, Verbose) 
            continue

        # 3) Sinon, choisir l'étage non vide le plus proche.
        if global_pending[0] > 0:
            target = choose_nearest_nonempty(elev.floor, queues)
        else:
            if cfg.policy == "passive":
                # Politique passive : on dort
                yield wakeup_ref[0]
                wakeup_ref[0] = simpy.Event(env)
                continue
            elif cfg.policy == "active":
                # On va vers l'étage central si aucune requête
                yield from active_idle_towards_center(env, elev, queues, cfg, wakeup_ref, Verbose, global_pending)
                continue
        # 5) Déplacement vers la cible et pickup en arrivant si la file est non vide.
        if target != elev.floor:
            yield from elev.travel_to(target)        
            record_snapshot(env, elev, queues, Verbose)
        if queues[target]:
            carrying = queues[target].popleft()      
            global_pending[0] = max(global_pending[0] - 1, 0)
            if elev.door > 0:
                yield env.timeout(elev.door)         
            carrying.t_pick = env.now                
            record_snapshot(env, elev, queues, Verbose) 