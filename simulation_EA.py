"""
Simulation d'un ascenseur (1 cabine, capacité 1) avec contrôleur explicite et files FIFO par étage.

Classes principales :
- SimConfig : paramètres de simulation (n étages, λ_i, loi de destination, etc.).
- Request   : un "colis" (origine, destination, timestamps).
- ElevatorState : position, vitesse, distance parcourue ; déplacements via env.timeout(...).
- arrivals_process (Processus simpy): arrivées modélisées par des processus de Poisson indépendants par étage (file FIFO).
- XXX_controller : logique de déplacement/prise en charge selon la politique demandée.
- run_one_rep : assemble, exécute, renvoie les métriques.

"""
import math
import random
from collections import deque
from typing import Dict, List, Optional, Tuple
import simpy  
from controllers.utils import record_snapshot

from controllers import get_controller

class SimConfig:
    """
    Paramètres d'entrée du simulateur.

    Args:
        n_floors (int): nombre total d'étages (>= 2).
        lambdas_per_floor (List[float]): intensités d'arrivée par étage [λ_0, ..., λ_{n-1}].
        uniform_dest (bool): si True, destination uniforme j≠i.
        P_dest (List[List[float]] | None): matrice n*n de probabilités de destination
            (ligne i normalisée / P[i][i] = 0) ; ignorée si uniform_dest=True.
        init_floor (int): étage de départ de l'ascenseur.
        speed (float): vitesse en étages / unité de temps (>= 0+).
        door_time (float): temps fixe par arrêt (appliqué au pickup et au drop).
        sim_time (float): horizon de simulation (temps simulé).
        warmup (float): période d'échauffement ; on exclut les demandes créées avant cette borne des stats.
        seed (int): graine aléatoire (reproductibilité).
        policy (str): politique de l'ascenseur ("passive", "active" : définit l'action quand le système entier est vide)
        Verbose(Dict): prises de 'photos' pour debug

    Raises:
        AssertionError/ValueError: si incohérences (dimensions de P, normalisation, diag).
    """
    def __init__(
        self,
        n_floors: int,
        lambdas_per_floor: List[float],
        uniform_dest: bool = True,
        P_dest: Optional[List[List[float]]] = None,
        init_floor: int = 0,
        speed: float = 1.0,
        door_time: float = 0.0,
        sim_time: float = 20_000.0,
        warmup: float = 2_000.0,
        seed: int = 123,
        policy: str = "passive",
        Verbose: Dict = None,
        M: int = 1
    ):
        assert n_floors >= 2
        assert len(lambdas_per_floor) == n_floors 
        if P_dest is not None:
            assert len(P_dest) == n_floors and all(len(row) == n_floors for row in P_dest) 
            for i in range(n_floors):
                if abs(sum(P_dest[i]) - 1.0) > 1e-9 or abs(P_dest[i][i]) > 1e-12:
                    raise ValueError("Chaque ligne de P doit sommer à 1 et P[i][i] = 0.")

        self.Verbose = Verbose
        self.policy = policy               
        self.n = int(n_floors)                      # nb d'étages
        self.lambdas = list(lambdas_per_floor)      # λ_i par étage
        self.P = P_dest                              # matrice P ou None
        self.uniform_dest = bool(uniform_dest)       # destination uniforme ?
        self.init_floor = int(init_floor)            # position initiale de l'ascenseur
        self.speed = float(speed)                    # vitesse (étages/unité de temps)
        self.door = float(door_time)                 # temps portes (par arrêt)
        self.sim_time = float(sim_time)              # horizon de simu
        self.warmup = float(warmup)                  # échauffement (exclusion des stats)
        self.seed = int(seed)                        # graine aléatoire
        self.M = M                                  # valeur pour tronquer la DP

class Request:
    """
    Représente un colis : (origin, dest) + timestamps (création/pickup/drop).

    Attributes:
        id (int): identifiant unique.
        origin (int): étage d'origine.
        dest (int): étage de destination (<> origin).
        t_create (float): temps d'arrivée dans la file d'origine.
        t_pick (float): temps de prise en charge (pickup).
        t_drop (float): temps de livraison (drop).
    """
    __slots__ = ("id", "origin", "dest", "t_create", "t_pick", "t_drop")
    def __init__(self, rid: int, origin: int, dest: int, t: float):
        self.id = rid
        self.origin = int(origin)
        self.dest = int(dest)
        self.t_create = float(t)
        self.t_pick = math.nan
        self.t_drop = math.nan

def sample_destination(i: int, n: int, cfg: SimConfig) -> int:
    """
    Tire une destination j ≠ i selon la loi demandée.

    Examples:
        >>> cfg = SimConfig(3, [0.1,0.1,0.1], uniform_dest=True)
        >>> j = sample_destination(0, 3, cfg); assert j in {1,2}
    """
    if cfg.uniform_dest or cfg.P is None:
        j = i
        while j == i:
            j = random.randrange(n)  
        return j
    # tirage catégoriel sur la ligne i de P
    r, s = random.random(), 0.0
    for j in range(n):
        s += cfg.P[i][j]
        if r <= s:
            return j
    # repli (en cas de légères erreurs numériques sur la somme)
    return max(range(n), key=lambda j: cfg.P[i][j])

class ElevatorState:
    """
    État minimal de l'ascenseur : étage courant, vitesse, temps portes, distance cumulée.

    Args:
        env (simpy.Environment): environnement de simulation.
        cfg (SimConfig): configuration (n, init_floor, speed, door_time).

    Methods:
        travel_to(target_floor): simule le déplacement vers un étage (avance l'horloge).
    """
    def __init__(self, env: simpy.Environment, cfg: SimConfig):
        self.env = env                              
        self.n = cfg.n                               
        self.floor = cfg.init_floor                  
        self.speed = cfg.speed                       
        self.door = cfg.door                         
        self.distance_total = 0.0                    

    def travel_to(self, target_floor: int):
        """
        Simule le déplacement de l'ascenseur jusqu'à `target_floor`.

        Effet :
            - Accumule la distance (en étages).
            - Attend (dist / speed) en temps simulé.
            - Met à jour self.floor.
        """
        if target_floor == self.floor:
            return
        dist = abs(target_floor - self.floor)       
        self.distance_total += dist                  
        # Avance le temps simulé d'une durée (dist / speed).                    
        yield self.env.timeout(dist / max(self.speed, 1e-12))  
        self.floor = target_floor                    

def arrivals_process(env: simpy.Environment,
                     floor_id: int,
                     queues: Dict[int, deque],
                     cfg: SimConfig,
                     id_counter,
                     wakeup_ref: List[simpy.events.Event],
                     Verbose: Dict[str, List],
                     global_pending: List[int]):
    """
    Processus SimPy qui génère les arrivées à un étage donné (Poisson λ_i).

    Args:
        env: environnement SimPy (horloge).
        floor_id: index d'étage (0..n-1).
        queues: dictionnaire {étage -> deque FIFO}.
        cfg: configuration générale (notamment λ_i, loi de destination).
        id_counter: générateur d'identifiants (yield i).
        wakeup_ref: [Event] mutable pour réveiller le contrôleur quand une arrivée survient.
        Verbose: permet de prendre des photos si <> None pour debug
        global_pending: var globale qui enregistre nombre de colis présents dans le système

    Comportement :
        - Inter-arrivées ~ Exp(λ_i) => attente via env.timeout(dt).
        - À chaque arrivée : choix de destination j ≠ i, création d'un Request, append en queue.
        - On déclenche wakeup_ref[0] pour sortir le contrôleur de sa sieste si besoin.
    """
    lam = cfg.lambdas[floor_id]                      
    if lam <= 0:
        return
    while True:
        dt = random.expovariate(lam)                 
        yield env.timeout(dt)                        
        rid = next(id_counter)                      
        dest = sample_destination(floor_id, cfg.n, cfg)  
        req = Request(rid, floor_id, dest, env.now) 
        queues[floor_id].append(req)
        global_pending[0] += 1                 
        record_snapshot(env, None, queues, Verbose)
        if not wakeup_ref[0].triggered:
            wakeup_ref[0].succeed()                 
        wakeup_ref[0] = simpy.Event(env)            

def run_one_rep(cfg: SimConfig, controller_type: str = "naif") -> Dict[str, float]:
    """
    Construit l'environnement, lance les process d'arrivées + le contrôleur,
    exécute jusqu'à cfg.sim_time, puis renvoie les métriques agrégées.

    Args:
        cfg (SimConfig): configuration complète de la réplication.

    Returns:
        dict: {
            "completed": int,
            "wait": list,
            "system": list,
        }
    """
    # Graine pour la reproductibilité des tirages aléatoires.
    random.seed(cfg.seed)

    env = simpy.Environment()                                                   
    queues: Dict[int, deque] = {f: deque() for f in range(cfg.n)}
    elev = ElevatorState(env, cfg)
    trace = {"t": [], "avgW": []}    
    stats = {
        "completed": 0,
        "wait_times": [],
        "system_times": [],
    }

    Verbose = cfg.Verbose 
    wakeup_ref = [simpy.Event(env)]    
    global_pending = [0]  # liste pour pouvoir être modifiée dans les processus                                     
    controller = get_controller(controller_type)

    def id_gen():
        i = 0
        while True:
            yield i
            i += 1
    idc = id_gen()
    for f in range(cfg.n):
        env.process(arrivals_process(env, f, queues, cfg, idc, wakeup_ref, Verbose, global_pending))

    env.process(controller(env, elev, queues, cfg, stats, wakeup_ref, trace, Verbose, global_pending))            

    env.run(until=cfg.sim_time)                                                   

    return {
        "completed": stats["completed"],
        "wait_times": stats["wait_times"],
        "system_times": stats["system_times"],
        "trace_t": trace["t"],             
        "trace_avgW": trace["avgW"],    
        "Verbose": Verbose,   
    }

    
if __name__ == "__main__":
    
    from user_input import (
    demander_int, demander_float, demander_politique, demander_controleur,
    demander_distribution, generer_graine, demander_lambdas, demander_verbose
    )
    
    # INTERACTION UTILISATEUR 
    print("Bienvenue dans le simulateur interactif ")
    n = demander_int("Combien d'étages dans le système ? ")
    lambdas = demander_lambdas(n)

    rep = input("Souhaitez-vous une loi uniforme de destination ? [y/n] : ").strip().lower()
    if rep == "y":
        uniform = True
        P = None
    else:
        uniform = False
        P = demander_distribution(n)


    init_floor = demander_int("À quel étage démarre l'ascenseur ? ")
    controller = demander_controleur()
    policy = demander_politique()

    Verbose = {  
    "t": [],
    "elevator_pos": [],
    "queues": [],
    } if demander_verbose() else None

    sim_time = demander_float("Durée de la simulation ? ")

    seed = generer_graine()
    warmup = 0

    cfg = SimConfig(
        n_floors=n,
        lambdas_per_floor=lambdas,
        P_dest=P,
        uniform_dest=uniform,
        init_floor=init_floor,
        speed=1.0,
        door_time=0.0,
        sim_time=sim_time,
        warmup=warmup,
        seed=seed,
        policy = policy,
        Verbose = Verbose,
        M = 1

    )

    # Lancer une réplication et afficher les résultats :
    out = run_one_rep(cfg, controller)
    mean = lambda x: (sum(x)/len(x)) if x else float("nan")

    avg_wait = mean(out["wait_times"])
    avg_system = mean(out["system_times"])
    
    print("=== Résultats ===")
    print(f"Colis livrés   : {out['completed']}")
    print(f"W moyen        : {avg_wait:.4f}")
    print(f"S moyen        : {avg_system:.4f}")
    print(f"Graine utilisée pour cette simulation : {seed}")
    #print(f"W théorique        : {W9:.4f}")
    from plotting import plot_all
    plot_all(out, cfg)


