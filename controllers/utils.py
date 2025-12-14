from collections import deque
from typing import Dict, List, Optional, Tuple
import simpy



def record_snapshot(env, elev, queues, Verbose):
    """Enregistre l'état actuel du système pour le mode verbose."""
    if Verbose is None :
        return
    Verbose["t"].append(env.now)
    Verbose["queues"].append([len(queues[f]) for f in range(len(queues))])
    if elev is not None:
        Verbose["elevator_pos"].append(elev.floor)
    else:
        # Repeat last elevator position if unknown
        last = Verbose["elevator_pos"][-1] if Verbose["elevator_pos"] else 0
        Verbose["elevator_pos"].append(last)

def active_idle_towards_center(env: simpy.Environment,
                               elev,
                               queues: Dict[int, deque],
                               cfg,
                               wakeup_ref: List[simpy.events.Event],
                               Verbose,
                               global_pending: List[int]):
    """
    Comportement en mode actif lorsque le système est vide :
      - Se déplacer étage par étage vers l'étage central (cfg.n // 2).
      - Après chaque étage parcouru, vérifier global_pending[0] :
        si la valeur est toujours 0, continuer à se rapprocher du centre ;
        si elle devient strictement positive, s'arrêter et rendre la main
        au contrôleur pour qu'il traite le système désormais non vide.
      - Si l'ascenseur atteint l'étage central et que le système est toujours vide,
        se mettre en attente sur wakeup_ref.
    """
    center = cfg.n // 2
    step = 1 if center > elev.floor else -1
    # Se déplacer étage par étage et vérifier si le système est encore vide à chaque fois
    while global_pending[0] == 0 and elev.floor != center:
        next_floor = elev.floor + step

        yield from elev.travel_to(next_floor)
        record_snapshot(env, elev, queues, Verbose)

    if global_pending[0] == 0:
        yield wakeup_ref[0]
        wakeup_ref[0] = simpy.Event(env)

def choose_nearest_nonempty(current_floor: int, queues: Dict[int, deque]) -> Optional[int]:
    """
    Retourne l'étage non vide le plus proche de `current_floor`, ou None si toutes les files sont vides.

    Args:
        current_floor (int): position actuelle de l'ascenseur.
        queues (Dict[int,deque]): files par étage.

    Returns:
        int | None: index d'étage cible, ou None si aucune file.

    Examples:
        >>> qs = {0: deque([1]), 1: deque([]), 2: deque([2,3])}
        >>> choose_nearest_nonempty(1, qs) in {0,2}
        True
    """
    nonempty = [f for f, q in queues.items() if q]
    if not nonempty:
        return None
    return min(nonempty, key=lambda f: abs(f - current_floor))