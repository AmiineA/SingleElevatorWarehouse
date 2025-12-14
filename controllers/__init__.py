from .naif_controller import naif_controller
from .dp_snapshot_controller import dp_controller      
from .dp_offline_controller import dp_offline_controller
from .global_fifo_controller import global_fifo_controller

def get_controller(name: str):
    """
    Retourne la fonction contrôleur correspondante.
    name ∈ {"naif", "dp", "offline_dp", "idiot"} par exemple.
    """
    table = {
        "naif": naif_controller,
        "dp": dp_controller,
        "idiot": global_fifo_controller,
        "offline_dp": dp_offline_controller,
    }
    try:
        return table[name]
    except KeyError:
        raise ValueError(f"Unknown controller_type: {name!r}")
