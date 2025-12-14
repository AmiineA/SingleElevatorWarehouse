# Simulation d'un ascenseur — Modèle à événements discrets avec SimPy

Ce projet simule un système multi-étages dans lequel des colis arrivent selon des processus de Poisson à chaque étage, et un ascenseur unique est responsable de leur prise en charge et de leur livraison. Le tout est implémenté avec **SimPy**, une bibliothèque Python pour la simulation à événements discrets.

---

##  Notions SimPy utilisées dans ce projet

SimPy repose sur des processus (fonctions génératrices Python) qui avancent dans le temps via des événements (`timeout`, `event.succeed`, etc.). Chaque action qui prend du temps dans la simulation se fait avec un `yield`.


### 1. `yield env.timeout(durée)`
> Simule l'attente d'une durée de temps.

```python
yield env.timeout(2.5)
```

Cela signifie que le processus est suspendu pendant 2.5 unités de temps simulé. Il ne reprend que lorsque cette durée est écoulée. Pendant ce temps, d'autres processus continuent à s'exécuter.

**Dans notre projet :**
- utilisé pour simuler le temps de déplacement entre les étages
- ou le temps d’ouverture/fermeture des portes (`elev.door`)



### 2. `yield from ...`
> Délègue l’exécution à une autre fonction génératrice. On l'utilise lorsqu'on veut inclure une sous-action dans un processus SimPy

```python
def travel_to(floor):
    # Calcul du temps de déplacement
    dist = abs(floor - self.floor)
    self.floor = floor
    yield env.timeout(dist / self.speed)

# Dans le contrôleur :
[...]
yield from elev.travel_to(target_floor)
[...]
```
Cela attend automatiquement que la sous-fonction ait fini (c'est-à-dire que son `yield env.timeout(dist / self.speed)` se termine).



### 3. `simpy.Event(env)` et `.succeed()`
> Permet de créer des événements personnalisés déclenchés manuellement.

Un `Event` est un objet SimPy vide qui n’est résolu que lorsque quelqu’un appelle `.succeed()`.

```python
# Création
wake_event = simpy.Event(env)

# Quelqu’un attend cet événement
yield wake_event

# Un autre processus peut le déclencher :
wake_event.succeed()
```
On peut donc synchroniser plusieurs processus:
- un processus dort jusqu’à ce qu’il soit réveillé (c'est-à-dire on sort de la fonction (=processus) à la ligne du yield)
- un autre le réveille en appelant `.succeed()` (réveille = revenir à la ligne suivante du yield et continuer l'exécution dans le processus)



### 4. `yield wakeup_ref[0]` et `triggered`
> Gère la mise en pause/réveil de l’ascenseur quand il n’y a plus de colis.

Dans notre code, `wakeup_ref` est une liste mutable contenant l’événement courant de réveil.

```python
# Côté contrôleur :
yield wakeup_ref[0]
wakeup_ref[0] = simpy.Event(env)  # Prépare le prochain dodo

# Côté arrivées :
if not wakeup_ref[0].triggered:
    wakeup_ref[0].succeed()  # Réveille le contrôleur
```

Ce mécanisme évite de boucler inutilement (car l'action de l'ascenseur est géré par un While True). L’ascenseur dort tant qu’il n’y a rien à faire — et il est réveillé dès qu’un colis arrive.


### 5. `env.process(...)`
> Lance une fonction comme **processus SimPy**.

Chaque fonction avec un `yield` ne fonctionne que si on l’enregistre comme processus dans l’environnement :

```python
def mon_processus(env):
    yield env.timeout(1)

env.process(mon_processus(env))  # Démarre le processus
```

---

## Résumé des commandes SimPy utilisées

| Instruction                      | Rôle                                                 |
|----------------------------------|-------------------------------------------------------|
| `yield env.timeout(t)`           | Attend pendant `t` unités de temps simulé            |
| `yield from elev.travel_to(f)`   | Appelle un sous-processus de déplacement             |
| `simpy.Event(env)`               | Crée un événement à déclencher manuellement          |
| `event.succeed()`                | Déclenche un événement pour réveiller un processus   |
| `event.triggered`                | Vérifie si un événement a déjà été déclenché         |
| `env.process(...)`               | Démarre un processus dans la simulation              |


## Installation de SimPy


```bash
pip install simpy
```

##  Documentation officielle

 [SimPy — Docs Officielles](https://simpy.readthedocs.io/en/latest/index.html)