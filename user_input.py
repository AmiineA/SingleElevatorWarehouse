import random
from typing import List

def demander_float(message: str, strictly_positive: bool = False) -> float:
    while True:
        try:
            val = float(input(message))
            if strictly_positive and val <= 0:
                print("Erreur : veuillez entrer un nombre strictement positif.")
                continue
            return val
        except ValueError:
            print("Entrée invalide, veuillez entrer un nombre.")


def demander_int(message: str) -> int:
    while True:
        try:
            return int(input(message))
        except ValueError:
            print("Entrée invalide, veuillez entrer un entier.")


def demander_liste_float(message: str, n: int) -> List[float]:
    while True:
        try:
            ligne = input(message)
            valeurs = list(map(float, ligne.strip().split(",")))
            if len(valeurs) != n:
                print(f"Erreur : vous devez entrer exactement {n} valeurs.")
                continue
            return valeurs
        except ValueError:
            print("Entrée invalide, veuillez entrer des nombres séparés par des virgules.")


def demander_lambdas(n: int) -> List[float]:
    while True:
        ligne = input("Valeurs de λ_i (un seul nombre pour tous les étages ou une liste séparée par des virgules) : ").strip()
        try:
            # Cas 1 : liste
            if "," in ligne:
                lambdas = list(map(float, ligne.split(",")))
                if len(lambdas) != n:
                    print(f"Erreur : vous devez entrer exactement {n} valeurs.")
                    continue
                if any(l < 0 for l in lambdas):
                    print("Erreur : toutes les valeurs de λ doivent être positives ou nulles.")
                    continue
                return lambdas
            else:
                # Cas 2 : valeur unique
                val = float(ligne)
                if val < 0:
                    print("Erreur : λ doit être un nombre positif ou nul.")
                    continue
                return [val for _ in range(n)]
        except ValueError:
            print("Erreur de format : veuillez entrer un nombre ou une liste de nombres séparés par des virgules.")


def demander_distribution(n: int) -> List[List[float]]:
    matrice = []
    for i in range(n):
        while True:
            ligne = demander_liste_float(f"Ligne {i+1} de la matrice de probabilité (séparée par des virgules) : ", n)
            if ligne[i] != 0:
                print("La diagonale doit être nulle (pas de destination identique à l'origine).")
                continue
            if any(v < 0 for v in ligne):
                print("Erreur : les probabilités doivent être positives.")
                continue
            if abs(sum(ligne) - 1.0) > 1e-6:
                print("Erreur : la somme de la ligne doit être égale à 1.")
                continue
            matrice.append(ligne)
            break
    return matrice


def generer_graine() -> int:
    seed = random.randint(0, 10**6)
    return seed


def demander_verbose() -> bool:
    rep = input("Souhaitez-vous activer le mode verbose (affichage des files et ascenseur) ? [y/n] : ").strip().lower()
    return rep == "y"


def demander_politique() -> str:
    while True:
        rep = input("Choisir la politique de l'ascenseur : (p) passive ou (a) active ? ").strip().lower()
        if rep in ("p", "passive"):
            return "passive"
        elif rep in ("a", "active"):
            return "active"
        else:
            print("Erreur : entrée invalide. Veuillez entrer 'p' ou 'a'.")

def demander_controleur() -> str:
    while True:
        rep = input("Choisir le contrôleur : (n) naïf, (d) dynamique (DP), (i) idiot, (o) offline_dp ? ").strip().lower()
        if rep in ("n", "naif", "naïf"):
            return "naif"
        if rep in ("d", "dp", "dyn", "dynamique"):
            return "dp"
        if rep in ("i", "I", "idiot"):
            return "idiot"
        if rep in ("offline_dp", "o", "memoisation"):
            return "offline_dp"
        print("Erreur : veuillez entrer 'n','d' ou 'i'.")
