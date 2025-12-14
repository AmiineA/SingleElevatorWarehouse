import matplotlib.pyplot as plt

# Data from simulations
n_values = [3, 4, 5, 6, 7, 8, 9]

lambdas_critiques_idiot = [0.148, 0.0858, 0.0554, 0.0388, 0.0286, 0.0221, 0.0174]
W_critique_idiot = [82, 340, 235, 560, 278, 354, 232]

lambdas_critiques_naif = [0.2495, 0.1482, 0.0945, 0.0689, 0.0513, 0.0405, 0.0317]
W_critique_naif = [1187, 1417, 236, 634, 747, 1076, 737]

lambdas_critiques_dp = [0.2195, 0.14, 0.09, 0.06, 0.044, 0.034, 0.026]

theoretical_lambdas_critiques = [ 3/(n*(n+1)) for n in n_values]
theoretical_lambdas_fifo = [3/(2*n*n + n -1) for n in n_values]


plt.figure(figsize=(9, 5))

# --- empirical curves
plt.plot(n_values, lambdas_critiques_idiot,
         marker="o", linewidth=2, label="Idiot controller", color="C1")

plt.plot(n_values, lambdas_critiques_naif,
         marker="s", linewidth=2, label="Naïf controller", color="C0")

plt.plot(n_values, lambdas_critiques_dp,
         marker="^", linewidth=2, label="DP controller", color="C2")

# --- theoretical curves
plt.plot(n_values, theoretical_lambdas_critiques,
         linestyle="--", linewidth=2, color="black",
         label=r"Theory $3/n(n+1)$")

plt.plot(n_values, theoretical_lambdas_fifo,
         linestyle="--", linewidth=2, color="red",
         label=r"Theory FIFO $3/(2n^2+n-1)$")

# --- cosmetics
plt.xlabel("Number of floors $n$", fontsize=12)
plt.ylabel(r"Critical arrival rate $\lambda_{\mathrm{critical}}$", fontsize=12)
plt.title("Critical arrival rate vs number of floors", fontsize=14)

plt.grid(True, alpha=0.3)
plt.legend(frameon=False)
plt.tight_layout()
plt.show()
