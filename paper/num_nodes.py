import matplotlib.pyplot as plt
import numpy as np

FEEDBACK = 412
TR = 150
DATA = 284
ACK = 347

GUARD = 20
GUARD_END = 29
DQN_M = 32
DQN_N = 128

TOTAL_TIME = FEEDBACK + GUARD + TR * DQN_M + GUARD + DATA * DQN_N + GUARD + \
        ACK + GUARD_END

DATA_PER_SLOT = 12

trasmission_data = DATA_PER_SLOT * DQN_N


def calculate_rate(num_of_nodes):
    return trasmission_data / num_of_nodes / (TOTAL_TIME / 1000 / 60 / 60)

nodes = np.arange(1000, 5000, 200)

fig = plt.figure(figsize=(8,4))
plt.axhline(y=30,linewidth=1.5, color="g", linestyle="--")
plt.plot(nodes, calculate_rate(nodes), linewidth=2.5)
plt.title("DQ-N Number of Nodes vs. Bandwidth Per Node", fontsize=24)
plt.xlabel("Number of nodes", fontsize=18)
plt.ylabel("Bytes per Hour per Node", fontsize=18)
plt.tick_params(axis='both', which='major', labelsize=15)


plt.axis('tight')
#plt.show()
fig.savefig("nodes.pdf", dpi=300, transparent=True, bbox_inches="tight", pad_inches=0.0)

