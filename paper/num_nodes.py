import matplotlib as mpl
mpl.use('Agg')

import matplotlib.pyplot as plt
import numpy as np

FEEDBACK = 1592
TR = 150
DATA = 543
ACK = 281

GUARD = 15
GUARD_END = 15
DQN_M = 80
DQN_N = 64

TOTAL_TIME = FEEDBACK + GUARD + TR * DQN_M + GUARD + DATA * DQN_N + GUARD + \
        ACK + GUARD_END

DATA_PER_SLOT = 36

trasmission_data = DATA_PER_SLOT * DQN_N


def dqn_calculate_rate(num_of_nodes):
    return trasmission_data / num_of_nodes / (TOTAL_TIME / 1000 / 60 / 60)

def lorawan_calculate_rate(num_of_nodes):
    return DATA_PER_SLOT / num_of_nodes / (DATA / 1000 / 60 / 60) / 2 / np.e

nodes = np.arange(1000, 7000, 200)

fig = plt.figure(figsize=(8,4))
plt.axhline(y=30,linewidth=1.5, color="g", linestyle="--")
plt.plot(nodes, dqn_calculate_rate(nodes), linewidth=2.5, label="DQ-N")
plt.plot(nodes, lorawan_calculate_rate(nodes), linewidth=2.5, label="pure-ALOHA (LoRaWAN)")

plt.legend()

plt.title("Number of Nodes vs. Bandwidth Per Node", fontsize=24)
plt.xlabel("Number of nodes", fontsize=18)
plt.ylabel("Bytes per Hour per Node", fontsize=18)
plt.tick_params(axis='both', which='major', labelsize=15)


plt.axis('tight')
#plt.show()
fig.savefig("nodes.pdf", dpi=300, transparent=True, bbox_inches="tight", pad_inches=0.0)

