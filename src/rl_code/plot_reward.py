import matplotlib.pyplot as plt
import pickle


reward= list()
with open('rewards.pickle', "rb") as f:
    reward= pickle.load(f)

plt.plot(reward)
plt.show()