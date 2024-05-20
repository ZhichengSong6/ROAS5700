from model import DynamicsModel
import numpy as np

model = DynamicsModel(10, 20, np.eye(6), np.eye(2))
model.solve()