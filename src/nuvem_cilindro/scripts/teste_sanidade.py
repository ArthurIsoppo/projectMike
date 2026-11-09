import pybullet as p, pybullet_data, time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

r, h = 0.05, 0.20  # raio e altura em metros
col = p.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=h)
vis = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=[0.8, 0.2, 0.2, 1])
p.createMultiBody(baseMass=0.5, baseCollisionShapeIndex=col,
                  baseVisualShapeIndex=vis, basePosition=[0, 0, h/2])

while p.isConnected():
    p.stepSimulation()
    time.sleep(1/240)
