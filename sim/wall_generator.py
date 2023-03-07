import sys
import random

def generate_random_pose():
    x = random.uniform(-4.0, 4.0)
    y = random.uniform(-4.0, 4.0)
    yaw = random.uniform(-3.14, 3.14)
    return "%.4f %.4f 0.0001 0 0 %.4f" % (x, y, yaw)

# Forme de chaque model
model_template = """
  <model name="grey_wall_{idx}">
    <include>
      <uri>model://grey_wall</uri>
    </include>
    <pose>{pose}</pose>
  </model>
"""

if len(sys.argv) != 2:
    num_models = 3
else:
    num_models = int(sys.argv[1])

# Generer les modèles avec des positions aléatoires
models = []
for i in range(num_models):
    pose = generate_random_pose()
    models.append(model_template.format(idx=i, pose=pose))

# Combine all models into a single string
models_str = "\n".join(models)

file_path = "./personalised.world"

with open(file_path, "r") as file:
    content = file.readlines()

# Trouve la ligne où ajouter les modèle
for i, line in enumerate(content):
    if """<world name="default">""" in line:
        content.insert(i + 1, models_str)
        break

with open(file_path, "w") as f:
    f.write("".join(content))
