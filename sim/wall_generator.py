import sys
import random
import math

def is_position_valid(x, y, pose):
    # Calculate the distance between the point and the center of the object
    dx = x - pose[0]
    dy = y - pose[1]
    distance = math.sqrt(dx ** 2 + dy ** 2)

    # Calculate the length of the object along the x axis with a yaw applied
    length = 7
    angle = pose[2]
    length_x = abs(length * math.cos(angle))
    length_y = abs(length * math.sin(angle))

    # Calculate the coordinates of the four corners of the object
    corners = [(pose[0] + length_x / 2, pose[1] + length_y / 2),
               (pose[0] - length_x / 2, pose[1] - length_y / 2)]

    i=0
    j=1
    # Calculate the equation of the line segment in the form y = mx + b
    if corners[i][0] == corners[j][0]:
        # Vertical line, slope is infinite
        if x == corners[i][0] and min(corners[i][1], corners[j][1]) - 0.8 <= y <= max(corners[i][1], corners[j][1]) + 0.8:
            return False
    else:
        # Non-vertical line, calculate slope and y-intercept
        m = (corners[j][1] - corners[i][1]) / (corners[j][0] - corners[i][0])
        b = corners[i][1] - m * corners[i][0]
        if abs(y - (m * x + b)) < 0.8 and min(corners[i][0], corners[j][0]) <= x <= max(corners[i][0], corners[j][0]):
            return False
    return True

def generate_random_pose():
    x = random.uniform(-4.0, 4.0)
    y = random.uniform(-4.0, 4.0)
    yaw = random.uniform(-3.14, 3.14)
    return x, y, yaw

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

# Generer les mod�les avec des positions al�atoires
models = []
poses = []
for i in range(num_models):
    pose = generate_random_pose()
    poses.append(pose)
    models.append(model_template.format(idx=i, pose=("%.4f %.4f 0.0001 0 0 %.4f" % pose)))

# Combine all models into a single string
models_str = "\n".join(models)

file_path = "./personalised.world"

with open(file_path, "r") as file:
    content = file.readlines()

# Trouve la ligne o� ajouter les mod�le
for i, line in enumerate(content):
    if """<world name="default">""" in line:
        content.insert(i + 1, models_str)
        break

with open(file_path, "w") as f:
    f.write("".join(content))

valid_pos = False    
while not valid_pos:
    x = random.uniform(-5.0, 5.0)
    y = random.uniform(-5.0, 5.0)
    valid_pos = True
    wall_total_length = 7
    for walls in poses:
        if not is_position_valid(x, y, walls):
            valid_pos = False
            break
print(f"{x},{y}")
