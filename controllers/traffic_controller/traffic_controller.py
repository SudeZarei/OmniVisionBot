from controller import Supervisor, Keyboard

# Initialize Supervisor and timestep
supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Enable keyboard input
kb = Keyboard()
kb.enable(timestep)

# Access materials via DEF names
green_mat = supervisor.getFromDef("GreenMat")
yellow_mat = supervisor.getFromDef("YellowMat")
red_mat = supervisor.getFromDef("RedMat")

# Extract color fields for each material
green_base = green_mat.getField("baseColor")
green_emissive = green_mat.getField("emissiveColor")

yellow_base = yellow_mat.getField("baseColor")
yellow_emissive = yellow_mat.getField("emissiveColor")

red_base = red_mat.getField("baseColor")
red_emissive = red_mat.getField("emissiveColor")

# Helper function to toggle light colors
def update_lights(r_on, y_on, g_on):
    # Update red light
    if r_on:
        red_base.setSFColor([1.0, 0.0, 0.0])
        red_emissive.setSFColor([1.0, 0.0, 0.0])
    else:
        red_base.setSFColor([0.1, 0.1, 0.1])
        red_emissive.setSFColor([0.0, 0.0, 0.0])
        
    # Update yellow light
    if y_on:
        yellow_base.setSFColor([1.0, 1.0, 0.0])
        yellow_emissive.setSFColor([1.0, 1.0, 0.0])
    else:
        yellow_base.setSFColor([0.1, 0.1, 0.1])
        yellow_emissive.setSFColor([0.0, 0.0, 0.0])
        
    # Update green light
    if g_on:
        green_base.setSFColor([0.0, 1.0, 0.0])
        green_emissive.setSFColor([0.0, 1.0, 0.0])
    else:
        green_base.setSFColor([0.1, 0.1, 0.1])
        green_emissive.setSFColor([0.0, 0.0, 0.0])

# Set default state: Green on
update_lights(r_on=False, y_on=False, g_on=True)

# Main control loop
while supervisor.step(timestep) != -1:
    key = kb.getKey()
    
    if key == ord('1'):
        update_lights(r_on=False, y_on=False, g_on=True)
    elif key == ord('2'):
        update_lights(r_on=False, y_on=True, g_on=False)
    elif key == ord('3'):
        update_lights(r_on=True, y_on=False, g_on=False)