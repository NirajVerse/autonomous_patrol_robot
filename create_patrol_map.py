from pathlib import Path
import xml.etree.ElementTree as ET
import shutil

root = Path(__file__).resolve().parent

# Original TurtleBot3 empty simulation world
source_world = (
    root / "src/turtlebot3_simulations"
    / "turtlebot3_gazebo/worlds/empty_world.world"
)

tree = ET.parse(source_world)
world = tree.getroot().find("world")

def add_box(name, x, y, length, width, color):
    model = ET.SubElement(
        world, "model", name=name
    )
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = (
        f"{x} {y} 0.002 0 0 0"
    )

    link = ET.SubElement(model, "link", name="body")
    visual = ET.SubElement(link, "visual", name="visual")
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")

    ET.SubElement(box, "size").text = (
        f"{length} {width} 0.004"
    )

    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = color
    ET.SubElement(material, "diffuse").text = color


def add_station(name, x, y):
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = (
        f"{x} {y} 0.005 0 0 0"
    )

    link = ET.SubElement(model, "link", name="body")
    visual = ET.SubElement(link, "visual", name="visual")
    geometry = ET.SubElement(visual, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")

    ET.SubElement(cylinder, "radius").text = "0.09"
    ET.SubElement(cylinder, "length").text = "0.01"

    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = "0 0.9 0.2 1"
    ET.SubElement(material, "diffuse").text = "0 0.9 0.2 1"


blue = "0.1 0.35 0.95 1"

# Road width = 0.25 meters
add_box("left_road", -1.5, 0, 0.25, 3.0, blue)
add_box("center_road", 0, -0.25, 0.25, 2.5, blue)
add_box("right_road", 1.5, 0.75, 0.25, 1.5, blue)

add_box("upper_road", 0, 1.0, 3.0, 0.25, blue)
add_box("lower_road", 0, 0.0, 3.0, 0.25, blue)

# Three patrol stations
add_station("P1", -1.5, 1.5)
add_station("P2", -1.5, -1.5)
add_station("P3", 1.5, 1.5)

tree.write(
    root / "worlds/patrol.world",
    encoding="unicode",
    xml_declaration=True
)

# Copy the already working TurtleBot3 launch file
source_launch = (
    root / "src/turtlebot3_simulations"
    / "turtlebot3_gazebo/launch/empty_world.launch.py"
)
target_launch = root / "launch/patrol.launch.py"

content = source_launch.read_text()

old = """world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )"""

new = """world = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            '..', 'worlds', 'patrol.world'
        )
    )"""

if old not in content:
    raise RuntimeError("Could not find world path in launch file")

target_launch.write_text(content.replace(old, new))

print("Created worlds/patrol.world")
print("Created launch/patrol.launch.py")
