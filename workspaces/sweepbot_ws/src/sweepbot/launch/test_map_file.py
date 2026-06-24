import os
import yaml
from ament_index_python.packages import get_package_share_directory

package_name = 'sweepbot'

package_share_dir = get_package_share_directory(package_name)

# Path to your .yaml file inside the sweepbot package
map_yaml_path = os.path.join(package_share_dir, 'maps', 'holladaybsmtTWO.yaml')


if not os.path.exists(map_yaml_path):
    raise FileNotFoundError(f"YAML file not found: {map_yaml_path}")
print(f"Found map YAML: {map_yaml_path}")

with open(map_yaml_path, 'r') as f:
    data = yaml.safe_load(f)

print("YAML contents:")
for key, val in data.items():
    print(f"  {key}: {val}")


image_path = os.path.join(os.path.dirname(map_yaml_path), data['image'])


if not os.path.exists(image_path):
    raise FileNotFoundError(f"PGM image not found: {image_path}")
print(f"Found map image: {image_path}")


try:
    from PIL import Image
    img = Image.open(image_path)
    print(f"Image loaded: size={img.size}, mode={img.mode}")
except ImportError:
    print("Pillow not installed; skipping image validation.")
except Exception as e:
    print(f"Image load failed: {e}")
