from PIL import Image, ImageEnhance
from pathlib import Path

BASE = Path("/home/kimi/dev_ws/rover_ws/src/a2_isaac/isaac_sim/assets/markers/tier2_mineral/0")

base_color = BASE / "Default_OBJ_baseColor.jpg"
emissive = BASE / "Default_OBJ_emissive.jpg"

def tint(src, dst, color, strength=0.75):
    img = Image.open(src).convert("RGB")
    overlay = Image.new("RGB", img.size, color)
    out = Image.blend(img, overlay, strength)
    out = ImageEnhance.Contrast(out).enhance(1.15)
    out.save(dst, quality=95)

# red
tint(base_color, BASE / "Default_OBJ_baseColor_red.jpg", (255, 40, 20), 0.65)
tint(emissive, BASE / "Default_OBJ_emissive_red.jpg", (255, 20, 0), 0.8)

# yellow
tint(base_color, BASE / "Default_OBJ_baseColor_yellow.jpg", (255, 210, 30), 0.65)
tint(emissive, BASE / "Default_OBJ_emissive_yellow.jpg", (255, 220, 0), 0.8)