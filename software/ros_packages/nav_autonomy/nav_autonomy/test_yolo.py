from ultralytics import YOLO
import cv2
import os
import sys

models_dir = "yolo_models"

batches = [f for f in os.listdir(models_dir) if os.path.isdir(os.path.join(models_dir, f))]
print("Available batches:")
for i, name in enumerate(batches):
    print(f"  {i + 1}. {name}")
batch_choice = input(f"Select batch [1-{len(batches)}]: ").strip()
index = int(batch_choice) - 1
models_dir = os.path.join(models_dir, batches[index])
available = [f for f in os.listdir(models_dir) if f.endswith(".pt")]

if not available:
    print("No .pt models found in models/")
    sys.exit(1)

print("Available models:")
for i, name in enumerate(available):
    print(f"  {i + 1}. {name}")

choice = input(f"Select model [1-{len(available)}]: ").strip()
try:
    index = int(choice) - 1
    if not 0 <= index < len(available):
        raise ValueError
except ValueError:
    print("Invalid selection.")
    sys.exit(1)

model_path = os.path.join(models_dir, available[index])
print(f"Loading {model_path}...")
model = YOLO(model_path)

results = model(source=0, stream=True)

results = model(source=0, stream=True)

try:
    for result in results:
        frame = result.plot()
        cv2.imshow("YOLO Live", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
except Exception as e:
    print(f"Exception: {e}")
finally:
    cv2.destroyAllWindows()
