import os
import torch

from world_model import IndustrialWorldEngine

SAVE_DIR = "models"
SAVE_PATH = os.path.join(SAVE_DIR, "world_model_weights.pt")

if __name__ == "__main__":
    engine = IndustrialWorldEngine()
    engine.train_quick_dynamics()

    os.makedirs(SAVE_DIR, exist_ok=True)
    torch.save(engine.model.state_dict(), SAVE_PATH)
    print(f"저장 완료: {SAVE_PATH}")