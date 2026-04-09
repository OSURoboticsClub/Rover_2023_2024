import torch
print(f"PyTorch: {torch.__version__}")
print(f"CUDA: {torch.version.cuda}")
print(f"cuDNN: {torch.backends.cudnn.version()}")
print(f"GPU: {torch.cuda.get_device_name(0)}")

# Test a simple convolution directly
import torch.nn as nn
conv = nn.Conv2d(3, 64, 3, padding=1).cuda()
x = torch.randn(1, 3, 640, 640).cuda()

print("Testing FP32 conv...")
try:
    out = conv(x)
    print("FP32: OK")
except Exception as e:
    print(f"FP32 failed: {e}")

print("Testing with benchmark=True...")
torch.backends.cudnn.benchmark = True
try:
    out = conv(x)
    print("benchmark: OK")
except Exception as e:
    print(f"benchmark failed: {e}")

print("Testing with cudnn disabled...")
torch.backends.cudnn.enabled = False
try:
    out = conv(x)
    print("no cudnn: OK")
except Exception as e:
    print(f"no cudnn failed: {e}")
                    
