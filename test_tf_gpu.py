
import tensorflow as tf

print("TensorFlow version:", tf.__version__)
print("Num GPUs Available:", len(tf.config.experimental.list_physical_devices('GPU')))

# 确认CUDA和cuDNN版本
from tensorflow.python.client import device_lib
devices = device_lib.list_local_devices()
for device in devices:
    if 'GPU' in device.device_type:
        print(f"Device name: {device.name}")
        print(f"Device memory: {device.memory_limit}")
        print(f"Device description: {device.physical_device_desc}")

print("Build information:")
print(tf.sysconfig.get_build_info())
print("TensorFlow version:", tf.__version__)
print("Num GPUs Available:", len(tf.config.experimental.list_physical_devices('GPU')))
from tensorflow.python.client import device_lib
devices = device_lib.list_local_devices()
for device in devices:
    if 'GPU' in device.device_type:
        print(f"Device name: {device.name}")
        print(f"Device memory: {device.memory_limit}")
        print(f"Device description: {device.physical_device_desc}")
print("Build information:")
print(tf.sysconfig.get_build_info())
