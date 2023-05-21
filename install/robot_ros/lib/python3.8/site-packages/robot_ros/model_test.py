from tflite_runtime.interpreter import Interpreter
import numpy as np
import os
import cv2

# Load the TFLite model and allocate tensors.
# Vedi info modello: https://tfhub.dev/iree/lite-model/mobilenet_v1_100_224/uint8/1
interpreter = Interpreter(model_path="data/lite-model_mobilenet_v1_100_224_uint8_1.tflite")
interpreter.allocate_tensors()
main_dir_path=os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
input_file = os.path.join(main_dir_path, 'resource', 'airplan.jpg')


# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Testing model on real data
camera_img= cv2.imread(input_file)
input_image=cv2.resize(camera_img,[224, 224]) #resize
input_image=input_image.reshape(1, 224, 224, 3) 
input_image=input_image.astype(np.uint8)
input_shape=input_details[0].shape

# Test the model on random input data.
# input_data = np.array(np.random.random_sample(input_shape), dtype=np.uint8)

# test with real data
input_data = np.array(input_image, dtype=np.uint8)
interpreter.set_tensor(input_details[0]['index'], input_data)

interpreter.invoke()

# The function `get_tensor()` returns a copy of the tensor data.
# Use `tensor()` in order to get a pointer to the tensor.
output_data = interpreter.get_tensor(output_details[0]['index'])
print(output_data)