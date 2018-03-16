import numpy as np
import os
import keras
import cv2

model = keras.models.load_model("model.h5")
current_path = os.path.join("sim_data", "tl_00012.png")
image = np.array(cv2.imread(current_path))
image_array = np.asarray(image)
pred = model.predict(image_array[None,:,:,:], batch_size=1)
print pred
print np.argmax(pred)
