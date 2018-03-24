import csv
import os
import cv2
import numpy as np
import sklearn
import keras

lines = []
with open("sim_data/labels.csv") as csvfile:
    reader = csv.reader(csvfile)
    # skip the heading line if there is one
    # next(reader, None)
    for line in reader:
        lines.append(line)
print("Number of lines: ", len(lines))

# Read one image to get its shape
source_path = line[0]
filename = os.path.basename(source_path)
current_path = os.path.join("sim_data", filename)
image = np.array(cv2.imread(current_path))
ishape = np.shape(image)
print("ishape=", ishape)

from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelBinarizer

label_binarizer = LabelBinarizer()
label_binarizer.fit([0, 1, 2])

train_lines, validation_lines = \
    sklearn.model_selection.train_test_split(lines,
                                             test_size=0.2)

# Note: actual batch_size will be 2x because we append
# the horizontally flipped images
def generator(lines, batch_size=32):
    num_lines = len(lines)
    path = "sim_data"
    # Loop forever, the generator never terminates
    while 1:
        # shuffle every epoch
        sklearn.utils.shuffle(lines)
        for offset in range(0, num_lines, batch_size):
            batch_lines = lines[offset:offset+batch_size]
            images = []
            states = []
            for line in batch_lines:
                filename = line[0]
                image = cv2.imread(os.path.join(path,filename))
                images.append(image)
                state = int(line[1])
                states.append(state)
                # flipped image
                images.append(cv2.flip(image, 1))
                states.append(state)
                
            X_train = np.array(images)
            y_train = np.array(label_binarizer.transform(states))
            yield sklearn.utils.shuffle(X_train, y_train)

train_generator = generator(train_lines, batch_size=16)
validation_generator = generator(validation_lines, batch_size=16)

from keras.models import Sequential, Model
from keras.layers import Flatten, Dense, Conv2D, Dropout
from keras.layers import Lambda, MaxPooling2D, Cropping2D
import matplotlib.pyplot as plt

# if model.h5 already exists use it
if os.path.isfile("model.h5"):
    print "Using existing model"
    model = keras.models.load_model("model.h5")
else:
    model = Sequential()
    model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=ishape))
    # Crop the top 200 lines of the image since the lights are always below that
    # model.add(Cropping2D(cropping=((100, 0), (0, 0))))
    model.add(Conv2D(8, (5, 5), activation="relu"))
    model.add(MaxPooling2D())
    model.add(Conv2D(10, (5, 5), activation="relu"))
    model.add(MaxPooling2D())
    model.add(Conv2D(12, (5, 5), activation="relu"))
    model.add(MaxPooling2D())
    model.add(Conv2D(18, (3, 3), activation="relu"))
    model.add(MaxPooling2D())
    model.add(Conv2D(24, (3, 3), activation="relu"))
    model.add(MaxPooling2D())
    model.add(Flatten())
    model.add(Dense(120, activation="relu"))
    # model.add(Dropout(0.25))
    model.add(Dense(84, activation="relu"))
    # model.add(Dropout(0.25))
    model.add(Dense(10, activation="relu"))
    # model.add(Dropout(0.25))
    model.add(Dense(3, activation='softmax'))

    model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])

# from keras.utils import plot_model
# plot_model(model, to_file='model.png')
from keras.utils.vis_utils import plot_model
plot_model(model, to_file='model.png', show_shapes=True)

history_object = model.fit_generator(train_generator,
                                     samples_per_epoch=len(train_lines)*2,
                                     validation_data=validation_generator,
                                     nb_val_samples=len(validation_lines)*2,
                                     nb_epoch=4, verbose=1)

# history_object = model.fit_generator(train_generator,
#                                      steps_per_epoch=len(train_lines)/16,
#                                      validation_data=validation_generator,
#                                      validation_steps=len(validation_lines)/16,
#                                      epochs=4, verbose=1)

model.save('model.h5')

print(history_object.history.keys())

plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean square error loss')
plt.ylabel('mean square error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()
