#!/usr/bin/env python

import tensorflow as tf

from keras import layers, models

def create_model(input_shape):
    model = models.Sequential()
    
    model.add(layers.Conv2D(24, (5, 5), strides=(2, 2), activation='relu', input_shape=input_shape))
    model.add(layers.Conv2D(36, (5, 5), strides=(2, 2), activation='relu'))
    model.add(layers.Conv2D(48, (5, 5), strides=(2, 2), activation='relu'))
    model.add(layers.Conv2D(64, (3, 3), activation='relu'))
    model.add(layers.Conv2D(64, (3, 3), activation='relu'))
    model.add(layers.Flatten())
    model.add(layers.Dense(100, activation='relu'))
    model.add(layers.Dense(50, activation='relu'))
    model.add(layers.Dense(10, activation='relu'))
    model.add(layers.Dense(1, activation='tanh'))

    optimizer = tf.keras.optimizers.Adam(3e-4)
    model.compile(optimizer=optimizer, loss='huber')

    return model


model = create_model(input_shape=(66, 200, 3))

# Load the saved model
# model.load_weights("model/model.h5") 
# model.summary()
