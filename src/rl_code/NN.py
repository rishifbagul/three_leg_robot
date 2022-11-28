import tensorflow as tf
from tensorflow.keras.layers import Input, Dense, concatenate 


def critic_NN(state_size, action_size,hidden_layers):

    input_x = Input(shape=state_size)
    input_a = Input(shape=action_size)
    x = input_x
    for i,j in enumerate(hidden_layers[:-1]):
        if i==1:
            x = concatenate([x,input_a],axis=-1)
        x = Dense(j,activation='relu')(x)
    x = Dense(hidden_layers[-1])(x)
    
    return tf.keras.Model([input_x,input_a],x)

def actor_NN(state_size,action_size,hidden_layers,action_mult=1):
    input_x = Input(shape=state_size)
    x = input_x
    for i in hidden_layers:
        x = Dense(i,activation='relu')(x)
    x = Dense(action_size,activation='tanh')(x)
    x = tf.math.multiply(x,action_mult)
    return tf.keras.Model(input_x,x)

def visualise_NN(model,file_name):
    tf.keras.utils.plot_model(
    model,
    to_file=file_name,
    show_shapes=True,
    show_dtype=False,
    show_layer_names=True,
    rankdir='TB',
    expand_nested=True,
    dpi=96,
    layer_range=None,
    show_layer_activations=True
)