import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
from datetime import datetime
import NN
from collections import deque
from sys import exit
import memory_buffer
import random
from tensorflow import keras
import os

class DDPGAgent:
    
    def __init__(self,state_size, action_size, gamma, tau, buffer_maxlen, critic_learning_rate, actor_learning_rate):
        
        self.state_size = state_size
        self.action_size = action_size
        self.action_max=1
        self.gamma = gamma
        self.tau = tau
        
        #Network layers
        actor_layer = [700,400,128]
        critic_layer = [700,512,300,1]

        if os.path.exists("NN_model/actor/saved_model.pb") :
          print("-----model Loading --------------------")
          self.load()
        else:
          # Main networks
          self.actor = NN.actor_NN((self.state_size),(self.action_size),actor_layer,self.action_max)
          self.critic =NN.critic_NN((self.state_size),(self.action_size),critic_layer)

          # Target networks
          self.actor_target = NN.actor_NN((self.state_size),(self.action_size),actor_layer,self.action_max)
          self.critic_target =NN.critic_NN((self.state_size),(self.action_size),critic_layer)

          # Copying weights in,
          self.actor_target.set_weights(self.actor.get_weights())
          self.critic_target.set_weights(self.critic.get_weights())
      
          # optimizers
        self.actor_optimizer = tf.keras.optimizers.Adam(learning_rate=critic_learning_rate)
        self.critic_optimizer = tf.keras.optimizers.Adam(learning_rate=actor_learning_rate)
  
        self.replay_buffer = memory_buffer.BasicBuffer(buffer_maxlen, self.state_size, self.action_size)
        
        self.critic_losses = []
        
        self.actor_losses = []
        
    def get_action(self, s, noise_scale):                   #visit again here
        a =  self.actor.predict(s.reshape(1,-1))[0]
        print("pridicted action=",a)
        
        a += noise_scale * np.random.randn(self.action_size)
        return np.clip(a, -self.action_max, self.action_max)
       

    def update(self, batch_size):                           #learn about this
        
        #print("..learning...")
        X,A,R,X2,D = self.replay_buffer.sample(batch_size)
        X = np.asarray(X,dtype=np.float32)
        A = np.asarray(A,dtype=np.float32)
        R = np.asarray(R,dtype=np.float32)
        X2 = np.asarray(X2,dtype=np.float32)
        
        
        Xten=tf.convert_to_tensor(X)
        

        with tf.GradientTape() as tape:
          A2 =  self.actor_target(X2)
          q_target = R + self.gamma  * self.critic_target([X2,A2])
          qvals = self.critic([X,A]) 
          q_loss = tf.reduce_mean((qvals - q_target)**2)
          grads_q = tape.gradient(q_loss,self.critic.trainable_variables)
        self.critic_optimizer.apply_gradients(zip(grads_q, self.critic.trainable_variables))
        self.critic_losses.append(q_loss)


 
        with tf.GradientTape() as tape2:
          A_mu =  self.actor(X)
          Q_mu = self.critic([X,A_mu])
          mu_loss =  -tf.reduce_mean(Q_mu)
          grads_mu = tape2.gradient(mu_loss,self.actor.trainable_variables)
        self.actor_losses.append(mu_loss)
        self.actor_optimizer.apply_gradients(zip(grads_mu, self.actor.trainable_variables))



        
        temp1 = np.array(self.critic_target.get_weights(),dtype=object)
        temp2 = np.array(self.critic.get_weights(),dtype=object)
        temp3 = self.tau*temp2 + (1-self.tau)*temp1
        self.critic_target.set_weights(temp3)
      

        temp1 = np.array(self.actor_target.get_weights())
        temp2 = np.array(self.actor.get_weights())
        temp3 = self.tau*temp2 + (1-self.tau)*temp1
        self.actor_target.set_weights(temp3)
    
    def save(self):
      self.actor.save("NN_model/actor")
      self.critic.save("NN_model/critic")
      self.actor_target.save("NN_model/actor_target")
      self.critic_target.save("NN_model/critic_target")
      print("NN model saved")
    
    def load(self):
      self.actor=keras.models.load_model("NN_model/actor")
      self.critic=keras.models.load_model("NN_model/critic")
      self.actor_target=keras.models.load_model("NN_model/actor_target")
      self.critic_target=keras.models.load_model("NN_model/critic_target")
      print("NN model loaded")

