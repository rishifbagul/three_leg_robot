import numpy as np
import os
import pickle


class BasicBuffer:
    
    def __init__(self, size, state_dim, action_dim):
        self.file_name="memory.pickle"
        self.save_counter_max=100
        if os.path.exists(self.file_name) :
            self.load_buffer()
        else:
            print("initialising buffer")
            self.state_buf = np.zeros([size, state_dim], dtype=np.float32)
            self.next_state_buf = np.zeros([size, state_dim], dtype=np.float32)
            self.action_buf = np.zeros([size, action_dim], dtype=np.float32)
            self.reward_buf = np.zeros([size], dtype=np.float32)
            self.done_buf = np.zeros([size], dtype=np.float32)
            self.ptr, self.size, self.max_size = 0, 0, size
        self.save_counter=0

    def push(self, obs, act, rew, next_obs, done):
        self.state_buf[self.ptr] = obs
        self.next_state_buf[self.ptr] = next_obs
        self.action_buf[self.ptr] = act
        self.reward_buf[self.ptr] = np.asarray([rew])
        self.done_buf[self.ptr] = done
        self.ptr = (self.ptr+1) % self.max_size
        self.size = min(self.size+1, self.max_size)
        self.save_counter+=1
        if self.save_counter>self.save_counter_max:
            self.save_buffer()
            self.load_buffer()
            self.counter=0

    def sample(self, batch_size=32):
        idxs = np.random.randint(0, self.size, size=batch_size)
        temp_dict= dict(s=self.state_buf[idxs],
                    s2=self.next_state_buf[idxs],
                    a=self.action_buf[idxs],
                    r=self.reward_buf[idxs],
                    d=self.done_buf[idxs])
        return (temp_dict['s'],temp_dict['a'],temp_dict['r'].reshape(-1,1),temp_dict['s2'],temp_dict['d'])

    def save_buffer(self):
        with open(self.file_name,"wb") as f:
            pickle.dump(self.state_buf, f)
            pickle.dump(self.next_state_buf, f)
            pickle.dump(self.action_buf, f)
            pickle.dump(self.reward_buf, f)
            pickle.dump(self.done_buf, f)
            pickle.dump(self.ptr, f)
            pickle.dump(self.size, f)
            pickle.dump(self.max_size, f)
            
        print("saved buffer to file ", self.file_name)
        

    def load_buffer(self):
        with open(self.file_name, "rb") as f:
            self.state_buf= pickle.load(f)
            self.next_state_buf= pickle.load(f)
            self.action_buf= pickle.load(f)
            self.reward_buf= pickle.load(f)
            self.done_buf= pickle.load(f)
            self.ptr= pickle.load(f)
            self.size= pickle.load(f)
            self.max_size= pickle.load(f)

        print("loaded buffer from file ", self.file_name)
        