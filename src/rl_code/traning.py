
from agent import DDPGAgent
import numpy as np
import gazebo_env
import pickle





def trainer(env, agent, max_episodes, max_steps, batch_size, episilon):
    episode_rewards = []

    for episode in range(max_episodes):
        state = env.reset_robot()
        episode_reward = 0

        for step in range(max_steps):
            action = agent.get_action(state, episilon)
            print("actual action=",action)
            next_state, reward, done = env.perform_one_step(action)

            if step == max_steps-1 :
                done=True
            print("reward=",reward)
            agent.replay_buffer.push(state, action, reward, next_state, done)
            episode_reward += reward

            if agent.replay_buffer.size > batch_size:
                agent.update(batch_size)   


            if done or step == max_steps-1:
                episode_rewards.append(episode_reward)
                print("Episode " + str(episode) + ": " + str(episode_reward))
                break

            state = next_state
        
        with open("rewards.pickle","wb") as f:
            pickle.dump(episode_rewards, f)

        if episode%10==0 and episode>=1:
            agent.save()
    return episode_rewards

env = gazebo_env.Gazebo_enviorment()

max_episodes = 50
max_steps = 50
batch_size = 32

gamma = 0.99
tau = 1e-2
buffer_maxlen = 10000
critic_lr = 1e-3
actor_lr = 1e-3
episilon=0.5
state_size=6+4+1        #joints+orientation+direction of reward
action_size=6           # number of joints
agent = DDPGAgent(state_size,action_size, gamma, tau, buffer_maxlen, critic_lr, actor_lr)
episode_rewards = trainer(env, agent, max_episodes, max_steps, batch_size,episilon)