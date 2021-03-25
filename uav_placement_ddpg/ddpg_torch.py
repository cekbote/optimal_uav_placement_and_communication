import os
import torch as T
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np


class OUActionNoise:
    def __init__(self, mu, sigma=0.15, theta=0.2, dt=1e-2, x0=None):
        self.theta = theta
        self.mu = mu
        self.sigma = sigma
        self.dt = dt
        self.x0 = x0
        self.reset()

    def __call__(self):
        x = self.x_prev + self.theta * (self.mu - self.x_prev) * \
            self.dt + self.sigma + np.sqrt(self.dt) * \
            np.random.normal(size=self.mu.shape)
        self.x_prev = x
        return x

    def reset(self):
        self.x_prev = self.x0 if self.x0 is not None else \
            np.zeros_like(self.mu)

class ReplayBuffer:
    def __init__(self, max_size, input_shape, n_actions):
        self.mem_size = max_size
        self.mem_cntr = 0
        self.state_memory = np.zeros((self.mem_size, *input_shape))
        self.new_state_memory = np.zeros((self.mem_size, *input_shape))
        self.action_memory = np.zeros((self.mem_size, n_actions))
        self.reward_memory = np.zeros(self.mem_size)

    def store_transition(self, state, action, reward, state_):
        index = self.mem_cntr % self.mem_size
        self.state_memory[index] = state
        self.action_memory[index] = action
        self.reward_memory[index] = reward
        self.new_state_memory[index] = state_
        self.mem_cntr += 1

    def sample_buffer(self, batch_size):
        max_mem = min(self.mem_cntr, self.mem_size)
        batch = np.random.choice(max_mem, batch_size)

        states = self.state_memory[batch]
        new_states = self.new_state_memory[batch]
        rewards = self.reward_memory[batch]
        actions = self.action_memory[batch]

        return states, actions, rewards, new_states

class CrticNetwork(nn.Module):
    def __init__(self, lr, input_dims, n_actions):

        self.obsv = nn.Sequential(
            nn.Conv2d(1, 1, 2),
            nn.ReLU(),
            nn.Conv2d(1, 1, 1),
            nn.ReLU()
        )

        self.actions = nn.Sequential(
            nn.Linear(n_actions, 2),
            nn.ReLU(),
            nn.Linear(2, 2),
            nn.ReLU()
        )

        self.output = nn.Sequential(
            nn.Linear(6, 2),
            nn.ReLU(),
            nn.Linear(2, 1)
        )

        self.optimizer = optim.Adam(self.parameters(), lr=lr)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')
        self.to(self.device)

    def forward(self,  state, action):
        state_value = self.obsv(state)
        state_value = state_value.view(state_value.size(0), -1)
        action_value = self.actions(action)
        combined = T.cat((state_value, action_value), dim=1)
        q_value = self.output(combined)

        return q_value

class ActorNetwork(nn.Module):
    def __init__(self, lr, input_dims, n_actions):

        self.conv_actions = nn.Sequential(
            nn.Conv2d(1, 1, 2),
            nn.ReLU(),
            nn.Conv2d(1, 1, 2),
            nn.ReLU(),
            nn.Conv2d(1, 1, 2),
            nn.ReLU()
        )

        self.lin_actions=nn.Sequential(
            nn.Linear(2, 2),
            nn.ReLU(),
            nn.Linear(2, 2),
            nn.ReLU(),
            nn.Linear(2, n_actions),
            nn.ReLU()
        )

        self.optimizer = optim.Adam(params=self.parameters(), lr=lr)
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')
        self.to(self.device)

    def forward(self, state):
        action = self.conv_actions(state)
        action = self.lin_actions(action)

        return action


class Agent():
    def __init__(self, lr1, lr2, input_dims, tau, gamma=0.99, n_actions=2,
                 max_size=1000, batch_size=32):
        self.gamma = gamma
        self.tau = tau
        self.memory = ReplayBuffer(max_size, input_dims, n_actions)
        self.batch_size = batch_size

        self.actor = ActorNetwork(lr1, input_dims, n_actions)
        self.target_actor = ActorNetwork(lr1, input_dims, n_actions)

        self.critic = CrticNetwork(lr2, input_dims, n_actions)
        self.target_critic = CrticNetwork(lr2, input_dims, n_actions)

        self.noise = OUActionNoise(mu=np.zeros(n_actions))

        self.update_network_parameters(tau=1)

    def choose_action(self, observation):
        self.actor.eval()
        observation = T.tensor(observation, dtype=T.float).to(self.actor.device)
        mu = self.actor(observation).to(self.actor.device)
        mu_prime = mu + T.Tensor(self.noise(),
                                 dtype=T.float).to(self.actor.device)
        self.actor.train()
        return mu_prime.cpu().detach.numpy()

    def learn(self):
        if self.memory.mem_cntr < self.batch_size:
            return
        states, actions, rewards, next_states = \
            self.memory.sample_buffer(self.batch_size)
        rewards = T.tensor(rewards, dtype=T.float).to(self.critic.device)
        next_states = T.tensor(next_states, dtype=T.float).to(self.critic.device)
        actions = T.tensor(actions, dtype=T.float).to(self.critic.device)
        states = T.tensor(states, dtype=T.float).to(self.critic.device)

        self.target_actor.eval()
        self.target_critic.eval()
        self.critic.eval()

        target_actions = self.target_actor.forward(next_states)
        critic_value_ = self.target_critic.forward(next_states, target_actions)
        critic_value = self.target_critic.forward(states, actions)

        target = []
        for j in range(self.batch_size):
            target.append(rewards[j] + self.gamma*critic_value_[j])
        target = T.tensor(target).to(self.critic.device)
        target = target.view(self.batch_size, 1)

        self.critic.train()
        self.critic.optimizer.zero_grad()
        critic_loss = F.mse_loss(target, critic_value)
        critic_loss.backward()
        self.critic.optimizer.step()

        self.critic.eval()
        self.actor.optimizer.zero_grad()
        mu = self.actor.forward(states)
        self.actor.train()
        actor_loss = -self.critic.forward(states, mu)
        actor_loss = T.mean(actor_loss)
        actor_loss.backward()
        self.actor.optimizer.step()

        self.update_network_parameters()

    def update_network_parameters(self, tau=None):
        if tau is None:
            tau = self.tau

        actor_params = self.actor.named_parameters()
        critic_params = self.critic.named_parameters()
        target_actor_params = self.target_actor.named_parameters()
        target_critic_params = self.target_critic.named_parameters()

        critic_state_dict = dict(critic_params)
        actor_state_dict = dict(actor_params)
        target_critic_dict = dict(target_critic_params)
        target_actor_dict = dict(target_actor_params)

        for name in critic_state_dict:
            critic_state_dict[name] = tau * critic_state_dict[name].clone() + \
                                      (1 - tau) * \
                                      target_critic_dict[name].clone()
        self.target_critic.load_state_dict(critic_state_dict)

        for name in actor_state_dict:
            actor_state_dict[name] = tau * actor_state_dict[name].clone() + \
                                     (1 - tau) * \
                                     target_actor_dict[name].clone()
        self.target_actor.load_state_dict(actor_state_dict)
