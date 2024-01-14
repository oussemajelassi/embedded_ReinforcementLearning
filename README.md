# embedded_ReinforcementLearning
This Repo will follow me while learning RL Basics and deploying the model on embedded targets probably the STM32

## Reinforcement learning

I will start by taking the basics of reinforcement learning away from the embedded systems area,
As it seems there are some intros to the field, I will try to follow.

### RL: must know concepts :

#### Environment : 

The environment is the place where our agent is going to learn, it specifies "laws" and provides the agent with consequences of his **Actions**.
The Environment is mainly composed of :
* States : a state is a description to our system, It gives informations about our system that we need to **reward** or **punish** the agent for its actions.
* Action : An action is what the agent can do, as a car can turn right or left or as a man can jump or stay low, we need to specify what our agent can do.
* Reward : Every action is succeeded by a reward to our agent to indicate wether that action is good or not 

#### RL Algorithms :

To perform an action we need to use an RL model, as it seems there are many available and I do not yet get the difference however what's needed from a model is to feed him current **observation** and what's demanded of him is to get us the adequate action.
