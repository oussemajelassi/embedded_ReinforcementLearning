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

#### MARKOV Decision Process : 

So MDP is based on strcuturing the system into several **states**, We might think of states as in FSM thats kinda true a state is also the current **observation** of the system and a complete description for his behaviour now.
The Difference to FSM is that To go from a state to a state we need to take an **action** however the output of this action is **probabilistic** and we do not know for sure the final outcome.
For example, for a standing man that can either walk or run, the outcome can be resulting in him falling or reaching destination either he takes the first choce or the second, how ever when he walks the probability for falling is low on the other hand if he runs he will reach destination more quickly.

## Project : Signal Generation based on Reinforcement Learning

I want to develop a DSP node having a signal aquisition unit, the input will be a signal of a certain frequency, My node will then act based on that and try to generate a signal similar to the input signal manipulating the STM32 periphirals such as timer and ADC.
