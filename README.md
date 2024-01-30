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

### RL : common packages and tools : 

#### Stable baselines : 

It is a set of RL algorithms which aims to get it easier to use common models and it provides a great set of tools such as : 

* **Callbacks** : They are a function called every once and a while when training the model, used in order to show metrics, evaluate training process and logging, To use them we need to overwrite a class in stable-baselines called **BaseCallback**.
  
## Project : Robot Orientation using Reinforcement Learning

### Serial Communication :
To ensure serial communication between collab environment and the STM32, we should connect collab to local runtime.
to do so please follow [connect to local runtime](https://research.google.com/colaboratory/local-runtimes.html#:~:text=In%20Colab%2C%20click%20the%20%22Connect,connected%20to%20your%20local%20runtime.)

### Training Phase :

In order to set up a training envirenment, I will establish an RTOS program running on the STM32 in order to recieve orders, execute and then give back results.

There will be two tasks running for now, the first one is communicationTask , will wait for orders and then puts the new order in a queue and goes to sleep. The other task is initially asleep, once recieved the value will update the command and waits for 200ms and then calculates the results and puts them in another queue and goes back to sleep. 
