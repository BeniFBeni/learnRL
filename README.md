# learnRL
Learning reinforcement learning (in MATLAB®)

## Abstract

This software a playground and is aimed specifically at studying reinforcement learning (RL) in detail with a rich variety of settings.
The core of the playground is based upon a model of a mobile robot, referred to as the so called "extended non-holonomic double integrator" (ENDI).
See [these notes](ENDI-notes.pdf) for its description.
A flowchart of the overall code can be found in [here](MATLAB-RL-flowchart.pdf).
Basically, an *agent* (referred to also as the "controller") is attached to the *environment* (the system) and generates *actions* so as to minimize running costs (also called rewards or stage costs) over an infinite horizon in future.
The specific objective in this software package it so park the robot.
The controller is multi-modal and allows comparison with various baselines (nominal parking controller, model-predictive controller with and without on-the-fly model estimation).

## Major content of the package

* [init.m](init.m) - initialization script
* [CartENDI18a.slx](CartENDI18a.slx) - main file, a Simulink model, as the name says, in MATLAB 2018a
* [critic.m](critic.m) - script containing the definition of the critic as a function
* [mySSest.m](mySSest.m) - a standard estimator of a state-space model wrapped into a form suitable for the use in Simulink
* [myFindInitState.m](myFindInitState.m) - the same to determine a suitable initial state of the estimator
* [optCtrl.m](optCtrl.m) - script containing the definition of the actor as a function
* [theta_star.m](theta_star.m) - auxiliary function for the nominal parking controller

Details follow

## General description

The flowchart in [MATLAB-RL-flowchart.pdf](MATLAB-RL-flowchart.pdf) pretty much explains how the different parts of this software interact with each other.
The main ingredients of it are:

* the system
* the nominal controller
* the data-driven controller consisting of a model estimator, the critic and the actor

[CartENDI18a.slx](CartENDI18a.slx) resembles the flowchart, basically, except for additional stuff like scopes etc.
The functions are accompanied with extensive documentation.

## Usage

The basic usage cycle goes like this:

* browse into the folder containing the package in MATLAB
* run [init.m](init.m)
* open [CartENDI18a.slx](CartENDI18a.slx)
* run it
* check the results in scopes, the data will also be saved in the workspace

### To customize:

all the customization settings are pretty much in [init.m](init.m) and combined in the structure pars.
The key ones are:

`Nruns, Tsim`

Number of simulation runs each Tsim seconds long.
After each run, the system is reset to the initial state `x0`, while all the learned parameters (of the estimated model, the critic weights) are kept to emulate multi-trial RL

`delta_t`

Sampling time of the controller.
The system itself is continuous as physical process while the controller is digital.
What is to note:
1. the higher the sampling time, the more chattering in the control might occur.
It even may lead to instability and failure to park the robot
1. smaller sampling times lead to higher computation times
1. especially controllers that use the estimated model are sensitive to sampling time, because inaccuracies in estimation lead to problems when propagated over longer periods of time.
Experiment with `delta_t` and try achieve a compromise between stability and computational performance

`estBufferSize`

The size of the buffer to store data for model estimation.
The bigger the buffer, the more accurate the estimation may be achieved.
For successful model estimation, the system must be sufficiently excited.
Using bigger buffers is a way to achieve this.
estBufferSize is measured in numbers of periods of length `delta_t`.

`modelOrder`

The order of the estimation model, which reads:

    x_next = A x + B u,
    y = C x + D u

Here, `x` is the internal state of the model which is of no interest *per se*.
`u` is the input, `y` is the output.
We are interested in adequate predictions of `y` under given `u`'s.
The higher the model order, the better estimation results may be achieved, but be aware of overfitting.

`modelUpdDelay`

As the number of samples (number of `delta_t`'s) between model updates, this constant determines how often the estimated parameters are updated.
The more often the model is updated, the higher the computational burden is.
On the other hand, more frequent updates help keep the model actual.

`modelLearns`

Number of learning phases for model.
Set to something like 1e10 to have practically indefinite model updates.
Otherwise, if you want to have a learning phase, after which you fix the estimate, set `modelLearns` accordingly.
For instance, if you set

    pars.estBufferSize = Tsim-1
    pars.modelUpdDelay = Tsim-1
    pars.modelLearns = 1

you will have just the first run to learn the model and the consecutive ones will use that.

`estFromSim`

Determines if you start with a random model estimate or upload one from previous experiments.
It is meant to be a mat-file `SS0.mat` containing matrices `Aest0`, `Best0`, `Cest0` of your initial estimated state-space model.
There is a small script [saveSS.m](saveSS.m) to save the model after the simulation is done.

`optCtrlMode`

Optimal controller mode:

1. model-predictive control (MPC)
1. MPC with estimated model, a.k.a. adaptive MPC, or AMPC
1. RL/ADP (as [stacked Q-learning](https://doi.org/10.1016/j.ifacol.2017.08.803) with horizon N) using true model for prediction
1. RL/ADP (as stacked Q-learning with horizon N) using estimated model for prediction 
1. RL/ADP (as N-step roll-out Q-learning) using true model for prediction
1. RL/ADP (as N-step roll-out Q-learning) using estimated model for prediction 

The methods are also described in detail in [MATLAB-RL-flowchart.pdf](MATLAB-RL-flowchart.pdf).

`ctrlStackSize`

Controller horizon length.
This is the number of steps (in `delta_t` seconds) that controller looks "ahead".
The higher this number is, the more stably the controller tends to function.
Yet, computational burden grows as well.
If `ctrlStackSize = 1` the controller is not predictive and model-free.
Up to date, no guarantee can be given for such a setup to succeed, so use rather for experimentation.
In particular, you might require multiple trials to achieve appropriate critic weights and, possibly, further modifications (see **Further customization**).

`criticStackSize`

Size of the critic's buffer.
The critic optimizes the *temporal error* which is a measure of critic's ability to capture the optimal infinite-horizon cost (a.k.a. the *value function*).
The temporal errors are stacked up using the said buffer.
The principle here is pretty much the same as with the model estimation: accuracy against performance.

`criticUpdDelay, criticLearns`

These two constants act essentially in the same way as modelUpdDelay, modelLearns

`rcostS, rcostR`

Matrices that parameterize the running cost

`isProbNoise, probNoisePow`

Determine whether probing noise should be added to the final control and with what power.
It may be required to achieve proper excitation of the system for model estimation and critic weight update.

`critStruct`

A achoice of the critic's structure.
Please see [init.m](init.m) for the detailed description of those.

`criticFromSim`

The same as with the model estimation.

Finally, a hint to check the model prediction quality:

while running [CartENDI18a.slx]/(CartENDI18a.slx), open [optCtrl.m]/(optCtrl.m), place a breakpoint somewhere inside `costFncCtrl` and run the content of [DEBUG_modEst.m]/(DEBUG_modEst.m) to see how close the predicted estimates are from prediction using the true discretized model. 

## Further customization

The following possible customization is evident:

* use of different a system: provide your description in sys_rhs and sys_out of [CartENDI18a.slx](CartENDI18a.slx), as well as in [optCtrl.m](optCtrl.m)
* use of different running cost, not necessarily quadratic: this should be adjusted accordingly in [optCtrl.m](optCtrl.m) and [critic.m](critic.m)
* use of different critic structures, say, neural nets with multiple hidden layers: again, adjust [optCtrl.m](optCtrl.m) and [critic.m](critic.m)
* use of different RL techniques, say, dual learning
* use of different model estimators, say, neural nets: replace mySSest.m with your suitable estimator and adjust `dataDrivOptCtrl` in [CartENDI18a.slx](CartENDI18a.slx) and [optCtrl.m](optCtrl.m) accordingly

## Closing remarks

Please contact [me](mailto:p.osinenko@skoltech.ru) if you need a Simulink model of a lower version.
Although this software package uses mostly basic functionality, some compatibility issues might occur when converting. 

Also, in case of any suggestions, found bugs etc. feel free to contact.

Finally, regarding the nominal controller.
This one was designed using the technique called *nonsmooth backstepping*



