# Extra Files
## Overview
This folder has some documentation that didn't fit anywhere else.

## BOM
This is a spreadsheet with the Bill of Materials. You can open it with LibreOffice or with Excel.

## Linux Commands
This is a file that has some useful commands for the Linux Terminal.

## Lagrangian Simulation Derivation
In this folder are the jupyter notebooks that were used to derive the equations of motions for a triple-pendulum on a wheel. Those equations were then used in `control_ui/pendulum_simulation.cpp`, which is then used to simulate the robot. The comments in the corresponding header file also provide some context. See the comments in `control_ui/simulation_ui.cpp` on how to use the simulation.

The notebooks are these two files:
```
eulerlagrangian_cartpole.ipynb
eulerlagrangian.ipynb
```
The first one is a simplified version that just covers the classic Cart Pole. The second one derives the full equations that simulate the robot.

To try out these notebooks, you need a suitable Python environment. I used Anaconda with this Environment:
```
conda create -n sympy python=3.10
conda activate sympy
conda install -c anaconda sympy jupyter
```
But something similar with pip will probably also work. You can also try to open them directly with VS Code. See [here](https://www.dataquest.io/blog/jupyter-notebook-tutorial/) on how to use Jupyter Notebook.

The Lagrangian Method is used in those notebooks. It is explained [here](https://scholar.harvard.edu/files/david-morin/files/cmchap6.pdf), alongside some examples that explain how to apply this method manually.

The triple-pendulum is a straightforward extension of the double-pendulum, and there is a lot of information about that out there. [Here](https://scipython.com/blog/the-double-pendulum/) is a well described simple python implementation.

The wheel attached to the pendulum is very similar to a cart attached to a pole. And the Cart Pole is also something well researched. A good explanation is [here](http://piepieninja.github.io/2019/05/09/cart-pole-dynamics/) and [here](https://www.matthewpeterkelly.com/tutorials/cartPole/index.html).

A combination of a double-pendulum with a cart is covered [here](https://www3.math.tu-berlin.de/Vorlesungen/SoSe12/Kontrolltheorie/matlab/inverted_pendulum.pdf). It also contains a MatLab assisted derivation of its equations of motions.


