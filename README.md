# drone-ddp
Drone trajectory optimization using Differential Dynamic Programming. Written for AE 4803 ROB FA24 final project. Code written by myself and Kush Bandi (https://www.linkedin.com/in/kush-bandi/). <br>
<br>
Based on the dynamics for a quadrotor drone, a second-order cost function approximation, and second-order dynamics, a differential dynamic programming (alternating forward and backward passes) approach is utilized to bring the drone from its initial condition to a desired final state while avoiding obstacles. Obstacles are implemented in the form of barrier functions, which have their cost go to infinity as the drone approaches the obstacle's position. The differential dynamic programming (DDP) is extended to a Model-Predictive Control (MPC) implementation, which re-plans the optimal trajectory online during the drone's flight. Results from the MPC using DDP with obstacles are shown below.
<p align="center">
<img width="350" height="300" alt="image" src="https://github.com/user-attachments/assets/46c9a707-8a77-4351-8bb2-20e41e537675" />
<img width="350" height="300" alt="image" src="https://github.com/user-attachments/assets/e5e5ed3a-356b-4c7f-89df-529354c28b30" />
</p>
<p align="center">
<img width="350" height="300" alt="image" src="https://github.com/user-attachments/assets/b839340b-c9ce-485e-ad5d-58d283d4019b" />
</p>
