---
title: 'A C++ Implementation of a (Cartesian) Impedance Controller for Robotic Manipulators'
tags:
  - ROS
  - Compliant Control
  - Cartesian Impedance Controller
authors:
  - name: Matthias Mayr
    orcid: 0000-0002-8198-3154
    equal-contrib: false
    affiliation: "1, 2" # (Multiple affiliations must be quoted)
  - name: Julian M. Salt-Ducaju
    orcid: 0000-0001-5256-8245 
    equal-contrib: false
    affiliation: "1, 2"
affiliations:
 - name: Faculty of Engineering (LTH), Lund University, Sweden
   index: 1
 - name: Wallenberg AI, Autonomous Systems and Software Program (WASP), Sweden
   index: 2
date: 8 December 2022
bibliography: paper.bib

---

# Summary

Impedance control increases the safety in contact-rich environments where robots are present by establishing a mass-spring-damper relationship between external forces acting on the robot and variation from its reference of a set of coordinates that describe the motion of a robot. As a consequence, the controlled robot behaves in a compliant way with respect to its external forces, which has the added benefit of allowing that a human operator can manually guide the robot. 

In this package, we provide a C++ implementation of a controller that allows collaborative robots:

1. To achieve compliance in its, Cartesian, task-frame coordinates.
2. To achieve joint compliance in the null-space of its task-frame coordinates.
3. To be able to apply a desired force to the environment in a contact situation.

This package can be used in any torque-controller robotic manipulator, as long as a URDF description of its geometry is provided.

# Statement of Need
Modern robotics is moving more and more past the traditional robot systems that have hard-coded paths and stiff manipulators. Many use-cases require the robots to work in semi-structured environments. These environments impose uncertainties that could cause collisions. Furthermore, many advanced assembly, manufacturing and household scenarios such as insertions or wiping motions require the robot to excert a controlled force on the environment. Finally, the robot workspace is becoming increasingly shared with human workers in order to leverage both agents and allow them to complement each other.

A compliant control implementation for robotic manipulators is a valid solution for robots in contact-rich environments, since it fulfills the following criteria, allowing the robot to:

1. Dynamically adapt the end-effector reference point.
2. Dynamically adapt the Cartesian stiffnesses.
3. Apply commanded forces and torques in the frame of the end-effector of the robot.
4. Command a joint configuration and apply it in the nullspace of the Cartesian robotic task.
5. Execute joint-space trajectories.


Moreover, the Robot Operating System (ROS) is an open-source middleware that is widely used in the robotics community for the development of robotic sofware systems [@quigley:2009]. Within ROS, such a compliant control solution is available for position-commanded and velocity-commanded robotic manipulators with the `cartesian_controllers` package [@FDCC]. However, if a robotic manipulators supports to directly be commanded joint torques, *e.g.*, the `KUKA iiwa` or the `Franka Emika Robot (Panda)`, this is often the preferred control strategy, since a stable compliant behavior might be achieved for position-commanded and velocity-commanded robotic manipulators [@lawrence:1988]. 

A complete implementation of compliance for torque-commanded robtics manipulators is not available, and the existing solutions can only be used for a single type of robotic manipulator:

|                         | Reference<br> Pose<br> Update | Cartesian<br> Stiffness<br> Update | Cartesian<br> Wrench<br> Update | Nullspace<br> Control | Kinesthetic<br> Teaching | Trajectory<br> Execution | Multi-Robot<br> Support |
|-------------------------|:-----------------------------:|:----------------------------------:|:-------------------------------:|:---------------------:|:------------------------:|:------------------------:|:-----------------------:|
| **KUKA FRI Cart. Imp.** | x                             |                  x                 | ?                               | ?                     | (✓)1                     | ?                        | x                       |
| **franka_ros**          | ✓                             |                  x                 | ✓                               | ✓                     | ✓                        | x                        | x                       |
|  **libfranka**          | x                             |                  x                 | x                               | x                     | (✓)2                     | x                        | x                       |
| **This package**        | ✓                             |                  ✓                 | ✓                               | ✓                     | ✓                        | ✓                        | ✓                       |

1: Reaching a joint limit triggers a safety stop<br>
2: Can be implemented by setting the Cartesian stiffness to zero

- TODISCUSS: When I compile the paper with the Docker I can't see the ticks. ??? Also, note 2 would also be applied for our package (regarding kinesthetic teaching).

- TODO: Talk about RL use-case
- TODO: Cite papers: @mayr22skireil and @mayr22priors @ahmad2022generalizing

# Control Implementation

The gravity-compensated rigid-body dynamics of the controlled robot can be described, in the joint space of the robot $q\in  \mathbb{R}^{n}$, as [@springer:2016]:
\begin{equation}\label{eq:rigbod_q}
    M(q)\ddot{q} + C(q,\dot{q})\dot{q} = \tau_{\mathrm{c}} + \tau^{\mathrm{ext}}
\end{equation}
where $M(q)\in  \mathbb{R}^{n\times n}$ is the generalized inertia matrix, $C(q,\dot{q})\in  \mathbb{R}^{n\times n}$ captures the effects of Coriolis and centripetal forces, $\tau_{\mathrm{c}}\in  \mathbb{R}^{n}$ represents the input torques, and $\tau^{\mathrm{ext}}\in  \mathbb{R}^{n}$ represents the external torques, $n$ being the number of joints of the robot. The gravity-induced torques have been ignored in (\autoref{eq:rigbod_q}), since the studied robots (`KUKA LBR iiwa robot` and `Franka Emika Panda robot`) are automatically gravity-compensated.

Moreover, the torque signal commanded by the proposed controller to the robot, $\tau_{\mathrm{c}}$ in (\autoref{eq:rigbod_q}), is composed by the superposition of three joint-torque signals:
\begin{equation}\label{eq:tau_c}
    \tau_{\mathrm{c}} = \tau_{\mathrm{c}}^\mathrm{ca} + \tau_{\mathrm{c}}^\mathrm{ns} + \tau_{\mathrm{c}}^\mathrm{ext}
\end{equation}
where

*  $\tau_{\mathrm{c}}^\mathrm{ca}$ is the torque commanded to achieve a Cartesian impedance behavior [@hogan:1985] with respect to a Cartesian pose reference in the $m$-dimensional task space, $\xi^{\mathrm{D}}\in\mathbb{R}^{m}$, in the frame of the end-effector of the robot:
    \begin{equation}\label{eq:tau_sup}
        \tau_{\mathrm{c}}^\mathrm{ca} = J^{\mathrm{T}}(q)\left[-K^\mathrm{ca}\Delta \xi-D^\mathrm{ca}(J(q) \dot{q})\right]
    \end{equation}
    with $J(q)\in \mathbb{R}^{m \times n}$ being the Jacobian relative to the end-effector (task) frame of the robot, and $K^\mathrm{ca}\in \mathbb{R}^{m \times m}$ and $D^\mathrm{ca}\in \mathbb{R}^{m \times m}$ being the virtual Cartesian stiffness and damping matrices, respectively. Also, the Cartesian pose error, $\Delta \xi$ in (\autoref{eq:tau_sup}) is defined as $\Delta \xi_{\mathrm{tr}} = \xi_{\mathrm{tr}}-\xi_{\mathrm{tr}}^{\mathrm{D}}$ for the translational degrees of freedom of the Cartesian pose and as \mbox{$\Delta \xi_{\mathrm{ro}} = \xi_{\mathrm{ro}}\left(\xi_{\mathrm{ro}}^{\mathrm{D}}\right)^{-1}$} for the rotational degrees of freedom.

* $\tau_{\mathrm{c}}^\mathrm{ns}$ is the torque commanded to achieve a joint impedance behavior with respect to a desired configuration and projected in the null-space of the robot's Jacobian, to not affect the Cartesian motion of the robot's end-effector [@ott:2008]:
    \begin{equation}\label{eq:tau_ns}
        \tau_{\mathrm{c}}^\mathrm{ns} = \left(I_n-J^{\mathrm{T}}(q)(J^{\mathrm{T}}(q))^\mathrm{\dagger}\right)\tau_0
    \end{equation}
    with the superscript $^\mathrm{\dagger}$ denoting the Moore-Penrose pseudoinverse matrix\footnote{The Moore-Penrose pseudoinverse is computationally cheap and allows a null-space projection disregarding the dynamics of the robot. However, the use of this matrix for null-space projection may cause that a non-zero arbitrary torque, $\tau_0$ in (\autoref{eq:tau_ns}), generates interfering forces in the Cartesian space if the joint of the robot are not in static equilibrium ($\dot{q} = \ddot{q} = 0$).}[@khatib:1995] given by \mbox{$J^\dagger = (J^\mathrm{T}J)^{-1}J^\mathrm{T}$} [@ben:2003], and $\tau_0$ being the arbitrary joint torque formulated to achieve joint compliance, 
    \begin{equation}\label{eq:tau_0}
        \tau_0 = -K^\mathrm{ns}(q-q^{\mathrm{D}}) - D^\mathrm{ns} \dot{q}
    \end{equation}
    where $K^\mathrm{ns}\in \mathbb{R}^{n \times n}$ and $D^\mathrm{ns}\in \mathbb{R}^{n \times n}$ are the virtual joint stiffness and damping matrices, respectively.
    
* $\tau_{\mathrm{c}}^\mathrm{ext}$ is the torque commanded to achieve the desired external force command in the frame of the end-effector of the robot, $F_{\mathrm{c}}^\mathrm{ext}$:
    \begin{equation}\label{eq:tau_ext}
        \tau_{\mathrm{c}}^\mathrm{ext} = J^{\mathrm{T}}(q)F_{\mathrm{c}}^\mathrm{ext}
    \end{equation}

## Safety Measures

As described in \autoref{fig:flowchart}, there are several safety measures that have been implemented in the controller to achieve a smooth behavior of the robot:

### Filtering  \label{filt}
The proposed controller allows the online modification of relevant variables: $\xi^{\mathrm{D}}$, $K^\mathrm{ca}$ and $D^\mathrm{ca}$ in (\autoref{eq:tau_sup}), $q^{\mathrm{D}}$, $K^\mathrm{ns}$ and $D^\mathrm{ns}$ in (\autoref{eq:tau_0}), and $F_{\mathrm{c}}^\mathrm{ext}$ in (\autoref{eq:tau_ext}). However, for a smoother behavior of the controller, the value of these variables is low-pass filtered. The update law at each time-step $k$ is:
\begin{equation}
    \alpha_{k+1} = (1-a)\alpha_k + a \alpha^\mathrm{D}
\end{equation}
where $\alpha^\mathrm{D}$ is the desired new variable value and $a\in(0,1)$ is defined in such a way that $p$ percent of the difference between the desired value $\alpha^\mathrm{D}$ and the variable value at the time of the online modification instruction, $\alpha_0$, is applied after a user-defined amount of time. 



<!-- \begin{eqnarray}
   a & = & \frac{\delta t}{\kappa T + \delta t}\\
   \kappa & = & \frac{-1}{\log(1-p)}
\end{eqnarray}
being $\delta t$ the time between samples of the controller, *i.e.*, the inverse of the sampling frequency of the controller.) -->

- TODISCUSS: we could omit the expresisions for a and kappa.

### Saturation
To increase safety in the controller, some of the filtered variables (the stiffness and damping factors $K^\mathrm{ca}$, $D^\mathrm{ca}$, $K^\mathrm{ns}$ and $D^\mathrm{ns}$, and the desired external force command $F_{\mathrm{c}}^\mathrm{ext}$) can be saturated between user-defined maximum and minimum limits, *i.e.*, for an example variable $\alpha$:
\begin{equation}
    \alpha_\mathrm{min} \leq \alpha \leq \alpha_\mathrm{max} 
\end{equation}

### Rate Limiter
The rate of the commanded torque, $\tau_\mathrm{c}$ in (\autoref{eq:tau_c}), can be limited. For two consecutive commands at times $k$ and $k+1$:
\begin{equation}
    \Delta \tau_\mathrm{max} \geq \|\tau_\mathrm{c,k+1} - \tau_\mathrm{c,k}\|
\end{equation}


![Block diagram of the controller. \label{fig:flowchart}](flowchart.png){ width=80% }

# Acknowledgements

We thank Björn Olofsson and Anders Robertsson for the discussions and feedback. Furthermore, we thank Konstantinos Chatzilygeroudis for the permission to use the RBDyn wrapper code.

- TODO: Cite Konstantinos package as a Reference?

This work was partially supported by the Wallenberg AI, Autonomous Systems and Software Program (WASP) funded by Knut and Alice Wallenberg Foundation. The authors are members of the ELLIIT Strategic Research Area at Lund University.

- TODO: Are you part of ELLIT too?

# References
