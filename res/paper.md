---
title: 'A C++ Implementation of a Cartesian Impedance Controller for ROS'
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
date: 29 November 2022
bibliography: paper.bib

---

# Introduction
- TODO: Or make it a summary?

# Statement of need
Modern robotics is moving more and more past the traditional robot systems that have hard-coded paths and stiff manipulators. Many use-cases require the robots to work in semi-structured environments. These environments impose uncertainties that could cause collisions. Furthermore, many advanced assembly, manufacturing and household scenarios such as insertions or wiping motions require the robot to excert a controlled force on the environment. Finally, the robot workspace is becoming increasingly shared with human workers in order to leverage both agents and allow them to complement each other.

All of them have in common that a compliant control implementation for the robot arm is one solution. This needs to fulfill the following criteria:
1. Dynamically adapt the end-effector reference point
2. Dynamically adapt the Cartesian stiffnesses
3. Apply commanded forces and torques with the end effector
4. Command a nullspace configuration
5. Execute joint-space trajectories

Within the Robot Operating System (ROS) such a compliant control solution is available for position-controlled and velocity-controlled robot arms with the `cartesian_controllers` package [@FDCC]. However if a robot arm such the `KUKA iiwa` or the `Franka Emika Robot (Panda)` supports to directly command joint torques, this is often the preferred control strategy. A complete implementation of such a solution is not available:

|                         | Reference<br> Pose<br> Update | Cartesian<br> Stiffness<br> Update | Cartesian<br> Wrench<br> Update | Nullspace<br> Control | Kinesthetic<br> Teaching | Trajectory<br> Execution | Multi-Robot<br> Support |
|-------------------------|:-----------------------------:|:----------------------------------:|:-------------------------------:|:---------------------:|:------------------------:|:------------------------:|:-----------------------:|
| **KUKA FRI Cart. Imp.** | x                             |                  x                 | ?                               | ?                     | (✓)1                     | ?                        | x                       |
| **franka_ros**          | ✓                             |                  x                 | ✓                               | ✓                     | ✓                        | x                        | x                       |
|  **libfranka**          | x                             |                  x                 | x                               | x                     | (✓)2                     | x                        | x                       |
| **This package**        | ✓                             |                  ✓                 | ✓                               | ✓                     | ✓                        | ✓                        | ✓                       |

1: Reaching a joint limit triggers a safety stop<br>
2: Can be implemented by setting the Cartesian stiffness to zero

- TODO: Talk about RL use-case
- TODO: Cite papers: @mayr22skireil and @mayr22priors @ahmad2022generalizing

# Control Implementation

The rigid-body dynamics of the controlled robot can be described, in the joint space of the robot, $q$, as [@springer:2016]:
\begin{equation}\label{eq:rigbod_q}
    M(q)\ddot{q} + C(q,\dot{q})\dot{q} = \tau_{\mathrm{c}} + \tau^{\mathrm{ext}}
\end{equation}
where $M(q)\in  \mathbb{R}^{n\times n}$ is the generalized inertia matrix, $C(q,\dot{q})\in  \mathbb{R}^{n\times n}$ captures the effects of Coriolis and centripetal forces, $\tau_{\mathrm{c}}\in  \mathbb{R}^{n}$ represents the input torques, and $\tau^{\mathrm{ext}}\in  \mathbb{R}^{n}$ represents the external torques, $n$ being the number of joints of the robot. The gravity-induced torques have been ignored in (\autoref{eq:rigbod_q}), since the studied robots (KUKA LBR iiwa robot and Franka Emika Panda robot) are automatically gravity-compensated

The proposed controller sends a commanded torque signal to the robot, $\tau_{\mathrm{c}}\in \mathbb{R}^{n}$, that is composed by the superposition of three joint-torque signals:
\begin{equation}\label{eq:tau_c}
    \tau_{\mathrm{c}} = \tau_{\mathrm{c}}^\mathrm{ca} + \tau_{\mathrm{c}}^\mathrm{ns} + \tau_{\mathrm{c}}^\mathrm{ext}
\end{equation}
where

*  $\tau_{\mathrm{c}}^\mathrm{ca}$ is the torque commanded for implement Cartesian impedance control [@hogan:1985] with respect to a Cartesian pose reference in the $m$-dimensional task space, $\xi^{\mathrm{D}}\in\mathbb{R}^{m}$, in the frame of the end-effector of the robot:
    \begin{equation}\label{eq:tau_sup}
        \tau_{\mathrm{c}}^\mathrm{ca} = J^{\mathrm{T}}(q)\left[-K^\mathrm{ca}\Delta \xi-D^\mathrm{ca}(J(q) \dot{q})\right]
    \end{equation}
    with $J(q)\in \mathbb{R}^{m \times n}$ being the Jacobian relative to the end-effector (task) frame of the robot,   
    $K^\mathrm{ca}\in \mathbb{R}^{m \times m}$ and $D^\mathrm{ca}\in \mathbb{R}^{m \times m}$ being the virtual Cartesian stiffness and damping matrices, respectively. Also, the Cartesian pose error, $\Delta \xi$ in (\autoref{eq:tau_sup}) is defined as $\Delta \xi_{\mathrm{tr}} = \xi_{\mathrm{tr}}-\xi_{\mathrm{tr}}^{\mathrm{D}}$ for the translational degrees of freedom of the Cartesian pose and as \mbox{$\Delta \xi_{\mathrm{ro}} = \xi_{\mathrm{ro}}\left(\xi_{\mathrm{ro}}^{\mathrm{D}}\right)^{-1}$} for the rotational degrees of freedom.

* $\tau_{\mathrm{c}}^\mathrm{ns}$ is the torque commanded for joint impedance control with respect to a desired configuration and projected in the null-space of the robot's Jacobian, to not affect the Cartesian motion of the robot's end-effector [@ott:2008]:
    \begin{equation}\label{eq:tau_ns}
        \tau_{\mathrm{c}}^\mathrm{ns} = \left(I_n-J^{\mathrm{T}}(q)(J^{\mathrm{T}}(q))^\mathrm{\dagger}\right)\tau_0
    \end{equation}
    with the superscript $^\mathrm{\dagger}$ denoting the Moore-Penrose pseudoinverse matrix\footnote{The Moore-Penrose pseudoinverse is computationally cheap and allows a null-space projection disregarding the dynamics of the robot. However, the use of this matrix for null-space projection may cause that a non-zero arbitrary torque, $\tau_0$ in (\autoref{eq:tau_ns}), generates interfering forces in the Cartesian space if the joint of the robot are not in static equilibrium ($\dot{q} = \ddot{q} = 0$) [@khatib:1995].} given by \mbox{$J^\dagger = (J^\mathrm{T}J)^{-1}J^\mathrm{T}$} [@ben:2003], and $\tau_0$ being the arbitrary joint torque formulated to achieve joint compliance, 
    \begin{equation}\label{eq:tau_0}
        \tau_0 = -K^\mathrm{ns}(q-q^{\mathrm{D}}) - D^\mathrm{ns} \dot{q}
    \end{equation}
    where $K^\mathrm{ns}\in \mathbb{R}^{n \times n}$ and $D^\mathrm{ns}\in \mathbb{R}^{n \times n}$ are the virtual joint stiffness and damping matrices, respectively.
    
* $\tau_{\mathrm{c}}^\mathrm{ext}$ is the torque to achieve the desired external force command in the frame of the end-effector of the robot, $F_{\mathrm{c}}^\mathrm{ext}$:
    \begin{equation}\label{eq:tau_ext}
        \tau_{\mathrm{c}}^\mathrm{ext} = J^{\mathrm{T}}(q)F_{\mathrm{c}}^\mathrm{ext}
    \end{equation}

## Safety Measures

There are several safety measures that have been implemented to provide a smoother behavior of the controller:

### Filtering  \label{filt}
The proposed controller allows the online modification of relevant variables: $\xi^{\mathrm{D}}$, $K^\mathrm{ca}$ and $D^\mathrm{ca}$ in (\autoref{eq:tau_sup}), $q^{\mathrm{D}}$, $K^\mathrm{ns}$ and $D^\mathrm{ns}$ in (\autoref{eq:tau_0}), and $F_{\mathrm{c}}^\mathrm{ext}$ in (\autoref{eq:tau_ext}). However, for a smoother behavior of the controller, the value of these variables is low-pass filtered. For an example variable $\alpha$ is updated at each time-step $k$:
\begin{equation}
    \alpha_{k+1} = (1-a)\alpha_k + a \alpha^\mathrm{D}
\end{equation}
where $\alpha^\mathrm{D}$ is the desired new variable value and $a\in(0,1)$ is defined in such a way that $p$ percent of the difference between the desired value $\alpha^\mathrm{D}$ and the initial variable value $\alpha_0$ is applied after $T$ seconds:
\begin{eqnarray}
   a & = & \frac{\delta t}{\kappa T + \delta t}\\
   \kappa & = & \frac{-1}{\log(1-p)}
\end{eqnarray}
being $\delta t$ the time between samples of the controller, *i.e.*, the inverse of the sampling frequency of the controller.

### Saturation
To increase safety in the controller, some of the filtered variables in \mbox{Sec. \autoref{filt}} (the stiffness and damping factors $K^\mathrm{ca}$, $D^\mathrm{ca}$, $K^\mathrm{ns}$ and $D^\mathrm{ns}$, and the desired external force command $F_{\mathrm{c}}^\mathrm{ext}$) are saturated between user-defined maximum and minimum limits, *i.e.*, for an example variable $\alpha$.
\begin{equation}
    \alpha_\mathrm{min} \leq \alpha \leq \alpha_\mathrm{max} 
\end{equation}

### Rate Limiter
The rate of the commanded torque, $\tau_\mathrm{c}$ in (\autoref{eq:tau_c}), can be limited. For two consecutive commands at times $k$ and $k+1$:
\begin{equation}
    \Delta \tau_\mathrm{max} \geq \|\tau_\mathrm{c,k+1} - \tau_\mathrm{c,k}\|
\end{equation}

# Control Implementation
This contains some LaTeX example code. Feel free to remove.

Single dollars ($) are required for inline mathematics e.g. $f(x) = e^{\pi/x}$

Double dollars make self-standing equations:

$$\Theta(x) = \left\{\begin{array}{l}
0\textrm{ if } x < 0\cr
1\textrm{ else}
\end{array}\right.$$

You can also use plain \LaTeX for equations
\begin{equation}\label{eq:fourier}
\hat f(\omega) = \int_{-\infty}^{\infty} f(x) e^{i\omega x} dx
\end{equation}
and refer to \autoref{eq:fourier} from text.

# Citation Instructions

Citations to entries in paper.bib should be in
[rMarkdown](http://rmarkdown.rstudio.com/authoring_bibliographies_and_citations.html)
format.

For a quick reference, the following citation commands can be used:
- `@author:2001`  ->  "Author et al. (2001)"
- `[@author:2001]` -> "(Author et al., 2001)"
- `[@author1:2001; @author2:2001]` -> "(Author1 et al., 2001; Author2 et al., 2002)"

# Figures

Figures can be included like this:
![Caption for example figure.\label{fig:example}](flowchart.png)
and referenced from text using \autoref{fig:example}.

Figure sizes can be customized by adding an optional second parameter:
![Caption for example figure.](flowchart.png){ width=10% }

# Acknowledgements

We thank Björn Olofsson and Anders Robertsson for the discussions and feedback. Furthermore, we thank Konstantinos Chatzilygeroudis for the permission to use the RBDyn wrapper code.

This work was partially supported by the Wallenberg AI, Autonomous Systems and Software Program (WASP) funded by Knut and Alice Wallenberg Foundation.

# References
