---
title: 'A C++ Implementation of a Cartesian Impedance Controller'
tags:
  - ROS
  - Compliant Control
  - Cartesian Impedance Controller
authors:
  - name: Matthias Mayr
    orcid: 0000-0002-8198-3154
    equal-contrib: false
    affiliation: "1, 2" # (Multiple affiliations must be quoted)
  - name: Julian M. Salt Ducaju
    orcid: 0000-0001-5256-8245 
    equal-contrib: false # (This is how you can denote equal contributions between multiple authors)
    affiliation: 2
  - name: Author with no affiliation
    corresponding: false # (This is how to denote the corresponding author)
    affiliation: 3
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
