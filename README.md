# A-Pragmatic-Framework-for-Mobile-Redundant-Manipulator-Performing-Sequential-Tasks
In this paper, a framework combining base place- ment, path planning and redundancy resolution for a mobile manipulator performing sequential tasks, such as screwing, drilling or assembling tasks, is proposed. For a set of given tasks, the outputs of the proposed algorithm meet the following practical performance indicators: minimization of the number of the base positions, minimization of the number of manipulator joint configuration changes, feasibility of each task considering the force capacity of the manipulator (which takes benefit of redundancy resolution) and path planning of the end-effector motion with obstacle avoidance. The effectiveness of the proposed approach is evaluated considering a 3 DOFs mobile platform and a 7 DOFs manipulator performing screwing in a application with 42 tasks.


# Use-case description and objectives
The figure bellow illustrates the application considered in this work, where a 10 DOFs mobile manipulator composed of 7 DOFs serial robot arm mounted on a 3 DOFs holonomic mobile platform has to perform a sequence of discrete tasks, such as screwing. The spatial distribution of the tasks is large enough that several mobile base motions are required. From the starting position, the mobile manipulator moves to the first base position and performs a series of screwing tasks (group 1) while avoiding self-collision and the collision between the mobile manipulator and its environment, and so on for the other tasks group. 

 [My Image](../FIG%20GITHUB%20README/presentation.png)

The application objectives are :

## GOAL 1: minimizing the number of movements of the mobile base.  
The total operation time increases with the number of base positions due to: a) The mobile manipulator decelerates and then accelerates at each base position and b) due to the positioning accuracy of conventional base (a few centimeters), the resulting base position may deviate significantly from the desired position. The mobile robot has then to perform a time-consuming fine repositioning process. It is therefore essential to minimize the number of movements of the base.

## GOAL 2: task's feasibility (FCI)  
With its 10 DOFs, the mobile manipulator is a kinematically redundant system. Without loss of generality, three redundant parameters are considered in this work: the Y-axis and X-axis position of the mobile manipulator ***(Y<sub>B</sub>, X<sub>B</sub>)*** (see fig. \ref{explication})  and the swivel angle ***Swiv*** (see fig. \ref{Status_definition}).
% Thus, for each task, there exists an infinite number of solutions to the inverse kinematic's problem. 
The FCI method \cite{Management_Mobile_Redundant_Manipulators_2018}, which determines the manipulator’s FC inside the redundancy space according to the joints configuration, is used in this study to fix the choice of the redundancy parameters. 

## GOAL 3: generate short paths thanks to 1. status, 2. swivel angle and 3. task's order assignement (TSP)
To generate shorter paths and improve workstation safety by removing large movements, the choice of joints configuration and task order is essential. The choice of the manipulator configurations is done thanks to two sub-criteria (see fig. \ref{Status_definition}): the manipulator's form and the swivel angle $Swiv$. Hence, two constraints are added to maintain whenever possible: a) the same manipulator's form and b) the same swivel angle. The manipulator's form corresponds to a joint configuration for a given pose which is chosen thanks to the choice of a binary combination of the triplet (shoulder, elbow, wrist). They are eight possibles combinations for the considered redundant manipulator (Kuka IIWA). The tasks order choice may be solved by using a combinatorial optimization technique: the traveling salesman problem. Additionally, in case it is not possible to fix the same form, a strategy can be implemented to limit and reduce the traveling path between two tasks: a wrist motion is shorter than a wrist+elbow+shoulder motion. Thus, a qualification of the path planning is possible.
