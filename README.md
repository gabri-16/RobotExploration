# RobotExploration

Project that applies few different well-knwon tasks in swarm robotics scope. Robots should find few landmarks scattered around the arena and prepare to explore them using clustering techiques. For more details see the report in ```doc``` directory.

To run the application, execute the following command from the root of the project:

```
cd code
argos3 -c exploration.argos
```
The behavior of swarm robots and landmarks are contained in ```exploration.lua``` and ```landmark.lua``` files respectively (```code``` directory).

The file ```performance.ipynb``` is a pyhton notebook containing few simple tools to analyze performance data.
