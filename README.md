N-Dimensional Fast Methods Library v0.7

**Authors:**
 - [Javier V. Gomez](http://jvgomez.github.io) javvgomez _at_ gmail.com
 - Jose Pardeiro jose.pardeiro _at_ gmail.com
 - Pablo Gely

## ALGORITHMS
All the theory and algorithms implemented in this library can be found in my [PhD thesis](doc/files/phd_thesis.pdf). In fact, all the benchmarking data in chapter 4 has been produced with this library and it is stored on the `experiments` branch.


**Fast Marching Methods:**
- [FMM](http://jvgomez.github.io/fast_methods/classFMM.html): Fast Marching Method with Binary Queue and Fibonacci Queue (binary by default).
- [FMM*](http://jvgomez.github.io/fast_methods/classFMMStar.html): FMM with CostToGo heuristics.
- [SFMM](http://jvgomez.github.io/fast_methods/classSFMM.html): Simplified Fast Marhching Method.
- [SFMM*](http://jvgomez.github.io/fast_methods/classSFMMStar.html): SFMM with CostToGo heuristics..

**O(n) Fast Marching Methods:**
- [GMM](http://jvgomez.github.io/fast_methods/classGMM.html): Group Marching Method.
- [UFMM](http://jvgomez.github.io/fast_methods/classUFMM.html): Untidy Fast Marching Method.
- [FIM](http://jvgomez.github.io/fast_methods/classFIM.html): Fast Iterative Method.

**Fast Sweeping Methods:**
- [FSM](http://jvgomez.github.io/fast_methods/classFSM.html): Fast Sweeping Method.
- [LSM](http://jvgomez.github.io/fast_methods/classLSM.html): Lock Sweeping Method.
- [DDQM](http://jvgomez.github.io/fast_methods/classDDQM.html): Dynamic Double Queue Method.

**Fast Marching Square motion planning algorithms:**
- [FM2](http://jvgomez.github.io/fast_methods/classFM2.html): Fast Marching Square Method.
- [FM2*](http://jvgomez.github.io/fast_methods/classFM2Star.html): Fast Marching Square Star FM2 with CostToGo heuristics.

**ROS**

ROS nodes using this code (tested in the TurtleBot) are provided in a [separate repo](https://github.com/jpardeiro/fastmarching_node)


## DISCLAIMER and IMPORTANT NOTES

- The code is not deeply tested. I've just tested it works for the cases I need. If you find any problem, have any question (or whatever), please write to: javvgomez _at_ gmail.com

- The compilation time is highly increased due to CImg library. Please, omit it when possible as it is used only for visualization purposes.

- License [GNU/GPL V3](http://www.gnu.org/copyleft/gpl.html)

- This is a source code intended for my research. Although I want it to be useful for other people it is not intended to act as a library (there are many many points to improve). However, if you show interest or have feature request do not hesitate to contact me and I will try my best to improve the code for whoever needs it. I am also open to contributions and to create a formal library if necessary.


## Documentation

- [API Reference](jvgomez.github.io/fast_methods/)


## Building the code
Check the [building section](http://jvgomez.github.io/fast_methods/md_markdown_building.html) of the documentation.


## Design and folder structure
Check the [design section](http://jvgomez.github.io/fast_methods/md_markdown_design.html) of the documentation.


## KNOWN ISSUES

- Gradient Descent for FM2* could fail if very narrow passages are in the way of the path.
- It seems that UFMM can fail in maps with random (or similar) velocity changes.