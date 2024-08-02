Here’s an updated README file incorporating the NuSMV solver along with the revised example steps:

---

# Temporal Logic Guided Robot Navigation (TLG-RN)
An MILP Approach for faster controller synthesis of ground robot navigation using Linear Temporal Logic specifications. This approach is an improvement over our [previous](https://github.com/aparame/Temporal_Logic_Guided_Robot_Navigation) that used STL based approaches for robotic navigation. This work is to be part of a conference paper to be presented at [MECC 2024](https://mecc2024.a2c2.org/)

## Installing Dependencies

This code depends on the following tools:

1. **YALMIP**: Obtain it with the Multi-Parametric Toolbox (MPT3). See the [YALMIP Installation Guide](https://yalmip.github.io/tutorial/installation/). MPT is also required for plotting polyhedras.

2. **s-TaLiRo**: Required for computing robustness for the STL specifications. Download from [s-TaLiRo Download Page](https://sites.google.com/a/asu.edu/s-taliro/s-taliro/download).

3. **Gurobi Solver**: Used as the back-end for solving the optimization problem. For the user-interactive example to work without modifications, Gurobi needs to be installed and configured for Matlab. Visit [Gurobi](http://www.gurobi.com) for installation instructions.

4. **NuSMV Solver**: Required for model checking and verification. Install NuSMV by following these steps:
   - Download the NuSMV binaries from [NuSMV Download Page](https://nusmv.fbk.eu/).
   - Extract the downloaded archive to a directory of your choice.
   - Add the NuSMV bin directory to your system’s PATH environment variable.

5. **Custom ginput.m**: This example also uses a customized `ginput.m` file by Jiro Doke, available at [Custom ginput](http://www.mathworks.com/matlabcentral/fileexchange/38703-custom-ginput/content/ginputc.m), and is included here.

## Example

To test run the code:

1. **Run the Code**: Execute `main.m` in Matlab. At Line 19, select one of the two available maps: `simpleMap` or `complexMap`.

2. **Initial Position**: Choose a point on the map to set as the initial position for the robot.

3. **Number of Goals**: Enter the number of goals in the Matlab command window.

4. **Design Goal Region**: Design a quadrilateral (4-sided) goal region by selecting points anywhere on the map. Repeat this process based on the number of goals entered previously.

5. **Run the Code**: Press Enter one more time to run the code. The code will generate a fast temporal logic-guided safe path from the initial position to the goals.

**Note**: The robot dynamics used in the code are based on a 2D kinematic ground robot.

## Contact Us

For any queries or to report bugs, contact aparame@clemson.edu.
<!---
**Parameshwaran, Aditya**, and Yue Wang. Safety Verification and Navigation for Autonomous Vehicles Based on Signal Temporal Logic Constraints. No. 2023-01-0113. SAE Technical Paper, 2023.

--->

Feel free to adjust any specific details as needed!
