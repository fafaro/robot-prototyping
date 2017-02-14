# robot-prototyping
Playing with Bullet library to test out robotics algorithms

# Build Instructions
```
git clone https://github.com/fafaro/robot-prototyping.git
cd robot-prototyping
git clone https://github.com/bulletphysics/bullet3.git
git clone https://github.com/glfw/glfw.git
git clone https://github.com/g-truc/glm.git
git clone https://github.com/assimp/assimp.git
git clone https://github.com/leethomason/tinyxml2.git
Install Boost 1.63.0 to C:\local\boost_1_63_0
```
- Generate Visual Studio projects for all dependencies following respective instructions.
  When using CMAKE, create a 'build' sub folder in the library folder for project files.
- Open solution inside RobotPrototyping folder in Visual Studio. 
- Build.
