disneylearning
============

Required packages:
Opengl/Glu/Glut.
CMake building tools: http://www.cmake.org/
Google logging library: https://code.google.com/p/google-glog/
Boost library (1.54): http://www.boost.org/
Qt4 UI library (4.8): https://qt-project.org/search/tag/qt4
Box2D (2.3.0) : http://box2d.org/
Eigen Vector Library: http://eigen.tuxfamily.org/index.php?title=Main_Page
Shark machine learning: http://image.diku.dk/shark/sphinx_pages/build/html/index.html
Gaussian Process: https://github.com/mblum/libgp
Tinyxml2: http://www.grinninglizard.com/tinyxml2/
NLOpt: http://ab-initio.mit.edu/wiki/index.php/NLopt

Building instructions:
cd disneylearning
mkdir build
cd build
cmake ..
make
./disneylearning 
(or just for training, run ./disneylearning nogui)


