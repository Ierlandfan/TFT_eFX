# TFT_eFX

<<<<<<< HEAD
A template graphics extension library for TFT_eSPI.

Extra graphics functions can be added to this library to suit you projects. At the moment it has a function to draw curves and a simple 
myGraphicsFunction(x, y, color) example function that draws a 3x3 pixel block of the specified color centered on x,y

See the TFT_eFX_Your_Function example to get the general idea.

Use as a template when you add your graphics functions to the libraries files:

	TFT_eFX.h     - add function prototype, see myGraphicsFunction() line
	TFT_eFX.cpp   - add function with code, see myGraphicsFunction() lines
	keywords.txt  - add function name to highlight in IDE, see myGraphicsFunction line

If you have a Github acount you can create a pull request for possible inclusion of your graphics functions to the master library. The caveat is that functions should be of general appeal to other users and an example should be added to the examples folder.

Potentially this library could be adapted to work with other graphics libraries.

Enjoy!
=======
A template library that allows extra graphics function to be added to the TFT_eSPI library.

This library will be expanded over time to add new graphics capabilities to the library, this may include gauges, dials, progress bars, graph drawing etc.

At the moment the library includes functions for drawing bezier curves by defining a set of 3 points. The example demonstrates the drawing of the curves.

If you fork this library and add new functions plus a commented example, then do create a pull request!
>>>>>>> origin/master
