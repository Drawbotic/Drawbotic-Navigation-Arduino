.. _navigation_example:

Navigation Example
==================
This simple example uses the Navigation Library to get your Drawbotic robot to draw a simple polygon with sides of equal length.

The navigation queue is generated in the createPolygon function which is in turn called in setup. The createPolygon function allows you to create a queue to draw polygon with any number of sides you like and with any length of side you want. This is achieved through a for loop that runs as many times as there are sides, each interation adds a forward action to draw the side, then a rotate action is used to rotate the bot the correct amount to create the desired corner, the angle of this corner is equal to 360 / the total number of corners. This means that at the end of the polygon drawing the robot will have rotated a total of 360 degrees, i.e a full rotation.

By default the example draws an Octogon with a side length of 50mm, see if you are able to change this to a different shape!

.. literalinclude:: ../../examples/NavigationExample/NavigationExample.ino
   :language: c++