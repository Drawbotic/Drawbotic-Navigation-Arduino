.. Drawbotic Arduino Library documentation master file, created by
   sphinx-quickstart on Thu Mar 23 13:46:08 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Drawbotic Navigation Arduino Library
====================================

.. toctree::
    :maxdepth: 3
    :caption: Contents:

The Drawbotic Navigation Arduino library provides basic navigation functionality for Drawbotic drawing robots via an Arduino compatable C++ library. It is included by default in the Arduino Board Support Package for the Drawbotic platform.

Using the Navigation library allows you to perform a sequential queue of Navigation actions. With it you could, for example, get your Drawbotic robot to drive forward for 100mm, rotate 60 degrees clockwise, lower the pen, drive forward 40mm, then stop. You would create this queue (or any other you can think of!) ahead of time and the navigation library will ensure that each action is performed at the right time. To see a simple example of this check the Navigation Example page

Table of Contents
^^^^^^^^^^^^^^^^^
.. toctree::
    :maxdepth: 2

    self
    examples/index
    api/index
