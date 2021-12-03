Fade2D, v1.82, 2020, Nov. 15th
==============================

Welcome to the Fade2D Library!

The present download contains all library versions for
 
.) Windows (VS2010, VS2012, VS2013, VS2015, VS2017, VS2019) 
.) Linux (gcc) for x86_64 and Raspberry PI (ARMv6, ARMv7)
.) Apple (clang)

Documentation: http://www.geom.at/fade2d/html



Getting started
===============

This software package contains small C++ example source codes
that cover the different topics. They are documented here:
http://www.geom.at/fade-delaunay-triangulation/

Find instructions for compiling with Visual Studio OR 
for Linux/Apple below. 


* Visual Studio:
----------------
Open a solution file 

  examples_2D/vs20xx_exampleProject/examples_2D.sln   
  examples_25D/vs20xx_exampleProject/examples_25D.sln   

For maximum performance compile in x64 Release mode. Find the executable 
in fadeRelease/x64 and run it in a command line window (cmd.exe). When
you link the library with your own software you can use the same settings
that you find in the example solutions. 

VS2010 - version 10 - toolset v100 or Windows7.1SDK
VS2012 - version 11 - toolset v110 
VS2013 - version 12 - toolset v120
VS2015 - version 14 - toolset v140
VS2017 - version 15 - toolset v141
VS2019 - version 16 - toolset v142

* Linux and Apple users: 
------------------------
cd examples_2D   (or examples_25D)

* In the Makefile, choose your Linux distribution or Apple (if a specific 
Linux distro is not listed use a similar one). Be sure libgmp is installed.

$> make
$> ./allExamples_2D   (or ./allExamples_25D)





Directories
===========

.) include_fade2d and include_fade25d
Header files of Fade2D and Fade25D

.) x64 (Win32)
The *.dll and *.lib files. This is also the output directory for the
example executables compiled with Visual Studio.

.) lib_${DISTRO}_${ARCHITECTURE}
The shared libraries (*.so) for Linux and (*.dylib) Apple developers. The 
libraries work for a wide range of Linux distributions. Commercial users 
who need support for a certain additional Linux distribution: Please get 
in contact with the author. 

.) examples_2D
Source code of all examples using Fade2D

.) examples_25D
Source code of all examples using Fade2.5D

.) doc
Documentation




Student license
===============

Fade is free of charge for personal non-commercial research. But we ask 
you to put a link to Fade on your research homepage or project page and to 
cite Fade2D in scientific publications using it. Fade is not free software. 
Do not integrate it in a free software project without explicit permission.

Limits of the student version:
------------------------------
2D triangulation: 1 million points
2.5D triangulation: 100 000 points
Meshing: 50 000 output triangles
Cut&Fill: 10 000 triangles
SegmentChecker: 50 000 segments

Not enough for evaluation purposes? Ask the author (bkorn@geom.at) for an 
extended evaluation license if your project requires larger limits. Please 
describe your project and include a link to your research homepage. 


Commercial license
==================

All other applications, including commercial in-house usage, require a 
commercial license which has the advantage of maintenance, error corrections 
and personal support. The commercial license of Fade consists of one base 
component and several optional components:

* The Fade2D base component (mandatory): It covers 2D Delaunay triangulations 
and constrained Delaunay triangulations. Zones and the SegmentChecker module
are included. 

* The mesh generator and grid mesher (optional): Creates high quality triangles 
inside a specified area. The methods refine() and refineAdvanced() belong to 
this component.

* The Fade2.5D extension (optional): (x,y,z)-coordinates (2.5D) and
additional algorithms for terrains and other height fields.

* The Cut&Fill software for earthwork calculations (optional): Cut-And-Fill 
computes the volume between two overlapping TIN’s.

* The Segment Checker module is included in the base component but also 
available as a separate component without Delaunay triangulation: Given
a set of 2D or 2.5D line segments it identifies intersecting segments. 



Your feedback is appreciated:

 Geom Software
 https://www.geom.at
 Bernhard Kornberger
 bkorn@geom.at













In no case can Geom Software be made responsible for damages of any kind 
that arise in connection with the use or non-usability of the software 
or information provided on our internet pages. If you can't accept these 
terms, you are not allowed to use the software. Using Fade for military 
research and applications is not accepted.


