# Multi-robot Planning

This is a repository of algorithms and ROS nodes focused on multi-robot motion planning.  The core component is the base class, _CentralizedPlanner_ defined in mr\_generic\_planner which defines a class that creates a costmap and presents the generic planning API. Currently implemented children of _CentralizedPlanner_ are:

* _CentralizedRRTPlanner_ in mr\_rrt\_planner which is an RRT implementation based on the [RRTS](http://sertac.scripts.mit.edu/rrtstar/) source.
* _CentralizedSBPLPlanner_ in mr\_sbpl\_planner which is a search-based planning library ([SBPL](http://www.ros.org/wiki/sbpl)) developed by Maxim Likhachev.

This code was developed as part of the ground work for multi-robot planning with quality-of-service networking constraints described in the thesis [Communication for Teams of Networked Robots](http://www.seas.upenn.edu/~jonfink/thesis/fink_thesis_2011-06-15-1516.pdf) by Jonathan Fink.  The hope is that it may be useful for other applications.

## External code

There are two pieces of source code included in this repository developed by others:

* [RRTS](http://sertac.scripts.mit.edu/rrtstar/) - This is a great implementation of the basic RRT application (as well as the more recent RRT\* variant).  It is coded in C and quite fast.

* [SBPL](http://www.ros.org/wiki/sbpl) - Another great motion planning library.  Developed by Maxim Likhachev, it provides implementations of a number of search-based motion planning algorithms.  We are currently only using the ARA\* algorithm, but with the hooks in _CentralizedSBPLPlanner_, it is relatively straight forward to rely on other algorithms included in SBPL.

## License
Code is licensed under the BSD license.

Copyright (c) 2011, Jon Fink
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

