gandalf
=======

Gandalf - Generic ANd DistAnce-invariant Laser Features

The Gandalf detector is an open source C++ ROS package for object detection based on generic-distance invariant range scan features. 

The detector can be trained for detection and distinction of different objects. For our application we trained it for the detection and destinction of the three object classes 1) person without walking aid, 2) person in a wheelchair, and 3) person with a walker. The trained model for detection and distinction of these objects is provided with this detector. 

The detector is not fully ported to ROS until yet. Especially the multi object detection node (gandalf_mo_detector_node) is not commited yet. 
To test the detector without object classification run: roslaunch gandalf_detector gandalf_detector.launch

When using this software for your own research, please acknowledge the effort that went into its construction by citing the corresponding paper:

  C. Weinrich, T. Wengefeld, C. Schröter and H.-M. Gross
  People Detection and Distinction of their Walking Aids in 2D Laser Range Data based on Generic Distance-Invariant Features.
  In Proceedings of the IEEE International Symposium on Robot and Human Interactive Communication (RO-MAN), 2014, Edinburgh (UK)

License

Gandalf is licensed under the GNU General Public License (GPL) v3.0.

Since the detector was originally developed with "MIRA - Middleware for Robotic Applications", some components of the provided code are part of "MIRA - Middleware for Robotic Applications" licensed under the GNU General Public License (GPL) v3.0 and available at http://www.mira-project.org. These components are slidely modified to minimize software dependencies. 

Disclaimer

THIS CODE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Use at your own risk.
