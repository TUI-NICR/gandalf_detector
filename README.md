gandalf
=======

Gandalf - Generic ANd DistAnce-invariant Laser Features

The Gandalf detector is an open source C++ ROS package for object detection based on generic-distance invariant range scan features. 

The detector can be trained for detection and distinction of different objects. For our application we trained it for the detection and destinction of the three object classes 1) person without walking aid, 2) person in a wheelchair, and 3) person with a walker. The trained model for detection and distinction of these objects is provided with this detector. 

The detector is not fully ported to ROS until yet. Especially the training of a detector on your own data is not supported until yet. However, we plan to provide the necessary sources soon. 

To try out the detector run:
roslaunch gandalf_detector gandalf_detector.launch
Then you can visualize the laser range data and the markers (which show the detections) using rviz

When using this software for your own research, please acknowledge the effort that went into its construction by citing the corresponding paper:

  C. Weinrich, T. Wengefeld, C. Schröter and H.-M. Gross
  People Detection and Distinction of their Walking Aids in 2D Laser Range Data based on Generic Distance-Invariant Features.
  In Proceedings of the IEEE International Symposium on Robot and Human Interactive Communication (RO-MAN), 2014, Edinburgh (UK)

License

Gandalf is licensed under the GNU General Public License (GPL) v3.0.

Since the detector was originally developed with "MIRA - Middleware for Robotic Applications", some components of the provided code are part of "MIRA - Middleware for Robotic Applications" licensed under the GNU General Public License (GPL) v3.0 and available at http://www.mira-project.org. These components are slidely modified to minimize software dependencies. 

Disclaimer

THIS CODE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Use at your own risk.


Limitations

- Multiple detections for one object: In case of person detection, sometimes only one leg of a person is visible, for example if this leg covers the other leg. That's why the classifier learned to classify feature vectors of only one leg as a person as well. This results in two detections for one person, whose legs are both visible. This detections are merged to one hypothesis by covariance intersection in our tracker [Volkhardt-SMC-2013], which is not part of the gandalf_package.

- False detections: Our tracker [Volkhardt-SMC-2013] fuses the laser-based detections with futher detections (such as visual detections). Thereby laser-based detections, which do not move and are not confirmed by other detectors, do not result in confident tracker hypotheses. 

[Volkhardt-SMC-2013]
  Volkhardt, M., Weinrich, Ch., Gross, H.-M.
  People Tracking on a Mobile Companion Robot.
  In: Proc. IEEE Int. Conf. on Systems, Man, and Cybernetics (SMC), pp. 4354-4359, 2013 
