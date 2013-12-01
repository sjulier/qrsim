The QRSim Quadrotors Simulator
=====

Examples of quadrotor helicopters models described in the literature (e.g. [1],[2],[3]) tend to focus on reproducing only the dynamic aspects the aerial platform and their
primary use is in the domain of closed loop flight control. 

When the aim is to simulate more general higher level tasks that involve multiple platforms which sense and react in their environment, the usefulness of such models is limited.

The QRSim multi-vehicle software simulator is developed to devise and test algorithm that allow a set of UAVs to communicate and cooperate to achieve common goals. In addition to realistically simulate the dynamic of the aerial platform (learned from flight tests), the software simulates the sensors suite that typically equips a UAV (i.e. GPS, IMU, camera) and the endogenous and exogenous sources of inaccuracies that characterize both platforms and sensors. Environmental effects that act directly (e.g. wind) or indirectly (e.g. the plume dispersion) the task are also part of the simulation.

To offer a well structured and challenging set of control and machine learning problems the simulator also includes the implementation of a number of well defined non trivial task scenarios. Namely these are a pursue evasion game, a plume mapping application in presence of wind, and a search and rescue mission that uses the simulated on-board camera.

## Videos

The following two videos give a glimpse of some of the QRSim capabilities:  

[![QRSim capabilities: sensor noise and wind effects](https://github.com/UCL-CompLACS/qrsim/raw/master/doc/YoutubeVideo1.jpg)](http://www.youtube.com/watch?v=5ka4tP0z2RQ)


[![QRSim capabilities: large number of UAVs](https://github.com/UCL-CompLACS/qrsim/raw/master/doc/YoutubeVideo2.jpg)](http://www.youtube.com/watch?v=SjOaX4Z0iLk)

## Quick usage guide

To run a simple simulation of a single quadrotor holding a waypoint, follow these instructions:

1. Clone the Git repository: <b>git clone https://github.com/UCL-CompLACS/qrsim.git</b>
2. Open Matlab and switch to code: <b>cd /some/path/to/qrsim</b> 
3. Add the 'sim' directory to the Matlab search path: <b>addpath sim</b>
4. Change to the 'example' directory: <b>cd example</b>
5. Run an example: <b>main</b>

The simulation may also be controlled from outside Matlab. Currently, this is only supported in an Ubuntu Linux environment, and requires a little more work to set up. 

1. Configure Oracle Java 1.6 as Ubuntu default, because Matlab currently does not support other versions.
2. Install the required libraries: <b>sudo apt-get install cmake libprotoc-dev libprotobuf7 libprotobuf-lite7 libprotobuf-java</b>
3. Change to the 'tcp-linux' directory: <b>cd /some/path/to/qrsim/tcp-linux</b> 
4. Prepare the build environment: <b>cmake</b> 
5. Build the code: <b>make</b>
6. Install the buid products: <b>sudo make install</b>

To test the TCP interface, run the following:

1. In matlab: <b>QRSimTCPServer(10000)</b>
2. In linux: <b>/some/path/to/qrsim/tcp-linux/bin/testClient 127.0.0.1 10000</b>


## Documentation
The best way to have an understanding of how the QRSim software is structured and of what platforms, sensors and scenarios models are implemented is to read the following documentation:

* User manual [<a href="https://github.com/UCL-CompLACS/qrsim/raw/master/doc/manual.pdf">pdf</a>]
* Scenarios manual  [<a href="https://github.com/UCL-CompLACS/qrsim/raw/master/doc/scenarios.pdf">pdf</a>]
* Step by step tutorial [<a href="https://github.com/UCL-CompLACS/qrsim/raw/master/doc/tutorial.pdf">pdf</a>]

The documentation of the QRSim API can be accessed through the standard Matlab documentation system (i.e. using the Matlab command `doc`) 

## Citing
If you use this software in an academic context, please cite the following publication:

* Renzo De Nardi, <a href="http://www0.cs.ucl.ac.uk/staff/R.DeNardi/DeNardi2013rn.pdf">_The QRSim Quadrotors Simulator_<a/> Research Note RN/13/08, Department of Computer Science University College London, March 2013. [[bibtex](doc/qrsimcite.bib)]

## License
With the exception of the libraries in the [`3rdparty`](3rdparty) folder which are covered by their respective licenses, the QRSim software can be redistributed in accordance with the [Modified BSD License] (LICENSE).

## Support
Due to lack of time we are currently unable to provide direct support for the software, however we will do our best to address any problem reported via the GitHub <a href="https://github.com/UCL-CompLACS/qrsim/issues"> issue system<a/>.  

## Acknowledgments
The author wants to thank Guy Lever, Nicolas Hees, Simon Julier, John Shawe-Taylor, David Silver, Stephen Hailes and Luke Teacy for the fruitful discussing about the application scenarios and the classifier model. This work was carried out with the support of the European Research Council \#FP7-ICT-270327 (<a href="http://www.complacs.org">CompLACS</a>).


### References
1.<a id="one"></a>  S. Bouabdallah. _Design and control of quadrotors with application to autonomous flying._ PhD thesis, EPFL, 2007

2.<a id="two"></a>  C. Balas. _Modelling and linear control of a quadrotor._ Master's thesis, School of
Engineering, Cranfield University, 2007.

3.<a id="three"></a>  P. Pounds, R. Mahony, and P. Corke. _Modelling and control of a quad-rotor robot._
In ACRA, 2006.




