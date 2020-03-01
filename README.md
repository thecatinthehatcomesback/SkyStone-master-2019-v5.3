# Hello There!

Welcome to our GitHub page!

We're Team #10273, The Cat in the Hat Comes Back, a FIRST Tech Challenge team from Otsego,
Minnesota.  We enjoy learning new things and love helping other teams learn too!  (That's the
point of FIRST robotics, right?)

If you have any questions, please don't hesitate to contact us!  Our email is:
catcomesback10273@gmail.com


## What We Have Done Thus Far...

In this 2019-2020 season, we've set ourselves some pretty big goals.  In fact, we began setting
them back in April 2019 after attending the World's Championship tournament for our second time
in a row!  Some of the biggest goals was to implement odometry, motion profiling, and pure pursuit.
Although we were not able to finish our pure pursuit goal before our Minnesota State Championship
tournament, we were lucky enough to win Second Place Inspire and be the First Pick of the Winning
Alliance!  With our third trip to World's in Detroit coming up, we are now currently developing
pure pursuit with eager anticipation!

In our many journeys and adventures, we've learned from our own mistakes as well as from others,
so we'd love to be able to help other teams learn what we've learned with this repository!  Some
things we have include:

* __Asynchronous Hardware Classes:__ Used so that multiple sections of the robot can run
concurrently during an autonomous run.  By utilizing `isDone()` and `waitUntilDone()` methods
an autonomous can have multiple motors and servos moving at the same time.

* __Odometry:__ Used with odometry modules to keep track of the robot's absolute position on the
field.  Odometry is a much more accurate way of tracking a robot's position, particularly due to
the large slippage rates of mecanum wheels.

* __Updates Thread:__ Spawns a separate thread to keep track of the robot's position and motion
profiling.  Motion profiling is a way of speeding up and slowing down a robot based on time and
distance to the target point for smoother drive motions.

* __Pure Pursuit:__ We are currently developing Pure Pursuit on a seperate branch.