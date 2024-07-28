# Creating a Two-Wheeled Robot - And How to Teach it to Jump
In this "Blog Post", I want to demonstrate the challenges I faced while making this robot and explain the history of it a bit. I don't have an actual blog, so I'll just host it here on Github, together with the Source Code of the project.

It all started when I learned about [Raspibotics MABEL robot](https://raspibotics.wixsite.com/pibotics-blog/post/01-build-a-self-balancing-robot-with-legs-boston-dynamics-handle-inspired), together a list of materials and a description how to build it. I've always been interested in Robotics and this guide seemed like a good place to start. I also had some extra time due to Corona. My goal was to rebuild this MABEL robot, and then to improve upon it. The robot is basically a small version of [Handle by Boston Dynamics](https://www.youtube.com/watch?v=-7xvqQeoA8c) but lacks some of its features: Jumping, moving on a ramp and grabbing things. I wanted to add these features.

Building MABEL went surprisingly smooth. I bought a 3D printer, learned some electronics, learned how to do some basic 3D modeling in blender and wrote some code that I put on a Raspberry Pi and connected everything together. You can see the initial version below (right) next to my inspiration (left):

![MABEL Inspiration](https://8gadgetpack.net/milana/mabel2.jpg)
![Initial Version GIF](https://8gadgetpack.net/milana/mabel_copy3.gif)

You can't see it balancing there, but it has stepper motors on the wheels and a total of 4 servo motors for its "legs". Getting the robot to balance wasn't difficult. You put in an IMU sensor, implement a so called PID controller and tune 3 parameters until it balances. There are a lot of resources out there that explain this in detail.

The difficult part is what you don't see in the video: the jumping. The servo motors move with maximum speed in that video. And the wheels don't even leave the ground. I realized then that I needed completely different motors. Unfortunately I didn't know much about motors back then. So I started experimenting a bit. I took a brushed motor with a built in gear (those a very cheap and easy to use) and tried to load a spring with it. Once the spring is released, it could use all the stored energy at once for the jump. Kangaroos use a similar trick to jump, so it should work right? 

![Spring GIF](https://8gadgetpack.net/milana/spring.gif)

As you can see above, it didn't work at all. Again, the feet didn't even leave the ground. And the body of the robot wasn't even attached yet. It was clear then that my intuition about this was completely off and I decided to simulate it before actually building it. Making a simulation takes some time, but it would quickly pay off. You can try out lots of different motors and robot designs much faster than building it for real. I briefly tried to build a physics engine from scratch for this, after all, how difficuilt can it be to simulate a little robot? Well, the math was too hard, so I gave that up pretty quickly. I also searched for what people in the robotics industry use, but I couldn't find anything that was lightweight and free. I had some experience with 3d physics engines at that point, but I was a bit reluctant to use those. I didn't really need a 3D simulation for this, the robot can be easily reduced to 2 dimensions if you just care about the jumping. And all the 3D math and collision detection would complicate things quite a bit. Luckily I found out about [Box2D](https://box2d.org/), a lightweight 2D physics engine. It's primarily made for games, but I figured it would be accurate enough for this. And it was surprisingly simple to use. All I needed to do was to calculate the torque output for a given motor and current and recreate the robot in that simulation environment. I also checked that the simulation was sufficiently accurate with [Stanford Doggo](https://github.com/Nate711/StanfordDoggoProject) (a robot which [jumps on for legs](https://www.youtube.com/watch?v=YeUpceVrUfg)). That's an Open Source robot with all motor specs published, so I could simulate that robot and check that the jump height in the simulation matches the one in the video. And it did actually match. I had to use a very small delta time step, it was much slower than realtime, but the accuracy seemed pretty good.

With the simulation environment running and validated, the rest was pretty easy. I evaluated a couple of motors, varied the gear ratio and leg length a bit and then compared the cost of the motors to the jump height. This is the simulation of the configuration I decided to build: 

![Jump Simulation GIF](https://8gadgetpack.net/milana/sim_only2.gif)

It uses motors similar to what Stanford Doggo uses: Two brushless motors with a gear ratio of 3 to 1, controlled by an [ODrive Motor Controller](https://odriverobotics.com/).

Actually fitting the ODrive inside of the robot body and attaching the motors to the legs required quite a bit of modelling and printing, but the first prototype with one motor looked pretty promising:

![BLDC Test Jump GIF](https://8gadgetpack.net/milana/bldc_proto_jump2.gif)

We have successfully lifted off! Not much, and I still had to hold it a bit for balance and catch it after the jump. But fixing that would be easy, right?

Well, once fully assembled, the very first recorded jump was actually successful:

![Jumping GIF](https://8gadgetpack.net/milana/bldc_jump2.gif)

But that turned out to be a fluke. This initial version was so unreliable at jumping, it would only successfully land maybe 1 out of 10 times. Improving that successrate would turn out to be very difficult. Here is a small compilation of some of the many many test jumps, that wern't quite successful:

![Jump Fails GIF](https://8gadgetpack.net/milana/jump_fails8.gif)

Can you see why those jumps failed and how to fix it? If so, you are smarter than I am. I needed more information to debug these issues, and a conventional debugger is of little help here.

What I ended up doing is to write some software that connected to the robot via WIFI and queried its internal state (which updates roughly at 60Hz). This sofware was also recording these jump attempts with a webcam (also captured with 60Hz) and then displayed all this information in a plotting window. This software also has special handling of ODrive data. ODrive updates at 8000Hz, much faster than the main loop on the robot. ODrive needs to be this fast because it needs to control the current in the 3 wires of the motor very quickly, but sadly communication between the ODrive and the main computer is so slow that it cannot be queried in realtime. In order to still get that information with full precision, I had to store that data on its internal (very limited) RAM during the jump and then retrieve it from there after the jump.

In the end, such a plot of a successful jump looks something like this:

![Jump with Plots GIF](https://8gadgetpack.net/milana/jump_plot3.gif)

You can see the sensor readings of all the motors visualized as 2d and 3d models at the top. The angular position of both jumping motors is also plotted out.

This software helped me a great deal to make the jumps more reliable. One trick that helped a lot is to align both legs during the jump in such a way that they will touch the ground simultaneously. The robot will always slightly rotate to the left or right, but by moving the legs this can be compensated. Of course, you need to detect in which way the robot rotates. The built in accelerometer won't help here, you are weightless in free fall. But if you query a gyroscope with around 400Hz and make no mistakes in the 3d rotation math, you get pretty accurate results.

Another trick is to explicitly separate the time during a jump into different states. You can see the separate states in the video above visualized as a blue stairstep. I won't go into detail what all the states mean, but an interesting state transition is between falling and landing. While falling, you want to apply a lot of force to the motors, so that they can reach the correct position that is required for landing. Once landing with the ground is detected, you need to adjust the force in such a way that results in a smooth landing. If you apply too much force, the wheel might bounce off the ground. If you use too little force, the lower leg will rotate until it hits the upper leg and not slow down the robot in time. (In the landing part, the plot above is a bit crude. Thats where the ODrive RAM ran out and only data at 60Hz is available)

Another thing that became apparant in these recorded videos was that the initial motors that control the wheels were too weak. I replaced those with brushless motors and added a second ODrive to control them to fix this. I also had trouble finding rubber wheels that have sufficient grip with the ground. I ended up making my own with a 3d printed mold, similar to what James Bruton did in [this video](https://youtu.be/eKZIJwJBjEs?t=540).

An interesting issue that sometimes arose was that the robot would jump and land perfectly, but then lose balance and move either forwards or backwards until it fell down. It was clear that this was a PID Controller issue, the hardware was working perfectly fine. But I was at a loss how to fix it. Changing the gains didn't work, and experimenting with hacks to fix this wasn't very productive. Each unsuccessful fall could break something. And fixing things takes a lot of time. I decided to simulate this part of the robot as well, just to figure out whats's wrong with the PID algorithm. I didn't want to use Box2D for this though. At this time, it became apparant that something must be wrong with that simulation. The actual jump heights didn't come close to the simulated ones. So I bit the bullet and learned the math for physics simulations. And it turned out to be much simpler than thought. What you use is the so called "Lagrangian Method". It's something physics students learn. But if you are just interested in the results, it's enough to define an Energy function for a given robot state, let some numeric system differentiate and simplify some formulas, and voila: You get a linear equation that you solve in runtime and you get an acceleration for each timestep. Sounds fancy, but the difficult part really was just finding software that does the differentiation for you, all you need to provide is the kinetic and potential energy for a given robot state (few lines of python code, and similar to the double pendulum). And the resulting simulation was not only fast, it was very accurate. I could directly compare it to my recorded jumps, and it matched very well. With these simulations, it was also easy to find out the problem: When the motor cannot realize the commands of the PID controller, the integral would grow very fast and cause this issue. Now that I knew what to look for, I found the search term for it: [Integral Windup](https://en.wikipedia.org/wiki/Integral_windup). And one of its well known solutions, Backcalculation, worked like a charm to fix this. I made a little demonstration here that shows Backcalculation in action (simulated, but very similar to reality):

![Backcalculation GIF](https://8gadgetpack.net/milana/backcalculation.gif)

This plots the commanded velocity (blue), the control velocity (orange) and the realized velocity (brown).
You can see that the initial graph looks very similar, with and without Backcalculation. But the inserted sudden rotation in the last third can only be handled with Backcalculation. So yeah, it took me some time to find the cause of this problem. And the solution is implemented in one line of code. But at least I found a proper solution.

On the hardware side, I didn't always find one of those: The encoders, the devices that measure the angle of the joints, need to be very accurate so that the motor controller can work. Unfortunately, the strong acceleration while jumping and landing caused the internal structure of these encoders to slip along the axis. I found a solution involving glue and burning plastic with a soldering iron, but it wasn't pretty. EM Noise also was a serious issue. Due to the heavy current the motors require while jumping and the close space due to the robots small size, I had lots of Heisenbugs and data corruption problems. I got them down pretty good by hacking some of the Linux kernel, the ODrive firmware and using insulated wiring, but I'm still not super happy with it.

In the end, I got the jumping to work pretty reliably. It doesn't jump as high as I initially simulated it or hoped, but I guess that's good enough.
![Jump Simulation and real Jump GIF](https://8gadgetpack.net/milana/sim5.webp)

I also implemented other features than jumping. It can balance while moving with one leg on a ramp. I added some arms with some Inverse Kinematics control. And I also added a way to communicate with the robot via voice and a local ChatGPT clone (see [here](https://www.youtube.com/watch?v=1e_AJBxF1MY)). But this post is getting long enough, so I just refer to the code and readmes in this repository and this [Youtube Video](https://www.youtube.com/watch?v=lOAjTAtRaGs).

All in all, I believe it's fair to say that this robot is very similar feature wise with Boston Dynamics Handle Robot. Although I admit that it doesn't look as fancy and isn't quite as elegant, but I'm still quite happy with it.


