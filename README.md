# Auto-PID-Tuning-RNN
### Autotuning of PID using Deep learning and RNN (Recurrent Neural Networks) in Autonomous carts
Due to physical parameters, the PID controller of the autonomous cart doesn't converge to a particular tuning of PID.
Hence, it requires real time tuning accroding to the surface on which it runs. 

The given code implements 2 networks(online tuning and offline tuning) in arduino MEGA(.ino), in order to initialise the online network with most appropriate values of tuning before it is made to run inside the factories or warehouses


