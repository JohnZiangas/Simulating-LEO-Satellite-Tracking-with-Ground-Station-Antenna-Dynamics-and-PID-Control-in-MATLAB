# Simulating-LEO-Satellite-Tracking-with-Ground-Station-Antenna-Dynamics-and-PID-Control-in-MATLAB.

This project was submitted as part of the semester project for the course **"Advanced Control Systems" (ECE_ELE760)** in the Department of Electrical and Computer Engineering at the University of the Peloponnese. The project explores fundamental concepts in orbital mechanics, antenna dynamics, and PID control, combining these elements to simulate a Low Earth Orbit (LEO) satellite. It is designed for beginners interested in entering the space industry, particularly in the field of satellite communications.

The repository includes the main MATLAB code, which simulates the motion of a LEO satellite for 25 minutes as it passes above a ground station. Additionally, the ***"PID Controllers"*** folder contains the MATLAB codes for modeling a simple PID controller to track the satellite's azimuth and elevation. The files ***"Azimuth_Values.mat"*** and ***"Elevation_Values.mat"*** provide the azimuth and elevation data of the satellite when it is within the antenna's local horizon. These values are manually extracted from the ***"LEO_Satellite_Orbit_Simulation.mlx"*** file and can be used in conjunction with the ***"Azimuth_Calc.m"*** and ***"Elevation_Calc.m"*** files.

I highly recommend exploring the following material if you wish to deepen your understanding of satellite orbits and communication systems.

1) https://orbitalmechanics.info/
2) https://youtube.com/playlist?list=PLbPW06RB5w6m_xizacLr6dMXP58IHq23D&si=4i3OxbtAztL6RIHD

*To run the code, ensure you have downloaded and installed the **Satellite Communications Toolbox**, **Aerospace Communications Toolbox** and **Control Systems Toolbox** in MATLAB.*
