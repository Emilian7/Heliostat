# Heliostat


The project aims to create a heliostat (a solar panel that follows the sun) and determine if it can generate more energy than a stationary solar panel, factoring in the energy required to follow the sun. 

The heliostat uses the INA219 sensor to monitor the charging current and the ESP HUZZAH32 to measure the battery's voltage. This data is transmitted to Adafruit AIO via Wi-Fi. To adjust the solar panel’s position and track the sun, the heliostat utilizes the Adafruit servo driver and SG90 servos. The hardware was both purchased and sourced from Pauliskolan's storage. The code was written in C++ using Arduino IDE with easy access to libraries and code sourced from previous attempts at creating a heliostat, which all can be found on Github. All the hardware used in the heliostat was soldered together to utilize I2C as communication. The static solar station was sourced from a previous project requiring minimal adjustments. 

The 3D model consists of components gathered from GitHub and custom parts designed in Fusion 360. The final design consists of nine parts, six bigger and three smaller, all made in Fusion 360 and 3D printed at Pauliskolan in Malmö. 

The heliostat and static readings were performed at the same time, next to each other, to ensure the same weather conditions for both. Readings were done during mixed, overcast and sunny weather to ensure a polarized result. Readings resulted in the heliostat exceeding the static in collected solar energy during all weather conditions, but it didn't always exceed the statics final battery percentage. During overcast days the heliostat collected about 11% more solar energy, but ended up with 6,2% less battery. During sunny days. During overcast days the heliostat collected about 18% more solar energy and ended up with 14,9% more battery. There were some setbacks and issues along the way, including hardware, software and printing, but these were all resolved. There were some uncertainties in the reading and hardware, leaving place for future improvements  if someone were to pick up and continue the project. 

The conclusion to whether or not a heliostat is a worthy investment is not a clear yes or no answer. If located in a mostly sunny location a heliostat is clearly a good investment. If located in a mostly cloudy location a static panel is most likely the way to go. 
