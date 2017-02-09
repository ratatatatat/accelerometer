
#ifndef _Accel_h_
#define _Accel_h_

#include <math.h>
#include <Sensor.h>
#include "Sensor.cpp"


//Get the Average Readings
//Set the offsets for each axis
//Determine the gravity axis and set a boolean
//Prepare Kalman Variables.


template<class T>
class Accel{
	public:
                Accel(T thresh);//Constructor, Initialize and set threshold to Sensor Objects;
                int getGravAxis(); //Return 1,2,3 (x,y,z) depending on the axis.
                bool ready = false; //To tell if accelerometer is good to go is good to go.
                bool xready; //Bool to know if the x axis is good to go,have all necessary data from the sensors
                bool yready; //Bool to know if the y axis is good to go, have all the necessary data from the sensors
                bool zready; //Bool to know if the z axis is good to go, have all the necessary data from the sensors.
                void caliAccel(T measX,T measY, T measZ);
                T getXMean(); //Return the mean for the X axis
                T getXVar(); // Return the Var for the X axis
                T getYMean(); //Return the mean for the Y axis
                T getYVar(); // Return the variance for the Y axis
                T getZMean(); // Return the mean for the Z axis
                T getZVar(); // Return the variance for the z axis
                //Return Offsets
                T getOffsetX();
                T getOffsetY();
                T getOffsetZ();
                //Return Angular Kalman
                T getAngKalX();
                T getAngKalY();
                T getAngKalZ();
                // Return Linear Kalman
                T getLinKalX();
                T getLinKalY();
                T getLinKalZ();

	private:
                Sensor <float> sensX;//Sensor Object for X; Call constructor for sens object inside accel constructor
                Sensor <float> sensY;//Sensor Object for y;
                Sensor <float> sensZ;//Sensor Object for z;
                bool xGrav = false;//Boolean to know if the x axis is gravity
                bool yGrav = false;//Boolean to know if the y axis is gravity
                bool zGrav = false;//Boolean to know if the z axis is gravity
                T xVal; //Hold the avg value for the xaxis;
                T yVal; //Hold the avg value for the y axis
                T zVal; //Hold the avg value for the z axis.
                T xOffset; //Offset for x axis
                T yOffset; //Offset for y axis
                T zOffset; //Offset for z axis

                ///// Hold the Variance, set from the sensor object;
                T xVar;
                T yVar;
                T zVar;
                //////

                /////Kalman Variable for Angular motion
                T xVarAng; //Kalman Var for x axis
                T yVarAng; //Kalman Var for y axis
                T zVarAng; //Kalman Var for z axis
                /////

                /////Kamlman variable for linear motion
                T xVarLin;
                T yVarLin;
                T zVarLin;
                ///////


                void calcOffsetAxis();
                //////
                void calcX(T measX);//Loop that through Sensor functions, sets the xready=true, when it is ready.
                void calcY(T measY);//Loop that through Sensor functions, sets the yready=true, when it is ready.
                void calcZ(T measZ);//Loop that through Sensor functions, sets the xready=true, when it is ready.
                ////// After running these the avg and sd for the axis shoud be available

                ///// Once available set the avg axis values.
                void setXVal();
                void setYVal();
                void setZVal();

                ///// Determine the Gravity Axis, once average values are set

                void detGravAxis();//Check the average value see which one is 9.8, update gravity booleans.

                ////// After avg and and sd and gravity axis are set, calculate axis are avaiable, calculate the offsets
                void calcOffsetX(); //What it should be reading vs what it is actually reading update the offset variables.
                void calcOffsetY();
                void calcOffsetZ();
                /////

                /////Determine Kalman

                void setKalmanVars(); //Set all the kalman variables.



};
#endif
