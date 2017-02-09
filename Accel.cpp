#include "Accel.h"
#include <iostream>
#include <cmath>
using namespace std;

//constructor
template<class T>
Accel<T>::Accel(T thresh) {
    sensX.initVar(thresh);
    sensY.initVar(thresh);
    sensZ.initVar(thresh);
    cout << "Inside Constructor of Accel Object" << endl;

}
/*
template<class T>
Accel<T>::Accel(T thresh):sensX():sensy():sensZ(){
    //Initialize the Sensor Object and set the thresh to the variable.
    sensX.initVar(thresh);
    sensY.initVar(thresh);
    sensZ.initVar(thresh);

}*/

template<class T>
void Accel<T>::caliAccel(T measX, T measY, T measZ){
    ///This is function is triggered while the ready variable is false
    calcX(measX);
    calcY(measY);
    calcZ(measZ);
    cout << "Inside caliAccel, the measurements are" << endl;


    if(xready && yready && zready){
        cout << "Inside the ready statement meaning all axis are ready" << endl;

        ////if all necessary data is ready for the axis, start setting/ calculating the necessary variables.
        detGravAxis();//Determine the gravity axis
        calcOffsetX();
        calcOffsetY();
        calcOffsetZ();
        setKalmanVars();
        ready = true;
    }
}

template<class T>
int Accel<T>::getGravAxis(){
    if(xGrav == true){
        return 1;
    }
    else if(yGrav == true){
        return 2;
    }
    else if(zGrav == true){
        return 3;
    }
}

template<class T>
void Accel<T>::calcX(T measX){
    cout << "Inside the calcX function" << endl;
    if(sensX.statReady == false){
        cout << "Inside sensX, meaning sensX.statReady is false" << endl;
        sensX.calcStat(measX);
    }
    else{
        cout << "sensX.statReady is true" << endl;
        xready = true;
        xVal = sensX.getMean();
        xVar = sensX.getVar();
    }
}

template<class T>
void Accel<T>::calcY(T measY){
    if(sensY.statReady == false){
        sensY.calcStat(measY);
    }
    else{
        yready = true;
        yVal = sensY.getMean();
        yVar = sensY.getVar();
    }
}

template<class T>
void Accel<T>::calcZ(T measZ){
    if(sensZ.statReady == false){
        sensZ.calcStat(measZ);
    }
    else{
        zready = true;
        zVal = sensZ.getMean();
        zVar = sensZ.getVar();
    }
}

template<class T>
void Accel<T>::detGravAxis(){
    //Device should be completely flat/motionless during calibration
    //Compare the three axis averages to see which one is the closest to 9.8
    T xComp = abs(xVal);
    T yComp = abs(yVal);
    T zComp = abs(zVal);

    //Only Grav Axis will be bigger than both.

    if((zComp > yComp) && (zComp > xComp)){
        zGrav = true;
    }

    else if ((yComp > zComp) && (yComp > xComp)){
        yGrav = true;
    }

    else if ((xComp > zComp) && (xComp > yComp)){
        xGrav = true;
    }
}

template<class T>
void Accel<T>::calcOffsetX(){
    T expected;
    if(xGrav){
        expected = -9.8;
    }
    else{
        expected = 0.0;
    }
    xOffset = xVal - expected;
}

template<class T>
void Accel<T>::calcOffsetY(){
    T expected;
    if(yGrav){
        expected = -9.8;
    }
    else{
        expected = 0.0;
    }
    yOffset = yVal - expected;
}

template<class T>
void Accel<T>::calcOffsetZ(){
    T expected;
    if(zGrav){
        expected = -9.8;
    }
    else{
        expected = 0.0;
    }
    zOffset = zVal - expected;
}

template<class T>
void Accel<T>::setKalmanVars(){
    //Angluar Kalman Variables
    if(xGrav){
        yVarAng=(atan2(yVar,9.8)*180/3.14); //Gives it to me in degress, CONSIDER INCREASING VALUE OF PI FOR IMPROVED ACCURACY
        zVarAng=(atan2(zVar,9.8)*180/3.14);
    }
    else if(yGrav){
        xVarAng=(atan2(xVar,9.8)*180/3.14); //Gives it to me in degress, CONSIDER INCREASING VALUE OF PI FOR IMPROVED ACCURACY
        zVarAng=(atan2(zVar,9.8)*180/3.14);
    }
    else if(zGrav){
        xVarAng=(atan2(xVar,9.8)*180/3.14); //Gives it to me in degress, CONSIDER INCREASING VALUE OF PI FOR IMPROVED ACCURACY
        yVarAng=(atan2(yVar,9.8)*180/3.14);
    }
/////////////////////FIGURE SOMETHING OUT FOR YAW!!! ie the GRAV AXIS. LATER, WILL DO THOUGH!

    //Linear Kalman Variable
    xVarLin = xVal;
    yVarLin = yVal;
    zVarLin = zVal;

}

template<class T>
T Accel<T>::getOffsetX(){
    return xOffset;
}

template<class T>
T Accel<T>::getOffsetY(){
    return yOffset;
}

template<class T>
T Accel<T>::getOffsetZ(){
    return zOffset;
}

template<class T>
T Accel<T>::getAngKalX(){
    return xVarAng;
}

template<class T>
T Accel<T>::getAngKalY(){
    return yVarAng;
}

template<class T>
T Accel<T>::getAngKalZ(){
    return zVarAng;
}

template<class T>
T Accel<T>::getLinKalX(){
    return xVarLin;
}

template<class T>
T Accel<T>::getLinKalY(){
    return yVarLin;
}

template<class T>
T Accel<T>::getLinKalZ(){
    return zVarLin;
}

template<class T>
T Accel<T>::getXMean(){
    float avg;
    avg = sensX.getMean();
    return avg;
}

template<class T>
T Accel<T>::getXVar(){
    float var;
    var = sensX.getVar();
    return var;
}

template<class T>
T Accel<T>::getYMean(){
    float avg;
    avg = sensY.getMean();
    return avg;
}

template<class T>
T Accel<T>::getYVar(){
    float var;
    var = sensY.getVar();
    return var;
}

template<class T>
T Accel<T>::getZMean(){
    float avg;
    avg = sensZ.getMean();
    return avg;
}

template<class T>
T Accel<T>::getZVar(){
    float var;
    var = sensZ.getVar();
    return var;
}
