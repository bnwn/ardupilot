float kalmanFilter(float inData)
    {
    static float prevData=0;
    static float p=10, q=0.0001, r=0.05, kGain=0;

    //Kalman filter function start*******************************
    p = p+q;
    kGain = p/(p+r);

    inData = prevData+(kGain*(inData-prevData));

    p = (1-kGain)*p;

    prevData = inData;
    //Kalman filter function stop********************************

    return inData;
    }
