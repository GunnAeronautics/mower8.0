#include "headers/utils/RollingAverage.h"



RollingAverage::RollingAverage(int rollLen){
    rollingLen = rollLen;//default
    dataIndex = 0;
    average = 0.0;
    rawData = new float[rollingLen]();
  }

void RollingAverage::newData(float data){
    //what the average should divide by
    divideBy ++;
    if (divideBy > rollingLen){
      divideBy = rollingLen;
    }
    
    average = average + data;
    average = average - rawData[dataIndex];
    rawData[dataIndex] = data;
    dataIndex ++;
    dataIndex %= rollingLen;

    
  }

float RollingAverage::getData(){
    if (divideBy == 0){//prevent divide by 0
      return 0;
    }
    return average / divideBy;
  }
