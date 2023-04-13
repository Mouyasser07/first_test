import 'package:first_test/deplacement.dart';
import 'package:flutter/material.dart';
import 'package:scidart/numdart.dart';
import 'package:sensors_plus/sensors_plus.dart';
import 'package:syncfusion_flutter_charts/charts.dart';
import 'dart:math' as math ;
import 'dart:async';


class Deplacement extends StatefulWidget {
  @override
  State<StatefulWidget> createState() {
    // TODO: implement createState
    return _DeplacementState();
  }
}

class _DeplacementState extends State<Deplacement> {
  late AccelerometerEvent eventAcc;
  late GyroscopeEvent eventGyr;
  late StreamSubscription accStream;
  late StreamSubscription gyrStream;
  late Timer timer;
  List<KeyPoint> keyPoints=[];
  double threshold=0;
  double prevAccMean=0;
  double increase=0;
  bool sensorIsActivated=false;

  double rollAngleA=0;
  double previousRollAngleA=0;
  double rollAngleG=0;
  double previousRollAngleG=0;

  double pitchAngleA=0;
  double previousPitchAngleA=0;
  double pitchAngleG=0;
  double previousPitchAngleG=0;

  double eventThreshold=1;




  int listCounter=0;

  Calibration c=Calibration(0, 0, 0);
  double calibStateX=0;
  double calibStateY=0;
  double calibStateZ=0;
  late List<AccelerometerData> dataList=[];
  late List<AccelerometerData> accelDataList=[];
  late ChartSeriesController dataSerieController;
  List<StreamSubscription<dynamic>> _streamSubscriptions =
  <StreamSubscription<dynamic>>[];
  late GyroscopeData g=GyroscopeData(0,0,0);
  late AccelerometerData a=AccelerometerData(0, 0, 0, 0);

  int counterItem=0;
  int counterList=0;

  double pitch=0;
  double roll=0;

  //Observation Matrix
  Array2d h=Array2d([Array([1,0,0,0,0]),Array([0,1,0,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])]);
  Array2d h1=Array2d([Array([1,0,0,0,0,0,0,0]),Array([0,1,0,0,0,0,0,0]),Array([0,0,1,0,0,0,0,0]),Array([0,0,0,1,0,0,0,0]),
    Array([0,0,0,0,1,0,0,0]),Array([0,0,0,0,0,1,0,0]),Array([0,0,0,0,0,0,1,0]),Array([0,0,0,0,0,0,0,1])]);

  //Measurment Matrix
  Array2d u=Array2d([Array([0,0,1])]);
  //double u=0;
  Array2d y=Array2d.empty();
  Array2d y1=Array2d.empty();

  Array2d q=Array2d([Array([0]),Array([0]),Array([0])]);
  Array2d q_estimate=Array2d([Array([0]),Array([0]),Array([0]),Array([0]),Array([0])]);
  Array2d q_estimate1=Array2d([Array([0]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0])]);

  Array2d p=Array2d([Array([1,0,0,0,0]),Array([0,1,0,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])]);
  Array2d p1=Array2d([Array([1,0,0,0,0,0,0,0]),Array([0,1,0,0,0,0,0,0]),Array([0,0,1,0,0,0,0,0]),Array([0,0,0,1,0,0,0,0]),
    Array([0,0,0,0,1,0,0,0]),Array([0,0,0,0,0,1,0,0]),Array([0,0,0,0,0,0,1,0]),Array([0,0,0,0,0,0,0,1])]);

  Array2d ex=Array2d([Array([1,0,0,0,0]),Array([0,1,0,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])]);
  Array2d ex1=Array2d([Array([1,0,0,0,0,0,0,0]),Array([0,1,0,0,0,0,0,0]),Array([0,0,1,0,0,0,0,0]),Array([0,0,0,1,0,0,0,0]),
    Array([0,0,0,0,1,0,0,0]),Array([0,0,0,0,0,1,0,0]),Array([0,0,0,0,0,0,1,0]),Array([0,0,0,0,0,0,0,1])]);
  Array2d r=Array2d([Array([1,0,0,0,0]),Array([0,1,0,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])]);
  Array2d r1=Array2d([Array([1,0,0,0,0,0,0,0]),Array([0,1,0,0,0,0,0,0]),Array([0,0,1,0,0,0,0,0]),Array([0,0,0,1,0,0,0,0]),
    Array([0,0,0,0,1,0,0,0]),Array([0,0,0,0,0,1,0,0]),Array([0,0,0,0,0,0,1,0]),Array([0,0,0,0,0,0,0,1])]);
  Array2d k=Array2d.empty();
  Array2d k1=Array2d.empty();

  Array2d bResult=Array2d([Array([0]),Array([0]),Array([0]),Array([0]),Array([0])]);
  Array2d q_estimate_previous=Array2d([Array([0]),Array([0]),Array([0]),Array([0]),Array([0])]);

  //output variables
  Array2d zp=Array2d.empty();

  Array2d zv=Array2d.empty();

  Array2d za=Array2d.empty();

  Array2d zroll=Array2d.empty();

  Array2d zpitch=Array2d.empty();

  Array2d d_estimate_az=Array2d.empty();
  Array2d d_estimate_ax=Array2d.empty();
  Array2d d_estimate_ay=Array2d.empty();

  Array2d v_estimate_az=Array2d.empty();

  Array2d a_estimate_az=Array2d.empty();

  Array2d roll_estimate_az=Array2d.empty();

  Array2d pitch_estimate_az=Array2d.empty();

  double counterX = 0;
  double counterY = 0;
  double counterZ = 0;

  double u1Previous=0;
  double u2Previous=0;
  double u3Previous=0;

  double positionX=0;
  double positionY=0;
  double positionZ=0;
  double diffX=0;
  double diffY=0;
  double diffZ=0;
  double firstX=0;
  double firstY=0;
  double firstZ=0;
  double lastX=0;
  double lastY=0;
  double lastZ=0;


  double countF=0;
  double newAccelerationX=0;
  double newAccelerationY=0;
  double newAccelerationZ=0;

  double oldAccelerationX=0;
  double oldAccelerationY=0;
  double oldAccelerationZ=0;

  double newVelocityX=0;
  double newVelocityY=0;
  double newVelocityZ=0;

  double oldVelocityX=0;
  double oldVelocityY=0;
  double oldVelocityZ=0;

  double newPositionX=0;
  double newPositionY=0;
  double newPositionZ=0;

  double oldPositionX=0;
  double oldPositionY=0;
  double oldPositionZ=0;

  @override
  void initState(){
    //Timer.periodic(const Duration(seconds:1), updateData);
    //setAccelerometerData();
    //dataList=getAccelerometerData();
    //Timer.periodic(const Duration(seconds: 1), updateData);
    super.initState();
  }
  @override
  void dispose(){
    super.dispose();
    //pauseTimer();
    for (final subscription in _streamSubscriptions) {
      subscription.cancel();
    }
  }
  @override
  Widget build(BuildContext context) {
    return SafeArea(
        child: Scaffold(
            appBar: AppBar(
              title: Text("Deplacement"),
              backgroundColor: Colors.pink,
            ),
            body: Container(
                child:Column(
                    children:<Widget>[
                      Center(
                      child:Row(
                        children: <Widget>[
                          ElevatedButton(
                              onPressed: (){
                                /*if(!sensorIsActivated){
                                  startTimer();
                                  setState(() {
                                    sensorIsActivated=true;
                                  });
                                }
                                else{
                                  pauseTimer();
                                  setState(() {
                                    sensorIsActivated = false;
                                  });
                                }*/
                                setState(()  {
                                  dataList=[];
                                  setGyroscopeData();
                                  setAccelerometerData();
                                  //dataList= getDataList() ;
                                });

                              },
                              child: Text("Begin ")
                          ),
                          ElevatedButton(
                              onPressed: (){
                                setState(()  {
                                  for (final subscription in _streamSubscriptions) {
                                    subscription.cancel();
                                  }
                                  /*if(timer.isActive)
                                  pauseTimer();*/
                                });

                              },
                              child: Text("End ")
                          ),
                          ElevatedButton(
                              onPressed: (){
                                /*_streamSubscriptions.add(accelerometerEvents.listen((AccelerometerEvent event) {
                                  if(listCounter<300) {
                                    calibStateX=calibStateX+event.x;
                                    calibStateY=calibStateY+event.y;
                                    calibStateZ=calibStateZ+event.z;
                                    listCounter++;
                                  }
                                  else{

                                    calibStateX=calibStateX/300;
                                    calibStateY=calibStateY/300;
                                    calibStateZ=calibStateZ/300;
                                    print("calibStateX= ${math.sqrt(calibStateX*calibStateX+calibStateY*calibStateY+calibStateZ*calibStateZ)} ");
                                    Calibration c=Calibration(calibStateX, calibStateY, calibStateZ);
                                    print("calibration done");
                                    _streamSubscriptions.first.pause();
                                  }
                                }));*/
                                setState(()  {
                                  dataList=getAccelerometerData();
                                });

                              },
                              child: Text("Calibrate ")
                          ),
                        ],
                      ),
                      ),

            Expanded(
                child:
        SfCartesianChart(
            primaryXAxis: CategoryAxis(),
            series:<ChartSeries>[
              LineSeries<AccelerometerData,double>(
                /*onRendererCreated: (ChartSeriesController controller){
                      dataSerieController=controller;
                    },*/
                  dataSource: dataList,
                  color:const Color.fromRGBO(192,108,132,1),
                  xValueMapper: (AccelerometerData data,_) => data.time.toDouble(),
                  yValueMapper: (AccelerometerData data,_) => data.xAxis
              ),
              LineSeries<AccelerometerData,double>(
                /*onRendererCreated: (ChartSeriesController controller){
                      dataSerieController=controller;
                    },*/
                  dataSource: dataList,
                  color:const Color.fromRGBO(30, 54, 172, 1.0),
                  xValueMapper: (AccelerometerData data,_) => data.time.toDouble(),
                  yValueMapper: (AccelerometerData data,_) => data.yAxis
              ),
              LineSeries<AccelerometerData,double>(
                /*onRendererCreated: (ChartSeriesController controller){
                      dataSerieController=controller;
                    },*/
                  dataSource: dataList,
                  color:const Color.fromRGBO(56, 196, 15, 1.0),
                  xValueMapper: (AccelerometerData data,_) => data.time.toDouble(),
                  yValueMapper: (AccelerometerData data,_) => data.zAxis
              )
            ],
            /*primaryXAxis: NumericAxis(
                title:AxisTitle(
                    text: "Time unit"
                )
            ),
            primaryYAxis: NumericAxis(
                title: AxisTitle(
                    text: "Displacement on X"
                )
            ),*/
        )
            ),
                    ]
                )
            )

));
  }


  void updateData(Timer timer) {
    print("taw");
    if (dataList.length == 9) {
      dataList.removeAt(0);
      dataSerieController.updateDataSource(
          addedDataIndex: 9, removedDataIndex: 0);
    }
  }

  /*List<AccelerometerData> getAccelerometerData() {
    return dataList;
  }*/

  List<AccelerometerData> getAccelerometerData() {
    return dataList;
  }

  Future<void> setGyroscopeData() async {
    _streamSubscriptions.add(gyroscopeEvents.listen((GyroscopeEvent event) {
      //if(event.x!=0 || event.y!=0 || event.z!=0){
         g=GyroscopeData(event.x, event.y, event.z);
      //}
    }));
  }

  GyroscopeData getGyroscopeData(){
    return g;
  }

   Future<void> setAccelerometerData() async {
    //int listCounter=0;
    DateTime lastUpdate= DateTime.fromMicrosecondsSinceEpoch(0);
    /*_streamSubscriptions.add(accelerometerEvents.listen((AccelerometerEvent event) {
      a=AccelerometerData(event.x, event.y, event.z, 0);

    }));*/
    _streamSubscriptions.add(
        userAccelerometerEvents.listen((UserAccelerometerEvent event) {
          DateTime now = DateTime.now();
          double dt =(lastUpdate != DateTime.fromMicrosecondsSinceEpoch(0)) ? now
              .difference(lastUpdate)
              .inMilliseconds / 1000 : 0.0;
          lastUpdate = now;
         // print("on tilt acceleration =${math.sqrt(event.x*event.x+event.y*event.y+event.z*event.z)}");

          if(listCounter<300) {
            calibStateX=calibStateX+event.x;
            calibStateY=calibStateY+event.y;
            calibStateZ=calibStateZ+event.z;

                listCounter++;
          }
          else{

          c=Calibration(calibStateX, calibStateY, calibStateZ);
          //if(counterItem<10){

            kalmanFilter(event.x, event.y,event.z,dt,c);
            //linearAcceleration(event.x, event.y, event.z, dt, c);

            //position(event.x, event.y, event.z, dt, c);

            //counterItem++;
          //}
            //updateDataList(event.x,event.y,event.z,dt);
          }

        })   );

  }


  void startTimer() {
    // if the accelerometer subscription hasn't been created, go ahead and create it
    if (_streamSubscriptions.length==1) {
      _streamSubscriptions.add(accelerometerEvents.listen((AccelerometerEvent event) {
        setState(() {
          a=AccelerometerData(event.x, event.y, event.z, 0);
        });
      })
      );
      _streamSubscriptions.add(gyrStream = gyroscopeEvents.listen((GyroscopeEvent event) {
        setState(() {
          g=GyroscopeData(event.x, event.y, event.z);
        });
      })
      );
    } else {
      // it has already ben created so just resume it
      for (int i=0;i<_streamSubscriptions.length;i++) {
        _streamSubscriptions[i].resume();
      }
    }
      print("begin");
      timer = Timer.periodic(Duration(milliseconds: 20), (_) {
        // (20*500) = 1000 milliseconds, equals 1 seconds.
        // proccess the current event

        //findKeyPoints(getAccelerometerData(), getGyroscopeData());
      });
  }


  void pauseTimer() {
    // stop the timer and pause the accelerometer and gyro stream
    timer.cancel();
    for (final sub in _streamSubscriptions){
      sub.pause();
    }
  }


  void findKeyPoints(AccelerometerData accEvent,GyroscopeData gyrEvent){
    double stability=stabilityScore(accEvent,gyrEvent);
    print("stability = $stability");
    print("threshold = $threshold");
    if(stability<=threshold){
        KeyPoint keyPoint= new KeyPoint(stability,math.sqrt(accEvent.xAxis*accEvent.xAxis+accEvent.yAxis*accEvent.yAxis+accEvent.zAxis*accEvent.zAxis));
        keyPoints.add(keyPoint);
        print("point added : the new point's stability is ${keyPoint.stability} , gravity ${keyPoint.gravity}");
        threshold=stability;
        increase=stability*0.01;
      }
      else{
        threshold=threshold+increase;
      }
  }


  double stabilityScore(AccelerometerData accEvent,GyroscopeData gyrEvent){

    double actualAccMean=math.sqrt(accEvent.xAxis*accEvent.xAxis+accEvent.yAxis*accEvent.yAxis+accEvent.zAxis*accEvent.zAxis);//actual accelerometer mean
    double diffMean=actualAccMean-prevAccMean;//Difference in mean of accelerometer event
    double devMeas = actualAccMean;
    if(listCounter==300) {
      devMeas = actualAccMean - math.sqrt(
          calibStateX * calibStateX + calibStateY * calibStateY +
              calibStateZ * calibStateZ); //measurement deviation
    }
    double magGyr=math.sqrt(gyrEvent.xAxis*gyrEvent.xAxis+gyrEvent.yAxis*gyrEvent.yAxis+gyrEvent.zAxis*gyrEvent.zAxis);//magnitude of gyroscope event
    print("devMeas $devMeas");
    print("diffMean $devMeas");
    print("magGyr $magGyr");

    double stability=0.1*devMeas+0.45*diffMean+0.45*magGyr;//stability
    prevAccMean=actualAccMean;//previous accelerometer mean

    return stability.abs();

  }

  void validateKeyPoints(AccelerometerEvent accEvent,GyroscopeEvent gyrEvent, List<KeyPoint> keyPoints){
    double eventRange=findPeakLimits(accEvent,keyPoints);
  }
  double findPeakLimits(AccelerometerEvent accEvent,List<KeyPoint> keyPoints){
    return 0;
  }


  double getTilt(AccelerometerEvent accEvent1,AccelerometerEvent accEvent2){
      double cosAngle=(accEvent1.x*accEvent2.x+accEvent1.y*accEvent2.y+accEvent1.z*accEvent2.z)
          /(math.sqrt(accEvent1.x*accEvent1.x+accEvent1.y*accEvent1.y+accEvent1.z+accEvent1.z)
              +math.sqrt(accEvent2.x*accEvent2.x+accEvent2.y*accEvent2.y+accEvent2.z+accEvent2.z));
    return math.acos(cosAngle);
  }

  double getPitchG(GyroscopeEvent gyrEvent, double dt){
      previousPitchAngleG=pitchAngleG;
      pitchAngleG+=gyrEvent.x*dt;

    return pitchAngleG*(180/math.pi);
  }

  double getRollG(GyroscopeEvent gyrEvent, double dt){
    previousRollAngleG=rollAngleG;
    rollAngleG+=gyrEvent.z*dt;

    return rollAngleG*(180/math.pi);
  }

  double getPitchA(AccelerometerEvent accEvent){
    previousPitchAngleA=pitchAngleA;
    pitchAngleA=math.atan2(accEvent.x, accEvent.y);

    return pitchAngleA;
  }
  double getRollA(AccelerometerEvent accEvent){
    previousRollAngleA=rollAngleA;
    rollAngleA=math.atan2(-accEvent.z, math.sqrt(accEvent.y*accEvent.y+accEvent.x*accEvent.x));

    return rollAngleA;
  }


  void orientationChange(AccelerometerEvent accEvent ,GyroscopeEvent gyrEvent){
    double magnitude=math.sqrt(gyrEvent.x*gyrEvent.x+gyrEvent.y*gyrEvent.y+gyrEvent.z*gyrEvent.z);
    if(magnitude>eventThreshold){
      double p=(1-0.01)*getPitchG(gyrEvent,0.01)+0.01*getPitchA(accEvent);
      double r=(1-0.01)*getRollG(gyrEvent,0.01)+0.01*getRollA(accEvent);
      double previousP=(1-0.01)*previousPitchAngleG+0.01*previousPitchAngleA;
      double previousR=(1-0.01)*previousRollAngleG+0.01*previousRollAngleA;
      double diffPitch=(p-previousP).abs();
      double diffRoll=(r-previousR).abs();
      if(diffPitch+diffRoll>10){
        
      }
    }
  }


  void trackOrientation(){

  }
  void position(double x,double y,double z,double t, Calibration c){



    if(countF<64) {
      newAccelerationX = newAccelerationX + x;
      newAccelerationY = newAccelerationY + y;
      newAccelerationZ = newAccelerationZ + z;
      countF++;
    }
    else{

    newAccelerationX=newAccelerationX/64;
    newAccelerationY=newAccelerationY/64;
    newAccelerationZ=newAccelerationZ/64;

    newAccelerationX=newAccelerationX-c.calibrationX;
    newAccelerationY=newAccelerationY-c.calibrationY;
    newAccelerationZ=newAccelerationZ-c.calibrationZ;

    if(newAccelerationX>0.05&&newAccelerationX<-0.05){
      newAccelerationX=0;
    }
    if(newAccelerationY>0.05&&newAccelerationY<-0.05){
      newAccelerationY=0;
    }
    if(newAccelerationZ>0.05&&newAccelerationZ<-0.05){
      newAccelerationZ=0;
    }
    newVelocityX=oldVelocityX+oldAccelerationX+(newAccelerationX-oldAccelerationX)/2;
    newPositionX=oldPositionX+oldVelocityX+(newVelocityX-oldVelocityX)/2;

    newVelocityY=oldVelocityY+oldAccelerationY+(newAccelerationY-oldAccelerationY)/2;
    newPositionY=oldPositionY+oldVelocityY+(newVelocityY-oldVelocityY)/2;

    newVelocityZ=oldVelocityZ+oldAccelerationZ+(newAccelerationZ-oldAccelerationZ)/2;
    newPositionZ=oldPositionZ+oldVelocityZ+(newVelocityZ-oldVelocityZ)/2;

    oldAccelerationX=newAccelerationX;
    oldAccelerationY=newAccelerationY;
    oldAccelerationZ=newAccelerationZ;


    oldVelocityX=newVelocityX;
    oldVelocityY=newVelocityY;
    oldVelocityZ=newVelocityZ;

    oldPositionX=newPositionX;
    oldPositionY=newPositionY;
    oldPositionZ=newPositionZ;

    countF=0;
    dataList.add(AccelerometerData(oldAccelerationX, oldAccelerationY, oldAccelerationZ, counterList));
    counterList++;
    }

  }

  void linearAcceleration(double x1, double x2, double x3,double t,Calibration c){
    double gravityX = x1;
    double gravityY = x2;
    double gravityZ = x3;

    double angularVelocityX = getGyroscopeData().xAxis;
    double angularVelocityY = getGyroscopeData().yAxis;
    double angularVelocityZ = getGyroscopeData().zAxis;

    double pitch = atan2(gravityY, sqrt(gravityX * gravityX + gravityZ * gravityZ));
    double roll = atan2(-gravityX, gravityZ);
    print("pitch= $pitch");
    print("roll= $roll");

    double specificForceX = gravityX * -sin(pitch);
    double specificForceY = gravityY * sin(roll) * cos(pitch);
    double specificForceZ = gravityZ * cos(roll) * cos(pitch);

    double rotationMatrix0 = cos(roll) * cos(pitch);
    double rotationMatrix1 = -sin(roll) * cos(pitch);
    double rotationMatrix2 = sin(pitch);
    double rotationMatrix3 = cos(roll) * sin(pitch) * sin(angularVelocityZ) + sin(roll) * cos(angularVelocityZ);
    double rotationMatrix4 = cos(roll) * cos(angularVelocityZ) - sin(roll) * sin(pitch) * sin(angularVelocityZ);
    double rotationMatrix5 = -cos(pitch) * sin(angularVelocityZ);
    double rotationMatrix6 = cos(roll) * sin(pitch) * cos(angularVelocityZ) - sin(roll) * sin(angularVelocityZ);
    double rotationMatrix7 = sin(roll) * sin(pitch) * cos(angularVelocityZ) + cos(roll) * sin(angularVelocityZ);
    double rotationMatrix8 = cos(pitch) * cos(angularVelocityZ);

    double linearAccelerationX =
        rotationMatrix0 * specificForceX + rotationMatrix1 * specificForceY + rotationMatrix2 * specificForceZ;
    double linearAccelerationY =
        rotationMatrix3 * specificForceX + rotationMatrix4 * specificForceY + rotationMatrix5 * specificForceZ;
    double linearAccelerationZ =
        rotationMatrix6 * specificForceX + rotationMatrix7 * specificForceY + rotationMatrix8 * specificForceZ;

    dataList.add(AccelerometerData(linearAccelerationX,linearAccelerationY,linearAccelerationZ,counterList));
    counterList++;

  }


  void kalmanFilter(double x1,double x2 , double x3,double t,Calibration c){
    //setting variabels
    print("start");
    //event input on X,Y and Z
    double u1=x1;
    double u2=x2;
    double u3=x3;
    double g1=getGyroscopeData().xAxis;
    double g2=getGyroscopeData().yAxis;
    double g3=getGyroscopeData().zAxis;
    /*if(u1<0.05&&u1>(-0.05)){
      u1=0;
    }
    if(u2<0.05&&u2>(-0.05)){
      u2=0;
    }
    if(u3<0.05&&u3>(-0.05)){
      u3=0;
    }*/
    print("Acc x = $u1");
    print("Acc y= $u2");
    print("Acc z=$u3");
    //print("input before gy x: $g1, y: $g2, z: $g3");
    //double x=0;
    Array2d gyInput=Array2d([Array([g1]),Array([g2]),Array([g3])]);
    //Euler rates transformation matrix
    Array2d gyControlMatrix=Array2d([Array([1,math.sin(roll)*math.tan(pitch),math.cos(roll)*math.tan(pitch)]),
      Array([0,math.cos(roll),-math.sin(roll)])]);

    //input bias

     /*double u1_bias=u1-c.calibrationX/300;
     double u2_bias=u2-c.calibrationY/300;
     double u3_bias=u3-c.calibrationZ/300;

     u1=u1-u1_bias;
     u2=u2-u2_bias;
     u3=u3-u3_bias;*/

     Array2d uInput=Array2d([Array([u1]),Array([u2]),Array([u3]),Array([u1]),Array([u2]),Array([u3]),Array([0]),Array([0])]);
    //input without bias
   /* u1=u1-u1_bias;
    u2=u2-u2_bias;
    u3=u3-u3_bias;*/

    Array2d sumAcc=Array2d([Array([0]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0])]);
    //time delay
    double dt=0.01;
    //control vector
    Array2d u =Array2d([Array([u1]) ,Array([u2]),Array([u3]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0])]);
    //State Transition Matrix
    //Array2d a=Array2d([Array([1,dt,dt*dt/2,0,0]),Array([0,1,dt,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])]);
    Array2d a1=Array2d([Array([1,0,0,3*dt*dt/4,0,0,0,0]),Array([0,1,0,0,3*dt*dt/2,0,0,0]),Array([0,0,1,0,0,3*dt*dt/2,0,0]),Array([0,0,0,1,0,0,0,0]),Array([0,0,0,0,1,0,0,0]),
      Array([0,0,0,0,0,1,0,0]),Array([0,0,0,0,0,0,1,0]),Array([0,0,0,0,0,0,0,1])]);

    /*Array2d a2=Array2d([Array([1,0,0,0,0,0,dt*dt/2,0,0,0,0]),Array([0,1,0,0,0,0,0,dt*dt/2,0,0,0]),Array([0,0,1,0,0,0,0,0,dt*dt/2,0,0]),Array([0,0,0,1,0,0,dt,0,0,0,0]),
      Array([0,0,0,0,1,0,0,dt,0,0,0]),Array([0,0,0,0,0,1,0,0,dt,0,0]),Array([0,0,0,0,0,0,1,0,0,0,0]),Array([0,0,0,0,0,0,0,1,0,0,0]),Array([0,0,0,0,0,0,0,0,1,0,0]),
      Array([0,0,0,0,0,0,0,0,0,1,0]),Array([0,0,0,0,0,0,0,0,0,0,1])]);*/
    Array2d gy=Array2d([Array([0]),Array([0]),Array([0]),Array([pitch]),Array([roll])]);
    Array2d ac=Array2d([Array([0]),Array([0]),Array([0]),Array([math.atan2(u2, u3)]),Array([math.atan2(-u1, math.sqrt(u2*u2+u3*u3))])]);
    Array2d ac1=Array2d([Array([0]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0]),Array([math.atan2(u2, u3)]),Array([math.atan2(-u1, math.sqrt(u2*u2+u3*u3))])]);
    Array2d sum=Array2d.empty();


    /*if(u1<0){
      u1=-u1;
    }
    if(u2<0){
      u2=-u2;
    }
    if(u3<0){
      u3=-u3;
    }*/

    //double u= math.sqrt(u1*u1+u2*u2+u3*u3);

    /*if (u<math.sqrt(c.calibrationY*c.calibrationY+c.calibrationX*c.calibrationX+c.calibrationZ*c.calibrationZ)){
      u=0;
    }*/
    //abs of negative values

    /*double u3_bias=0;
    double u3_perfect=0;
    u3_bias=u3-u3_bias;*/
    //First Step (Prediction)
    //predict State
    array2dMultiplyToScalar(u, dt*dt/4);
    if(counterList<2) {
      sumAcc=addition(sumAcc,Array2d([Array([u1]),Array([u2]),Array([u3]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0])]));
    }
    sum=sumAcc;
    array2dMultiplyToScalar(sum, dt * dt);

    //array2dMultiplyToScalar(gy, dt);
    //array2dMultiplyToScalar(ac,0.02);


    /*print("gy input = $gyInput");
    print("gy control matrix $gyControlMatrix");*/
    //Array2d gy1=array2dMultiplyToScalar(gy,180/math.pi);



    //Array2d q_estimate_curr=addition((mult(a,q_estimate)),ac);

    Array2d q_estimate_curr1=addition(addition(addition((mult(a1,q_estimate1)),u),sum),ac1);
    //print("q_estimate_curr1 dx = ${q_estimate_curr1.first}");


    //predict next covariance
    //p=addition(mult(mult(a,p),matrixTranspose(a)),ex);
    p1=addition(mult(mult(a1,p1),matrixTranspose(a1)),ex1);
    //Second Step (Update)
    //kalman gain
    //k=mult(mult(p,matrixTranspose(h)),inverse(addition(mult(mult(h,p),matrixTranspose(h)),r)));
    k1=mult(mult(p1,matrixTranspose(h1)),inverse(addition(mult(mult(h1,p1),matrixTranspose(h1)),r1)));

    gy=mult(gyInput,gyControlMatrix);
    roll=roll+gy.elementAt(0).first*dt;
    pitch=pitch+gy.elementAt(1).first*dt;

    //Gyroscope Data
    print("pitch= $pitch");
    print("roll= $roll");

    //update the state estimate
    //y=Array2d([q_estimate_curr.first,q_estimate_curr.elementAt(1),Array([u]),Array([roll]),Array([pitch])]);
    y1=Array2d([q_estimate_curr1.first,q_estimate_curr1.elementAt(1),q_estimate_curr1.elementAt(2),
      Array([u1]),Array([u2]),Array([u3]),Array([roll]),Array([pitch])]);



    //q_estimate=addition(q_estimate_curr,mult(k,(y-mult(h,q_estimate_curr))));
    q_estimate1=addition(q_estimate_curr1,mult(k1,(y1-mult(h1,q_estimate_curr1))));
    //update covariance
    //p=mult((Array2d([Array([1,0,0,0,0]),Array([0,1,0,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])])-(mult(k,h))),p);
    p1=mult((Array2d([Array([1,0,0,0,0,0,0,0]),Array([0,1,0,0,0,0,0,0]),Array([0,0,1,0,0,0,0,0]),Array([0,0,0,1,0,0,0,0]),
      Array([0,0,0,0,1,0,0,0]),Array([0,0,0,0,0,1,0,0]),Array([0,0,0,0,0,0,1,0]),Array([0,0,0,0,0,0,0,1])])-(mult(k1,h1))),p1);


    /*d_estimate_az.add(q_estimate.first);
    v_estimate_az.add(q_estimate.elementAt(1));
    a_estimate_az.add(q_estimate.elementAt(2));
    roll_estimate_az.add(q_estimate.elementAt(3));
    pitch_estimate_az.add(q_estimate.last);*/

    d_estimate_ax.add(q_estimate1.first);
    d_estimate_ay.add(q_estimate1.elementAt(1));
    d_estimate_az.add(q_estimate1.elementAt(2));
    roll_estimate_az.add(q_estimate1.elementAt(6));
    pitch_estimate_az.add(q_estimate1.last);
    /*print("d_estimate_ax= ${d_estimate_ax.last}");
    print("d_estimate_ay= ${d_estimate_ay.last}");
    print("d_estimate_az= ${d_estimate_az.last}");*/


    counterList++;
    q_estimate_previous=q_estimate;
    double last=d_estimate_az.elementAt(d_estimate_az.row-1).last;

    /*if(u1<0&&u1Previous<0&&firstX==0){
      firstX=d_estimate_ax.elementAt(counterList-1).first;
    }
    if(u1>0 && u1Previous<0&&firstX!=0){
      lastX=d_estimate_ax.elementAt(counterList-1).first;
    }

    if(u1>0&&u1Previous>0&&firstX==0){
      firstX=d_estimate_ax.elementAt(counterList-1).first;
    }
    if(u1<0 && u1Previous>0&&firstX!=0){
      lastX=d_estimate_ax.elementAt(counterList-1).first;
    }*/

    if((u1<0 && u1Previous>0)||(u1>0 && u1Previous<0)){
      positionX=positionX+d_estimate_ax.last.first;
      /*firstX=0;
      lastX=0;*/
      //print("positionX= $positionX");
    }
    /*if(u2<0&&u2Previous<0&&firstY==0){
      firstY=d_estimate_ay.elementAt(counterList-1).first;
    }
    if(u2>0 && u2Previous<0&&firstY!=0){
      lastY=d_estimate_ay.elementAt(counterList-1).first;
    }

    if(u2>0&&u2Previous>0&&firstY==0){
      firstY=d_estimate_ay.elementAt(counterList-1).first;
    }
    if(u2<0 && u2Previous>0&&firstY!=0){
      lastY=d_estimate_ay.elementAt(counterList-1).first;
    }*/


    if((u2<0 && u2Previous>0)||(u2>0 && u2Previous<0)){
      positionY=positionY+d_estimate_ay.last.first;
      /*firstY=0;
      lastY=0;*/
    }

    //il y a qlq chose qui cloche
    /*if(u3<0&&u3Previous<0&&firstZ==0){
      firstZ=d_estimate_az.elementAt(counterList-1).first;
    }
    if(u3>0 && u3Previous<0&&firstZ!=0){
      lastZ=d_estimate_az.elementAt(counterList-1).first;
    }


    if(u3>0&&u3Previous>0&&firstZ==0){
      firstZ=d_estimate_az.elementAt(counterList-1).first;
    }
    if(u3<0 && u3Previous>0&&firstZ!=0){
      lastZ=d_estimate_az.elementAt(counterList-1).first;
    }*/

    if((u3<0 && u3Previous>0)||(u3>0 && u3Previous<0)){
      positionZ=positionZ+d_estimate_az.last.first;
      /*firstZ=0;
      lastZ=0;*/
    }
    //storing last acceleration variables
    u1Previous=u1;
    u2Previous=u2;
    u3Previous=u3;
    //number of element of dataList

    //dataList.add(AccelerometerData(positionX, positionY, positionZ, counterList));
    //print("roll and pitch = ${roll_estimate_az.last}    ${pitch_estimate_az.last}");
    dataList.add(AccelerometerData(
        d_estimate_ax.elementAt(counterList-1).first/**math.cos(roll_estimate_az.elementAt(counterList-1).first) * math.cos(pitch_estimate_az.elementAt(counterList-1).first)*/,
        d_estimate_ay.elementAt(counterList-1).first/**math.sin(roll_estimate_az.elementAt(counterList-1).first) * math.cos(pitch_estimate_az.elementAt(counterList-1).first)*/,
        d_estimate_az.elementAt(counterList-1).first/**math.sin(pitch_estimate_az.elementAt(counterList-1).first)*/, counterList));
    //dataList.add(AccelerometerData(d_estimate_az.elementAt(counterList-1).first, v_estimate_az.elementAt(counterList-1).first, a_estimate_az.elementAt(counterList-1).first, counterList));

  }

  Array2d inverse(Array2d a) {
    if (a.row != a.column) {
      return matrixPseudoInverse(a);
    } else {
      return matrixSolve(a, matrixIdentity(a.row, a.row));
    }
  }

  Array2d addition(Array2d a,Array2d b){
    if(a.column==b.column && a.row==b.row){
    for (int i=0;i<a.row;i++){
      for(int j=0;j<a.column;j++) {
        a[i][j]+=b[i][j];
      }
    }
    return a;
  }
    else{
      throw new Exception("can't do addition");
    }
  }
  Array2d mult(Array2d a,Array2d b){

    if(a.column==b.row){
      var c=Array2d.fixed(a.row, b.column);
      for(int i=0;i<a.row;i++){
        for(int j=0;j<b.column;j++){
          double sum=0;
          for(int k=0;k<a.column;k++){
            sum+=a[i][k]*b[k][j];
          }
          c[i][j]=sum;
        }
      }
      return c;
    }
    else if(a.row==b.column){
      return mult(b,a);
    }
    else{
      throw new Exception("can't multiply");
    }
  }

}

class Calibration{
  double calibrationX;
  double calibrationY;
  double calibrationZ;
  Calibration(this.calibrationX,this.calibrationY,this.calibrationZ);
}

class AccelerometerData{
  double xAxis;
  double yAxis;
  double zAxis;
  int time;
  AccelerometerData(this.xAxis,this.yAxis,this.zAxis,this.time);
}
class GyroscopeData{
  double xAxis;
  double yAxis;
  double zAxis;
  GyroscopeData(this.xAxis,this.yAxis,this.zAxis);
}
class KeyPoint{
  double stability;
  double gravity;
  KeyPoint(this.stability,this.gravity);
}




/*class PointPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    // TODO: implement paint
    //paint parameters
    final paint = Paint()
      ..strokeWidth = 5
      ..color = Colors.black
      ..style = PaintingStyle.stroke;
    //3 axis variabels
    double xAxisFrom = 0;
    double yAxisFrom = 0;
    double zAxisFrom = 0;
    double xAxisTo = 0;
    double yAxisTo = 0;
    double zAxisTo = 0;
    //list of points
    List<Point> points = <Point>[];
    //path variable
    final path = Path();
    //counter variable
    int c = 0;

    accelerometerEvents.listen((AccelerometerEvent event2) {
      xAxisTo = event2.x;
      print("x=$xAxisTo");
      yAxisTo = event2.y;
      print("y=$yAxisTo");
      zAxisTo = event2.z;
      print("z=$zAxisTo");
      Point p = Point(xAxisTo, yAxisTo);
      if (c == 0 || (p.x != points[0].x) || (p.y != points[0].y)) {
        points.add(p);
        print("Point added");
        c++;
        print("counter= $c");
      }
      if (points.length >= 2) {
        print("points[0]");
        print(points[0].x! * 100 / size.width);
        print(points[0].y! * 100 / size.height);
        print("points[1]");
        print(points[1].x! * 100 / size.width);
        print(points[1].y! * 100 / size.height);
        if (points.length == 2) {
          path.moveTo(points[0].x! * 100, points[0].y! * 100);
        }
        path.lineTo(points[1].x! * 100, points[1].y! * 100);
        canvas.drawPath(path, paint);

        points.removeAt(0);
        print("line exist");
      }
    });
  }



  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    // TODO: implement shouldRepaint
    return false;
  }
}

class Point {
  double? x;
  double? y;

  Point(double x, double y) {
    this.x = x;
    this.y = y;
  }
}

void updateDataList(double x,double y, double z, double dt){
    //velocity variabels
    double velocityX = 0.0;
    double velocityY = 0.0;
    double velocityZ = 0.0;
    //displacement variabels
    double displacementX = 0.0;
    double displacementY = 0.0;
    double displacementZ = 0.0;

    print("dt=$dt");

    //pitch and roll variabels
    double pitch;
    double roll;
    //direction
    String direction="";

    int counter=0;

    // event.x, event.y, event.z contain the accelerometer data
    double accelerationX = x;
    double accelerationY = y;
    double accelerationZ = z;

    //filtering
    while (counter < 64) {
      accelerationX = accelerationX + x;
      accelerationY = accelerationY + y;
      accelerationZ = accelerationZ + z;
      counter++;
    }
    accelerationX = accelerationX / 64;
    accelerationY = accelerationY / 64;
    accelerationZ = accelerationZ / 64;
    counter = 0;


    //calibration routine
    //Calibration calibration=calibrate();

    // real acceleration
    /*accelerationX = accelerationX - calibration.calibrationX;
    accelerationY = accelerationY - calibration.calibrationY;
    accelerationZ = accelerationZ - calibration.calibrationZ;*/
    //print("1 accelerationX= $accelerationX");

    //mechanical filtering
    /*if((accelerationX<=3 && accelerationX>=-3)||(accelerationY<=3||accelerationY>=-3)||(accelerationZ<=3||accelerationZ>=-3)){
        accelerationX=0;
        accelerationY=0;
        accelerationZ=0;
      }*/
    //pitch and roll
    roll = math.atan2(accelerationY, accelerationZ); //*180/math.pi
    pitch = math.atan2(-accelerationX, math.sqrt(
        accelerationY * accelerationY + accelerationZ * accelerationZ));

    //real acceleration affected by pitch and roll (orientation)

    //get the direction
    /*if(accelerationX>0){

      }*/

    double counterX = 0;
    double counterY = 0;
    double counterZ = 0;

    // integrate acceleration to obtain velocity
    velocityX += accelerationX * dt;
    velocityY += accelerationY * dt;
    velocityZ += accelerationZ * dt;

    //movement end check
    if (accelerationX == 0) {
      counterX++;
    }
    else
      (counterX = 0);
    if (accelerationY == 0) {
      counterY++;
    }
    else
      (counterY = 0);
    if (accelerationZ == 0) {
      counterZ++;
    }
    else {
      counterZ = 0;
    }
    if (counterX >= 25) {
      velocityX = 0;
    }
    if (counterY >= 25) {
      velocityY = 0;
    }
    if (counterZ >= 25) {
      velocityZ = 0;
    }
    //print("VelocityX= $velocityX");

    //print("dt=$dt");

    //print("accelerationX= $accelerationX accelerationY= $accelerationY accelerationZ= $accelerationZ");


    //print("velocityX= $velocityX velocityY= $velocityY velocityZ=$velocityZ");


    // integrate velocity to obtain displacement
    displacementX += velocityX * dt;
    displacementY += velocityY * dt;
    displacementZ += velocityZ * dt;
    //print("displacementX= $displacementX");

    //print("0 displacementX= $displacementX displacementY= $displacementY displacementZ=$displacementZ");
    //displacement over the orientation
    displacementX = math.cos(roll) * math.cos(pitch) * displacementX;
    displacementY = math.sin(roll) * math.cos(pitch) * displacementY;
    displacementZ = math.sin(pitch) * displacementZ;

    //print("1 displacementX= $displacementX displacementY= $displacementY displacementZ=$displacementZ");

      if (displacementX != 0) {
        dataList.add(AccelerometerData(
            displacementX, displacementY, displacementZ, counterItem++));
      }

    int l=dataList.length;
    print("dataList.length= $l");

  }*/*/
