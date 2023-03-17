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
  late List<AccelerometerData> dataList=[];
  late List<AccelerometerData> accelDataList=[];
  late ChartSeriesController dataSerieController;
  List<StreamSubscription<dynamic>> _streamSubscriptions =
  <StreamSubscription<dynamic>>[];
  late GyroscopeData g;

  int counterItem=0;

  //Observation Matrix
  Array2d h=Array2d([Array([1,0,0,0,0]),Array([0,1,0,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])]);
  //Measurment Matrix
  Array2d u=Array2d([Array([0,0,1])]);
  //double u=0;
  Array2d y=Array2d.empty();
  Array2d q=Array2d([Array([0]),Array([0]),Array([0])]);
  Array2d q_estimate=Array2d([Array([0]),Array([0]),Array([0]),Array([0]),Array([0])]);
  Array2d p=Array2d([Array([1,0,0,0,0]),Array([0,1,0,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])]);
  Array2d ex=Array2d([Array([1,0,0,0,0]),Array([0,1,0,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])]);
  Array2d r=Array2d([Array([1,0,0,0,0]),Array([0,1,0,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])]);
  Array2d k=Array2d.empty();
  //output variables
  Array2d zp=Array2d.empty();

  Array2d zv=Array2d.empty();

  Array2d za=Array2d.empty();

  Array2d zroll=Array2d.empty();

  Array2d zpitch=Array2d.empty();

  Array2d d_estimate_az=Array2d.empty();

  Array2d v_estimate_az=Array2d.empty();

  Array2d a_estimate_az=Array2d.empty();

  Array2d roll_estimate_az=Array2d.empty();

  Array2d pitch_estimate_az=Array2d.empty();



  @override
  void initState(){
    setGyroscopeData();
    print(getGyroscopeData());
    //setAccelerometerData();
    //dataList=getAccelerometerData();
    //Timer.periodic(const Duration(seconds: 1), updateData);
    super.initState();
  }
  @override
  void dispose(){
    super.dispose();
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
                      Row(
                        children: <Widget>[
                          ElevatedButton(
                              onPressed: (){
                                setState(()  {
                                  //dataList= getDataList() ;
                                });

                              },
                              child: Text("Begin ")
                          ),
                          ElevatedButton(
                              onPressed: (){
                                setState(()  {
                                  setAccelerometerData();
                                  setGyroscopeData();
                                });

                              },
                              child: Text("End ")
                          ),
                          ElevatedButton(
                              onPressed: (){
                                setState(()  {

                                });

                              },
                              child: Text("See data ")
                          ),
                        ],
                      ),





            Expanded(
                child:
        SfCartesianChart(
            series:<ChartSeries>[
              LineSeries<AccelerometerData,double>(
                /*onRendererCreated: (ChartSeriesController controller){
                      dataSerieController=controller;
                    },*/
                  dataSource: dataList,
                  color:const Color.fromRGBO(192,108,132,1),
                  xValueMapper: (AccelerometerData data,_) => data.time.toDouble(),
                  yValueMapper: (AccelerometerData data,_) => data.yAxis
              )
            ],
            primaryXAxis: NumericAxis(
                title:AxisTitle(
                    text: "Time unit"
                )
            ),
            primaryYAxis: NumericAxis(
                title: AxisTitle(
                    text: "Displacement on X"
                )
            ),
        )
            ),
                    ]
                )
            )

));
  }

  /*void updateData(Timer timer) {
    dataList.removeAt(0);
    print("first element removed");
    setAccelerometerData();
    int length = dataList.length;
    if (dataList.length == 9) {
      dataSerieController.updateDataSource(
          addedDataIndex: 9, removedDataIndex: 0);
    }
  }*/


  Future<List<AccelerometerData>> getDataList() async {
      await setAccelerometerData();
      int ll=dataList.length;
      print("dataList length=$ll");
      return dataList;
  }
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
    int listCounter=0;
    DateTime lastUpdate= DateTime.fromMicrosecondsSinceEpoch(0);


    _streamSubscriptions.add(
        userAccelerometerEvents.listen((UserAccelerometerEvent event) {
          DateTime now = DateTime.now();
          double dt =(lastUpdate != DateTime.fromMicrosecondsSinceEpoch(0)) ? now
              .difference(lastUpdate)
              .inMilliseconds / 1000 : 0.0;
          lastUpdate = now;
          /*if(listCounter<1000) {
            /*accelDataList.add(
                AccelerometerData(event.x, event.y, event.z, dt.toInt()));*/
            Calibration c=calibrate();
                listCounter++;
          }*/
          //else{

          if(counterItem<10){
            kalmanFilter(event.x, event.y,event.z,dt);
            counterItem++;
          }
            //updateDataList(event.x,event.y,event.z,dt);
          //}


          if(dataList.length>9){
            _streamSubscriptions.first.cancel();
            _streamSubscriptions.removeAt(0);
          }
        })   );

  }

   /*Calibration calibrate(){
    //calibration state variabels
    double calibStateX=0;
    double calibStateY=0;
    double calibStateZ=0;
    for(int i=0;i<accelDataList.length;i++){
        calibStateX = calibStateX + accelDataList[i].xAxis;
        calibStateY = calibStateX + accelDataList[i].yAxis;
        calibStateZ = calibStateX + accelDataList[i].zAxis;
    }
      calibStateX = calibStateX / 1000;
      calibStateY = calibStateY / 1000;
      calibStateZ = calibStateZ / 1000;
      print("calibration is done");
      return Calibration(calibStateX, calibStateY, calibStateZ);
  }*/

  void kalmanFilter(double x1,double x2 , double x3,double t){
    //event output on X,Y and Z
    double u1=x1;
    double u2=x2;
    double u3=x3;
    //double x=0;
    double pitch=0;
    double roll=0;

    print("setting variabels");
    //time delay
    double dt=t;
    //control vector
    Array2d b =Array2d([Array([dt*dt/2]) ,Array([dt]),Array([1]),Array([0]),Array([0])]);
    //State Transition Matrix
    Array2d a=Array2d([Array([1,dt,dt*dt/2,0,0]),Array([0,1,dt,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])]);
    Array2d gy=Array2d([Array([0]),Array([0]),Array([0]),Array([pitch]),Array([roll])]);
    Array2d ac=Array2d([Array([0]),Array([0]),Array([0]),Array([math.atan2(u2, u3)]),Array([math.atan2(-u1, math.sqrt(u2*u2+u3*u3))])]);
    if(u1<0){
      u1=-u1;
    }
    if(u2<0){
      u2=-u2;
    }
    if(u3<0){
      u3=-u3;
    }
    double u= math.sqrt(u1*u1+u2*u2+u3*u3)-9.80;
    print("abs of negatif values");

    /*double u3_bias=0;
    double u3_perfect=0;
    u3_bias=u3-u3_bias;*/
    //First Step (Prediction)
    //predict State
    print("prediction of state vector");

    array2dMultiplyToScalar(b, u);
    array2dMultiplyToScalar(gy, dt*0.98);
    array2dMultiplyToScalar(ac, 0.02);
    Array2d q_estimate_curr=addition(addition(addition(mult(a,q_estimate),b),gy),ac);//addition(mult(a,q_estimate),mult(b,u));
    print("predction is $q_estimate_curr");
    zp.add(q_estimate_curr.first);
    zv.add(q_estimate_curr.elementAt(1));
    za.add(q_estimate_curr.elementAt(2));
    zroll.add(q_estimate_curr.elementAt(3));
    zpitch.add(q_estimate_curr.last);

    //predict next covariance
    print("prediction of covaraince");
    p=addition(mult(mult(a,p),matrixTranspose(a)),ex);
    print("p = $p");
    //Second Step (Update)
    //kalman gain

    k=mult(mult(p,matrixTranspose(h)),inverse(addition(mult(mult(h,p),matrixTranspose(h)),r)));
    print("kalman gain $k");
    //update the state estimate
    y=Array2d([q_estimate_curr.first,q_estimate_curr.elementAt(1),Array([u]),q_estimate_curr.elementAt(3),q_estimate_curr.last]);
    q_estimate=addition(q_estimate_curr,mult(k,(y-mult(h,q_estimate_curr))));
    print("q_estimate= $q_estimate");
    //update covariance
    p=mult((Array2d([Array([1,0,0,0,0]),Array([0,1,0,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])])-(mult(k,h))),p);
    print("updated covariance = $p");
    d_estimate_az.add(q_estimate.first);
    v_estimate_az.add(q_estimate.elementAt(1));
    a_estimate_az.add(q_estimate.elementAt(2));
    roll_estimate_az.add(q_estimate.elementAt(3));
    pitch_estimate_az.add(q_estimate.last);

    print("Estimated acceleration= $a_estimate_az");
    print("Real acceleration = $u");

    print("Estimated roll= $roll_estimate_az");
    double rollResult=roll*dt;
    print("Real roll = $rollResult");
    print("Estimated pitch= $pitch_estimate_az");
    double pitchResult=pitch*dt;
    print("Real acceleration = $pitchResult");

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
}*/
