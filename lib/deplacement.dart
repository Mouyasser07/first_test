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
                                setState(()  {
                                  dataList=[];
                                  setAccelerometerData();
                                  setGyroscopeData();
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
                                });

                              },
                              child: Text("End ")
                          ),
                          ElevatedButton(
                              onPressed: (){
                                setState(()  {
                                  dataList=getAccelerometerData();
                                });

                              },
                              child: Text("See data ")
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
          print("on tilt acceleration =${math.sqrt(event.x*event.x+event.y*event.y+event.z*event.z)}");

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
            //counterItem++;
          //}
            //updateDataList(event.x,event.y,event.z,dt);
          }

        })   );

  }

   Calibration calibrate(){
    //calibration state variabels
    double calibStateX=0;
    double calibStateY=0;
    double calibStateZ=0;
    for(int i=0;i<accelDataList.length;i++){
        calibStateX = calibStateX + accelDataList[i].xAxis;
        calibStateY = calibStateX + accelDataList[i].yAxis;
        calibStateZ = calibStateX + accelDataList[i].zAxis;
    }
      calibStateX = calibStateX / 300;
      calibStateY = calibStateY / 300;
      calibStateZ = calibStateZ / 300;
      print("calibration is done");
      return Calibration(calibStateX, calibStateY, calibStateZ);
  }

  void kalmanFilter(double x1,double x2 , double x3,double t,Calibration c){
    //setting variabels
    print("start");
    //event input on X,Y and Z
    double u1=x1;
    double u2=x2;
    double u3=x3;
    if(u1<0.05&&u1>(-0.05)){
      u1=0;
    }
    if(u2<0.05&&u2>(-0.05)){
      u2=0;
    }
    if(u3<0.05&&u3>(-0.05)){
      u3=0;
    }
    //print("input before acc x: $u1, y: $u2, z: $u3");
    //double x=0;
    Array2d gyInput=Array2d([Array([getGyroscopeData().xAxis]),Array([getGyroscopeData().yAxis]),Array([getGyroscopeData().zAxis])]);
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
    double dt=t;
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
    /*print("gy= $gy");
    roll=gy.first.first;
    pitch=gy.last.first;
    print("roll= $roll");
    print("pitch = $pitch ");*/
    //print("gy1= $gy1");

    //Array2d q_estimate_curr=addition((mult(a,q_estimate)),ac);
    //print("input acc x: $u1, y: $u2, z: $u3");

    Array2d q_estimate_curr1=addition(addition(addition((mult(a1,q_estimate1)),u),sum),ac1);
    //print("q_estimate_curr1 dx = ${q_estimate_curr1.first}");
    /*zp.add(q_estimate_curr1.first);
    zv.add(q_estimate_curr1.elementAt(1));
    za.add(q_estimate_curr1.elementAt(2));
    zroll.add(q_estimate_curr1.elementAt(3));
    zpitch.add(q_estimate_curr1.last);*/

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
    //update the state estimate
    //y=Array2d([q_estimate_curr.first,q_estimate_curr.elementAt(1),Array([u]),Array([roll]),Array([pitch])]);
    y1=Array2d([q_estimate_curr1.first,q_estimate_curr1.elementAt(1),q_estimate_curr1.elementAt(2),
      Array([u1]),Array([u2]),Array([u3]),Array([roll]),Array([pitch])]);
    //print("y1 dx= ${y1.first} ");
    //print("y1 ax= ${y1.elementAt(3)}");
    //print("y1 roll= ${y1.elementAt(6)} pitch=${y1.last} ");


    /*if(u!=uPrevious){
      q_estimate=q_estimate_previous;
      print("taw");
    }*/
    //q_estimate=addition(q_estimate_curr,mult(k,(y-mult(h,q_estimate_curr))));
    q_estimate1=addition(q_estimate_curr1,mult(k1,(y1-mult(h1,q_estimate_curr1))));
    //print("q_estimate1 dx =${q_estimate1.first}");
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
    if((u1<0 && u1Previous>0)||(u1>0 && u1Previous<0)){
      positionX=positionX+d_estimate_ax.elementAt(counterList-1).first;
      //print("positionX= $positionX");
    }
    if((u2<0 && u2Previous>0)||(u2>0 && u2Previous<0)){
      positionY=positionY+d_estimate_ay.elementAt(counterList-1).first;
    }
    if((u3<0 && u3Previous>0)||(u3>0 && u3Previous<0)){
      positionZ=positionZ+d_estimate_az.elementAt(counterList-1).first;
    }
    //storing last acceleration variables
    u1Previous=u1;
    u2Previous=u2;
    u3Previous=u3;
    //number of element of dataList

    //dataList.add(AccelerometerData(positionX, positionY, positionZ, counterList));
    //print("roll and pitch = ${roll_estimate_az.last}    ${pitch_estimate_az.last}");
    dataList.add(AccelerometerData(
        d_estimate_ax.elementAt(counterList-1).first*math.cos(roll_estimate_az.elementAt(counterList-1).first) * math.cos(pitch_estimate_az.elementAt(counterList-1).first),
        d_estimate_ay.elementAt(counterList-1).first*math.sin(roll_estimate_az.elementAt(counterList-1).first) * math.cos(pitch_estimate_az.elementAt(counterList-1).first),
        d_estimate_az.elementAt(counterList-1).first*math.sin(pitch_estimate_az.elementAt(counterList-1).first), counterList));
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
