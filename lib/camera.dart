import 'dart:async';
import 'dart:math';
import 'package:arcore_flutter_plugin/arcore_flutter_plugin.dart';
import 'package:flutter/material.dart';
import 'package:vector_math/vector_math_64.dart' as vector;
import 'dart:ui' as ui;
import 'dart:math' as math ;
import 'package:first_test/deplacement.dart';
import 'package:ar_flutter_plugin/ar_flutter_plugin.dart';
import 'package:scidart/numdart.dart';
import 'package:sensors_plus/sensors_plus.dart';

class Camera extends StatefulWidget {
  @override
  _RuntimeMaterialsState createState() => _RuntimeMaterialsState();
}

class _RuntimeMaterialsState extends State<Camera> {
  ArCoreController? arCoreController;
  ArCoreNode? sphereNode;
  ArCoreNode? gridNode;
  int numObject=0;

  ui.Image? _textImage;
  bool _isTextImageLoaded = false;

  double metallic = 0.0;
  double roughness = 0.4;
  double reflectance = 0.5;
  Color color = Colors.red;




  Calibration c=Calibration(0, 0, 0);
  double calibStateX=0;
  double calibStateY=0;
  double calibStateZ=0;
  late List<AccelerometerData> dataList=[];
  late List<AccelerometerData> accelDataList=[];
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
  void initState() {
    // TODO: implement initState
    super.initState();
    //_loadTextImage();
  }

  /*void _loadTextImage() async {
    final recorder = ui.PictureRecorder();
    final canvas = Canvas(recorder);
    final textSpan = TextSpan(
      text: 'Hello, AR!',
      style: TextStyle(fontSize: 40, color: Colors.white),
    );
    final textPainter = TextPainter(
      text: textSpan,
      textDirection: TextDirection.ltr,
    )..layout();
    textPainter.paint(canvas, Offset.zero);
    final picture = recorder.endRecording();
    final image = await picture.toImage(textPainter.width.toInt(), textPainter.height.toInt());
    setState(() {
      _textImage = image;
      _isTextImageLoaded = true;
    });
  }*/

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(
          title: const Text('Materials Runtime Change'),
          actions: <Widget>[
            IconButton(
              icon: Icon(Icons.update),
              onPressed: () {},
            )
          ],
        ),
        body: Column(
          children: <Widget>[
            /*SphereControl(
              initialColor: color,
              initialMetallicValue: metallic,
              initialRoughnessValue: roughness,
              initialReflectanceValue: reflectance,
              onColorChange: onColorChange,
              onMetallicChange: onMetallicChange,
              onRoughnessChange: onRoughnessChange,
              onReflectanceChange: onReflectanceChange,
            ),*/Container(
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
                  double m=0.001;
                  double x=0;
                  double y=0;
                  double z=-1;
                  for(int i =0;i<4;i++){
                    for (int j=0;j<4;j++){
                      _addHeatMap(x, z+(m/2), y, m);
                      m=m+0.01;
                      x+=0.21;
                    }
                    x=0;
                    y+=0.21;
                  }
                });

              },
              child: Text("Show heat map ")
          ),
      ElevatedButton(
          onPressed: (){
            setState(()  {
              dataList=getAccelerometerData();
              for(int i=0;i<dataList.length;i++) {
                if(i%50==0) {
                  _addSphere(dataList[i].xAxis,dataList[i].yAxis,dataList[i].zAxis);
                }
              }
              });

          },
          child: Text("See data ")
      ),
      ],
    ),
    ),

    /*Expanded(
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
                  yValueMapper: (AccelerometerData data,_) => data.xAxis
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
            ),*/
    ]
    )
    ),
            Expanded(
              child: ArCoreView(
                onArCoreViewCreated: _onArCoreViewCreated,
                enableTapRecognizer: true,
                enableUpdateListener: true,
              ),
            ),
          ],
        ),
      ),
    );
  }

  void _onArCoreViewCreated(ArCoreController controller) {
    arCoreController = controller;
    arCoreController?.onNodeTap = (name) => onTapHandler(name);
    //arCoreController?.onPlaneTap = _onPlaneTapHandler;
    arCoreController?.onPlaneDetected= _onPlaneDetected;
  }
  void onTapHandler(String name) {
    showDialog<void>(
      context: context,
      builder: (BuildContext context) =>
          AlertDialog(content: Text('onNodeTap on $name')),
    );
  }
  /*void _onPlaneTapHandler(List<ArCoreHitTestResult> hits) async {
    while (numObject<=3) {

      final material = ArCoreMaterial(
        color: Colors.red,
      );

     // }
      final sphere = ArCoreSphere(
      materials: [material],
      radius: 0.1,
    );

      final cube = ArCoreCube(
          size: vector.Vector3(0.2, 0.2, 0.2), materials: [material]);
      sphereNode = ArCoreNode(
        shape: cube,
        position: vector.Vector3(-0.005, -1,numObject.toDouble()*0.505),
      );
      arCoreController?.addArCoreNode(sphereNode!);
      numObject++;
    }
  }*/
  void _addHeatMap(double x,double y,double z ,double m){
    print('m= $m');
    var material =ArCoreMaterial(
      color:Color.fromRGBO(15, 72, 196, 0.8),
    );
    if(m>=0.068){
       material =ArCoreMaterial(
        color:Color.fromRGBO(75, 196, 15, 0.8),
      );
    }
    if(m>=0.102) {
       material =ArCoreMaterial(
        color:Color.fromRGBO(196, 181, 15, 0.8),
      );
    }
    if(m>=0.136) {
       material =ArCoreMaterial(
        color:Color.fromRGBO(196, 106, 15, 0.8),
      );
    }
    if(m>=0.17) {
       material =ArCoreMaterial(
        color:Color.fromRGBO(196, 15, 36, 0.8),
      );
    }

      final cube = ArCoreCube(
          size: vector.Vector3(0.2, m,0.2),
          materials: [material]
      );
      gridNode = ArCoreNode(
          shape:cube,
          position:vector.Vector3(x*10,y*10,z*10)
      );
      arCoreController?.addArCoreNode(gridNode!);

  }

  void _addSphere(double x, double y,double z) {

      final material = ArCoreMaterial(
        color: Colors.red,
      );
      final sphere = ArCoreSphere(
      materials: [material],
      radius: 0.05,
    );
     /* final cube = ArCoreCube(
          size: vector.Vector3(0.2, 0.2, 0.001), materials: [material]);*/
      sphereNode = ArCoreNode(
        shape: sphere,
        position: vector.Vector3(x, y, z),
      );

      arCoreController?.addArCoreNode(sphereNode!);
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


          if(listCounter<300) {
            calibStateX=calibStateX+event.x;
            calibStateY=calibStateY+event.y;
            calibStateZ=calibStateZ+event.z;

            listCounter++;
          }
          else{
            print("taw");
            c=Calibration(calibStateX, calibStateY, calibStateZ);
            //if(counterItem<10){

            kalmanFilter(event.x, event.y,event.z,dt,c);
            //counterItem++;
            //}
            //updateDataList(event.x,event.y,event.z,dt);
          }



        })   );

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
    //time delay
    double dt=t;
    //control vector
    Array2d b =Array2d([Array([dt*dt/2]) ,Array([dt*dt/2]),Array([dt*dt/2]),Array([1]),Array([1]),Array([1]),Array([0]),Array([0])]);
    //State Transition Matrix
    //Array2d a=Array2d([Array([1,dt,dt*dt/2,0,0]),Array([0,1,dt,0,0]),Array([0,0,1,0,0]),Array([0,0,0,1,0]),Array([0,0,0,0,1])]);
    /*Array2d a1=Array2d([Array([1,0,0,dt*dt/2,0,0,0,0]),Array([0,1,0,0,dt*dt/2,0,0,0]),Array([0,0,1,0,0,dt*dt/2,0,0]),Array([0,0,0,1,0,0,0,0]),Array([0,0,0,0,1,0,0,0]),
      Array([0,0,0,0,0,1,0,0]),Array([0,0,0,0,0,0,1,0]),Array([0,0,0,0,0,0,0,1])]);*/
    Array2d sumAcc=Array2d([Array([0]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0])]);
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
    //array2dMultiplyToScalar(b, u);
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
    array2dMultiplyToScalar(u, dt*dt/4);
    if(counterList<2) {
      sumAcc=addition(sumAcc,Array2d([Array([u1]),Array([u2]),Array([u3]),Array([0]),Array([0]),Array([0]),Array([0]),Array([0])]));
    }
    sum=sumAcc;
    array2dMultiplyToScalar(sum, dt * dt);

    //Array2d q_estimate_curr=addition((mult(a,q_estimate)),ac);
    //print("input acc x: $u1, y: $u2, z: $u3");

    Array2d q_estimate_curr1=addition((mult(a1,q_estimate1)),ac1);
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



    q_estimate_previous=q_estimate;
    double last=d_estimate_az.elementAt(d_estimate_az.row-1).last;
    if((u1<0 && u1Previous>0)||(u1>0 && u1Previous<0)){
      positionX=positionX+d_estimate_ax.elementAt(counterList).first;
      //print("positionX= $positionX");
    }
    if((u2<0 && u2Previous>0)||(u2>0 && u2Previous<0)){
      positionY=positionY+d_estimate_ay.elementAt(counterList).first;
    }
    if((u3<0 && u3Previous>0)||(u3>0 && u3Previous<0)){
      positionZ=positionZ+d_estimate_az.elementAt(counterList).first;
    }
    //storing last acceleration variables
    u1Previous=u1;
    u2Previous=u2;
    u3Previous=u3;
    //number of element of dataList
    counterList++;
    dataList.add(AccelerometerData(positionX, positionY, positionZ, counterList));
    //print("roll and pitch = ${roll_estimate_az.last}    ${pitch_estimate_az.last}");
    /*dataList.add(AccelerometerData(
        d_estimate_ax.elementAt(counterList-1).first/**math.cos(roll_estimate_az.elementAt(counterList-1).first) * math.cos(pitch_estimate_az.elementAt(counterList-1).first)*/,
        d_estimate_ay.elementAt(counterList-1).first/**math.sin(roll_estimate_az.elementAt(counterList-1).first) * math.cos(pitch_estimate_az.elementAt(counterList-1).first)*/,
        d_estimate_az.elementAt(counterList-1).first/**math.sin(pitch_estimate_az.elementAt(counterList-1).first)*/, counterList));*/
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





  onColorChange(Color newColor) {
    if (newColor != this.color) {
      this.color = newColor;
      updateMaterials();
    }
  }

  onMetallicChange(double newMetallic) {
    if (newMetallic != this.metallic) {
      metallic = newMetallic;
      updateMaterials();
    }
  }

  onRoughnessChange(double newRoughness) {
    if (newRoughness != this.roughness) {
      this.roughness = newRoughness;
      updateMaterials();
    }
  }

  onReflectanceChange(double newReflectance) {
    if (newReflectance != this.reflectance) {
      this.reflectance = newReflectance;
      updateMaterials();
    }
  }

  updateMaterials() {
    debugPrint("updateMaterials");
    if (gridNode == null) {
      return;
    }
    debugPrint("updateMaterials sphere node not null");
    final material = ArCoreMaterial(
      color: color,
      metallic: metallic,
      roughness: roughness,
      reflectance: reflectance,
    );
    gridNode?.shape?.materials.value = [material];
  }

  @override
  void dispose() {
    arCoreController?.dispose();
    for (final subscription in _streamSubscriptions) {
      subscription.cancel();
    }
    super.dispose();
  }

  void _onPlaneDetected(ArCorePlane plane) {
  print("plane detected");
  }
}

class SphereControl extends StatefulWidget {
  final double? initialRoughnessValue;
  final double? initialReflectanceValue;
  final double? initialMetallicValue;
  final Color? initialColor;
  final ValueChanged<Color>? onColorChange;
  final ValueChanged<double>? onMetallicChange;
  final ValueChanged<double>? onRoughnessChange;
  final ValueChanged<double>? onReflectanceChange;

  const SphereControl(
      {Key? key,
        this.initialRoughnessValue,
        this.initialReflectanceValue,
        this.initialMetallicValue,
        this.initialColor,
        this.onColorChange,
        this.onMetallicChange,
        this.onRoughnessChange,
        this.onReflectanceChange})
      : super(key: key);

  @override
  _SphereControlState createState() => _SphereControlState();
}

class _SphereControlState extends State<SphereControl> {
  late double metallicValue;
  late double roughnessValue;
  late double reflectanceValue;
  Color? color;

  @override
  void initState() {
    roughnessValue = widget.initialRoughnessValue ?? 0.0;
    reflectanceValue = widget.initialReflectanceValue ?? 0.0;
    metallicValue = widget.initialRoughnessValue ?? 0.0;
    color = widget.initialColor;
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: <Widget>[
          Row(
            children: <Widget>[
              ElevatedButton(
                child: Text("Random Color"),
                onPressed: () {
                  final newColor = Colors.accents[Random().nextInt(14)];
                  widget.onColorChange?.call(newColor);
                  setState(() {
                    color = newColor;
                  });
                },
              ),
              Padding(
                padding: const EdgeInsets.only(left: 20.0),
                child: CircleAvatar(
                  radius: 20.0,
                  backgroundColor: color,
                ),
              ),
            ],
          ),
          Row(
            children: <Widget>[
              Text("Metallic"),
              Checkbox(
                value: metallicValue == 1.0,
                onChanged: (value) {
                  metallicValue = (value ?? false) ? 1.0 : 0.0;
                  widget.onMetallicChange?.call(metallicValue);
                  setState(() {});
                },
              )
            ],
          ),
          Row(
            children: <Widget>[
              Text("Roughness"),
              Expanded(
                child: Slider(
                  value: roughnessValue,
                  divisions: 10,
                  onChangeEnd: (value) {
                    roughnessValue = value;
                    widget.onRoughnessChange?.call(roughnessValue);
                  },
                  onChanged: (double value) {
                    setState(() {
                      roughnessValue = value;
                    });
                  },
                ),
              ),
            ],
          ),
          Row(
            children: <Widget>[
              Text("Reflectance"),
              Expanded(
                child: Slider(
                  value: reflectanceValue,
                  divisions: 10,
                  onChangeEnd: (value) {
                    reflectanceValue = value;
                    widget.onReflectanceChange?.call(reflectanceValue);
                  },
                  onChanged: (double value) {
                    setState(() {
                      reflectanceValue = value;
                    });
                  },
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }
}