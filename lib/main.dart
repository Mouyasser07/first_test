import 'dart:async';

import 'package:first_test/courbe.dart';

import 'camera.dart';
import 'package:flutter/material.dart';
import 'package:sensors_plus/sensors_plus.dart';
import 'deplacement.dart';

void main() {
  runApp(App());
}

class App extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
        debugShowCheckedModeBanner: false,
        title: "Sensors Detector",
        home: Home());
  }
}

class Home extends StatefulWidget {
  @override
  State<Home> createState() => _HomeState();
}

class _HomeState extends State<Home> {
  List<StreamSubscription<dynamic>> _streamSubscriptions =
  <StreamSubscription<dynamic>>[];
  String? sensors;
  double xAxis = 0;
  double yAxis = 0;
  double zAxis = 0;

  void getxAxis() {}

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          backgroundColor: Colors.pink,
          title: Text("Sensors Detector"),
        ),
        body: Material(
            color: Colors.white,
            child: Center(
                child: Container(
              alignment: Alignment.topLeft,
              //width: 200.0,
              //height: 200.0,
              padding: EdgeInsets.only(left: 10.0, top: 10.0),
              child: Column(
                children: <Widget>[
                  Expanded(
                    child: RadioListTile(
                      title: Text(
                        "Magnetometer",
                        textDirection: TextDirection.ltr,
                        style: TextStyle(
                          color: Colors.black,
                          fontSize: 20.0,
                        ),
                      ),
                      value: "Magnetometer",
                      groupValue: sensors,
                      toggleable: true,
                      onChanged: (value) {
                        sensors=value;
                        if(_streamSubscriptions.length>0){
                          _streamSubscriptions.first.cancel();
                          _streamSubscriptions.removeAt(0);
                        }
                        _streamSubscriptions.add(
                        magnetometerEvents.listen((MagnetometerEvent event) {
                          setState(() {
                              xAxis = event.x;
                              yAxis = event.y;
                              zAxis = event.z;
                          });
                        })
                        );
                      },
                    ),
                  ),
                  Expanded(
                    child: RadioListTile(
                      title: Text(
                        "Accelerometer",
                        textDirection: TextDirection.ltr,
                        style: TextStyle(
                          color: Colors.black,
                          fontSize: 20.0,
                        ),
                      ),
                      value: "Accelerometer",
                      groupValue: sensors,
                      toggleable: true,
                      onChanged: (value) {
                        sensors=value;
                        if(_streamSubscriptions.length>0){
                          _streamSubscriptions.first.cancel();
                          _streamSubscriptions.removeAt(0);
                        }
                        _streamSubscriptions.add(
                        accelerometerEvents.listen((AccelerometerEvent event1) {
                          setState(() {
                            xAxis = event1.x;
                            yAxis = event1.y;
                            zAxis = event1.z;
                          });
                        })
                        );
                      },
                    ),
                  ),
                  Expanded(
                    child: RadioListTile(
                      title: Text(
                        "Gyroscope",
                        textDirection: TextDirection.ltr,
                        style: TextStyle(
                          color: Colors.black,
                          fontSize: 20.0,
                        ),
                      ),
                      value: "Gyroscope",
                      groupValue: sensors,
                      toggleable: true,
                      onChanged: (value) {
                        sensors=value;
                        if(_streamSubscriptions.length>0){
                          _streamSubscriptions.first.cancel();
                          _streamSubscriptions.removeAt(0);
                        }
                        _streamSubscriptions.add(
                        gyroscopeEvents.listen((GyroscopeEvent event2) {
                          setState(() {
                              xAxis = event2.x;
                              yAxis = event2.y;
                              zAxis = event2.z;
                          });
                        })
                        );
                      },
                    ),
                  ),
                  Expanded(
                      child: Row(children: <Widget>[
                    Text(
                      " X axis :",
                      textDirection: TextDirection.ltr,
                      style: TextStyle(
                        color: Colors.black,
                        fontSize: 20.0,
                      ),
                    ),
                    Text(
                      xAxis.toString(),
                      textDirection: TextDirection.ltr,
                      style: TextStyle(
                        color: Colors.black,
                        fontSize: 20.0,
                      ),
                    ),
                        ElevatedButton(
                            onPressed: (){
                              if(_streamSubscriptions.length>0){
                                _streamSubscriptions.first.cancel();
                                _streamSubscriptions.removeAt(0);
                              }
                              Navigator.push(
                                  context,
                                  MaterialPageRoute(builder: (context) => Courbe())
                              );
                            },
                            child: Text("See graph ")
                        )
                  ])),
                  Expanded(
                      child: Row(children: <Widget>[
                    Text(
                      " Y axis :",
                      textDirection: TextDirection.ltr,
                      style: TextStyle(
                        color: Colors.black,
                        fontSize: 20.0,
                      ),
                    ),
                    Text(
                      yAxis.toString(),
                      textDirection: TextDirection.ltr,
                      style: TextStyle(
                        color: Colors.black,
                        fontSize: 20.0,
                      ),
                    ),
                        ElevatedButton(
                            onPressed: (){
                              if(_streamSubscriptions.length>0){
                              _streamSubscriptions.first.cancel();
                              _streamSubscriptions.removeAt(0);
                              }
                              Navigator.push(
                                context,
                                MaterialPageRoute(builder: (context) => Deplacement())
                              );
                        },
                            child: Text("See displacement ")
                        )
                  ])),
                  Expanded(
                      child: Row(children: <Widget>[
                    Text(
                      " Z axis :",
                      textDirection: TextDirection.ltr,
                      style: TextStyle(
                        color: Colors.black,
                        fontSize: 20.0,
                      ),
                    ),
                    Text(
                      zAxis.toString(),
                      textDirection: TextDirection.ltr,
                      style: TextStyle(
                        color: Colors.black,
                        fontSize: 20.0,
                      ),
                    ),
                        ElevatedButton(
                            onPressed: (){
                              if(_streamSubscriptions.length>0){
                                _streamSubscriptions.first.cancel();
                                _streamSubscriptions.removeAt(0);
                              }
                              Navigator.push(
                                  context,
                                  MaterialPageRoute(builder: (context) => Camera())
                              );
                            },
                            child: Text("See camera ")
                        )
                  ]))
                  //)
                ],
              ),
            ))));
  }
}
