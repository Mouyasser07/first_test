import 'package:arcore_flutter_plugin/arcore_flutter_plugin.dart';
//import 'package:camera/camera.dart';
import 'package:flutter/material.dart';
class Camera extends StatefulWidget{
  @override
  State<StatefulWidget> createState() {
    return CameraState();
  }

}
class CameraState extends State<Camera>{
  @override
  Widget build(BuildContext context) {
    // TODO: implement build
    throw Scaffold(
      appBar: AppBar(
        title: Text("Vision de camera"),
        backgroundColor: Colors.pink,
      ),
      body:Text("Camera"),

      );

  }

}