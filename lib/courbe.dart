
//import 'package:camera/camera.dart';
import 'package:flutter/material.dart';
import 'package:arcore_flutter_plugin/arcore_flutter_plugin.dart';


class Courbe extends StatefulWidget{
  @override
  State<StatefulWidget> createState() {
    // TODO: implement createState
    return CourbeState();
  }

}
class CourbeState extends State<Courbe>{
  @override
  Widget build(BuildContext context) {
    // TODO: implement build
    return Scaffold(
      appBar: AppBar(
        title: Text(
            "Vision de courbe"),
        backgroundColor: Colors.pink,
      ),
      body:Text("Courbe"),
    );
  }

}