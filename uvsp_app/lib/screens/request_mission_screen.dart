import 'package:flutter/material.dart';
import 'package:uvsp_app/models/Vehicle.dart';

class RequestMissionScreen extends StatefulWidget {
  final Vehicle vehicleSelected;
  const RequestMissionScreen({Key? key, required this.vehicleSelected}) : super(key: key);

  @override
  State<RequestMissionScreen> createState() => _RequestMissionScreenState();
}

class _RequestMissionScreenState extends State<RequestMissionScreen> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
        backgroundColor: const Color.fromRGBO(58, 66, 86, 1.0),
    appBar: AppBar(
    // Here we take the value from the MyHomePage object that was created by
    // the App.build method, and use it to set our appbar title.
    title: const Text("UVS Platform"),
    automaticallyImplyLeading: true,
    centerTitle: true,),
    body: SafeArea(child: Text('Missions availables ${widget.vehicleSelected.name}')
    ),);

  }
}
