import 'package:flutter/material.dart';
import 'package:uvsp_app/models/Vehicle.dart';

import '../models/MissionPlan.dart';
import '../services/http_service.dart';
import '../widgets/list_missions.dart';

class RequestMissionScreen extends StatefulWidget {
  final Vehicle vehicleSelected;
  const RequestMissionScreen({Key? key, required this.vehicleSelected}) : super(key: key);

  @override
  State<RequestMissionScreen> createState() => _RequestMissionScreenState();
}

class _RequestMissionScreenState extends State<RequestMissionScreen> {
  final HttpService httpService = HttpService();
  late Future<List<MissionPlan>> _missionsAvailablesList;


  Future<void> _refreshListMissions() async {
    //Rebuild UI
    setState(() {
      _missionsAvailablesList = httpService.getMissionsAvailables(widget.vehicleSelected.type);
    });
  }

  @override
  void initState(){
    super.initState();
    _missionsAvailablesList = httpService.getMissionsAvailables(widget.vehicleSelected.type);

  }
  FutureBuilder<List<MissionPlan>> _builderListMissions(BuildContext context) {
    return FutureBuilder<List<MissionPlan>>(
      future: _missionsAvailablesList,
      builder: (context, snapshot) {
        // if(snapshot.connectionState == ConnectionState.done){
        if (snapshot.hasData) {
          final List<MissionPlan>? missionsAvailables = snapshot.data;

          print("hola");
          print(snapshot.data);
          return Expanded(
            child: RefreshIndicator(
                color: const Color.fromRGBO(58, 66, 86, 1.0),
                onRefresh: () async {
                  _refreshListMissions();
                },
                child: buildListMissionsAvailables(context, missionsAvailables!, httpService, widget.vehicleSelected)),
          );
        } else if (snapshot.hasError) {
          return RefreshIndicator(color: const Color.fromRGBO(58, 66, 86, 1.0),
            onRefresh: () async {
              _refreshListMissions();
            },
            child: SingleChildScrollView(
              physics: const AlwaysScrollableScrollPhysics(),
              child: Container(
                height: 100,
                width: 300,
                color: Colors.white,
                child: Padding(
                    padding: const EdgeInsets.all(16.0),
                    child: Center(
                        child: Text(
                          'Error ->${snapshot.error}',
                          style: const TextStyle(
                              fontSize: 16.0,
                              color: Color.fromRGBO(58, 66, 86, 1.0)),
                        ))),
              ),
            ),
          );
        } else {
          return Center(
              child: Column(children: const [
                SizedBox(height: 200),
                SizedBox(
                    height: 75,
                    width: 75,
                    child: CircularProgressIndicator(
                      strokeWidth: 6.0,
                      color: Colors.white70,
                    )),
              ]));
        }
      },
    );
  }

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
    body: SafeArea(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.center,
        mainAxisAlignment: MainAxisAlignment.start,
        children: <Widget>[
          Padding(
              padding: const EdgeInsets.all(12.0),
              child: SizedBox(
                width: 400,
                height: 50,
                child: Card(
                    color: Colors.white,
                    child: Center(
                        child: Text(
                          'Missions availables ${widget.vehicleSelected.name}',
                          style: const TextStyle(
                              fontWeight: FontWeight.bold,
                              fontSize: 16.0,
                              color: Color.fromRGBO(58, 66, 86, 1.0)),
                          semanticsLabel: 'availables missions for ${widget.vehicleSelected.name}',
                        ))),
              )),
          _builderListMissions(context),
          //ElevatedButton(onPressed: () async => fetchInfo(), child: const Text('API request')),
        ],
      ),
    ),);

  }
}
