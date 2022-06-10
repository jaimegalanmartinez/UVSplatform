import 'dart:convert';
import 'dart:io';

import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:uvsp_app/models/Vehicle.dart';
import 'package:uvsp_app/screens/login_screen.dart';
import 'package:http/http.dart' as http;

import 'package:uvsp_app/services/http_service.dart';
import 'package:font_awesome_flutter/font_awesome_flutter.dart';
class HomeScreen extends StatefulWidget {
  const HomeScreen({Key? key, required this.title}) : super(key: key);

  // This widget is the home page of your application. It is stateful, meaning
  // that it has a State object (defined below) that contains fields that affect
  // how it looks.

  // This class is the configuration for the state. It holds the values (in this
  // case the title) provided by the parent (in this case the App widget) and
  // used by the build method of the State. Fields in a Widget subclass are
  // always marked "final".

  final String title;

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  final currentUser = FirebaseAuth.instance.currentUser;
  String userToken = "";
  String username = "";
  final HttpService httpService = HttpService();
  late Future<List<Vehicle>> vehiclesAvailables;

  Future<void> _signOut() async {
    await FirebaseAuth.instance.signOut().then((value) => {
          Navigator.pushReplacement(context,
              MaterialPageRoute(builder: (context) => const LoginScreen()))
        });
  }

  Future<void> _getFirebaseToken() async {
    await currentUser!
        .getIdTokenResult()
        .then((result) => userToken = result.token!);
  }
  Future<List<Vehicle>> getVehiclesAvailables(String userToken) async {
    //https://mobikul.com/http-api-calling-in-flutter/
    final auth = await currentUser!.getIdTokenResult();
    userToken = auth.token!;
    const String uriMissionManager = "http://192.168.1.14:3000";
    final response = await http.get(Uri.parse("$uriMissionManager/api/v1/missions/vehiclesAvailables"),
        headers: {
          HttpHeaders.authorizationHeader: 'Bearer $userToken',
        });
    if (response.statusCode == 200){
      //If the server did return a 200 OK response,
      //then parse the JSON.
      //return Album.fromJson(jsonDecode(response.body));
      List<dynamic> body = jsonDecode(response.body);
      List<Vehicle> vehiclesAvailables = body.map(
            (dynamic vehicleJSON) => Vehicle.fromJson(vehicleJSON),
      ).toList();

      if (kDebugMode) {
        print('Vehicles retrieved: ${vehiclesAvailables.length}');
      }

      if (kDebugMode) {
        print(vehiclesAvailables.first.name);
      }
      return vehiclesAvailables;
    }else {
      throw Exception('Unable to retrieve vehicles availables');
    }
  }

  ListView _buildListVehicles(BuildContext context, List<Vehicle> vehicles) {
    return ListView.builder(
      itemCount: vehicles.length,
      padding: const EdgeInsets.all(8),
      itemBuilder: (context, index) {
        return Card(
          elevation: 4,
          child: ListTile(
            leading: vehicles[index].type == VehicleType.uav ?
            Image.asset('assets/images/drone.png', color: const Color.fromRGBO(71, 82, 107, 1.0), height: 40,width: 40,):
            Image.asset('assets/images/ugv.png', color: const Color.fromRGBO(71, 82, 107, 1.0), height: 45,width: 45),
            title: Text(
              vehicles[index].name,
              style: const TextStyle(fontWeight: FontWeight.bold),
            ),
            subtitle: Text('Status: ${vehicles[index].status.name}'),
          ),
        );
      },
    );
  }

  FutureBuilder<List<Vehicle>> _builderListVehicles(
      BuildContext context) {
    return FutureBuilder<List<Vehicle>>(
      future: getVehiclesAvailables(userToken),
      builder: (context, snapshot) {
        if(snapshot.connectionState == ConnectionState.done){
          final List<Vehicle>? vehiclesAvailables = snapshot.data;
          return Expanded(child: _buildListVehicles(context, vehiclesAvailables!),);

        }else if(snapshot.hasError){
          return SizedBox(height: 75, width: 75, child: Text('Error: ${snapshot.error}'));

        } else {
          return Center(child: Column(children: const [
            SizedBox(height: 200),
            SizedBox(height: 75, width: 75, child: CircularProgressIndicator(strokeWidth: 6.0, color: Colors.white70,)),
          ]));
        }
      },
    );
  }

  @override
  void initState() {
    super.initState();
    username = currentUser?.displayName ?? 'No user';
    username = username.replaceFirst(username[0], username[0].toUpperCase());
    //_getFirebaseToken();
    //getVehiclesAvailables(userToken);

  }

  @override
  Widget build(BuildContext context) {
    // The Flutter framework has been optimized to make rerunning build methods
    // fast, so that you can just rebuild anything that needs updating rather
    // than having to individually change instances of widgets.

    return Scaffold(
      backgroundColor: const Color.fromRGBO(58, 66, 86, 1.0),
      appBar: AppBar(
        // Here we take the value from the MyHomePage object that was created by
        // the App.build method, and use it to set our appbar title.
        title: Text(widget.title),
        automaticallyImplyLeading: false,
        centerTitle: true,
        actions: [
          Padding(
            padding: const EdgeInsets.only(right: 20.0),
            child: GestureDetector(
              onTap: () {
                _signOut();
              },
              child: const Icon(
                Icons.logout,
                size: 26.0,
                semanticLabel: 'Logout',
              ),
            ),
          )
        ],
      ),
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
                        'Welcome $username',
                        style: const TextStyle(
                            fontWeight: FontWeight.bold,
                            fontSize: 16.0,
                            color: Color.fromRGBO(58, 66, 86, 1.0)),
                        semanticsLabel: 'user logged in',
                      ))),
                )),
            _builderListVehicles(context),
            //ElevatedButton(onPressed: () async => fetchInfo(), child: const Text('API request')),
          ],
        ),
      ),
    );
  }
}
