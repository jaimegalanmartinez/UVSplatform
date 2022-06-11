import 'package:flutter/material.dart';
import 'package:uvsp_app/models/Vehicle.dart';
import 'package:uvsp_app/screens/login_screen.dart';
import 'package:uvsp_app/services/http_service.dart';

import '../widgets/list_vehicles.dart';

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

  String username = "";
  final HttpService httpService = HttpService();
  late Future<List<Vehicle>> _vehiclesAvailablesList;

  Future<void> _signOut() async {
    await httpService.getAuth.signOut().then((value) => {
          Navigator.pushReplacement(context,
              MaterialPageRoute(builder: (context) => const LoginScreen()))
        });
  }

  Future<void> _refreshListVehicles() async {
    //Rebuild UI
    setState(() {
      _vehiclesAvailablesList = httpService.getVehiclesAvailables();
    });
  }

  FutureBuilder<List<Vehicle>> _builderListVehicles(BuildContext context) {
    return FutureBuilder<List<Vehicle>>(
      future: _vehiclesAvailablesList,
      builder: (context, snapshot) {
        // if(snapshot.connectionState == ConnectionState.done){
        if (snapshot.hasData) {
          final List<Vehicle>? vehiclesAvailables = snapshot.data;
          print("hola");
          print(snapshot.data);
          return Expanded(
            child: RefreshIndicator(
                color: const Color.fromRGBO(58, 66, 86, 1.0),
                onRefresh: () async {
                  _refreshListVehicles();
                },
                child: buildListVehicles(context, vehiclesAvailables!)),
          );
        } else if (snapshot.hasError) {
          return RefreshIndicator(color: const Color.fromRGBO(58, 66, 86, 1.0),
            onRefresh: () async {
              _refreshListVehicles();
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
  void initState() {
    super.initState();
    username = httpService.getCurrentUser?.displayName ?? 'No user';
    username = username.replaceFirst(username[0], username[0].toUpperCase());
    _vehiclesAvailablesList = httpService.getVehiclesAvailables();
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
