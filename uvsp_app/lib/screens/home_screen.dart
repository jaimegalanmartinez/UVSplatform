import 'dart:io';

import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:uvsp_app/screens/login_screen.dart';
import 'package:http/http.dart' as http;

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
  Future<void> _signOut() async {
    await FirebaseAuth.instance.signOut().then((value) => {
          Navigator.pushReplacement(context,
              MaterialPageRoute(builder: (context) => const LoginScreen()))
        });
  }

  Future<void> _getFirebaseToken() async {
    await currentUser!.getIdTokenResult().then((result) => userToken = result.token!);

  }
  Future<String> fetchInfo() async {
    const String uriMissionManager = "http://192.168.1.14:3000";
    final response = await http.get(Uri.parse("$uriMissionManager/api/v1/missions/vehiclesAvailables"),
    headers: {
      HttpHeaders.authorizationHeader: 'Bearer $userToken',
    });
    if (response.statusCode == 200){
      //If the server did return a 200 OK response,
      //then parse the JSON.
      //return Album.fromJson(jsonDecode(response.body));
      if (kDebugMode) {
        print(response.body);
      }
      return response.body;
    }else {
      throw Exception('Failed to load the info requested');
    }
  }

  @override
  void initState(){
    super.initState();
    username = currentUser?.displayName ?? 'No user';
    username = username.replaceFirst(username[0], username[0].toUpperCase());
    _getFirebaseToken();
  }

  @override
  Widget build(BuildContext context) {
    // The Flutter framework has been optimized to make rerunning build methods
    // fast, so that you can just rebuild anything that needs updating rather
    // than having to individually change instances of widgets.

    return Scaffold(
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
      body: Column(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        crossAxisAlignment: CrossAxisAlignment.center,
        children: <Widget>[
          Padding(
              padding: const EdgeInsets.all(12.0),
              child: SizedBox(
                width: 400,
                height: 50,
                child: Card(
                    color: Colors.blue.shade400,
                    child: Center(
                        child: Text(
                      'Welcome $username',
                      style: const TextStyle(
                          fontWeight: FontWeight.bold,
                          fontSize: 16.0,
                          color: Colors.white),
                      semanticsLabel: 'user logged in',
                    ))),
              )),
          ElevatedButton(onPressed: () async => fetchInfo(), child: const Text('API request'))
        ],
      ),
    );
  }
}
