import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;
import '../models/Vehicle.dart';
import 'package:firebase_auth/firebase_auth.dart';

class HttpService {
  static const String uriMissionManager = "http://192.168.1.14:3000";
  final FirebaseAuth _authInstance = FirebaseAuth.instance;

  Future<String?> _getFirebaseToken() async {
    final auth = await _authInstance.currentUser!.getIdTokenResult();
    return auth.token!;
  }

  User? get getCurrentUser{
    return _authInstance.currentUser;
  }
  FirebaseAuth get getAuth{
    return _authInstance;
  }
  Future<List<Vehicle>> getVehiclesAvailables() async {
    //https://mobikul.com/http-api-calling-in-flutter/
    final auth = await _authInstance.currentUser!.getIdTokenResult();
    String userToken = auth.token!;

    try {
      final response = await http.get(
          Uri.parse("$uriMissionManager/api/v1/missions/vehiclesAvailables"),
          headers: {
            HttpHeaders.authorizationHeader: 'Bearer $userToken',
          });
      if (response.statusCode == 200) {
        //If the server did return a 200 OK response,
        //then parse the JSON.
        //return Album.fromJson(jsonDecode(response.body));
        List<dynamic> body = jsonDecode(response.body);
        List<Vehicle> vehiclesAvailables = body
            .map(
              (dynamic vehicleJSON) => Vehicle.fromJson(vehicleJSON),
        )
            .toList();

        if (kDebugMode) {
          print('Vehicles retrieved: ${vehiclesAvailables.length}');
        }

        if (kDebugMode) {
          print(vehiclesAvailables.first.name);
        }
        return vehiclesAvailables;
      } else {
        throw Exception('Unable to retrieve vehicles availables');
      }
    } catch (error) {
      if (error is SocketException) {
        throw Exception(
            '${error.message}.Unable to retrieve vehicles availables');
      }
      throw Exception('Unable to retrieve vehicles availables');
    }
  }
}