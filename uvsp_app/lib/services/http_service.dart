import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;
import 'package:uvsp_app/models/MissionPlan.dart';
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

  Future<List<MissionPlan>> getMissionsAvailables(VehicleType vehicleType) async {
    final auth = await _authInstance.currentUser!.getIdTokenResult();
    String userToken = auth.token!;

    try {
      final response = await http.get(
          Uri.parse("$uriMissionManager/api/v1/missions/missionsAvailables/${vehicleType.name.toUpperCase()}"),
          headers: {
            HttpHeaders.authorizationHeader: 'Bearer $userToken',
          });
      if (response.statusCode == 200) {
        //If the server did return a 200 OK response,
        //then parse the JSON.
        //return Album.fromJson(jsonDecode(response.body));
        List<dynamic> body = jsonDecode(response.body);
        List<MissionPlan> missionsAvailables = body
            .map(
              (dynamic missionsJSON) => MissionPlan.fromJson(missionsJSON),
        )
            .toList();

        if (kDebugMode) {
          print('Missions Plans retrieved: ${missionsAvailables.length}');
        }

        if (kDebugMode) {
          print(missionsAvailables.first.name);
        }
        return missionsAvailables;
      } else {
        throw Exception('Unable to retrieve missions availables');
      }
    } catch (error) {
      if (error is SocketException) {
        throw Exception(
            '${error.message}.Unable to retrieve missions availables');
      }
      throw Exception('Unable to retrieve missions availables');
    }
  }

  //Requests the creation of a mission for a specific mission plan
  Future<String> requestMission(MissionPlan missionPlan, Vehicle vehicleSelected) async {
    final auth = await _authInstance.currentUser!.getIdTokenResult();
    String userToken = auth.token!;

    try {
      final response = await http.post(
          Uri.parse("$uriMissionManager/api/v1/missions/requestMission"),
          headers: {
            HttpHeaders.authorizationHeader: 'Bearer $userToken',
            HttpHeaders.contentTypeHeader:'application/json'

          },
      body: _createMissionJSONresponse(missionPlan, vehicleSelected));
      if (response.statusCode == 201) {
        // If the server did return a 201 CREATED response,
        // then parse the JSON.
        //return Album.fromJson(jsonDecode(response.body));
        return response.body;
      } else {
        // If the server did not return a 201 CREATED response,
        // then throw an exception.
        throw Exception('Failed to create mission.');
      }

    } catch (error) {
      if (error is SocketException) {
        throw Exception(
            '${error.message}.Unable to request the specific mission');
      }else{
        throw Exception(
            '${error.toString()}.Unable to request the specific mission');
      }
    }

  }
  String _createMissionJSONresponse(MissionPlan missionPlan, Vehicle vehicleSelected){
    var data = missionPlan.toJson();
    data['vehicle_id'] = vehicleSelected.id;
    data['fleet_id'] = vehicleSelected.fleetId;
    return json.encode(data);
  }
}