import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;
import '../models/Vehicle.dart';

class HttpService {
  final String postsURL = "https://jsonplaceholder.typicode.com/posts";

  Future<List<Vehicle>> getVehiclesAvailables(String userToken) async {
    //https://mobikul.com/http-api-calling-in-flutter/
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
        print(response.body);
      }
      return vehiclesAvailables;
    }else {
      throw Exception('Unable to retrieve vehicles availables');
    }
  }
}