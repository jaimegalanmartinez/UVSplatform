import 'package:flutter/material.dart';
import 'package:uvsp_app/models/Vehicle.dart';

import '../screens/request_mission_screen.dart';

ListView buildListVehicles(BuildContext context, List<Vehicle> vehicles) {
  return ListView.separated(
    separatorBuilder: (context, index) => const SizedBox(height: 12,),
    itemCount: vehicles.length,
    padding: const EdgeInsets.all(12.0),
    itemBuilder: (context, index) {
      return Card(
        elevation: 4,
        child: ExpansionTile(
          leading: vehicles[index].type == VehicleType.uav
              ? Image.asset(
            'assets/images/drone.png',
            color: const Color.fromRGBO(71, 82, 107, 1.0),
            height: 48,
            width: 48,
          )
              : Image.asset('assets/images/ugv.png',
              color: const Color.fromRGBO(71, 82, 107, 1.0),
              height: 48,
              width: 48),
          title: Text(
                vehicles[index].name,
                style: const TextStyle(fontWeight: FontWeight.bold),
              ),
          subtitle: Column(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const SizedBox(height: 10),
            Text('Fleet: ${vehicles[index].fleetName}'),
              const SizedBox(height: 10),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              crossAxisAlignment: CrossAxisAlignment.center,
              children: [
                Text('Status: ${vehicles[index].status.name}'),Text('Type: ${vehicles[index].type.name.toUpperCase()}'),
                Text('Battery: ${vehicles[index].battery}%'),

              ],
            ),
              const SizedBox(height: 10),
          ],),
          children: [
            ElevatedButton(style: ButtonStyle(
                backgroundColor: MaterialStateProperty.all<Color>(const Color.fromRGBO(58, 66, 86, 1.0)),
                minimumSize:
                MaterialStateProperty.all<Size>(const Size(140, 40))),
              onPressed: () {  Navigator.push(context, MaterialPageRoute(builder: (context) => RequestMissionScreen(vehicleSelected: vehicles[index])));},
            child:const Text("Request a mission", style: TextStyle(color: Colors.white),),
            )
          ],
        ),
      );
    },
  );
}