import 'package:flutter/material.dart';
import 'package:uvsp_app/models/MissionPlan.dart';
import 'package:uvsp_app/models/Vehicle.dart';

import 'package:uvsp_app/screens/request_mission_screen.dart';
import 'package:uvsp_app/services/http_service.dart';

ListView buildListMissionsAvailables(
    BuildContext context, List<MissionPlan> missions, HttpService httpService, Vehicle vehicleSelected) {
  bool isMissionRequested = false;
  return ListView.separated(
    separatorBuilder: (context, index) => const SizedBox(
      height: 12,
    ),
    itemCount: missions.length,
    padding: const EdgeInsets.all(12.0),
    itemBuilder: (context, index) {
      return Card(
        elevation: 4,
        child: ExpansionTile(
          title: Text(
            'Mission plan: ${missions[index].name}',
            style: const TextStyle(fontWeight: FontWeight.bold),
          ),
          subtitle: Column(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const SizedBox(height: 10),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  Text(
                      'Mission commands: ${missions[index].missionCommands.length}')
                ],
              ),
              const SizedBox(height: 10),
            ],
          ),
          children: [
            ElevatedButton(
              style: ButtonStyle(
                  backgroundColor: MaterialStateProperty.all<Color>(
                      const Color.fromRGBO(58, 66, 86, 1.0)),
                  minimumSize:
                      MaterialStateProperty.all<Size>(const Size(140, 40))),
              onPressed: () {
                ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                  backgroundColor: const Color.fromRGBO(71, 82, 107, 1.0),
                  content: Text("Are you sure to request the mission: ${missions[index].name} ?"),
                  duration: const Duration(seconds: 6),
                  action: SnackBarAction(textColor: Colors.green,
                  label: 'Confirm \nmission plan',
                  onPressed: () async {
                    httpService.requestMission(missions[index], vehicleSelected);
                  },
                ),));
              },
              child: const Text(
                "Request this mission plan",
                style: TextStyle(color: Colors.white),
              ),
            )
          ],
        ),
      );
    },
  );
}
