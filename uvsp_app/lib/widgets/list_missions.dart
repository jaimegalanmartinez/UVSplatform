import 'package:flutter/material.dart';
import 'package:uvsp_app/models/MissionPlan.dart';
import 'package:uvsp_app/models/Vehicle.dart';

import 'package:uvsp_app/screens/request_mission_screen.dart';
import 'package:uvsp_app/services/http_service.dart';

ListView buildListMissionsAvailables(
    BuildContext context, List<MissionPlan> missions, HttpService httpService) {
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
              onPressed: () async {

                httpService.requestMission(missions[index]);
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
