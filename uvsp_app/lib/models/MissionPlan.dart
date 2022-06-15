
import 'dart:ffi';
class MissionCommand {
  final String name;
  final String description;

  const MissionCommand({
      required this.name,
      required this.description
  });

  factory MissionCommand.fromJson(Map<String, dynamic> json){
    return MissionCommand(
      name: json['name'],
      description: json['description']);
  }
  Map<String, dynamic> toJson() =>
      {'name': name, 'description': description};


}
class MissionPlan {
  final String name;
  final String vehiclesSupported;
  final List<MissionCommand> missionCommands;

  const MissionPlan({
    required this.name,
    required this.vehiclesSupported,
    required this.missionCommands,
  });

  factory MissionPlan.fromJson(Map<String, dynamic> json){
    //https://stackoverflow.com/questions/57508596/flutter-json-array-of-objects
    var listCommands = json['mission_commands'] as List;
    final List<MissionCommand> commands = listCommands.map((command) => MissionCommand.fromJson(command)).toList();
    return MissionPlan(
        name: json['name'],
        vehiclesSupported: json['vehicles_supported'],
        missionCommands: commands
    );
  }
  Map<String, dynamic> toJson() =>
      {'name': name, 'vehicles_supported': vehiclesSupported, 'mission_commands': missionCommands};
}