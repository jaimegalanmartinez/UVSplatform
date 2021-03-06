
///Type of the vehicle (UAV: unmanned aerial vehicle, UGV: unmanned ground vehicle)
enum VehicleType {uav, ugv}
/// Status of a specific vehicle (idle, onMission or charging)
enum VehicleStatus {idle, onMission, charging}
/// Class that describes an unmanned vehicle
/// @author: Jaime Galán Martínez
///
///
class Vehicle {
  final String id;
  final VehicleType type;
  final VehicleStatus status;
  final String name;
  final int battery;
  final String fleetName;
  final String fleetId;

  const Vehicle({
    required this.id,
    required this.type,
    required this.status,
    required this.name,
    required this.battery,
    required this.fleetName,
    required this.fleetId
  });

  factory Vehicle.fromJson(Map<String, dynamic> json){
    return Vehicle(
        id: json['_id'],
        type: VehicleType.values.byName(json['vehicle_type'].toString().toLowerCase()),
        status: VehicleStatus.values.byName(json['vehicle_status']),
        name: json['vehicle_name'],
        battery: json['battery'],
        fleetName: json['fleet_id']['fleet_name'],
        fleetId: json['fleet_id']['_id']);
  }

}