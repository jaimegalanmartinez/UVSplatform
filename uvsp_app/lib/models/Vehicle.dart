
///Type of the vehicle (UAV: unmanned aerial vehicle, UGV: unmanned ground vehicle)
enum VehicleType {uav, ugv}
/// Status of a specific vehicle (idle, onMission or charging)
enum VehicleStatus {idle, onMission, charging}
/// Class that describes an unmanned vehicle
/// @author: Jaime Galán Martínez
///
///
class Vehicle {
  final int id;
  final VehicleType type;
  final VehicleStatus status;
  final String name;
  final int battery;

  const Vehicle({
    required this.id,
    required this.type,
    required this.status,
    required this.name,
    required this.battery
  });

  factory Vehicle.fromJson(Map<String, dynamic> json){
    //Check about fleet_id field, not retrieving from mongo
    return Vehicle(
        id: json['vehicle_id'],
        type: VehicleType.values.byName(json['vehicle_type'].toString().toLowerCase()),
        status: VehicleStatus.values.byName(json['vehicle_status']),
        name: json['vehicle_name'],
        battery: json['battery']);
  }

}