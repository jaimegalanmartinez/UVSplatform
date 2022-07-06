/**
 * Vehicle Manager - Vehicle Manager Controller
 * @author: Jaime Galán Martínez
 */
import Vehicle from "../models/Vehicle.js"
import Fleet from "../models/Fleet.js"
import mongoose from "mongoose";
import * as FleetInterface from "../fleets_interface.js";
const rclnodejs = require('rclnodejs');

const vehicleData = {
    fleet_id: mongoose.mongo.ObjectId("62a4b9fdaec3727326e3069a"),
    vehicle_id: 1,
    vehicle_type: "UAV",
    vehicle_status: "idle",
    vehicle_name: "UAV_Webots",
    battery: 100,
};

const vehicle2Data = {
    fleet_id: 2,
    vehicle_id: 2,
    vehicle_type: "UGV",
    vehicle_status: "onMission",
    vehicle_name: "UGV_1",
    battery: 80,
};

const fleetData = {
    fleet_name: "Fleet_Webots",
    location: "Farm_001",
    fleet_vehicles: [
        mongoose.mongo.ObjectId("62a1a84f20092252107b42ed")
    ]

};
const newFleet = new Fleet(fleetData);
const newVehicle = new Vehicle(vehicle2Data);
/*newVehicle.save((error) => {
    if (error){
        res.status(500).json(message:{"Internal server error"});
        console.log('Error, saving vehicle data', error);
    }else{
        console.log('Vehicle data has been saved');
    }
});
*/
// /:id
export const getVehicleInformation = async (req, res) => {
    try {
        const { id } = req.params;
        console.log(req.params.id)
        const vehicle = await Vehicle.findById(id)

        if (!vehicle) return res.status(404).json({ message: `Vehicle with ${id} doesn't exists` })

        res.json(vehicle)

    } catch (error) {
        res.status(500).json({ message: error.message || `Error retrieving vehicle with id: ${id}` })
  }

}
  
//Check vehicles availables and retrieve the vehicles with status "idle".
export const checkAvailability = async (req, res) => {
    Vehicle.find({ vehicle_status: "idle"}).populate({path:"fleet_id", select:'fleet_name'})
    .then((data) =>{
        console.log('Data', data);
        res.json(data);

    }).catch( (error) => {
        console.log('error ', error);
        res.status(500).json("All vehicles are busy. There aren't vehicles availables")
    });

}

/*
Retrieves a JSON array with all the vehicles information from the uvs-vehicles DB
*/
export const getAllVehicles = async (req, res) => {
    await Vehicle.find({ })
    .then((data) =>{
        console.log('Data', data);
        res.json(data);

    }).catch( (error) => {
        console.log('error ', error);
    });

}

export const updateFleetVehicles = async (req, res) => {
    res.json(
        {
            "Title": "UVS Platform"
        }
    );

}

export const abortMission = async (req, res) => {
   
}

export const sendMissionToFleetManager = async (req, res) => {
    console.log('VEHICLE MANAGER: RECEIVING MISSION');
    //req.body contains a json with the data of the mission requested
    console.log(req.body);
    //Delete field description of all mission commands
    req.body.mission_plan.mission_commands.forEach(element => delete element.description);
    
    FleetInterface.infoMessage("Hello from Fleet Interface module");
    FleetInterface.publish(JSON.stringify(req.body), `fleet_${req.body.fleet_id}/missionPlan`);
    
    res.status(200).json(
        {
            "VEHICLE MANAGER notification": `Received the mission: ${req.body.mission_id}`
        } 
    );
   
   

}