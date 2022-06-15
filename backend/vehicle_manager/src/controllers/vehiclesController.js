import Vehicle from "../models/Vehicle.js"
import Fleet from "../models/Fleet.js"
import mongoose from "mongoose";

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
    vehicle_status: "on-mission",
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
  
//comandos capaz de ejecutar un vehiculo incluirlo como array en un parámetro. el mission manager gestiona y hace el chequeo.
//que ocurre si un vehiculo tiene que abortar la mision parcial (alguien decide si saltar comandos o continuar)
// planificador de problemas o completamente
//Dron detecta error el fleet manager e informa al mission manager de que la mision ha sido abortada y vuelve a casa y aterriza.
//abortar por colision o por sin bateria, gestor flota recibe: vehiculo volver automaticamente o atterizar, no puedo ejecutar por medir: pues aterriza
// vuelve a casa.
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
    /*await Fleet.find({ fleet_name: "Fleet_Webots"}).populate({path: "fleet_vehicles"}).then((data) => {
        res.json(data);
    });*/
    /*Vehicle.findByIdAndUpdate("62a1aaf7d063093f592cae21", {fleet_id: mongoose.mongo.ObjectId("62a4b9fdaec3727326e3069a")}, function (err, docs){
        if (err){
            console.log(err);
        }else{
            console.log("Updated user: ", docs);
        }

    });*/
    /*newFleet.save((error) => {
        if (error){
            res.status(500).json("Internal server error");
            console.log('Error, saving vehicle data', error);
        }else{
            console.log('Fleet data has been saved');
        }
    });*/
   

}