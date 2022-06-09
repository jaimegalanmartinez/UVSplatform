import Vehicle from "../models/Vehicle.js"
//import Fleet from "../models/Fleet.js"


const vehicleData = {
    fleet_id: 1,
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
  

export const checkAvailability = async (req, res) => {
    Vehicle.find({ vehicle_status: "idle"})
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
    res.json(
        {
            "Title": "UVS Platform"
        }
    );

}