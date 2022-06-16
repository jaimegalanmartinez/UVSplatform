import Mission from "../models/Mission.js"
import axios from 'axios';
import MissionPlan from "../models/MissionPlan.js";
import crypto from 'crypto';
      /* MISSION DRONE
      NAV_TAKEOFF - Automatically
      NAV_WAYPOINT
      //In place for measure
      NAV_LAND
      NAV_TAKEOFF
      NAV_HOME
      NAV_LAND

      export const missionPlanSchema = new mongoose.Schema({
    name: {
      type: String,
      required: true,
      trim: true  
    },

    vehicles_supported: {
        type: String,
        required: true,
        trim: true
    },

    mission_commands: {
        type: [missionCommandSchema]
    },

},{
    versionKey: false,
    timestamps: true,
    collection: "missionsPlans"
});
      */

//solicitada, aceptada, en progreso, abortada, completada. missiones
const missionCommands = [
    {
        name: "NAV_TAKEOFF",
        description: "The drone will take off from ground/hand",
    },
    {
        name: "NAV_WAYPOINT",
        description: "List of locations to which the vehicle needs to go",
    },
    {
        name: "NAV_MEASURE",
        description: "The drone will land a specific height to measure. After that, it'll get height at a given location",
    }, 
    {
        name: "NAV_HOME",
        description: "The vehicle will return home",
    },
    {
        name: "NAV_LAND",
        description: "The drone will land at a given location",
    },

];

const missionPlanData = {
    name: "Webots drone",
    vehicles_supported: "UAV",
    mission_commands: missionCommands

};

const plan1 = new MissionPlan(missionPlanData);

/*newFleet.save((error) => {
        if (error){
            res.status(500).json("Internal server error");
            console.log('Error, saving vehicle data', error);
        }else{
            console.log('Fleet data has been saved');
        }
    });*/

export const getAllMissions = async (req, res) => {
    const missions = await Mission.find()
    res.json(missions)
}

// /:id
export const getMission = async (req, res) => {
    try {
        const { id } = req.params;
        console.log(req.params.id)
        const mission = await Mission.findById(id)

        if (!mission) return res.status(404).json({ message: `Mission with ${id} doesn't exists` })

        res.json(mission)

    } catch (error) {
        res.status(500).json({ message: error.message || `Error retrieving mission with id: ${id}` })

    }

}

export const requestMission = async (req, res) => {
    console.log(req.body);
    let vehicleId = req.body.vehicle_id;
    let fleetId = req.body.fleet_id;
    console.log(vehicleId);
    let keyToDelete = 'vehicle_id';
    //Remove vehicle_id field from req.body JSON
    delete req.body[keyToDelete];
    keyToDelete = 'fleet_id';
    //Remove fleet_id field from req.body JSON
    delete req.body[keyToDelete];
    let missionPlanReceived = req.body;
    console.log(missionPlanReceived);
    //req.body is a JSON with the MissionPlan.
    const missionIdGenerated = crypto.randomUUID();

    const missionData = {
        mission_id: missionIdGenerated,//autogenerate
        name: `mission-flt-${fleetId}-${missionIdGenerated}`,
        fleet_id: fleetId,
        vehicle_id: vehicleId,
        mission_plan: missionPlanReceived,
        mission_status: 'requested'
    };
    //Possible missions status: requested, accepted, running, aborted and completed (solicitada, aceptada, en progreso, abortada, completada)
    const missionToCreate = new Mission(missionData);
    //Create the requested mission in missions collection
    missionToCreate.save((error) => {
        if (error){
            res.status(500).json("Internal server error");
            console.log('Error, saving mission', error);
        }else{
            console.log(`Mission with id: ${missionIdGenerated} has been saved in db`);
            res.status(201).json(`Created a new mission with the following mission plan requested: ${req.body.name}`);
        }
    });
    //Send mission plan to Vehicle Manager
    //_sendMissionToVehicleManager(missionData);
    const urlVehicleManager = 'http://localhost:3001';
    try{
        const response = await axios({
            method: 'post',
            url: urlVehicleManager + '/api/v1/vehicles/receiveMission',
            responseType: 'json',
            data: missionData
        });
    
        if (response.status == 200){
            console.log(response.data);
            //Response, assuring the mission was received by the Vehicle Manager
            //res.json(response.data);
        }
    
    }catch(err){
        console.error(err);
        if(axios.isAxiosError(err)){
            if(err.response) {
                //Request mae and server responded with error
                // The client was given an error response (5xx, 4xx)
                console.error(err.response);
            } else if (err.request){
                // Request was made but no response received
                // The client never received a response, and the request was never left
                console.error(err.request);
                res.status(503).json("Vehicle manager server time out")
    
            } else {
                //Something happended in setting up the request that triggered an Error
                console.error(err);
    
            }
        }
           
    }

}

/*export const abortMission = async (req, res) => {
    await plan1.save((error) => {
        if (error){
            res.status(500).json('Internal server error');
            console.log('Error, saving vehicle data', error);
        }else{
            console.log('Plan data has been saved');
        }
    });
    

}*/

export const getMissionsStatus = async (req, res) => {
    res.json(
        {
            "Title": "UVS Platform"
        }
    );

}

export const getMissionsAvailables = async (req, res) => {
    console.log(req.params.typeVehicle);
    let typeVehicle = req.params.typeVehicle === 'UAV' ? 'UAV': 'UGV';
   
    MissionPlan.find({ vehicles_supported: typeVehicle })
    .then((data) =>{
        console.log('Data', data);
        res.json(data);

    }).catch( (error) => {
        console.log('error ', error);
        res.status(500).json("Cannot retrieve the missions availables")
    });

}

export const getVehiclesAvailables = async (req, res) => {
    const urlVehicleManager = 'http://localhost:3001';
    try{
        const response = await axios({
            method: 'get',
            url: urlVehicleManager + '/api/v1/vehicles/checkAvailability',
            responseType: 'json'
        });

        if (response.status == 200){
            console.log(response.data);
            //Returns JSON Array with all the vehicles availables
            res.json(response.data);
        }

    }catch(err){
        console.error(err);
        if(axios.isAxiosError(err)){
            if(err.response) {
                //Request mae and server responded with error
                // The client was given an error response (5xx, 4xx)
                console.error(err.response);
            } else if (err.request){
                // Request was made but no response received
                // The client never received a response, and the request was never left
                console.error(err.request);
                res.status(503).json("Vehicle manager server time out")

            } else {
                //Something happended in setting up the request that triggered an Error
                console.error(err);

            }
        }
        

    } 
}

const _sendMissionToVehicleManager = async (req, res) => {
    //console.log(req);
    
} 

