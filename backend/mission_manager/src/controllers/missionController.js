import Mission from "../models/Mission.js"
import axios from 'axios';

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
    res.json(
        {
            "Title": "UVS Platform"
        }
    );

    //Send mission plan to Vehicle Manager

}

export const abortMission = async (req, res) => {
    res.json(
        {
            "Title": "UVS Platform"
        }
    );

}

export const getMissionsStatus = async (req, res) => {
    res.json(
        {
            "Title": "UVS Platform"
        }
    );

}

export const getMissionsAvailables = async (req, res) => {
    res.json(
        {
            "Title": "UVS Platform"
        }
    );

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

    } 

}