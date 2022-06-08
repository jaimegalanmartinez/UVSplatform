import Vehicle from "../models/Vehicle.js"
import Fleet from "../models/Fleet.js"

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

}

export const abortMission = async (req, res) => {
    res.json(
        {
            "Title": "UVS Platform"
        }
    );

}