import mongoose from "mongoose";
import * as MissionPlan from "../models/MissionPlan.js";

const missionSchema = new mongoose.Schema({
    mission_id: {
        type: Number,
        required: true,
        trim: true
    },

    name: {
        type: String,
        required: true,
        trim: true,
    },

    fleet_id: {
        type: Number,
        required: true,
        trim: true

    },

    vehicle_id: {
        type: Number,
        required: true,
        trim: true

    },

    mission_plan: {
        type: MissionPlan.missionPlanSchema,
        required: true
    },

    mission_status: {
        type: String,
        required: true,
        trim: true,
        default: "Requested"

    },

    startedAt: {
        type: Date,
        required: true,

    },

    endedAt: {
        type: Date,

    }
}, {
    versionKey: false,
    timestamps: true
});

export default mongoose.model('Missions', missionSchema);