/**
 * Mission model - Mission Manager
 * @author: Jaime Galán Martínez
 */
import mongoose from "mongoose";
import * as MissionPlan from "../models/MissionPlan.js";

const missionSchema = new mongoose.Schema({
    mission_id: {
        type: String,
        required: true,
        trim: true
    },

    name: {
        type: String,
        required: true,
        trim: true,
    },

    fleet_id: {
        type: String,
        required: true,
        trim: true

    },

    vehicle_id: {
        type: String,
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
        default: 'requested'

    },

    startedAt: {
        type: Date,
    },

    endedAt: {
        type: Date,

    }
}, {
    versionKey: false,
    timestamps: true
});

export default mongoose.model('Missions', missionSchema);