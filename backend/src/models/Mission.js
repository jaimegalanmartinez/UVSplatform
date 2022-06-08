import mongoose from "mongoose";

const missionCommand = new mongoose.Schema({
    name: {
      type: String,
      required: true,
      trim: true  
    },

    missionOrder: {
        type: Number,
        required: true,

    },

});

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

    mission_commands: {
        type: [missionCommand]
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