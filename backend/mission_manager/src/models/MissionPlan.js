import mongoose from "mongoose";

const missionCommandSchema = new mongoose.Schema({
    name: {
      type: String,
      required: true,
      trim: true  
    },

    description: {
        type: String,
        required: true,
        maxlength: 400
    },
},{
    versionKey: false,
    timestamps: false,
});


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

export default mongoose.model('MissionsPlans', missionPlanSchema);