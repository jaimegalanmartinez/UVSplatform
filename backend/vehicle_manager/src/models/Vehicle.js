import mongoose from "mongoose";

const vehicleSchema = new mongoose.Schema({
    fleet_id: {
        type: Number,
        required: true,
        trim: true
    },

    vehicle_id: {
        type: Number,
        required: true,
        trim: true,
    },

    vehicle_type: {
        type: String,
        required: true,
        trim: true

    },

    vehicle_status: {
        type: String,
        required: true,
        trim: true

    },

    vehicle_name: {
        type: String,
        required: true,
        trim: true,

    },

    battery: {
        type: Number,
        required: true,

    },

}, {
    versionKey: false,
    timestamps: true
});

export default mongoose.model('Vehicles', vehicleSchema);