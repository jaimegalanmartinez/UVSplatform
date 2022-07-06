/**
 * Vehicle Manager - Fleet model
 * @author: Jaime Galán Martínez
 */
import mongoose from "mongoose";

const fleetSchema = new mongoose.Schema({

    fleet_name: {
        type: String,
        required: true,
        trim: true,

    },

    location: {
        type: String,
        required: true,

    },
    fleet_vehicles: {
        type: [ {
            type: mongoose.SchemaTypes.ObjectId, ref: 'Vehicles'
        }]
    }

}, {
    versionKey: false,
    timestamps: true,
    collection: "fleets"
});

export default mongoose.model('Fleets', fleetSchema);