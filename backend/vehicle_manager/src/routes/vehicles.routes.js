import { Router } from 'express';
import * as VehicleManager from '../controllers/vehiclesController.js'

const router = Router();

//Root
router.get('/', (req, res) => {
    res.json(
        {
            "Title": "UVS API Platform"
        }
    );

});

// Vehicles API
// Request a mission
router.post('/missions/requestMission', VehicleManager.requestMission);

// Abort a mission
router.post('/missions/abortMission', VehicleManager.abortMission);


export default router;