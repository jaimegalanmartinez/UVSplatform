import { Router } from 'express';
import * as VehicleManager from '../controllers/vehiclesController.js'

const router = Router();

// Vehicles API

// Receive a mission from the Mission Manager, and send it to the specific fleet manager
router.post('/vehicles/receiveMission', VehicleManager.sendMissionToFleetManager);

//Check vehicles availability
router.get('/vehicles/checkAvailability', VehicleManager.checkAvailability);

// Abort a mission
//router.get('/vehicles/abortMission', VehicleManager.abortMission);

router.get('/vehicles/info/:id', VehicleManager.getVehicleInformation);

// Fleets API
router.post('/fleets/:id/updateFleetVehicles', VehicleManager.updateFleetVehicles);

export default router;