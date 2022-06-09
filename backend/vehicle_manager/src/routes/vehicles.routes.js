import { Router } from 'express';
import * as VehicleManager from '../controllers/vehiclesController.js'

const router = Router();

// Vehicles API
// Request a mission
router.get('/vehicles/checkAvailability', VehicleManager.checkAvailability);

// Abort a mission
router.post('/vehicles/abortMission', VehicleManager.abortMission);

router.get('/vehicles/info/:id', VehicleManager.getVehicleInformation);

// Fleets API
router.post('/fleets/:id/updateFleetVehicles', VehicleManager.updateFleetVehicles);

export default router;