/**
 * Vehicle Manager - Vehicles API
 * @author: Jaime Galán Martínez
 */
import { Router } from 'express';
import * as VehicleManager from '../controllers/vehiclesController.js'

const router = Router();

// Vehicles API

// Receive a mission from the Mission Manager, and send it to the specific fleet manager
router.post('/vehicles/receiveMission', VehicleManager.sendMissionToFleetManager);

//Check vehicles availability
router.get('/vehicles/checkAvailability', VehicleManager.checkAvailability);

export default router;