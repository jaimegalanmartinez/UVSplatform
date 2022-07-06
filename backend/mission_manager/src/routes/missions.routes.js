/**
 * Missions API routes - Mission Manager
 * @author: Jaime Galán Martínez
 */
import { Router } from 'express';
import * as MissionManager from '../controllers/missionController.js'
import * as FirebaseMiddleware from '../firebase_middleware.js';

const router = Router();

// https://www.tonyvu.co/posts/jwt-authentication-node-js
// https://www.xmatters.com/blog/blog-four-rest-api-versioning-strategies/

//Root
router.get('/', (req, res) => {
    res.json(
        {
            "Title": "UVS API Platform"
        }
    );

});

// Missions API
// Request a mission
router.post('/missions/requestMission', FirebaseMiddleware.authenticateJWT, MissionManager.requestMission);

// Get missions availables for an unmanned vehicle
router.get('/missions/missionsAvailables/:typeVehicle', FirebaseMiddleware.authenticateJWT, MissionManager.getMissionsAvailables);

// Get vehicles availables for requesting a mission
router.get('/missions/vehiclesAvailables', FirebaseMiddleware.authenticateJWT, MissionManager.getVehiclesAvailables);

export default router;