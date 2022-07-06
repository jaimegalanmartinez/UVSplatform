/**
 * Mission Manager - HTTP Server with expressjs server.
 * @author: Jaime Galán Martínez
 */
//https://bravedeveloper.com/2021/03/22/crear-un-api-rest-con-nodejs-y-express-nunca-fue-tan-provechoso-y-sencillo/
//https://docs.microsoft.com/en-us/azure/cosmos-db/mongodb/connect-using-mongoose

import express, { urlencoded, json } from 'express';
import morgan from 'morgan';
import MissionsRoutes from './routes/missions.routes.js';

const app = express();

//Setup
app.set('port', process.env.PORT || 3000);
app.set('json spaces', 2);

//Middleware
app.use(morgan('dev'));
app.use(urlencoded({ extended: false }));
app.use(json());

//Routes
app.use('/api/v1', MissionsRoutes);

export default app;