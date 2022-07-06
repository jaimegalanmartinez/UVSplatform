/**
 * Vehicle Manager - HTTP server using expressjs
 * @author: Jaime Galán Martínez
 */
import express, { urlencoded, json } from 'express';
import morgan from 'morgan';
import VehiclesRoutes from './routes/vehicles.routes.js';

const app = express();

//Setup
app.set('port', process.env.PORT || 3001);
app.set('json spaces', 2);

//Middleware
app.use(morgan('dev'));
app.use(urlencoded({ extended: false }));
app.use(json());

//Routes
app.use('/api/v1', VehiclesRoutes);

export default app;