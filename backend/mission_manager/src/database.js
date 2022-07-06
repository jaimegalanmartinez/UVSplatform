/**
 * Mission Manager - Connect to mongo DB ATLAS using mongoose at server startup.
 * @author: Jaime Galán Martínez
 */
import mongoose from 'mongoose';
import config from './config.js';

let connectToDB;

(connectToDB = async () => {
    const db = await mongoose.connect(config.databaseURI, {
        tls: true,
        useNewUrlParser: true,
        useUnifiedTopology: true,
        retryWrites: false
    }).then((data) => {
        console.log('Connection to UVS-missions successful')
        connectToDB = data;
        return data;
    }).catch((err) => console.log(err));

})();

mongoose.connection.on('connected', () => {
    console.log('Mongoose is connected to ATLAS')
});
