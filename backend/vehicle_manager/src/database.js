import config from './config.js';
import mongoose from 'mongoose';

(async () => {
    console.log(config.databaseURI);
    const db = await mongoose.connect(config.databaseURI, {
        tls: true,
        useNewUrlParser: true,
        useUnifiedTopology: true,
        retryWrites: false,
    }).then((data) => {
        console.log('Connection to UVS-VEHICLES DB successful')
        return data;
    }).catch((err) => console.log(err));

})();

mongoose.connection.on('connected', () => {
    console.log('Mongoose is connected to ATLAS')
});


