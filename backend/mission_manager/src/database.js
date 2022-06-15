import mongoose from 'mongoose';
import config from './config.js';

let connectToDB;

(connectToDB = async () => {
    console.log(config.databaseURI);
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

/*console.log(connectToDB);
export default {
    database: connectToDB.success()
}*/