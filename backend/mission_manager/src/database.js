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
        console.log('Connection to CosmosDB successful')
        connectToDB = data;
        return data;
    }).catch((err) => console.log(err));

})();

/*console.log(connectToDB);
export default {
    database: connectToDB.success()
}*/