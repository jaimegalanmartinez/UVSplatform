import config from './config.js';
import mongoose from 'mongoose';

/*const { MongoClient, ServerApiVersion } = require('mongodb');
const client = new MongoClient(config.databaseURI, { useNewUrlParser: true, useUnifiedTopology: true, serverApi: ServerApiVersion.v1 });
client.connect(err => {
  const collection = client.db("uvs-vehicles").collection("vehicles");
  // perform actions on the collection object
  client.close();
});
*/

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


