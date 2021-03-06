/**
 * Vehicle Manager - Config file to setup the environment variables
 * @author: Jaime Galán Martínez
 */
import { config } from "dotenv";
//import { URL } from 'url';

//const __dirname = new URL('.', import.meta.url).pathname;
//console.log(__filename);
//Load environment variables defined
config({path: __dirname + '/.env'});

export default {
    databaseURI: "mongodb+srv://" + process.env.ATLASDB_USER + ":" + process.env.ATLASDB_PASSWORD + "@sandbox.jemvt.mongodb.net/uvs-vehicles?retryWrites=true&w=majority"
};

