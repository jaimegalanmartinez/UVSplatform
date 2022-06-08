import { config } from "dotenv";
import { URL } from 'url';

//https://stackoverflow.com/questions/46745014/alternative-for-dirname-in-node-js-when-using-es6-modules
const __dirname = new URL('.', import.meta.url).pathname;
//Load environment variables defined
config({path:__dirname + '/.env'});

export default {
    databaseURI: "mongodb://"+ process.env.COSMOSDB_USER + ":" + process.env.COSMOSDB_PASSWORD + "@"+ process.env.COSMOSDB_HOST + ":" + 
                process.env.COSMOSDB_PORT + "/"+process.env.COSMOSDB_DBNAME+"?ssl=true&retrywrites=false&maxIdleTimeMS=120000&appName=@" + process.env.COSMOSDB_USER + "@",
    fbAdminPrivateKey: process.env.FIREBASE_ADMIN_PRIVATE_KEY,
    fbAdminClientEmail: process.env.FIREBASE_ADMIN_CLIENT_EMAIL,
    fbAdminProjectId: process.env.FIREBASE_ADMIN_PROJECT_ID
};