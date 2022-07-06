/**
 * Mission Manager - Config file for environment variables (Firebase and ATLAS DB credentials).
 * @author: Jaime Galán Martínez
 */
import { config } from "dotenv";
import { URL } from 'url';

//https://stackoverflow.com/questions/46745014/alternative-for-dirname-in-node-js-when-using-es6-modules
const __dirname = new URL('.', import.meta.url).pathname;
//Load environment variables defined
config({path:__dirname + '/.env'});

export default {
    databaseURI: "mongodb+srv://" + process.env.ATLASDB_USER + ":" + process.env.ATLASDB_PASSWORD + "@uvs-cluster.am9a2cl.mongodb.net/uvs-missions?retryWrites=true&w=majority",
    fbAdminPrivateKey: process.env.FIREBASE_ADMIN_PRIVATE_KEY,
    fbAdminClientEmail: process.env.FIREBASE_ADMIN_CLIENT_EMAIL,
    fbAdminProjectId: process.env.FIREBASE_ADMIN_PROJECT_ID
};