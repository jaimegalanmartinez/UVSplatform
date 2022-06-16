import app from './app.js';
import './database.js';
import './fleets_interface.js'


//Starting server, listening on port 3001 ...
const server = app.listen(app.get('port'), () => {
    console.log(`HTTP server listening on port ${app.get('port')}`);
});