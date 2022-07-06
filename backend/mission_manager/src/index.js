/**
 * Mission Manager 
 * @author: Jaime Galán Martínez
 */
import app from './app.js';
import './database.js';

//Starting server, listening on port 3000 ...
const server = app.listen(app.get('port'), () => {
    console.log(`HTTP server listening on port ${app.get('port')}`);
});

process.on('SIGINT', function(){
    server.close((err) => {  
        console.log('Mission manager closed')
        process.exit(err ? 1 : 0)
    });
});
