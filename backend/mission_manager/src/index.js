import app from './app.js';
import './database.js';

//Starting server, listening on port 3000 ...
const server = app.listen(app.get('port'), () => {
    console.log(`HTTP server listening on port ${app.get('port')}`);
});

var gracefulExit = () => { 
    database.connection.close( () => {
      console.log('Mongoose default connection with DB is disconnected through app termination');
      console.log('Disconnected from CosmosDB');
      process.exit(0);
    });
  }

process.on('SIGINT', function(){
    server.close((err) => {  
        console.log('Mission manager closed')
        process.exit(err ? 1 : 0)
    });
});
