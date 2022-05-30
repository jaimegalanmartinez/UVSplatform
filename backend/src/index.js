
//https://bravedeveloper.com/2021/03/22/crear-un-api-rest-con-nodejs-y-express-nunca-fue-tan-provechoso-y-sencillo/

const express = require('express');
const app = express();
const morgan = require ('morgan');

//Setup
app.set('port', process.env.PORT || 3000);
app.set('json spaces', 2);

//Middleware
app.use(morgan('dev'));
app.use(express.urlencoded({extended:false}));
app.use(express.json());

//Routes
app.use(require('./routes/index'));

//Starting server, listening on port 3000 ...
app.listen(app.get('port'), () => {
    console.log(`HTTP server listening on port ${app.get('port')}`);
});