import admin from 'firebase-admin';
import config from './config.js';

//Initialize Firebase Admin SDK
admin.initializeApp({
    credential: admin.credential.cert({
        private_key:
            config.fbAdminPrivateKey[0] === "-" ? config.fbAdminPrivateKey : JSON.parse(config.fbAdminPrivateKey),
        client_email: config.fbAdminClientEmail,
        project_id: config.fbAdminProjectId

    }),  
});

export const authenticateJWT = async (req, res, next) => {
    const authHeader = req.headers.authorization;
  
    if (authHeader) {
      const idToken = authHeader.split(" ")[1];
      admin.auth()
        .verifyIdToken(idToken)
        .then((decodedToken) => {
          const uid = decodedToken.uid;
          return next();
        })
        .catch( (error) => {
          console.log(error);
          return res.sendStatus(403);
        });
    } else {
      res.sendStatus(401);
    }
  };

