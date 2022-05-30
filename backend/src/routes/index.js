const { Router } = require('express');
const router = Router();

//Root
router.get('/', (req, res) => {
    res.json(
        {
            "Title": "UVS Platform"
        }
    );

});

module.exports = router;