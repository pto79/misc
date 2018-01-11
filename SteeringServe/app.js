var express = require('express'),
    fs = require('fs');
    //url = require('url');
var app = express();

//app.use('/public', express.static(__dirname + '/public'));  
//app.use(express.static(__dirname + '/public')); 

app.get('/', function (req, res) {
  res.send('Serve works fine')
})

app.post('/receive', function(request, respond) {
    
    var body = '';
    filePath = __dirname + '/public/data.txt';
    request.on('data', function(data) {
        body += data;
    });

    request.on('end', function (){
        fs.writeFile(filePath, body, function() {
            respond.end();
        });
    });
    

/*
  var writeStream = fs.createWriteStream('./output');

  // This pipes the POST data to the file
  request.pipe(writeStream);

  // After all the data is saved, respond with a simple html form so they can post more data
  request.on('end', function () {
    //res.writeHead(200, {"content-type":"text/html"});
    //res.end('<form method="POST"><input name="test" /><input type="submit"></form>');
    respond.end();
  });

  // This is here incase any errors occur
  writeStream.on('error', function (err) {
    console.log(err);
  });
*/

});


app.listen(3000, function () {
  console.log('Example app listening on port 3000!')
})