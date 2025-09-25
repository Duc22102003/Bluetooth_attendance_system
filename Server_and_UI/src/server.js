const express = require('express');
const cron = require('node-cron');
const sqlite3 = require('sqlite3').verbose();
const bodyParser = require('body-parser');
const app = express();
const path = require('path');
const port = 1602;
const host = "127.0.0.1";



app.use(bodyParser.urlencoded({ extended: true }));
app.use(bodyParser.json());
app.use(express.json());
app.use(express.static('public'));
app.set('view engine', 'ejs');
app.set('views', path.join(__dirname, 'views'));
app.use(express.static(path.join(__dirname, 'public')));

const DB_PATH = 'C:/sqlite/db/dulieudiemdanh.db'

// mở csdl qlite3


let db = new sqlite3.Database(DB_PATH, sqlite3.OPEN_READWRITE, (err) => {
  if (err) {
    console.error(err.message);
  }
  console.log('Connected to the dulieu.');
})


// Function to perform the task
function doTask() {

  // Perform the task: Delete all data from the 'flash' table
  db.run('DELETE FROM flash', (err) => {
      if (err) {
          console.error(err.message);
          db.close((err) => {
              if (err) {
                  console.error(err.message);
              }
              console.log('Closed the database connection due to an error.');
          });
          return;
      }
      console.log('All data from the "flash" table has been deleted.');
  });
}

// Initialize the cron job with the initial schedule
let cronExpression = '*/5 * * * *'; // Every 5 minutes
let scheduledTask = cron.schedule(cronExpression, doTask);

// Function to update the cron schedule
function updateCronSchedule(newCronExpression) {
  // Stop the current cron job
  if (scheduledTask) {
      scheduledTask.stop();
      console.log('Stopped the existing cron job.');
  }

  // Set up the new cron job with the new schedule
  cronExpression = newCronExpression;
  scheduledTask = cron.schedule(cronExpression, doTask);
  console.log(`Scheduled new cron job with expression: ${cronExpression}`);
}

app.post('/reSchedule', (req, res) => {
  let hour = req.body.hour;
  let minute = req.body.minute

  console.log(hour, minute);
  
  let reCronExpression = `*${minute == 1 ? '' : '/' + minute} *${hour == 1 ? '' : '/' + hour} * * *`;
  console.log('là ' + reCronExpression);
  updateCronSchedule(reCronExpression);

  db.serialize(() => {
      db.all("SELECT * FROM dulieu", (err, rows) => {
          if (err) {
              console.error(err.message);
              res.status(500).send(err.message);
          }
           res.render('thanhvien.ejs', {data: rows});
      });
  });

});

app.get('/schedule', (req, res) => {

  res.render('schedule.ejs');

})

app.get('/history', (req, res) => {

  db.serialize(() => {
    db.all("SELECT * FROM history", (err, rows) => {
        if (err) {
            console.error(err.message);
            res.status(500).send(err.message);
        }
         res.render('tra.ejs', {data: rows});
    });
  });

})

app.post('/searchMSSV', (req, res) => {
  let mssv = req.body.mssv;

  console.log(mssv);

  db.serialize(() => {
    db.all("SELECT * FROM history WHERE MSSV = ?", [mssv], (err, rows) => {
        if (err) {
            console.error(err.message);
            res.status(500).send(err.message);
        }
         res.render('tra.ejs', {data: rows});
    });
  });

})

app.post('/searchDate', (req, res) => {

  let date = req.body.date;

  console.log(date);

  db.serialize(() => {
    db.all("SELECT * FROM history WHERE date = ?", [date], (err, rows) => {
        if (err) {
            console.error(err.message);
            res.status(500).send(err.message);
        }
         res.render('searchDate.ejs', {defaultDate: date, data: rows});
    });
  });
})

app.post('/register', (req, res) => {
  let name = req.body.name;
  let MSSV = req.body.MSSV;
  let mac_address = req.body.mac_address;
  
  // Tiếp theo, bạn có thể sử dụng thông tin người dùng đã nhập để lưu vào cơ sở dữ liệu hoặc thực hiện một số thao tác khác.

  console.log(name, mac_address);
  res.send("Đăng ký thành công!");

  let datadulieu = [name, MSSV, mac_address];
  let sqlInsertdulieu = "INSERT INTO dulieu (Ten,MSSV,Addr_Bluetooth) VALUES (?,?,?);"

  db.run(sqlInsertdulieu, datadulieu, function (err) {
    if (err) {
      return console.error(err.message);
    };
    console.log(`Row inserted ${this.changes}`)
  });
});


app.get('/dangki', function (req, res) {
  //Gửi file HTML trong thư mục "public"
  res.sendFile(__dirname + '/public/dangki.html');
  

});



app.get('/danhsach', (req, res) => {

  db.serialize(() => {
      db.all("SELECT * FROM dulieu", (err, rows) => {
          if (err) {
              console.error(err.message);
              res.status(500).send(err.message);
          }
           res.render('thanhvien.ejs', {data: rows});
      });
  });

});



app.get('/danhsachdiemdanh', (req, res) => {

  db.serialize(() => {
      db.all("SELECT * FROM output", (err, rows) => {
          if (err) {
              console.error(err.message);
              res.status(500).send(err.message);
          }
           res.render('output.ejs', {data: rows});
      });
  });
});



function getTime()
        {
          const now = new Date();
          const hours = now.getHours().toString().padStart(2, '0');
          const minutes = now.getMinutes().toString().padStart(2, '0');
          const seconds = now.getSeconds().toString().padStart(2, '0');

                return `${hours}:${minutes}:${seconds}`;
        }

function getDay()
        {
          const today = new Date();
          const year = today.getFullYear();
          const month = (today.getMonth() + 1).toString().padStart(2, '0'); // Tháng tính từ 0
          const day = today.getDate().toString().padStart(2, '0');

                return `${day}-${month}-${year}`;
        }

        function checkAddr(mac, nameTable) {
          return new Promise((resolve, reject) => {
              const query = `SELECT * FROM ${nameTable} WHERE Addr_Bluetooth = ?;`;
              db.get(query, [mac], (err, row) => {
                  if (err) {
                      console.error(err.message);
                      return reject(err);
                  }
                  if (row) {
                      console.log(row);
                      resolve(row);
                  } else {
                      console.log(`No record found with the MAC: ${mac}`);
                      resolve(null);
                  }
              });
          });
      }
      
      function saveAddr(mac, nameTable) {
          return new Promise((resolve, reject) => {
              const query = `INSERT INTO ${nameTable} (Addr_Bluetooth) VALUES (?);`;
              db.run(query, [mac], function(err) {
                  if (err) {
                      console.error(err.message);
                      return reject(err);
                  }
                  resolve();
              });
          });
      }
      
        

      app.post("/fromesp", async (req, res) => {
        let datasend = "";
        let lcd = [];
        let i = 0;
        console.log(req.body); // log ra thông tin nhận được
    
        try {
            for (let key in req.body) { // duyệt qua từng key trong body
                if (req.body[key] != '') {
                    let Mac = req.body[key];
                    
                    // Kiểm tra và lưu địa chỉ MAC trong bảng 'flash'
                    let flashRow = await checkAddr(Mac, 'flash');
                    if (!flashRow) {
                        await saveAddr(Mac, 'flash');
                    
    
                    // Kiểm tra địa chỉ MAC trong bảng 'dulieu'
                    let dulieuRow = await checkAddr(Mac, 'dulieu');
                    if (dulieuRow) {
                        let query = `INSERT INTO history (Name, MSSV, Addr_Bluetooth, time, date) VALUES (?,?,?,?,?);`;
                        let data = [dulieuRow.Name, dulieuRow.MSSV, dulieuRow.Addr_Bluetooth, getTime(), getDay()];
                        lcd[i] = dulieuRow.MSSV;
                        console.log(lcd[i]);
                        i++;
    
                        await new Promise((resolve, reject) => {
                            db.run(query, data, function(err) {
                                if (err) {
                                    console.error(err.message);
                                    return reject(err);
                                }
                                resolve();
                            });
                        });
                    }
                  }
                }
            }
    
            for (let n = 0; n < i; n++) {
                datasend = datasend + lcd[n] + ",";
            }
            console.log(datasend);
    
            res.send(`!${datasend}`);
            i = 0;
        } catch (err) {
            console.error(err);
            res.status(500).json({ error: "Internal Server Error" });
        }
    });
    app.post('/Nowifi',(req,res)=>
    {
      console.log(req.body);
      
    })




app.listen(port, host, () => {
  console.log(`App listening on IP ${host} and port ${port}`);
});