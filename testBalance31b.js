/* Note: 10/09/2015
-Overcompensation while balancing
-Slow response from motors/ Slow reading from sensors

-Alternative flow of voltage into motor, indexMotorFl
--Smooth transactions
--Still not balancing

-Include data storage: 11/09/2015
--Less responsive to sensor after fs stream was added
*/

//3-axis
global.XAXIS = 0;
global.YAXIS = 1;
global.ZAXIS = 2;

//Set sensors
var ITG3205 = require('./index/indexITG.js');
var ADXL345 = require('./index/indexADXL.js');

//Set PID controller
var Controller = require('node-pid-controller');
var ctr = new Controller(7, 0, 3);
var correction = 0;
ctr.setTarget(0);

//Set Motor
var IM = require('./index/indexMotorFl.js');
var SM = new setMotor();

//Set Kalman
var math = require('mathjs');
var Kal = require('./index/indexKalman.js');
var KM = new KalmanModel();
var KO = new KalmanObservation();

//Set fs for data storage;
var fs = require('fs');
var kstream = fs.createWriteStream("data/kalmanValue.txt");
var astream = fs.createWriteStream("data/accelValue.txt");
var gstream = fs.createWriteStream("data/gyroValue.txt");
var fstream = fs.createWriteStream("data/feedValue.txt");

//Set variables
var gyroAngle = 0;
var accelAngle = 0;
var angleKalman = 0;
var angleComp = 0;
var gyroBias = 0;
var accelBias = 0;
var tick = 0;
var corr_p = 0;
var slope = 0;

//Set gyroscope variables
var gyro = new ITG3205(function(err){
    gyro.gyroScaleFactor[global.XAXIS] = 0.006779661016;
    gyro.gyroScaleFactor[global.YAXIS] = -0.006779661016;
    gyro.gyroScaleFactor[global.ZAXIS] = -0.06779661016;
    if(!err){
        computeGyroBias();
    } else{
        console.log(err);
    }
})

//Set accelerometer variables
var accel = new ADXL345(function(err){
    accel.accelScaleFactor[global.XAXIS] = 0.0371299982;
    accel.accelScaleFactor[global.YAXIS] = -0.0374319982;
    accel.accelScaleFactor[global.ZAXIS] = -0.0385979986;
    if(!err){
        computeAccelBias();
    } else {
        console.log(err);
    }
})

//Compute Bias
function computeGyroBias(){
    gyro.computeGyroBias(function(){
        gyroBias = gyro.runTimeGyroBias[global.XAXIS];
    })
}

function computeAccelBias(){
    accel.computeAccelBias(function(){
        accelBias = accel.runTimeAccelBias[global.XAXIS]
    })
}

//Kalman Filter
function computeKalman(){
    KM.u_k = gyroAngle;
    KO.z_k = accelAngle;
    KM.update(KO);
    angleKalman = KM.x_k.subset(math.index(0,0));
}

measureVal();

//Main Loop
function measureVal(){
    setInterval(function(){
        gyro.measureGyro(function(err) {
            if(!err){
                tick += 1;
                gyroAngle =(0.95 * gyroAngle) + (0.05 * gyro.gyroAngle[global.XAXIS]) - slope;
                slope += gyroAngle / tick;
                
            } else{
                console.log(err);
            }
        })
        accel.measureAccel(function(err){
            if(!err){
                if(accel.accelAngle[global.XAXIS] < 0){
                    corr_p = 0.5;
                } else{
                    corr_p = 1;
                }
                accelAngle =(0.95 * accelAngle) - (0.05 * corr_p * accel.accelAngle[global.XAXIS]);
            } else {
                console.log(err);
            }
        })
            
            computeKalman();
            angleComp = (((angleComp + gyroAngle) * (0.7)) + ((accelAngle) * 0.3))/1.45;
            
            correction = ctr.update(angleKalman);
            SM.update(correction);
            console.log('|Kal: ' + angleKalman + '|         |Gy: ' + gyroAngle + '|         |Acc: ' + accelAngle + '|       |PID: ' + correction + '|       |Fd: ' + SM.FeedIn + '|');
            
            kstream.write(angleKalman + '\n');
            astream.write(accelAngle + '\n');
            gstream.write(gyroAngle + '\n');
            fstream.write(SM.FeedIn + '\n');
    }, 10);
}