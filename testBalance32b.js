/* Note: 10/09/2015
-Overcompensation while balancing
-Slow response from motors/ Slow reading from sensors

-Add switch: 12/09/2015
--Control motor function
--Not sure wheather it reduces response time of motor 

-Use indexMotorFl
--Not balancing
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

//Set Switch
var b = require('bonescript');
var Switch = "P9_21";
b.pinMode(Switch, b.INPUT);
b.digitalWrite(Switch, b.HIGH);

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
            
            b.digitalRead(Switch, state);
            
            function state(x){
                if(x.value==b.LOW){           
                    SM.update(correction);
                } else if(x.value==b.HIGH){
                    SM.runMotor(0, 0, 0);
                }    
            }
            
            console.log('|Kal: ' + angleKalman + '|     |Gy: ' + gyroAngle + '|     |Acc: ' + accelAngle   + '|');
    }, 10);
}


