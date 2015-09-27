// === Kalman ===
// Kalman filter for Javascript
// Copyright (c) 2012 Itamar Weiss
// 
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
var math = require('mathjs');

var Kalman = {
  version: '0.0.1'
};

KalmanModel = (function(){

  function KalmanModel(){
    this.x_k = math.matrix([[0],[0]]);
    this.u_k = math.matrix([[0]]);
    this.P_k = math.matrix([[0, 0], [0, 0]]);
    this.F_k = math.matrix([[1, -0.01], [0, 1]]);
    this.B_k = math.matrix([[1],[0]]);
    this.Q_k = math.matrix([[0.001, 0], [0, 0.003]]);
    this.d_T = 0.01;
  }
  
  KalmanModel.prototype.update =  function(o){
    this.I = math.eye(this.P_k.size()[0]);
    //init
    this.x_k_ = this.x_k;
    this.P_k_ = this.P_k;

    //Predict
    this.x_k_k_ = math.add(math.multiply(this.F_k, this.x_k_), math.multiply(this.B_k, this.u_k));
    this.P_k_k_ = math.add(math.multiply(this.F_k, math.multiply(this.P_k_, math.transpose(this.F_k))), this.Q_k);

    //update
    this.y_k = math.subtract(o.z_k, math.multiply(o.H_k, this.x_k_k_));//observation residual
    this.S_k = math.add(math.multiply(o.H_k, math.multiply(this.P_k_k_, math.transpose(o.H_k))), o.R_k);//residual covariance
    this.K_k = math.multiply(this.P_k_k_, math.multiply(math.transpose(o.H_k), math.inv(this.S_k)));//Optimal Kalman gain
    this.x_k = math.add(this.x_k_k_, math.multiply(this.K_k, this.y_k));
    this.P_k = math.multiply(math.subtract(this.I, math.multiply(this.K_k, o.H_k)), this.P_k_k_);
  }
  
  return KalmanModel;
})();

KalmanObservation = (function(){

  function KalmanObservation(){
    this.z_k = math.matrix([[0]]);//observation
    this.H_k = math.matrix([[1,0]]);//observation model
    this.R_k = math.matrix([[0.03]]);//observation noise covariance
  }
  
  return KalmanObservation;
})();
exports.KalmanModel;
exports.KalmanObservation;