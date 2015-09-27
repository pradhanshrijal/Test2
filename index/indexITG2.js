var i2c = require('i2c');
var async = require('async');
var address = 0x68;

/**
 *
 * @param {Object} vars
 * @param {Object} callback
 */

function ITG3205(callback)
{
	this.gyroSAMPLECOUNT = 400;
 	this.gyroScaleFactor = [0.0,0.0,0.0];
	this.runTimeGyroBias = [0, 0, 0];
	this.gyroOneG = 0.0;
	this.thetaPerSec = [0.0,0.0,0.0];
	this.gyroSample = [0,0,0];
	this.gyroSampleCount = 0;
	this.gyroAngle = [0.0,0.0,0.0];
	this.gyroDt = 0.005;
	this.gyroD_TO_G = 57.2957795131; 
	//init stuff here
	this.wire = new i2c(address, {
		device : '/dev/i2c-1'
	});
	
	var self = this;
	
	async.waterfall([
	function(cb) {
		self.wire.writeBytes(0x3E, 0x00, function(err) {
			cb(err);
		});
	},
	function(cb) {
		self.wire.writeBytes(0x15, 0x0D, function(err) {
			cb(err);
		});
	},
	function(cb) {
		self.wire.writeBytes(0x16, 0x14, function(err) {
			cb(err);
		});
	},
	function(cb) {
		self.wire.writeBytes(0x17, 0x00, function(err) {
			cb(err);
		});
	}], function(err) {
		if (!err) {
			setTimeout(function() {
				callback(null);
			}, 10);
		} else {
			callback(err);
		}
	});
}
ITG3205.prototype.measureGyro = function(callback) {

	var self = this;

	this.wire.readBytes(0x1D, 6, function(err, res) {
		if (!err) {
			for (var axis = XAXIS; axis <= ZAXIS; axis++) {
				self.thetaPerSec[axis] = res.readInt16LE(axis*2) * self.gyroScaleFactor[axis] + self.runTimeGyroBias[axis];
				self.gyroAngle[axis] = (self.thetaPerSec[axis]) * self.gyroDt * self.gyroD_TO_G;
			}
			callback(null);
		} else {
			callback(err);
		}
	});

}

ITG3205.prototype.measureGyroSum = function(callback) {
	//get values from sensor here
	var self = this;
	this.wire.readBytes(0x1D, 6, function(err, res) {
		if (!err) {
			for (var axis = XAXIS; axis <= ZAXIS; axis++) {
				self.gyroSample[axis] += res.readInt16LE(axis*2)
			}
			self.gyroSampleCount++;
			callback(null);
		} else {
			callback(err);
		}
	});

}

ITG3205.prototype.evaluateThetaPerSec = function() {
	for (var axis = XAXIS; axis <= ZAXIS; axis++) {
		this.thetaPerSec[axis] = (this.gyroSample[axis] / this.gyroSampleCount) * this.gyroScaleFactor[axis] + this.runTimeGyroBias[axis];
		this.gyroSample[axis] = 0;
	}
	this.gyroSampleCount = 0;
}

ITG3205.prototype.computeGyroBias = function(callback) {
	var self = this;
	function getSamples() {
		if (self.gyroSampleCount < self.gyroSAMPLECOUNT) {
			self.measureGyroSum(function() {
				setTimeout(getSamples, 2.5);
			});
		} else {
			for (var axis = 0; axis < 3; axis++) {
				self.thetaPerSec[axis] = (self.gyroSample[axis] / self.gyroSAMPLECOUNT) * self.gyroScaleFactor[axis];
				self.gyroSample[axis] = 0;
			}
			self.gyroSampleCount = 0;

			self.runTimeGyroBias[XAXIS] = -self.thetaPerSec[XAXIS];
			self.runTimeGyroBias[YAXIS] = -self.thetaPerSec[YAXIS];
			self.runTimeGyroBias[ZAXIS] = -9.8065 - self.thetaPerSec[ZAXIS];

			self.gyroOneG = Math.abs(self.thetaPerSec[ZAXIS] + self.runTimeGyroBias[ZAXIS]);
			callback();
		}
	}

	getSamples();
}

module.exports = ITG3205;
