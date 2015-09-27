var b = require('bonescript');
var math = require('mathjs');

setMotor = (function(){
    
    function setMotor(){
        this.M1In1 = "P9_11";
        this.M1In2 = "P9_12";
        this.M2In1 = "P9_17";
        this.M2In2 = "P9_18";
        this.Enable1 = "P9_14";
        this.Enable2 = "P9_16";
        
        b.pinMode(this.M1In1, b.OUTPUT);
        b.pinMode(this.M1In2, b.OUTPUT);
        b.pinMode(this.M2In1, b.OUTPUT);
        b.pinMode(this.M2In2, b.OUTPUT);
        b.pinMode(this.Enable1, b.OUTPUT);
        b.pinMode(this.Enable2, b.OUTPUT);
        
        b.digitalWrite(this.M1In1, b.HIGH);
        b.digitalWrite(this.M1In2, b.HIGH);
        b.digitalWrite(this.M2In1, b.HIGH);
        b.digitalWrite(this.M2In2, b.HIGH);
        b.digitalWrite(this.Enable1, b.HIGH);
        b.digitalWrite(this.Enable2, b.HIGH);
        
        b.analogWrite(this.Enable1, b.LOW);
        b.analogWrite(this.Enable2, b.LOW);
    }
    
    setMotor.prototype.update = function(correction){
        this.eXX = math.abs(correction/650); 
        this.FeedIn = 0.9 + 0.1 * this.eXX;
        
        if(this.FeedIn > 1){
            this.FeedIn = 1;
        } else if(this.FeedIn < 0){
            this.FeedIn = 0;
        }
        
        if(correction < -5){
            this.runMotor(1, 0, this.FeedIn);
        } else if(correction > 5){
            this.runMotor(0, 1, this.FeedIn);
        }
    }
    
    setMotor.prototype.runMotor = function(In1, In2, PWM){
        b.analogWrite(this.Enable1, 0);
        b.analogWrite(this.Enable2, 0);
        
        b.digitalWrite(this.M1In1, In1);
        b.digitalWrite(this.M1In2, In2);
        b.digitalWrite(this.M2In1, In1);
        b.digitalWrite(this.M2In2, In2);
        
        b.analogWrite(this.Enable1, PWM);
        b.analogWrite(this.Enable2, PWM);
    }
    
    return setMotor;
})();

exports.setMotor;