/**
 * Visual Blocks Language
 *
 * Copyright 2012 Fred Lin.
 * https://github.com/gasolin/BlocklyDuino
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @fileoverview Helper functions for generating Arduino blocks.
 * @author gasolin@gmail.com (Fred Lin)
 */
'use strict';

goog.provide('Blockly.Arduino.pinoo');

goog.require('Blockly.Arduino');

var isLiveMode = false;

Blockly.Arduino['Robotistan_Start'] = function(block) {
  // TODO: Assemble JavaScript into code variable.
  var code = "";

  return code;
}

Blockly.Arduino['clearScreen'] = function(block) {

    var code = 'oled.fill(0)\n';

    return code;
};

Blockly.Arduino['showScreen'] = function(block) {

    var code = 'oled.show()\n';

    return code;
};

Blockly.Arduino['print'] = function(block) {

    var writeValue =  Blockly.Arduino.valueToCode(block, 'WriteValue', Blockly.Arduino.ORDER_NONE);

    var code = "";

    code += 'print(' + writeValue + ')\n';

    return code;
};

Blockly.Arduino['setDigitalPinValue'] = function(block) {

    var pin =  Blockly.Arduino.valueToCode(block, 'PIN', Blockly.Arduino.ORDER_NONE) || '0';
    var value = block.getFieldValue('VALUE');

    var code = '';

    Blockly.Arduino.imports_['import_machine'] = "import machine";
    Blockly.Arduino.definitions_['define_digital_pin' + pin] ='digital_pin_' + pin + ' = machine.Pin(' + pin + ', machine.Pin.OUT)';   

    code += 'digital_pin_' + pin + '.value(' + value + ')\n';

    return code;
};

Blockly.Arduino['readDigitalPinValue'] = function(block) {
 
    var pin =  Blockly.Arduino.valueToCode(block, 'PIN', Blockly.Arduino.ORDER_NONE) || '0';
    var code = '';

    Blockly.Arduino.imports_['import_machine'] = "import machine";
    Blockly.Arduino.definitions_['define_digital_pin' + pin] = 'digital_pin_' + pin + ' = machine.Pin(' + pin + ', machine.Pin.IN)';  
    
    code = 'digital_pin_' + pin + '.value()';
    return [code, Blockly.Arduino.ORDER_NONE];  
};

Blockly.Arduino['setAnalogPinValue'] = function(block) {

    var pin =  block.getFieldValue('PIN');
    var freq = Blockly.Arduino.valueToCode(block, 'FREQUENCY', Blockly.Arduino.ORDER_NONE) || '0';
    var duty = Blockly.Arduino.valueToCode(block, 'DUTY', Blockly.Arduino.ORDER_NONE) || '0';

    var code = '';

    Blockly.Arduino.imports_['import_Pin'] = "from machine import Pin";
    Blockly.Arduino.imports_['import_PWM'] = "from machine import PWM";

    Blockly.Arduino.definitions_['define_analog_pin' + pin] = 'analog_pin_' + pin + ' = Pin(' + pin +')';   
    Blockly.Arduino.definitions_['define_pwm' + pin] = 'pwm_' + pin + ' = PWM(analog_pin_' + pin + ')';   
    Blockly.Arduino.definitions_['define_pwmFreq' + pin] = 'pwm_' + pin + '.freq(1000)';   

    code += 'pwm_' + pin + '.duty_u16((' + freq + ' * 65535) / ' + duty + ')';

    return code;
};

Blockly.Arduino['readAnalogPinValue'] = function(block) {
 
    var pin =  block.getFieldValue('PIN');
    var code = '';

    Blockly.Arduino.imports_['import_machine'] = "import machine";
    Blockly.Arduino.imports_['import_ADC']  = "from machine import ADC";

    Blockly.Arduino.definitions_['define_adc' + pin] = 'adc_' + pin + ' = machine.ADC(' + pin + ')';  
    
    code = 'adc_' + pin + '.read_u16()';
    return [code, Blockly.Arduino.ORDER_NONE];  
};

Blockly.Arduino['playBuzzer'] = function(block) {

    var frequency =  Blockly.Arduino.valueToCode(block, 'FREQUENCY', Blockly.Arduino.ORDER_NONE) || '0';

    var code = "";
    var pin = BuzzerPin;

    Blockly.Arduino.imports_['import_Pin'] = 'from machine import Pin';
    Blockly.Arduino.imports_['import_PWM'] = 'from machine import PWM';
    Blockly.Arduino.definitions_['define_buzzer1'] = 'buzzer = PWM(Pin(' + pin + '))\n';
    code = 
            'buzzer.freq(' + frequency + ')\n' +
            'buzzer.duty_u16(100)\n';

    return code;
};

Blockly.Arduino['buzzerInterval'] = function(block) {

    var noteTime = block.getFieldValue('INTERVAL');
    var code = "";
    var interval = 0;

    if(noteTime == "1")
        interval = 0.250;
    else if(noteTime == "2")
        interval = 0.500;
    else if(noteTime == "3")
        interval = 1;

    code =  'sleep(' + interval + ')\n' +
            'buzzer.duty_u16(0)\n';

    return code;
};

Blockly.Arduino['DirectionSpeed'] = function (block) {

    var direction = block.getFieldValue('Direction');
    var speed = Blockly.Arduino.valueToCode(block, 'SPEED', Blockly.Arduino.ORDER_NONE) || '0';
    var code = '';

    Blockly.Arduino.imports_['import_Pin'] = 'from machine import Pin';
    Blockly.Arduino.imports_['import_ADC'] = 'from machine import ADC';
    Blockly.Arduino.imports_['import_PWM'] = 'from machine import PWM';

    Blockly.Arduino.definitions_['define_direction_motorA'] = '#motorA\n' +
                                                              'motor_A1 = PWM(Pin(15))\n' +
                                                              'motor_A1.duty_u16(0)\n' +
                                                              'motor_A2 = PWM(Pin(23))\n' +
                                                              'motor_A2.duty_u16(0)\n\n';

    Blockly.Arduino.definitions_['define_direction_motorB'] = '#motorB\n' +
                                                              'motor_B1 = PWM(Pin(32))\n' +
                                                              'motor_B1.duty_u16(0)\n' +
                                                              'motor_B2 = PWM(Pin(33))\n' +
                                                              'motor_B2.duty_u16(0)\n\n';

    Blockly.Arduino.definitions_['define_direction_motorC'] = '#motorC\n' +
                                                              'motor_C1 = PWM(Pin(5))\n' +
                                                              'motor_C1.duty_u16(0)\n' +
                                                              'motor_C2 = PWM(Pin(4))\n' +
                                                              'motor_C2.duty_u16(0)\n\n';

    Blockly.Arduino.definitions_['define_direction_motorD'] = '#motorD\n' +
                                                              'motor_D1 = PWM(Pin(27))\n' +
                                                              'motor_D1.duty_u16(0)\n' +
                                                              'motor_D2 = PWM(Pin(14))\n' +
                                                              'motor_D2.duty_u16(0)\n\n';

    if (direction == "forward") {
        Blockly.Arduino.definitions_['define_direction_forward'] =
            'def ' + direction + '(speedForward):\n' +
            '   motor_A1.duty_u16(speedForward * 650)\n' +
            '   motor_A2.duty_u16(0 * 650)\n\n' +
            '   motor_B1.duty_u16(speedForward * 650)\n' +
            '   motor_B2.duty_u16(0 * 650)\n\n' +
            '   motor_C1.duty_u16(speedForward * 650)\n' +
            '   motor_C2.duty_u16(0 * 650)\n\n' +
            '   motor_D1.duty_u16(speedForward * 650)\n' +
            '   motor_D2.duty_u16(0 * 650)\n\n' +
            '   return\n\n';

    }
    else if (direction == "backward") {
        Blockly.Arduino.definitions_['define_direction_backward'] =
            'def ' + direction + '(speedBackward):\n' +
            '   motor_A1.duty_u16(0 * 650)\n' +
            '   motor_A2.duty_u16(speedBackward * 650)\n\n' +
            '   motor_B1.duty_u16(0 * 650)\n' +
            '   motor_B2.duty_u16(speedBackward * 650)\n\n' +
            '   motor_C1.duty_u16(0 * 650)\n' +
            '   motor_C2.duty_u16(speedBackward * 650)\n\n' +
            '   motor_D1.duty_u16(0 * 650)\n' +
            '   motor_D2.duty_u16(speedBackward * 650)\n\n' +
            '   return\n\n';

    }
    else if (direction == "left") {
        Blockly.Arduino.definitions_['define_direction_left'] =
            'def ' + direction + '(speedLeft):\n' +
            '   motor_A1.duty_u16(0 * 650)\n' +
            '   motor_A2.duty_u16(speedLeft * 650)\n\n' +
            '   motor_B1.duty_u16(0 * 650)\n' +
            '   motor_B2.duty_u16(speedLeft * 650)\n\n' +
            '   motor_C1.duty_u16(speedLeft * 650)\n' +
            '   motor_C2.duty_u16(0 * 650)\n\n' +
            '   motor_D1.duty_u16(speedLeft * 650)\n' +
            '   motor_D2.duty_u16(0 * 650)\n' +
            '   return\n\n';

    }

    else if (direction == "right") {
        Blockly.Arduino.definitions_['define_direction_right'] =
            'def ' + direction + '(speedRight):\n' +
            '   motor_A1.duty_u16(speedRight * 650)\n' +
            '   motor_A2.duty_u16(0 * 650)\n\n' +
            '   motor_B1.duty_u16(speedRight * 650)\n' +
            '   motor_B2.duty_u16(0 * 650)\n\n' +
            '   motor_C1.duty_u16(0 * 650)\n' +
            '   motor_C2.duty_u16(speedRight * 650)\n\n' +
            '   motor_D1.duty_u16(0 * 650)\n' +
            '   motor_D2.duty_u16(speedRight * 650)\n' +
            '   return\n\n';
            
    }
    code = direction + '(' + speed + ')\n\n';

    return code;
};

Blockly.Arduino['stopMotors'] = function (block) {
    var code = '';

    Blockly.Arduino.imports_['import_Pin'] = 'from machine import Pin';
    Blockly.Arduino.imports_['import_PWM'] = 'from machine import PWM';
    Blockly.Arduino.imports_['import_ADC'] = 'from machine import ADC';

    Blockly.Arduino.definitions_['define_stop_motors'] =
        'def stop():\n' +
        '   motor_A1.duty_u16(0 * 650)\n' +
        '   motor_A2.duty_u16(0 * 650)\n\n' +
        '   motor_B1.duty_u16(0 * 650)\n' +
        '   motor_B2.duty_u16(0 * 650)\n\n' +
        '   motor_C1.duty_u16(0 * 650)\n' +
        '   motor_C2.duty_u16(0 * 650)\n\n' +
        '   motor_D1.duty_u16(0 * 650)\n' +
        '   motor_D2.duty_u16(0 * 650)\n\n';

    code = 'stop()\n';

    return code;
};

Blockly.Arduino['trackingState'] = function (block) {
    var value = block.getFieldValue('VALUE');
    var code = '';

    Blockly.Arduino.imports_['import_machine'] = 'import machine';

    if (value == "forward") {
        code = 'leftSensor.read_u16() >= THRESHOLD and rightSensor.read_u16() >= THRESHOLD'
    }
    else if (value == "right") {
        code = 'leftSensor.read_u16() < THRESHOLD and rightSensor.read_u16() > THRESHOLD'
    }
    else if (value == "left") {
        code = 'leftSensor.read_u16() > THRESHOLD and rightSensor.read_u16() < THRESHOLD'
    }
    else if (value == "backward") {
        code = 'leftSensor.read_u16() < THRESHOLD and rightSensor.read_u16() < THRESHOLD and directionStt != STOP'
    }

    return [code, Blockly.Arduino.ORDER_NONE];
};

Blockly.Arduino['trackingSensor'] = function (block) {
    var value = block.getFieldValue('VALUE');
    var code = '';

    Blockly.Arduino.imports_['import_machine'] = 'import machine';
    Blockly.Arduino.imports_['import_Pin'] = 'from machine import Pin';
    Blockly.Arduino.imports_['import_ADC'] = 'from machine import ADC';

    if (value == 'left') {
        Blockly.Arduino.definitions_['define_tracking_sensor1'] =
            'leftSensor = ADC(Pin(34))\n' +
            'leftSensor.read_u16()\n'
    }
    else {
        Blockly.Arduino.definitions_['define_tracking_sensor2'] =
            'rightSensor = ADC(Pin(35))\n' +
            'rightSensor.read_u16()\n'
    }
    return [code, Blockly.Arduino.ORDER_NONE];
};
  
Blockly.Arduino['axisAcceleration'] = function (block) {

    var value = block.getFieldValue('VALUE');
    var code = '';

    Blockly.Arduino.imports_['import_MPU6050'] = 'import mpu6050';

    Blockly.Arduino.definitions_['define_axis'] = 'i2c = I2C(scl=Pin(22), sda=Pin(21))\n' +
        '#initializing the I2C method for ESP32\n' +
        'mpu = mpu6050.accel(i2c)\n';

    code = 'mpu.get_values()["Ac' + value + '"]';

    return [code, Blockly.Arduino.ORDER_NONE];
};

Blockly.Arduino['pinControl'] = function (block) {

    var value = block.getFieldValue('VALUE');
    var pin = Blockly.Arduino.valueToCode(block, 'INPUT', Blockly.Arduino.ORDER_NONE) || '0';
    var code = '';

    if (value === "PWM") {
        Blockly.Arduino.imports_['import_PWM'] = 'from machine import PWM';
    } else {
        Blockly.Arduino.imports_['import_ADC'] = 'from machine import ADC';
    }

    code = value + '(Pin(' + pin + '))';

    return [code, Blockly.Arduino.ORDER_NONE];
};

Blockly.Arduino['readDistance'] = function (block) {
    var meter = block.getFieldValue('VALUE');

    Blockly.Arduino.imports_['import_time_pulse_us'] = 'from machine import time_pulse_us';
    Blockly.Arduino.imports_['import_HCSR04'] = 'from hcsr04 import HCSR04';

    Blockly.Arduino.definitions_['define_distance'] = 'sensor = HCSR04(trigger_pin=17, echo_pin=16, echo_timeout_us=10000)';

    var code = 'sensor.distance_'+ meter +'()';

    return [code, Blockly.Arduino.ORDER_NONE];
};

Blockly.Arduino['servoMotor'] = function(block) {

    var motor = block.getFieldValue('MOTOR');
    var angle =  Blockly.Arduino.valueToCode(block, 'ANGLE', Blockly.Arduino.ORDER_NONE) || '0';
    var code = "";
    var pin = 0;

    if(motor == "1")
        pin = ServoPin1;
    else if(motor == "2")
        pin = ServoPin2;
    else if(motor == "3")
        pin = ServoPin3;
    else
        pin = ServoPin4;

    Blockly.Arduino.imports_['import_Pin'] = 'from machine import Pin';
    Blockly.Arduino.imports_['import_PWM'] = 'from machine import PWM';
     Blockly.Arduino.imports_['import_fabs'] = 'from math import fabs';

    Blockly.Arduino.definitions_['define_servo1' + motor] = 'pwm_' + motor + ' = PWM(Pin(' + pin + '))';
    Blockly.Arduino.definitions_['define_servo2' + motor] = 'pwm_' + motor + '.freq(50)';

    Blockly.Arduino.definitions_['define_angleFunc'] = 
                                                        'def CalculateAngle(angle): \n' +
                                                        '   angle = fabs((angle * (6000 / 180)) + 2000) \n' +
                                                        '   angle = round(angle) \n' +
                                                        '   return angle \n';
    
    code = 'pwm_' + motor + '.duty_u16(CalculateAngle(' + angle + '))\n';

    return code;
};

Blockly.Arduino['dcMotor'] = function(block) {

    var motor = block.getFieldValue('MOTOR');
    var speed =  Blockly.Arduino.valueToCode(block, 'SPEED', Blockly.Arduino.ORDER_NONE) || '0';

    var code = "";
    var pin = 0;

    if(motor == "1")
        pin = MotorPin1;
    else
        pin = MotorPin2;

    Blockly.Arduino.imports_['import_PWM'] = 'from machine import PWM';
    Blockly.Arduino.imports_['import_Pin'] = 'from machine import Pin';
    Blockly.Arduino.imports_['import_ADC'] = 'from machine import ADC';

    Blockly.Arduino.definitions_['define_motor1' + motor] = 'motor_' + motor + ' = PWM(Pin(' + pin + '))';
    Blockly.Arduino.definitions_['define_motor2' + motor] = 'motor_' + motor + '.duty_u16(0)';

    code = 'motor_' + motor + '.duty_u16(' + speed + ' * 650) \n';

    return code;
};

Blockly.Arduino['timer'] = function(block) {

    var code = "";
    Blockly.Arduino.imports_['import_Timer'] = 'from machine import Timer';
    Blockly.Arduino.imports_['import_utime'] = 'import utime';
    
    code = 'utime.ticks_ms()';

    return [code, Blockly.Arduino.ORDER_NONE];  
}

Blockly.Arduino['resettimer'] = function(block) {

    var timer =  Blockly.Arduino.valueToCode(block, 'Timer', Blockly.Arduino.ORDER_ASSIGNMENT);

    var code = "";
    Blockly.Arduino.imports_['import_Timer'] = 'from machine import Timer';
    Blockly.Arduino.imports_['import_utime'] = 'import utime';

    code = 'tm_react = utime.ticks_diff(utime.ticks_ms(), ' + timer + ')\n';

    return code;
}

Blockly.Arduino['connect_wifi'] = function(block) {

    var ssid =  Blockly.Arduino.valueToCode(block, 'SSID', Blockly.Arduino.ORDER_NONE);
    var password =  Blockly.Arduino.valueToCode(block, 'Password', Blockly.Arduino.ORDER_NONE);
    
    ssid = ssid.replaceAll("\"", "");
    password = password.replaceAll("\"", "");

    var code = "";
    Blockly.Arduino.imports_['import_network'] = 'import network';
    Blockly.Arduino.imports_['import_socket'] = 'import socket';

    code = 
            "ssid = '" + ssid + "'\n" +
            "password = '" + password + "' \n" +
            "wlan = network.WLAN(network.STA_IF) \n" +
            "wlan.active(True) \n" +
            "wlan.connect(ssid, password) \n";

    return code;
}

Blockly.Arduino['html_input'] = function(block) {
 
    var html =  Blockly.Arduino.valueToCode(block, 'HTML', Blockly.Arduino.ORDER_NONE);
    var code = '';

    code = '\"\"' + html + '\"\"';
    return [code, Blockly.Arduino.ORDER_NONE];  
};

Blockly.Arduino['ip_address'] = function(block) {
    return ["status[0]", Blockly.Arduino.ORDER_NONE];  
};

Blockly.Arduino['show_ip'] = function(block) {

    var ipadress =  Blockly.Arduino.valueToCode(block, 'IP_Address', Blockly.Arduino.ORDER_NONE);

    ipadress = ipadress.replaceAll("\"", "");

    var code = "";

    code = 
            "if wlan.status() != 3: \n" +
            "   raise RuntimeError('network connection failed') \n" +
            "else: \n" +
            "   print('Connected') \n" +
            "   status = wlan.ifconfig() \n" +
            "   print( 'ip = ' + " + ipadress + " ) \n";

    return code;
}

Blockly.Arduino['open_socket'] = function(block) {

    var ipadress =  Blockly.Arduino.valueToCode(block, 'IP_Address', Blockly.Arduino.ORDER_NONE);
    var port =  Blockly.Arduino.valueToCode(block, 'Port', Blockly.Arduino.ORDER_NONE);
    
    ipadress = ipadress.replaceAll("\"", "");

    var code = "";

    code = 
            "addr = socket.getaddrinfo('" + ipadress + "', " + port + ")[0][-1] \n" +
            "s = socket.socket() \n" +
            "s.bind(addr) \n" +
            "s.listen(1) \n";

    return code;
}

Blockly.Arduino['wait_for_connection'] = function(block) {

    var time =  Blockly.Arduino.valueToCode(block, 'Time', Blockly.Arduino.ORDER_NONE);

    Blockly.Arduino.imports_['import_utime'] = "import utime";
    Blockly.Arduino.imports_['import_time'] = "import time";
    var code = "";
    code = 
            "max_wait = " + time + " \n" +
            "while max_wait > 0: \n" +
            "    if wlan.status() < 0 or wlan.status() >= 3: \n" +
            "        break \n" +
            "    max_wait -= 1 \n" +
            "    print('waiting for connection...') \n" +
            "    time.sleep(1) \n";

    return code;
}

Blockly.Arduino['accept_connection'] = function(block) {

    var html =  Blockly.Arduino.valueToCode(block, 'HTML', Blockly.Arduino.ORDER_NONE);

    var code = "";
    
    code = 
            "cl, addr = s.accept() \n" +
            "cl.send('HTTP/1.0 200 OK\\r\\nContent-type: text/html\\r\\n\\r\\n') \n" +
            "cl.send(" + html + ") \n" +
            "cl.close() \n";

    return code;
}

Blockly.Arduino['request'] = function(block) {

    var code = "";
    
    code = 
            "cl, addr = s.accept() \n" +
            "request = cl.recv(1024) \n" +
            "request = str(request) \n";

    return code;
}

Blockly.Arduino['request_find'] = function(block) {

    var find =  Blockly.Arduino.valueToCode(block, 'Find', Blockly.Arduino.ORDER_NONE);
    
    var code = "request.find(" + find + ")"

    return [code, Blockly.Arduino.ORDER_NONE];  
}


Blockly.Arduino['isButtonPressed'] = function(block) {

    var value = block.getFieldValue('VALUE');
  
    var code = "ir_data == " + value;

    return [code, Blockly.Arduino.ORDER_NONE];
};